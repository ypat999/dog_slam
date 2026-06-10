#!/usr/bin/env python3

# 订阅/map获取二维占用栅格地图
# 订阅/scan获取当前激光扫描数据
# 使用AMCL粒子滤波：在全地图自由空间均匀播撒粒子
# 使用似然场模型评估每个粒子的得分
# 记录得分最高的粒子，发布到/initialpose话题

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from collections import deque


class AmclInitialPoseEstimator(Node):
    """
    AMCL 初始位姿估计节点

    功能：
    1. 订阅 /map 话题，获取占用栅格地图并预计算似然场
    2. 订阅 /scan 话题，缓存激光扫描数据
    3. 在地图自由空间中均匀播撒粒子
    4. 使用似然场模型对每个粒子评分
    5. 发布得分最高粒子的位姿到 /initialpose
    """

    def __init__(self):
        super().__init__('amcl_initial_pose_estimator')

        # ---------- 参数 ----------
        self.declare_parameter('particle_count', 3000)       # 粒子数量
        self.declare_parameter('map_resolution', 0.05)       # 地图分辨率 (m/pixel)
        self.declare_parameter('scan_range_max', 50.0)       # 扫描最大范围 (m)
        self.declare_parameter('laser_min_range', 0.1)       # 激光最小范围 (m)
        self.declare_parameter('laser_max_range', 100.0)     # 激光最大范围 (m)
        self.declare_parameter('max_beams', 180)             # 每次评分使用的激光束数
        self.declare_parameter('sigma_hit', 0.08)            # 似然场高斯标准差 (m)
        self.declare_parameter('z_hit', 0.60)                # 命中权重
        self.declare_parameter('z_rand', 0.40)               # 随机权重
        self.declare_parameter('likelihood_max_dist', 2.0)   # 似然场最大搜索距离 (m)
        self.declare_parameter('trigger_on_map', True)       # 收到地图后自动触发估计

        self.particle_count = self.get_parameter('particle_count').value
        self.map_resolution = self.get_parameter('map_resolution').value
        self.scan_range_max = self.get_parameter('scan_range_max').value
        self.laser_min_range = self.get_parameter('laser_min_range').value
        self.laser_max_range = self.get_parameter('laser_max_range').value
        self.max_beams = self.get_parameter('max_beams').value
        self.sigma_hit = self.get_parameter('sigma_hit').value
        self.z_hit = self.get_parameter('z_hit').value
        self.z_rand = self.get_parameter('z_rand').value
        self.likelihood_max_dist = self.get_parameter('likelihood_max_dist').value
        self.trigger_on_map = self.get_parameter('trigger_on_map').value

        # ---------- 数据存储 ----------
        self.map_image = None          # numpy 2D array: 0=占用, 255=空闲
        self.map_info = None           # OccupancyGrid.info
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_width = 0
        self.map_height = 0
        self.likelihood_field = None   # 预计算的似然场 (距离变换)
        self.free_space_indices = None # 自由空间像素坐标 (rows, cols)
        self.scan_msg = None           # 最新一帧激光扫描
        self.has_estimated = False     # 是否已经完成估计

        # ---------- QoS ----------
        best_effort_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # ---------- 订阅 ----------
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, best_effort_qos)

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, best_effort_qos)

        # ---------- 发布 ----------
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        self.get_logger().info('AMCL 初始位姿估计节点已启动')

    # ============================================================
    #  地图回调
    # ============================================================
    def map_callback(self, msg):
        try:
            self.map_info = msg.info
            self.map_origin_x = msg.info.origin.position.x
            self.map_origin_y = msg.info.origin.position.y
            self.map_width = msg.info.width
            self.map_height = msg.info.height

            # 转换为 numpy 数组
            map_data = np.array(msg.data, dtype=np.int8).reshape(
                (self.map_height, self.map_width))

            # 0=占用, 255=空闲
            self.map_image = np.zeros((self.map_height, self.map_width), dtype=np.uint8)
            self.map_image[map_data == -1] = 128   # 未知 (归为空闲以增加粒子覆盖)
            self.map_image[map_data == 0] = 255
            self.map_image[map_data == 100] = 0

            # 预计算似然场 (距离变换)
            self._build_likelihood_field()

            # 收集自由空间像素坐标（用于均匀采样粒子）
            free_mask = (self.map_image >= 128)  # 空闲 + 未知
            rows, cols = np.where(free_mask)
            self.free_space_indices = np.stack([rows, cols], axis=1)

            self.get_logger().info(
                f'地图已更新: {self.map_width}x{self.map_height}, '
                f'自由空间像素: {len(self.free_space_indices)}, '
                f'原点: ({self.map_origin_x:.2f}, {self.map_origin_y:.2f})')

            # 如果有扫描数据且需要自动触发
            if self.trigger_on_map and self.scan_msg is not None and not self.has_estimated:
                self.get_logger().info('收到地图，自动触发初始位姿估计...')
                self.estimate_initial_pose()

        except Exception as e:
            self.get_logger().error(f'处理地图数据时出错: {str(e)}')

    # ============================================================
    #  构建似然场 (距离变换)
    # ============================================================
    def _build_likelihood_field(self):
        """对占用栅格地图做距离变换，生成似然场"""
        # 障碍物二值图: 1=障碍物, 0=空闲
        obstacle_binary = (self.map_image == 0).astype(np.uint8)

        # 使用 OpenCV 距离变换
        import cv2
        max_dist_pixels = self.likelihood_max_dist / self.map_resolution
        dist = cv2.distanceTransform(
            1 - obstacle_binary, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)

        # 截断到最大距离
        dist = np.clip(dist, 0, max_dist_pixels)
        self.likelihood_field = dist.astype(np.float32)

        self.get_logger().info(
            f'似然场已构建, 最大距离: {self.likelihood_max_dist}m '
            f'({max_dist_pixels:.0f}像素)')

    # ============================================================
    #  扫描回调
    # ============================================================
    def scan_callback(self, msg):
        self.scan_msg = msg

        # 如果已有地图且未估计，自动触发
        if self.trigger_on_map and self.map_image is not None and not self.has_estimated:
            self.estimate_initial_pose()

    # ============================================================
    #  均匀采样粒子
    # ============================================================
    def _sample_particles_uniform(self):
        """在地图自由空间中均匀采样粒子"""

        if self.free_space_indices is None or len(self.free_space_indices) == 0:
            self.get_logger().error('没有自由空间可采样')
            return None

        n_free = len(self.free_space_indices)
        n_sample = min(self.particle_count, n_free)

        # 均匀随机采样像素位置
        idx = np.random.choice(n_free, size=n_sample, replace=False)
        sampled_pixels = self.free_space_indices[idx]  # (N, 2): [row, col]

        # 像素坐标 -> 世界坐标
        # row = (map_origin_y - y) / resolution + height, 反转后
        # 地图数组第0行对应世界坐标y_max
        world_x = (sampled_pixels[:, 1] + 0.5) * self.map_info.resolution + self.map_origin_x
        world_y = (self.map_height - sampled_pixels[:, 0] - 0.5) * self.map_info.resolution + self.map_origin_y

        # 均匀随机采样朝向
        world_yaw = np.random.uniform(-math.pi, math.pi, size=n_sample)

        particles = np.stack([world_x, world_y, world_yaw], axis=1)  # (N, 3)
        return particles

    # ============================================================
    #  似然场评分
    # ============================================================
    def _score_particle(self, px, py, pyaw):
        """对单个粒子评分，使用似然场模型"""

        if self.scan_msg is None or self.likelihood_field is None:
            return 0.0

        # 下采样激光束
        ranges = self.scan_msg.ranges
        angle_min = self.scan_msg.angle_min
        angle_increment = self.scan_msg.angle_increment
        num_beams = len(ranges)

        step = max(1, num_beams // self.max_beams)
        indices = range(0, num_beams, step)

        total_weight = 0.0
        z_hit_denom = 1.0 / (self.sigma_hit * math.sqrt(2.0 * math.pi))
        z_rand_mult = 1.0 / self.laser_max_range

        for i in indices:
            r = ranges[i]
            if r < self.laser_min_range or r > self.laser_max_range:
                continue

            # 激光端点在机器人坐标系
            angle = angle_min + i * angle_increment
            lx_local = r * math.cos(angle)
            ly_local = r * math.sin(angle)

            # 转换到世界坐标系
            cos_yaw = math.cos(pyaw)
            sin_yaw = math.sin(pyaw)
            lx_world = px + cos_yaw * lx_local - sin_yaw * ly_local
            ly_world = py + sin_yaw * lx_local + cos_yaw * ly_local

            # 查似然场
            dist = self._lookup_likelihood(lx_world, ly_world)
            if dist < 0:
                continue

            # 似然场模型
            dist_m = dist * self.map_resolution
            p_hit = z_hit_denom * math.exp(-0.5 * (dist_m / self.sigma_hit) ** 2)
            p = self.z_hit * p_hit + self.z_rand * z_rand_mult

            if p > 0:
                total_weight += math.log(p)

        return total_weight

    def _lookup_likelihood(self, wx, wy):
        """查询世界坐标(wx, wy)处的似然场距离（像素单位）"""
        # 世界坐标 -> 像素坐标
        col = int((wx - self.map_origin_x) / self.map_info.resolution)
        row = int((wy - self.map_origin_y) / self.map_info.resolution)
        # 翻转row（地图第0行对应y_max）
        row = self.map_height - 1 - row

        if 0 <= row < self.map_height and 0 <= col < self.map_width:
            return self.likelihood_field[row, col]
        return -1.0

    # ============================================================
    #  主估计流程
    # ============================================================
    def estimate_initial_pose(self):
        """执行初始位姿估计"""
        if self.map_image is None:
            self.get_logger().warn('地图尚未就绪')
            return
        if self.scan_msg is None:
            self.get_logger().warn('扫描数据尚未就绪')
            return
        if self.likelihood_field is None:
            self.get_logger().warn('似然场尚未构建')
            return
        if self.has_estimated:
            self.get_logger().info('已经完成估计，跳过')
            return

        self.get_logger().info('开始初始位姿估计...')

        # 1. 均匀采样粒子
        particles = self._sample_particles_uniform()
        if particles is None or len(particles) == 0:
            self.get_logger().error('粒子采样失败')
            return

        self.get_logger().info(f'已采样 {len(particles)} 个粒子')

        # 2. 对每个粒子评分
        best_score = -float('inf')
        best_particle = None
        scores = []

        for i, (px, py, pyaw) in enumerate(particles):
            score = self._score_particle(px, py, pyaw)
            scores.append(score)

            if score > best_score:
                best_score = score
                best_particle = (px, py, pyaw)

            if (i + 1) % 500 == 0:
                self.get_logger().info(f'评分进度: {i+1}/{len(particles)}')

        if best_particle is None:
            self.get_logger().error('未找到有效粒子')
            return

        # 3. 输出统计信息
        scores_arr = np.array(scores)
        self.get_logger().info(
            f'评分统计: best={best_score:.2f}, mean={scores_arr.mean():.2f}, '
            f'median={np.median(scores_arr):.2f}')

        # 4. 发布 /initialpose
        bx, by, byaw = best_particle
        self.get_logger().info(
            f'最佳初始位姿: x={bx:.3f}, y={by:.3f}, yaw={byaw:.3f}rad ({math.degrees(byaw):.1f}deg)')

        self._publish_initial_pose(bx, by, byaw)
        self.has_estimated = True

        self.get_logger().info('初始位姿已发布到 /initialpose')

    # ============================================================
    #  发布 initialpose
    # ============================================================
    def _publish_initial_pose(self, x, y, yaw):
        """发布 PoseWithCovarianceStamped 到 /initialpose"""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.pose.position = Point(x=x, y=y, z=0.0)
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)

        # 协方差矩阵 (与 nav2_amcl 默认一致)
        cov = [0.0] * 36
        cov[0] = 0.25   # x 方差
        cov[7] = 0.25   # y 方差
        cov[35] = 0.06853892326654787  # yaw 方差
        msg.pose.covariance = cov

        self.initial_pose_pub.publish(msg)

    # ============================================================
    #  手动触发估计的服务接口 (可选扩展)
    # ============================================================
    def reset_and_estimate(self):
        """重置状态并重新估计"""
        self.has_estimated = False
        if self.map_image is not None and self.scan_msg is not None:
            self.estimate_initial_pose()
        else:
            self.get_logger().warn('地图或扫描数据未就绪，等待数据...')


def main(args=None):
    rclpy.init(args=args)

    node = AmclInitialPoseEstimator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
