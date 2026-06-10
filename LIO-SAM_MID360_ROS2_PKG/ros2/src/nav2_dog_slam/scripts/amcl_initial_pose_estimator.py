#!/usr/bin/env python3

# AMCL 全局搜索初始位姿估计器
# 策略：
# 1. 订阅 /map 获取占用栅格地图，预计算似然场
# 2. 订阅 /scan 获取当前激光扫描
# 3. 按地图尺寸每隔 3 米在自由空间生成网格候选点
# 4. 每个候选点每个朝向按噪声规则生成散布粒子群（模拟 AMCL 收到 initialpose 后的粒子初始化）
# 5. 第一轮：全地图搜索，对每个散布群评分，记录得分最高的前 5 个散布粒子
# 6. 第二轮：对前 5 个散布粒子中心重新生成散布群精细评分，取最优
# 7. 发布最终 /initialpose 实现全局定位

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion


class AmclGlobalSearchEstimator(Node):
    """
    AMCL 全局搜索初始位姿估计节点

    通过在地图上按网格搜索所有自由空间候选点，
    每个候选点按噪声规则生成散布粒子群，
    使用似然场模型内部评分，两轮筛选找到最佳初始位姿。
    """

    def __init__(self):
        super().__init__('amcl_global_search_estimator')

        # ---------- 参数（与 nav2_params_zg_3d_airy.yaml 中 amcl 参数对齐）----------
        # 网格搜索参数
        self.declare_parameter('grid_spacing', 3.0)             # 网格间距 (m)

        # 散布粒子参数（模拟 AMCL 收到 initialpose 后的粒子初始化）
        self.declare_parameter('scatter_count', 30)             # 每个候选朝向的散布粒子数
        self.declare_parameter('scatter_sigma_xy', 0.5)         # 散布位置噪声标准差 (m)，对应 AMCL initial_cov[0]=0.25
        self.declare_parameter('scatter_sigma_yaw', 0.26)       # 散布朝向噪声标准差 (rad)，对应 AMCL initial_cov[35]=0.0685
        self.declare_parameter('refine_scatter_count', 100)     # 第二轮精细散布粒子数

        # AMCL 似然场模型参数
        self.declare_parameter('sigma_hit', 0.08)               # 似然场高斯标准差 (m)
        self.declare_parameter('z_hit', 0.60)                   # 命中权重
        self.declare_parameter('z_rand', 0.40)                  # 随机权重
        self.declare_parameter('laser_min_range', 0.1)          # 激光最小范围 (m)
        self.declare_parameter('laser_max_range', 200.0)        # 激光最大范围 (m)
        self.declare_parameter('max_beams', 360)                # 第一轮评分使用的激光束数
        self.declare_parameter('likelihood_max_dist', 100.0)    # 似然场最大搜索距离 (m)

        # 搜索策略参数
        self.declare_parameter('top_k', 5)                      # 第一轮取前 K 个候选
        self.declare_parameter('refine_beams', 720)             # 第二轮精细评分使用的激光束数
        self.declare_parameter('trigger_on_map', True)          # 收到地图后自动触发
        self.declare_parameter('free_threshold', 128)           # 自由空间阈值

        # 读取参数
        self.grid_spacing = self.get_parameter('grid_spacing').value
        self.scatter_count = self.get_parameter('scatter_count').value
        self.scatter_sigma_xy = self.get_parameter('scatter_sigma_xy').value
        self.scatter_sigma_yaw = self.get_parameter('scatter_sigma_yaw').value
        self.refine_scatter_count = self.get_parameter('refine_scatter_count').value
        self.sigma_hit = self.get_parameter('sigma_hit').value
        self.z_hit = self.get_parameter('z_hit').value
        self.z_rand = self.get_parameter('z_rand').value
        self.laser_min_range = self.get_parameter('laser_min_range').value
        self.laser_max_range = self.get_parameter('laser_max_range').value
        self.max_beams = self.get_parameter('max_beams').value
        self.likelihood_max_dist = self.get_parameter('likelihood_max_dist').value
        self.top_k = self.get_parameter('top_k').value
        self.refine_beams = self.get_parameter('refine_beams').value
        self.trigger_on_map = self.get_parameter('trigger_on_map').value
        self.free_threshold = self.get_parameter('free_threshold').value

        # 4 个朝向：上(0°) / 右(90°) / 下(180°) / 左(270°)
        self.direction_yaws = [0.0, math.pi / 2, math.pi, -math.pi / 2]
        self.direction_names = ['上(0°)', '右(90°)', '下(180°)', '左(270°)']

        # ---------- 数据存储 ----------
        self.map_image = None
        self.map_info = None
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_width = 0
        self.map_height = 0
        self.likelihood_field = None
        self.scan_msg = None
        self.has_estimated = False

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

        self.get_logger().info('AMCL 全局搜索初始位姿估计节点已启动')
        self.get_logger().info(
            f'网格间距: {self.grid_spacing}m, 朝向: 4方向, '
            f'散布粒子: {self.scatter_count}/候选, '
            f'噪声: sigma_xy={self.scatter_sigma_xy}m, sigma_yaw={self.scatter_sigma_yaw}rad, '
            f'Top-K: {self.top_k}')

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

            # 0=占用, 255=空闲, 128=未知
            self.map_image = np.zeros((self.map_height, self.map_width), dtype=np.uint8)
            self.map_image[map_data == -1] = 128
            self.map_image[map_data == 0] = 255
            self.map_image[map_data == 100] = 0

            # 预计算似然场
            self._build_likelihood_field()

            self.get_logger().info(
                f'地图已更新: {self.map_width}x{self.map_height}, '
                f'分辨率: {self.map_info.resolution:.3f}m/pixel, '
                f'原点: ({self.map_origin_x:.2f}, {self.map_origin_y:.2f}), '
                f'尺寸: {self.map_width * self.map_info.resolution:.1f}m x '
                f'{self.map_height * self.map_info.resolution:.1f}m')

            if self.trigger_on_map and self.scan_msg is not None and not self.has_estimated:
                self.get_logger().info('收到地图，自动触发全局搜索...')
                self.run_global_search()

        except Exception as e:
            self.get_logger().error(f'处理地图数据时出错: {str(e)}')

    # ============================================================
    #  构建似然场 (距离变换)
    # ============================================================
    def _build_likelihood_field(self):
        """对占用栅格地图做距离变换，生成似然场"""
        import cv2

        obstacle_binary = (self.map_image == 0).astype(np.uint8)
        max_dist_pixels = self.likelihood_max_dist / self.map_info.resolution
        dist = cv2.distanceTransform(
            1 - obstacle_binary, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
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
        if self.trigger_on_map and self.map_image is not None and not self.has_estimated:
            self.run_global_search()

    # ============================================================
    #  生成网格候选点
    # ============================================================
    def _generate_grid_candidates(self):
        """
        按地图尺寸每隔 grid_spacing 米在自由空间生成候选点。
        每个候选点生成 4 个朝向（上/下/左/右）。
        返回: list of (x, y, yaw, direction_name)
        """
        resolution = self.map_info.resolution
        map_w_m = self.map_width * resolution
        map_h_m = self.map_height * resolution

        candidates = []
        x = self.map_origin_x
        while x < self.map_origin_x + map_w_m:
            y = self.map_origin_y
            while y < self.map_origin_y + map_h_m:
                if self._is_free(x, y):
                    for yaw, name in zip(self.direction_yaws, self.direction_names):
                        candidates.append((x, y, yaw, name))
                y += self.grid_spacing
            x += self.grid_spacing

        return candidates

    def _is_free(self, wx, wy):
        """检查世界坐标 (wx, wy) 是否在自由空间"""
        col = int((wx - self.map_origin_x) / self.map_info.resolution)
        row = int((wy - self.map_origin_y) / self.map_info.resolution)
        row = self.map_height - 1 - row

        if 0 <= row < self.map_height and 0 <= col < self.map_width:
            return self.map_image[row, col] >= self.free_threshold
        return False

    # ============================================================
    #  散布粒子生成（模拟 AMCL 收到 initialpose 后的粒子初始化）
    # ============================================================
    def _generate_scatter(self, cx, cy, cyaw, count, sigma_xy, sigma_yaw):
        """
        围绕中心位姿 (cx, cy, cyaw) 按高斯噪声生成散布粒子群。
        模拟 AMCL 收到 /initialpose 后根据协方差播撒粒子的过程。

        参数:
            cx, cy, cyaw: 中心位姿
            count: 散布粒子数量
            sigma_xy: 位置噪声标准差 (m)
            sigma_yaw: 朝向噪声标准差 (rad)

        返回:
            numpy array (count, 3): [x, y, yaw]
        """
        dx = np.random.normal(0, sigma_xy, count)
        dy = np.random.normal(0, sigma_xy, count)
        dyaw = np.random.normal(0, sigma_yaw, count)

        particles = np.stack([cx + dx, cy + dy, cyaw + dyaw], axis=1)
        return particles

    # ============================================================
    #  似然场评分
    # ============================================================
    def _score_particle(self, px, py, pyaw, max_beams=None):
        """对单个粒子评分，使用似然场模型"""
        if self.scan_msg is None or self.likelihood_field is None:
            return 0.0

        if max_beams is None:
            max_beams = self.max_beams

        ranges = self.scan_msg.ranges
        angle_min = self.scan_msg.angle_min
        angle_increment = self.scan_msg.angle_increment
        num_beams = len(ranges)

        step = max(1, num_beams // max_beams)
        indices = range(0, num_beams, step)

        total_weight = 0.0
        z_hit_denom = 1.0 / (self.sigma_hit * math.sqrt(2.0 * math.pi))
        z_rand_mult = 1.0 / self.laser_max_range

        cos_yaw = math.cos(pyaw)
        sin_yaw = math.sin(pyaw)

        for i in indices:
            r = ranges[i]
            if r < self.laser_min_range or r > self.laser_max_range:
                continue

            angle = angle_min + i * angle_increment
            lx_local = r * math.cos(angle)
            ly_local = r * math.sin(angle)

            lx_world = px + cos_yaw * lx_local - sin_yaw * ly_local
            ly_world = py + sin_yaw * lx_local + cos_yaw * ly_local

            dist = self._lookup_likelihood(lx_world, ly_world)
            if dist < 0:
                continue

            dist_m = dist * self.map_info.resolution
            p_hit = z_hit_denom * math.exp(-0.5 * (dist_m / self.sigma_hit) ** 2)
            p = self.z_hit * p_hit + self.z_rand * z_rand_mult

            if p > 0:
                total_weight += math.log(p)

        return total_weight

    def _lookup_likelihood(self, wx, wy):
        """查询世界坐标处的似然场距离（像素单位）"""
        col = int((wx - self.map_origin_x) / self.map_info.resolution)
        row = int((wy - self.map_origin_y) / self.map_info.resolution)
        row = self.map_height - 1 - row

        if 0 <= row < self.map_height and 0 <= col < self.map_width:
            return self.likelihood_field[row, col]
        return -1.0

    # ============================================================
    #  对散布粒子群评分，返回最佳粒子
    # ============================================================
    def _score_scatter_cluster(self, cx, cy, cyaw, scatter_count, sigma_xy, sigma_yaw,
                                max_beams=None):
        """
        对一个候选位姿生成散布粒子群并评分，返回最佳粒子及其得分。

        参数:
            cx, cy, cyaw: 候选中心位姿
            scatter_count: 散布粒子数
            sigma_xy: 位置噪声标准差
            sigma_yaw: 朝向噪声标准差
            max_beams: 评分使用的激光束数

        返回:
            (best_score, best_x, best_y, best_yaw, mean_score, cluster_scores)
        """
        particles = self._generate_scatter(cx, cy, cyaw, scatter_count, sigma_xy, sigma_yaw)

        best_score = -float('inf')
        best_x, best_y, best_yaw = cx, cy, cyaw
        scores = []

        for px, py, pyaw in particles:
            # 跳过不在自由空间的粒子
            if not self._is_free(px, py):
                scores.append(-float('inf'))
                continue
            score = self._score_particle(px, py, pyaw, max_beams=max_beams)
            scores.append(score)
            if score > best_score:
                best_score = score
                best_x, best_y, best_yaw = px, py, pyaw

        mean_score = np.mean([s for s in scores if s > -float('inf')]) if scores else -float('inf')

        return best_score, best_x, best_y, best_yaw, mean_score, scores

    # ============================================================
    #  主搜索流程
    # ============================================================
    def run_global_search(self):
        """执行两轮全局搜索"""
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

        self.get_logger().info('=' * 60)
        self.get_logger().info('开始 AMCL 全局搜索初始位姿估计')
        self.get_logger().info('=' * 60)

        # ============================================================
        # 第一轮：全地图网格搜索 + 散布粒子评分
        # ============================================================
        candidates = self._generate_grid_candidates()
        total = len(candidates)
        total_scatter_particles = total * self.scatter_count
        self.get_logger().info(
            f'第一轮: {total} 个候选 (网格 {self.grid_spacing}m x 4朝向), '
            f'每候选散布 {self.scatter_count} 粒子, '
            f'共 {total_scatter_particles} 粒子待评分')

        if total == 0:
            self.get_logger().error('未生成任何候选点，请检查地图')
            return

        # 记录所有散布粒子中表现最好的粒子
        # 格式: (score, x, y, yaw, dname, center_x, center_y, center_yaw)
        all_best_particles = []

        for idx, (cx, cy, cyaw, dname) in enumerate(candidates):
            # 对该候选生成散布粒子群并评分
            best_score, best_x, best_y, best_yaw, mean_score, _ = \
                self._score_scatter_cluster(
                    cx, cy, cyaw,
                    scatter_count=self.scatter_count,
                    sigma_xy=self.scatter_sigma_xy,
                    sigma_yaw=self.scatter_sigma_yaw,
                    max_beams=self.max_beams)

            # 记录该散布群中的最佳粒子
            all_best_particles.append(
                (best_score, best_x, best_y, best_yaw, dname, cx, cy, cyaw))

            if (idx + 1) % 50 == 0 or idx == total - 1:
                self.get_logger().info(
                    f'第一轮评分进度: {idx + 1}/{total} '
                    f'({100.0 * (idx + 1) / total:.1f}%), '
                    f'已评分粒子: {(idx + 1) * self.scatter_count}')

        # 按散布群最佳粒子得分降序排序，取前 top_k
        all_best_particles.sort(key=lambda t: t[0], reverse=True)
        top_particles = all_best_particles[:self.top_k]

        self.get_logger().info('-' * 60)
        self.get_logger().info(f'第一轮完成，Top-{self.top_k} 散布粒子:')
        for rank, (s, bx, by, byaw, dname, cx, cy, cyaw) in enumerate(top_particles, 1):
            self.get_logger().info(
                f'  #{rank}: score={s:.2f}, '
                f'最佳粒子=({bx:.3f}, {by:.3f}, {math.degrees(byaw):.1f}°), '
                f'中心=({cx:.3f}, {cy:.3f}, {math.degrees(cyaw):.1f}°) [{dname}]')

        # ============================================================
        # 第二轮：对 Top-K 粒子中心重新生成散布群精细评分
        # ============================================================
        self.get_logger().info('-' * 60)
        self.get_logger().info(
            f'第二轮: 对 Top-{self.top_k} 候选重新生成散布群精细评分 '
            f'(散布 {self.refine_scatter_count} 粒子, {self.refine_beams} 激光束)...')

        refined = []
        for rank, (s, bx, by, byaw, dname, cx, cy, cyaw) in enumerate(top_particles, 1):
            # 以第一轮最佳粒子为中心，重新生成更密集的散布群
            refine_best, refine_x, refine_y, refine_yaw, refine_mean, _ = \
                self._score_scatter_cluster(
                    bx, by, byaw,
                    scatter_count=self.refine_scatter_count,
                    sigma_xy=self.scatter_sigma_xy * 0.5,   # 第二轮缩小搜索范围
                    sigma_yaw=self.scatter_sigma_yaw * 0.5,
                    max_beams=self.refine_beams)

            refined.append((refine_best, refine_x, refine_y, refine_yaw, dname))
            self.get_logger().info(
                f'  #{rank}: 第一轮={s:.2f} -> 第二轮最佳={refine_best:.2f}, '
                f'粒子=({refine_x:.3f}, {refine_y:.3f}, {math.degrees(refine_yaw):.1f}°) '
                f'[{dname}], 散布均值={refine_mean:.2f}')

        # 取最终最优
        refined.sort(key=lambda t: t[0], reverse=True)
        best_score, best_x, best_y, best_yaw, best_dname = refined[0]

        self.get_logger().info('=' * 60)
        self.get_logger().info('全局搜索完成！最终结果:')
        self.get_logger().info(
            f'  最佳位姿: x={best_x:.3f}, y={best_y:.3f}, '
            f'yaw={math.degrees(best_yaw):.1f}° ({best_dname})')
        self.get_logger().info(f'  最佳得分: {best_score:.2f}')
        self.get_logger().info('=' * 60)

        # ============================================================
        # 发布 /initialpose
        # ============================================================
        self._publish_initial_pose(best_x, best_y, best_yaw)
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
        cov[0] = 0.25
        cov[7] = 0.25
        cov[35] = 0.06853892326654787
        msg.pose.covariance = cov

        self.initial_pose_pub.publish(msg)

    # ============================================================
    #  手动重置
    # ============================================================
    def reset_and_estimate(self):
        """重置状态并重新估计"""
        self.has_estimated = False
        if self.map_image is not None and self.scan_msg is not None:
            self.run_global_search()
        else:
            self.get_logger().warn('地图或扫描数据未就绪，等待数据...')


def main(args=None):
    rclpy.init(args=args)

    node = AmclGlobalSearchEstimator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
