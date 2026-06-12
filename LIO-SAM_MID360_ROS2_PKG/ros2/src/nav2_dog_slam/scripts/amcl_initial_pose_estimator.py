#!/usr/bin/env python3

# AMCL 全局搜索初始位姿估计器 (v3 - timer 驱动, 无阻塞, 一轮)
# 策略：
# 1. 订阅 /rkbot/map 获取自由空间
# 2. 生成网格候选位姿 (4朝向)
# 3. Timer 驱动状态机：发 /initialpose → 等 AMCL 回调 → 记录协方差 → 下一个
# 4. 一轮完成后取 Top-K 中最优，发布最终 /initialpose
#
# 本节点不自己做任何计算；所有评分由 AMCL 通过协方差反馈完成。
# 无嵌套 spin_once，回调由主 executor 自然处理。

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import copy
import cv2
import numpy as np
import math

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion


class AmclGlobalSearchEstimator(Node):
    """
    AMCL 全局搜索初始位姿估计节点 (v3 - timer 驱动, 一轮)
    """

    def __init__(self):
        super().__init__('amcl_global_search_estimator')

        # ---------- 参数 ----------
        self.declare_parameter('grid_spacing', 2.0)
        self.declare_parameter('top_k', 3)
        self.declare_parameter('trigger_on_map', True)
        self.declare_parameter('free_threshold', 250)
        self.declare_parameter('feedback_timeout', 2.5)
        self.declare_parameter('fine_search_radius', 1.0)
        self.declare_parameter('min_topk_distance', 4.0)
        self.declare_parameter('verify_settle_time', 3.0)
        self.declare_parameter('verify_cov_threshold', 0.000000001)

        self.grid_spacing = self.get_parameter('grid_spacing').value
        self.top_k = self.get_parameter('top_k').value
        self.trigger_on_map = self.get_parameter('trigger_on_map').value
        self.free_threshold = self.get_parameter('free_threshold').value
        self.feedback_timeout = self.get_parameter('feedback_timeout').value
        self.fine_search_radius = self.get_parameter('fine_search_radius').value
        self.min_topk_distance = self.get_parameter('min_topk_distance').value
        self.verify_settle_time = self.get_parameter('verify_settle_time').value
        self.verify_cov_threshold = self.get_parameter('verify_cov_threshold').value

        # 1 个朝向：右(-90°)
        self.direction_yaws = [0]
        self.direction_names = ['右(0°)']

        # ---------- 数据 ----------
        self.map_image = None
        self.map_info = None
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_width = 0
        self.map_height = 0
        self.has_estimated = False

        # AMCL 反馈
        self.latest_amcl_pose = None
        self._pending_feedback = False

        # ---------- 搜索状态机 ----------
        self._search_timer = None
        self._search_state = 'idle'      # idle | searching | done
        self._candidates = []
        self._results = []
        self._current_idx = 0
        self._publish_time = None
        self._waiting_for_feedback = False
        self._current_x = 0.0
        self._current_y = 0.0
        self._current_yaw = 0.0
        self._current_name = ''
        self._t_start = None
        self._fine_phase = False
        self._fine_candidates = []
        self._fine_results = []

        # ---------- 验证状态 ----------
        self._verify_results = []
        self._verify_idx = 0
        self._verify_settle_start = None
        self._verify_settled = False
        self._verify_snapshot = None  # 验证通过时锁定的 AMCL 消息副本

        # ---------- QoS ----------
        map_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # ---------- 订阅 ----------
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/rkbot/map', self.map_callback, map_qos)

        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/rkbot/amcl_pose',
            self.amcl_pose_callback, 10)

        # ---------- 发布 ----------
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        self.get_logger().info('AMCL 全局搜索初始位姿估计节点 v3 (一轮) 已启动')
        self.get_logger().info(
            f'网格间距: {self.grid_spacing}m, 朝向: 右(-90°), Top-K: {self.top_k}, '
            f'反馈超时: {self.feedback_timeout}s')

    # ============================================================
    #  回调
    # ============================================================
    def amcl_pose_callback(self, msg):
        self.latest_amcl_pose = msg
        self._pending_feedback = False

    def map_callback(self, msg):
        try:
            self.map_origin_x = msg.info.origin.position.x
            self.map_origin_y = msg.info.origin.position.y
            self.map_width = msg.info.width
            self.map_height = msg.info.height
            self.map_info = msg.info
            resolution = msg.info.resolution

            map_data = np.array(msg.data, dtype=np.int8).reshape(
                (self.map_height, self.map_width))
            self.map_image = np.zeros((self.map_height, self.map_width), dtype=np.uint8)
            self.map_image[map_data == -1] = 128
            self.map_image[map_data == 0] = 255
            self.map_image[map_data == 100] = 0

            # 将白色(自由空间)膨胀 2 米，候选点仅从膨胀后的区域选取
            inflate_pixels = max(1, int(round(2.0 / resolution)))
            free_mask = np.zeros_like(self.map_image, dtype=np.uint8)
            free_mask[self.map_image >= self.free_threshold] = 1
            kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE,
                (inflate_pixels * 2 + 1, inflate_pixels * 2 + 1))
            dilated = cv2.dilate(free_mask, kernel)
            self.map_free_dilated = np.full_like(self.map_image, 0, dtype=np.uint8)
            self.map_free_dilated[dilated == 1] = 255

            self.get_logger().info(
                f'地图已更新: {self.map_width}x{self.map_height}, '
                f'分辨率: {resolution:.3f}m/pixel, '
                f'白色膨胀+2m 内核半径={inflate_pixels}pix')

            if self.trigger_on_map and not self.has_estimated:
                self.get_logger().info('收到地图，自动触发全局搜索...')
                self._start_search()

        except Exception as e:
            self.get_logger().error(f'处理地图数据时出错: {str(e)}')

    # ============================================================
    #  启动搜索
    # ============================================================
    def _start_search(self):
        if self.map_image is None or self.has_estimated:
            return

        self._candidates = self._generate_grid_candidates()
        if not self._candidates:
            self.get_logger().error('未生成任何候选点，请检查地图')
            return

        total = len(self._candidates)
        self.get_logger().info(f'第一轮(粗搜索)开始: {total} 候选点')
        self._results = []
        self._current_idx = 0
        self._waiting_for_feedback = False
        self._search_state = 'searching'
        self._t_start = self.get_clock().now()

        if self._search_timer is None:
            self._search_timer = self.create_timer(0.01, self._search_tick)
        else:
            self._search_timer.reset()

    # ============================================================
    #  Timer 驱动核心 (10ms)
    # ============================================================
    def _search_tick(self):
        if self._search_state == 'searching':
            if self._fine_phase:
                self._tick_fine()
            else:
                self._tick_coarse()
        elif self._search_state == 'verifying':
            self._tick_verify()
        elif self._search_state == 'done':
            pass

    def _tick_coarse(self):
        if self._current_idx >= len(self._candidates):
            self._search_state = 'idle'
            self._start_fine_search()
            return

        if not self._waiting_for_feedback:
            self._publish_candidate()
            return

        if self._pending_feedback:
            elapsed = (self.get_clock().now() - self._publish_time).nanoseconds / 1e9
            if elapsed >= self.feedback_timeout:
                self.get_logger().info(
                    f'[超时] ({self._current_x:.1f},{self._current_y:.1f},'
                    f'{math.degrees(self._current_yaw):.0f}°)')
                self._results.append(
                    (-float('inf'), self._current_x, self._current_y, self._current_yaw,
                     self._current_name, self._current_x, self._current_y, self._current_yaw,
                     float('inf')))
                self._current_idx += 1
                self._waiting_for_feedback = False
            return

        if self.latest_amcl_pose is not None:
            self._record_amcl_result()
        self._current_idx += 1
        self._waiting_for_feedback = False

    # ============================================================
    #  第二轮搜索：在 Top-K 周围做精细检索
    # ============================================================
    def _start_fine_search(self):
        # 取粗搜索 Top-K (带距离去重)
        self._results.sort(key=lambda t: t[0], reverse=True)
        topk = self._select_topk_with_distance(self._results)

        fine_spacing = self.grid_spacing / 4.0
        radius = self.fine_search_radius
        self._fine_candidates = []
        for r in topk:
            _, _, _, _, _, ax, ay, ayaw, _ = r
            x_start = ax - radius
            x_end = ax + radius
            y_start = ay - radius
            y_end = ay + radius
            x = x_start
            while x <= x_end:
                y = y_start
                while y <= y_end:
                    if self._is_free(x, y):
                        for yaw, name in zip(self.direction_yaws, self.direction_names):
                            self._fine_candidates.append((x, y, yaw, name))
                    y += fine_spacing
                x += fine_spacing

        if not self._fine_candidates:
            self.get_logger().warn('第二轮未生成候选点，直接使用粗搜索结果')
            self._search_state = 'idle'
            self._on_search_done()
            return

        self.get_logger().info(
            f'第二轮搜索: {len(self._fine_candidates)} 候选点, '
            f'间距={fine_spacing:.2f}m, 半径={radius}m')
        self._fine_results = []
        self._current_idx = 0
        self._waiting_for_feedback = False
        self._fine_phase = True
        self._search_state = 'searching'

    def _tick_fine(self):
        if self._current_idx >= len(self._fine_candidates):
            self._search_state = 'idle'
            self._fine_phase = False
            self._results = self._fine_results
            self._on_search_done()
            return

        if not self._waiting_for_feedback:
            self._publish_fine_candidate()
            return

        if self._pending_feedback:
            elapsed = (self.get_clock().now() - self._publish_time).nanoseconds / 1e9
            if elapsed >= self.feedback_timeout:
                self.get_logger().info(
                    f'[超时] ({self._current_x:.1f},{self._current_y:.1f},'
                    f'{math.degrees(self._current_yaw):.0f}°)')
                self._fine_results.append(
                    (-float('inf'), self._current_x, self._current_y, self._current_yaw,
                     self._current_name, self._current_x, self._current_y, self._current_yaw,
                     float('inf')))
                self._current_idx += 1
                self._waiting_for_feedback = False
            return

        if self.latest_amcl_pose is not None:
            self._record_fine_result()
        self._current_idx += 1
        self._waiting_for_feedback = False

    def _publish_fine_candidate(self):
        cx, cy, cyaw, dname = self._fine_candidates[self._current_idx]
        self._current_x, self._current_y, self._current_yaw, self._current_name = \
            cx, cy, cyaw, dname
        self._pending_feedback = True
        self._waiting_for_feedback = True
        self.latest_amcl_pose = None
        self._publish_time = self.get_clock().now()
        self._publish_initial_pose(cx, cy, cyaw)

    def _record_fine_result(self):
        msg = self.latest_amcl_pose
        ax = msg.pose.pose.position.x
        ay = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        ayaw = 2.0 * math.atan2(qz, qw)
        cov = msg.pose.covariance
        cov_trace = abs(cov[0]) + abs(cov[7]) + abs(cov[35])

        # 主要指标：AMCL 收敛位姿离初始猜测的距离
        # AMCL 大幅漂移 → 初始猜测跟激光对不上 → 坏点
        move_dist = math.hypot(ax - self._current_x, ay - self._current_y)
        move_yaw = abs(self._normalize_angle(ayaw - self._current_yaw))

        # 综合评分：偏移距离为主，yaw偏差为辅，协方差做微调
        quality = -(move_dist + move_yaw * 0.5 + cov_trace * 1e3)

        delay = (self.get_clock().now() - self._publish_time).nanoseconds / 1e9
        self.get_logger().info(
            f'[精{self._current_idx+1}/{len(self._fine_candidates)}] '
            f'converged=({ax:.2f},{ay:.2f},{math.degrees(ayaw):.0f}°) '
            f'init=({self._current_x:.1f},{self._current_y:.1f}) '
            f'cov={cov_trace:.2e} move={move_dist:.2f}m quality={quality:.2f}')

        self._fine_results.append(
            (quality, self._current_x, self._current_y, self._current_yaw,
             self._current_name, ax, ay, ayaw, cov_trace))

    # ============================================================
    #  搜索完成 → 取 Top-K 中最优 (最终)
    # ============================================================
    def _on_search_done(self):
        elapsed = (self.get_clock().now() - self._t_start).nanoseconds / 1e9
        self._results.sort(key=lambda t: t[0], reverse=True)

        topk_log = self._select_topk_with_distance(self._results)
        self.get_logger().info(f'两轮搜索完成, 耗时 {elapsed:.1f}s, Top-{len(topk_log)}:')
        for i, r in enumerate(topk_log):
            self.get_logger().info(
                f'  #{i+1}: quality={r[0]:.4e} '
                f'pos=({r[5]:.2f},{r[6]:.2f},{math.degrees(r[7]):.0f}°)')

        # 进入验证阶段：按 quality 顺序，逐一发布 /initialpose，等待 AMCL 充分收敛后检查协方差
        self._verify_results = list(self._results)
        self._verify_idx = 0
        self._search_state = 'verifying'
        self._publish_verify_candidate()

    # ============================================================
    #  验证阶段：逐一发布候选位姿，等 AMCL 充分收敛，检查协方差
    # ============================================================
    def _publish_verify_candidate(self):
        if self._verify_idx >= len(self._verify_results):
            self.get_logger().error('所有候选点验证均失败，无法找到可靠初始位姿')
            self.has_estimated = True
            self._search_state = 'done'
            return

        r = self._verify_results[self._verify_idx]
        ax, ay, ayaw = r[5], r[6], r[7]
        self.get_logger().info(
            f'验证候选 #{self._verify_idx+1}: '
            f'({ax:.2f},{ay:.2f},{math.degrees(ayaw):.0f}°)')
        self._publish_initial_pose(ax, ay, ayaw)
        self._verify_settle_start = self.get_clock().now()
        self._verify_settled = False
        self.latest_amcl_pose = None

    def _tick_verify(self):
        if self._verify_settled:
            return

        if self._verify_settle_start is None:
            return

        elapsed = (self.get_clock().now() - self._verify_settle_start).nanoseconds / 1e9
        if elapsed < self.verify_settle_time:
            return  # 仍在等待 AMCL 收敛

        # 收敛时间到，检查 AMCL 协方差
        self._verify_settled = True

        if self.latest_amcl_pose is None:
            self.get_logger().warn('验证超时：未收到 AMCL 反馈')
            self._verify_idx += 1
            self._publish_verify_candidate()
            return

        cov = self.latest_amcl_pose.pose.covariance
        cov_trace = abs(cov[0]) + abs(cov[7]) + abs(cov[35])
        self.get_logger().info(f'验证结果: cov_trace={cov_trace:.4e} (阈值={self.verify_cov_threshold})')

        if cov_trace < self.verify_cov_threshold:
            # 锁定当前 AMCL 消息的快照（后续回调可能覆盖 self.latest_amcl_pose）
            self._verify_snapshot = copy.deepcopy(self.latest_amcl_pose)
            self._accept_verify_result()
        else:
            self.get_logger().warn(f'协方差过大 ({cov_trace:.4e})，尝试下一个候选...')
            self._verify_idx += 1
            self._publish_verify_candidate()

    def _accept_verify_result(self):
        msg = self.latest_amcl_pose
        ax = msg.pose.pose.position.x
        ay = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        ayaw = 2.0 * math.atan2(qz, qw)

        self.get_logger().info('=' * 60)
        self.get_logger().info(f'验证通过! 最终位姿: x={ax:.3f}, y={ay:.3f}, yaw={math.degrees(ayaw):.1f}°')
        self.get_logger().info('=' * 60)

        # 重新发布最终确认的位姿（用 AMCL 收敛后的位姿）
        self._publish_initial_pose(ax, ay, ayaw)
        self.has_estimated = True
        self._search_state = 'done'

    # ============================================================
    #  辅助
    # ============================================================
    def _select_topk_with_distance(self, results):
        """从已按 quality 降序排列的 results 中选出 Top-K，且任意两点的 AMCL 估计位置距离 >= min_topk_distance。
        如果候选点与已选点距离太近但质量更高，则替换已选点。"""
        selected = []
        for r in results:
            ax, ay = r[5], r[6]
            # 检查与所有已选点的距离冲突
            conflicts = [(i, s) for i, s in enumerate(selected)
                         if math.hypot(ax - s[5], ay - s[6]) < self.min_topk_distance]

            if not conflicts:
                if len(selected) < self.top_k:
                    selected.append(r)
            else:
                # 有冲突 — 如果比冲突中任意一个质量更好，替换之
                for i, s in conflicts:
                    if r[0] > s[0]:
                        selected[i] = r
                        break

        if not selected:
            selected = [results[0]]
        return selected
    def _publish_candidate(self):
        cx, cy, cyaw, dname = self._candidates[self._current_idx]
        self._current_x, self._current_y, self._current_yaw, self._current_name = \
            cx, cy, cyaw, dname
        self._pending_feedback = True
        self._waiting_for_feedback = True
        self.latest_amcl_pose = None
        self._publish_time = self.get_clock().now()
        self._publish_initial_pose(cx, cy, cyaw)

    def _record_amcl_result(self):
        msg = self.latest_amcl_pose
        ax = msg.pose.pose.position.x
        ay = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        ayaw = 2.0 * math.atan2(qz, qw)
        cov = msg.pose.covariance
        cov_trace = abs(cov[0]) + abs(cov[7]) + abs(cov[35])
        move_dist = math.hypot(ax - self._current_x, ay - self._current_y)
        move_yaw = abs(self._normalize_angle(ayaw - self._current_yaw))

        quality = -(move_dist + move_yaw * 0.5 + cov_trace * 1e3)

        delay = (self.get_clock().now() - self._publish_time).nanoseconds / 1e9
        self.get_logger().info(
            f'[{self._current_idx+1}/{len(self._candidates)}] '
            f'converged=({ax:.2f},{ay:.2f},{math.degrees(ayaw):.0f}°) '
            f'init=({self._current_x:.1f},{self._current_y:.1f}) '
            f'cov={cov_trace:.2e} move={move_dist:.2f}m quality={quality:.2f}')

        self._results.append(
            (quality, self._current_x, self._current_y, self._current_yaw,
             self._current_name, ax, ay, ayaw, cov_trace))

    def _is_free(self, wx, wy):
        if self.map_image is None:
            return False
        col = int((wx - self.map_origin_x) / self.map_info.resolution)
        row = int((wy - self.map_origin_y) / self.map_info.resolution)
        if 0 <= row < self.map_height and 0 <= col < self.map_width:
            # 候选点选取使用膨胀后的地图（白色膨胀2m），确保离障碍物足够远
            map_to_use = getattr(self, 'map_free_dilated', self.map_image)
            return map_to_use[row, col] >= self.free_threshold
        return False

    @staticmethod
    def _normalize_angle(angle):
        """将角度归一化到 [-pi, pi)"""
        return math.atan2(math.sin(angle), math.cos(angle))

    def _generate_grid_candidates(self):
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

    def _publish_initial_pose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'rkbot/map'
        msg.pose.pose.position = Point(x=x, y=y, z=0.0)
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)
        cov = [0.0] * 36
        cov[0] = 1.0
        cov[1] = 1.0
        cov[6] = 1.0
        cov[7] = 1.0
        cov[35] = 3.14
        msg.pose.covariance = cov
        self.initial_pose_pub.publish(msg)

    def reset_and_estimate(self):
        self.has_estimated = False
        if self.map_image is not None:
            self._start_search()
        else:
            self.get_logger().warn('地图未就绪')


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
