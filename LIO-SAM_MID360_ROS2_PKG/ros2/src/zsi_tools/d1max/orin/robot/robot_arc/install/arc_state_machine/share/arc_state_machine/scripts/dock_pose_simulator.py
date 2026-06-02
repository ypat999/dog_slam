#!/usr/bin/env python3
"""
Dock Pose Simulator

功能：模拟SLAM和Perception两个模块的dock pose数据发布
- 用户输入dock真值（map坐标系）
- 监听/odom/current_pose获取机器人位置
- 通过TF2进行map→base_link坐标转换
- 添加可配置噪声后发布dock_pose
- 手动键盘控制模块状态切换

作者: Claude
日期: 2025-09-30
"""

import copy
import math
import select
import sys
import termios
import threading
import time
import tty
from enum import IntEnum
from pathlib import Path
from typing import Optional, Tuple, Dict

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
# ROS消息类型
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from robots_dog_msgs.msg import DockPoseStamped, ArcModuleState
from std_msgs.msg import Header

# TF转换工具（仅用于欧拉角和四元数转换）
try:
    from tf_transformations import euler_from_quaternion, quaternion_from_euler
except ImportError:
    # 如果tf_transformations不可用，使用内置实现
    def euler_from_quaternion(quaternion):
        """从四元数转换为欧拉角 (roll, pitch, yaw)"""
        x, y, z, w = quaternion

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


    def quaternion_from_euler(roll, pitch, yaw):
        """从欧拉角转换为四元数 [x, y, z, w]"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return [x, y, z, w]


# ============================================================================
# 模块状态枚举
# ============================================================================

class ModuleState(IntEnum):
    """模块状态枚举（与ArcModuleState对应）"""
    PASSIVE = 2  # 被动状态，不发布数据
    STANDBY = 0  # 待机状态，不发布数据
    RUNNING = 1  # 运行状态，发布数据


# ============================================================================
# 配置管理器
# ============================================================================

class ConfigManager:
    """配置加载和验证"""

    def __init__(self, config_path: Optional[str] = None):
        self.config = {}

        if config_path is None:
            config_path = self._find_default_config()

        self.load_config(config_path)

    def _find_default_config(self) -> str:
        """查找默认配置文件（使用相对于包的路径）"""
        # 获取当前脚本所在目录
        script_dir = Path(__file__).resolve().parent

        possible_paths = []

        # 情况1: 源码路径
        # 结构: arc_state_machine/scripts/dock_pose_simulator.py
        # 配置: arc_state_machine/config/dock_pose_simulator_config.yaml
        source_config = script_dir.parent / "config" / "dock_pose_simulator_config.yaml"
        possible_paths.append(source_config)

        # 情况2: ROS2安装路径
        # 脚本: install/arc_state_machine/lib/arc_state_machine/dock_pose_simulator.py
        # 配置: install/arc_state_machine/share/arc_state_machine/config/dock_pose_simulator_config.yaml
        # 从 lib/arc_state_machine 向上2级到 install/arc_state_machine，再进入 share
        install_config = script_dir.parent.parent / "share" / "arc_state_machine" / "config" / "dock_pose_simulator_config.yaml"
        possible_paths.append(install_config)

        for path in possible_paths:
            if path.exists():
                return str(path)

        raise FileNotFoundError(
            f"无法找到配置文件 dock_pose_simulator_config.yaml\n"
            f"已搜索路径:\n" + "\n".join(f"  - {p}" for p in possible_paths)
        )

    def load_config(self, config_path: str):
        """加载配置文件"""
        try:
            with open(config_path, 'r') as f:
                self.config = yaml.safe_load(f)
            print(f"✓ 配置文件已加载: {config_path}")
        except Exception as e:
            raise RuntimeError(f"加载配置文件失败: {e}")

    def get(self, *keys, default=None):
        """获取嵌套配置项"""
        value = self.config
        for key in keys:
            if isinstance(value, dict) and key in value:
                value = value[key]
            else:
                return default
        return value


# ============================================================================
# 用户输入处理器
# ============================================================================

class UserInputHandler:
    """用户输入处理器"""

    @staticmethod
    def get_dock_pose_input() -> Tuple[float, float, float]:
        """获取dock pose用户输入（map坐标系）"""
        print("\n" + "=" * 50)
        print("请输入充电桩在map坐标系下的位置:")
        print("=" * 50)

        x = UserInputHandler._get_validated_float(
            "  X位置 (米)", -100.0, 100.0, 0.0
        )
        y = UserInputHandler._get_validated_float(
            "  Y位置 (米)", -100.0, 100.0, 0.0
        )
        yaw_deg = UserInputHandler._get_validated_float(
            "  朝向角度 (度)", -180.0, 360.0, 0.0
        )

        yaw_rad = math.radians(yaw_deg)

        print(f"\n✓ Dock位置已设置: ({x:.2f}, {y:.2f}, {yaw_deg:.1f}°)")
        print("=" * 50 + "\n")

        return (x, y, yaw_rad)

    @staticmethod
    def _get_validated_float(prompt: str, min_val: float,
                             max_val: float, default: float) -> float:
        """验证浮点数输入"""
        while True:
            try:
                user_input = input(f"{prompt} [{default}]: ").strip()

                # 空输入使用默认值
                if not user_input:
                    return default

                value = float(user_input)

                if min_val <= value <= max_val:
                    return value
                else:
                    print(f"  ❌ 错误: 值必须在 {min_val} 到 {max_val} 之间")
            except ValueError:
                print("  ❌ 错误: 请输入有效的数字")
            except KeyboardInterrupt:
                print("\n\n用户取消输入")
                sys.exit(0)

    @staticmethod
    def create_pose_from_xyyaw(x: float, y: float, yaw: float) -> Pose:
        """从x, y, yaw创建Pose消息"""
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0.0

        # 欧拉角转四元数
        quat = quaternion_from_euler(0, 0, yaw)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]

        return pose


# ============================================================================
# 坐标转换处理器
# ============================================================================

class TransformHandler:
    """坐标转换处理器 - 使用里程计数据直接建立坐标转换"""

    def __init__(self, node: Node, config: ConfigManager):
        self.node = node
        self.config = config
        self.logger = node.get_logger()

        # 机器人里程计数据（认为是机器人在map坐标系下的pose）
        self.robot_pose_in_map = None
        self.odom_received = False

        # 订阅odom话题
        odom_topic = config.get('topics', 'odom_input', default='/odom/current_pose')

        # 使用BEST_EFFORT QoS以兼容大多数里程计发布者
        # 这样可以订阅RELIABLE或BEST_EFFORT的发布者
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        self.logger.info(f"📡 准备订阅话题: {odom_topic}")
        self.logger.info(f"📡 QoS配置: reliability=BEST_EFFORT, depth=10")

        self.odom_sub = node.create_subscription(
            Odometry,
            odom_topic,
            self._odom_callback,
            qos_profile
        )

        self.logger.info(f"✓ 坐标转换处理器已启动")
        self.logger.info(f"  订阅话题: {odom_topic}")
        self.logger.info(f"  转换策略: 直接使用里程计建立 map → base_link 转换关系")
        self.logger.info(f"  等待回调触发...")

    def _odom_callback(self, msg: Odometry):
        """里程计回调 - 认为接收到的是机器人在map坐标系下的pose"""
        # 调试：每次回调都打印（用于诊断）
        if not self.odom_received:
            self.logger.info("🔔 里程计回调被触发！")

        # 不管frame_id是什么，都认为这是机器人在map坐标系下的位置
        self.robot_pose_in_map = msg.pose.pose

        if not self.odom_received:
            self.odom_received = True
            self.logger.info("✓ 里程计数据已接收")
            # 打印首次接收到的位置信息
            x = self.robot_pose_in_map.position.x
            y = self.robot_pose_in_map.position.y
            _, _, yaw = euler_from_quaternion([
                self.robot_pose_in_map.orientation.x,
                self.robot_pose_in_map.orientation.y,
                self.robot_pose_in_map.orientation.z,
                self.robot_pose_in_map.orientation.w
            ])
            self.logger.info(f"  机器人位置(map): x={x:.2f}m, y={y:.2f}m, yaw={math.degrees(yaw):.1f}°")

    def get_robot_pose_in_map(self) -> Optional[Pose]:
        """获取机器人在map坐标系的位置"""
        return self.robot_pose_in_map

    def transform_dock_to_base_link(self, dock_map: Pose) -> Optional[Pose]:
        """将dock pose从map坐标系转换到base_link坐标系

        转换原理：
        1. robot_map: 机器人在map坐标系的位置 (来自/odom/current_pose)
        2. dock_map: dock在map坐标系的位置 (用户输入)
        3. dock_base: dock在base_link坐标系的位置 (计算结果)

        转换公式：
        dock_base = T_base_map * dock_map
                  = inverse(T_map_base) * dock_map
                  = inverse(robot_map) * dock_map
        """
        if self.robot_pose_in_map is None:
            return None

        try:
            # 获取机器人在map坐标系的位姿
            robot_x = self.robot_pose_in_map.position.x
            robot_y = self.robot_pose_in_map.position.y
            _, _, robot_yaw = euler_from_quaternion([
                self.robot_pose_in_map.orientation.x,
                self.robot_pose_in_map.orientation.y,
                self.robot_pose_in_map.orientation.z,
                self.robot_pose_in_map.orientation.w
            ])

            # 获取dock在map坐标系的位姿
            dock_x = dock_map.position.x
            dock_y = dock_map.position.y
            _, _, dock_yaw = euler_from_quaternion([
                dock_map.orientation.x,
                dock_map.orientation.y,
                dock_map.orientation.z,
                dock_map.orientation.w
            ])

            # 计算dock相对于机器人的位置（map坐标系）
            dx = dock_x - robot_x
            dy = dock_y - robot_y

            # 旋转到base_link坐标系
            # base_link坐标系相对于map坐标系旋转了 -robot_yaw
            cos_yaw = math.cos(-robot_yaw)
            sin_yaw = math.sin(-robot_yaw)

            dock_base_x = dx * cos_yaw - dy * sin_yaw
            dock_base_y = dx * sin_yaw + dy * cos_yaw

            # 角度转换
            dock_base_yaw = dock_yaw - robot_yaw

            # 归一化角度到 [-pi, pi]
            while dock_base_yaw > math.pi:
                dock_base_yaw -= 2 * math.pi
            while dock_base_yaw < -math.pi:
                dock_base_yaw += 2 * math.pi

            # 创建结果Pose
            dock_base = Pose()
            dock_base.position.x = dock_base_x
            dock_base.position.y = dock_base_y
            dock_base.position.z = 0.0

            # 转换为四元数
            quat = quaternion_from_euler(0, 0, dock_base_yaw)
            dock_base.orientation.x = quat[0]
            dock_base.orientation.y = quat[1]
            dock_base.orientation.z = quat[2]
            dock_base.orientation.w = quat[3]

            return dock_base

        except Exception as e:
            self.logger.error(f"坐标转换失败: {e}")
            return None

    def wait_for_odom(self, timeout: float = 30.0) -> bool:
        """等待里程计数据可用

        注意：使用系统时间(time.time())而非ROS时钟来计算超时，
        因为use_sim_time=true时ROS时钟可能不会前进（无仿真时钟源）
        """
        self.logger.info("等待里程计数据...")

        # 使用系统时间而非ROS时钟，避免use_sim_time=true时的超时问题
        start_time = time.time()

        while rclpy.ok():
            # 关键：必须 spin 才能触发回调！
            rclpy.spin_once(self.node, timeout_sec=0.1)

            if self.odom_received:
                self.logger.info("✓ 里程计数据就绪")
                return True

            # 使用系统时间计算经过的时间
            elapsed = time.time() - start_time
            if elapsed > timeout:
                self.logger.error(f"等待里程计数据超时 ({timeout}s)")
                self.logger.error("请检查 /odom/current_pose 话题是否正在发布:")
                self.logger.error("  ros2 topic echo /odom/current_pose")
                self.logger.error(f"调试提示: 订阅者是否成功创建? 检查 ros2 topic info /odom/current_pose")
                return False

        return False


# ============================================================================
# 双模块状态管理器
# ============================================================================

class DualModuleStateManager:
    """双模块状态管理器"""

    def __init__(self, initial_slam: str = "PASSIVE",
                 initial_perception: str = "PASSIVE"):
        # 初始化状态
        self.slam_state = ModuleState[initial_slam.upper()]
        self.perception_state = ModuleState[initial_perception.upper()]

        # 状态序列
        self.state_sequence = [
            ModuleState.PASSIVE,
            ModuleState.STANDBY,
            ModuleState.RUNNING
        ]

    def transition_slam_next(self):
        """SLAM状态向前切换"""
        self.slam_state = self._next_state(self.slam_state)

    def transition_slam_prev(self):
        """SLAM状态向后切换"""
        self.slam_state = self._prev_state(self.slam_state)

    def transition_perception_next(self):
        """Perception状态向前切换"""
        self.perception_state = self._next_state(self.perception_state)

    def transition_perception_prev(self):
        """Perception状态向后切换"""
        self.perception_state = self._prev_state(self.perception_state)

    def reset_all_to_passive(self):
        """重置所有模块到PASSIVE"""
        self.slam_state = ModuleState.PASSIVE
        self.perception_state = ModuleState.PASSIVE

    def _next_state(self, current: ModuleState) -> ModuleState:
        """获取下一个状态"""
        idx = self.state_sequence.index(current)
        next_idx = (idx + 1) % len(self.state_sequence)
        return self.state_sequence[next_idx]

    def _prev_state(self, current: ModuleState) -> ModuleState:
        """获取上一个状态"""
        idx = self.state_sequence.index(current)
        prev_idx = (idx - 1) % len(self.state_sequence)
        return self.state_sequence[prev_idx]

    def should_publish_slam_dock_pose(self) -> bool:
        """SLAM是否应该发布dock pose"""
        return self.slam_state == ModuleState.RUNNING

    def should_publish_perception_dock_pose(self) -> bool:
        """Perception是否应该发布dock pose"""
        return self.perception_state == ModuleState.RUNNING

    def get_state_name(self, state: ModuleState) -> str:
        """获取状态名称"""
        return state.name


# ============================================================================
# 噪声生成器
# ============================================================================

class NoiseGenerator:
    """高斯噪声生成器"""

    def __init__(self, config: ConfigManager):
        # SLAM噪声参数
        self.slam_x_std = config.get('noise', 'slam', 'x_stddev', default=0.05)
        self.slam_y_std = config.get('noise', 'slam', 'y_stddev', default=0.05)
        self.slam_yaw_std = config.get('noise', 'slam', 'yaw_stddev', default=0.087)

        # Perception噪声参数
        self.perc_x_std = config.get('noise', 'perception', 'x_stddev', default=0.01)
        self.perc_y_std = config.get('noise', 'perception', 'y_stddev', default=0.01)
        self.perc_yaw_std = config.get('noise', 'perception', 'yaw_stddev', default=0.017)

    def add_slam_noise(self, pose: Pose) -> Pose:
        """为SLAM添加噪声"""
        return self._add_noise(pose, self.slam_x_std, self.slam_y_std, self.slam_yaw_std)

    def add_perception_noise(self, pose: Pose) -> Pose:
        """为Perception添加噪声"""
        return self._add_noise(pose, self.perc_x_std, self.perc_y_std, self.perc_yaw_std)

    def _add_noise(self, pose: Pose, x_std: float, y_std: float, yaw_std: float) -> Pose:
        """添加高斯噪声"""
        noisy_pose = copy.deepcopy(pose)

        # 位置噪声
        noisy_pose.position.x += np.random.normal(0, x_std)
        noisy_pose.position.y += np.random.normal(0, y_std)

        # 姿态噪声
        roll, pitch, yaw = euler_from_quaternion([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ])

        yaw += np.random.normal(0, yaw_std)

        # 转换回四元数
        quat = quaternion_from_euler(roll, pitch, yaw)
        noisy_pose.orientation.x = quat[0]
        noisy_pose.orientation.y = quat[1]
        noisy_pose.orientation.z = quat[2]
        noisy_pose.orientation.w = quat[3]

        return noisy_pose


# ============================================================================
# 发布管理器
# ============================================================================

class PublishManager:
    """发布管理器"""

    def __init__(self, node: Node, config: ConfigManager):
        self.node = node

        # 创建发布器 - 模块状态
        slam_state_topic = config.get('topics', 'slam_state_output',
                                      default='/arc/slam_state')
        self.slam_state_pub = node.create_publisher(
            ArcModuleState,
            slam_state_topic,
            10
        )

        perc_state_topic = config.get('topics', 'perception_state_output',
                                      default='/arc/perception_state')
        self.perception_state_pub = node.create_publisher(
            ArcModuleState,
            perc_state_topic,
            10
        )

        # 创建发布器 - Calibration状态
        self.calibration_state_pub = node.create_publisher(
            ArcModuleState,
            '/arc/calibration_state',
            10
        )

        # 创建发布器 - Dock Pose
        slam_dock_topic = config.get('topics', 'slam_dock_pose_output',
                                     default='/arc/slam_dock_pose')
        self.slam_dock_pose_pub = node.create_publisher(
            DockPoseStamped,
            slam_dock_topic,
            10
        )

        perc_dock_topic = config.get('topics', 'perception_dock_pose_output',
                                     default='/arc/perception_dock_pose')
        self.perception_dock_pose_pub = node.create_publisher(
            DockPoseStamped,
            perc_dock_topic,
            10
        )

    def publish_module_states(self, slam_state: ModuleState,
                              perception_state: ModuleState):
        """发布模块状态"""
        now = self.node.get_clock().now().to_msg()

        # SLAM状态
        slam_msg = ArcModuleState()
        slam_msg.stamp = now
        slam_msg.state = int(slam_state)
        slam_msg.error_code = 0
        slam_msg.error_msg = ""
        self.slam_state_pub.publish(slam_msg)

        # Perception状态
        perc_msg = ArcModuleState()
        perc_msg.stamp = now
        perc_msg.state = int(perception_state)
        perc_msg.error_code = 0
        perc_msg.error_msg = ""
        self.perception_state_pub.publish(perc_msg)

        # Calibration状态 (固定为STANDBY)
        calibration_msg = ArcModuleState()
        calibration_msg.stamp = now
        calibration_msg.state = 0  # STATE_STANDBY
        calibration_msg.tag_id = 0
        calibration_msg.error_code = 0
        calibration_msg.error_msg = ""
        self.calibration_state_pub.publish(calibration_msg)

    def publish_dock_poses(self, slam_pose: Optional[Pose],
                           perception_pose: Optional[Pose],
                           should_publish_slam: bool,
                           should_publish_perception: bool):
        """发布dock pose"""
        now = self.node.get_clock().now().to_msg()

        # SLAM dock pose
        if should_publish_slam and slam_pose:
            slam_msg = DockPoseStamped()
            slam_msg.header.stamp = now
            slam_msg.header.frame_id = "base_link"
            slam_msg.current_pose = slam_pose
            slam_msg.source = 0  # SOURCE_LIDAR
            slam_msg.confidence = 0.9
            self.slam_dock_pose_pub.publish(slam_msg)

        # Perception dock pose
        if should_publish_perception and perception_pose:
            perc_msg = DockPoseStamped()
            perc_msg.header.stamp = now
            perc_msg.header.frame_id = "base_link"
            perc_msg.current_pose = perception_pose
            perc_msg.source = 1  # SOURCE_CAMERA
            perc_msg.confidence = 0.95
            self.perception_dock_pose_pub.publish(perc_msg)


# ============================================================================
# UI管理器
# ============================================================================

class UIManager:
    """UI显示和键盘控制管理器"""

    def __init__(self):
        self.running = True
        self.last_key = None
        self.keyboard_thread = None

        # 检测是否运行在真实终端
        self.is_tty = sys.stdout.isatty()

        # 保存终端设置
        try:
            self.old_settings = termios.tcgetattr(sys.stdin)
        except:
            self.old_settings = None

    def start_keyboard_listener(self):
        """启动键盘监听线程"""
        self.keyboard_thread = threading.Thread(
            target=self._keyboard_listener_loop,
            daemon=True
        )
        self.keyboard_thread.start()

    def _keyboard_listener_loop(self):
        """键盘监听循环"""
        if self.old_settings is None:
            return

        try:
            tty.setcbreak(sys.stdin.fileno())

            while self.running:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    self.last_key = key
        except Exception as e:
            print(f"键盘监听错误: {e}")

    def get_last_key(self) -> Optional[str]:
        """获取最后按下的键（非阻塞）"""
        key = self.last_key
        self.last_key = None
        return key

    def display_status(self, data: Dict):
        """显示实时状态"""
        # 构建完整输出（使用字符串拼接，一次性输出减少闪烁）
        lines = []

        # 仅在真实 tty 时使用 ANSI 清屏，通过 tee 重定向时跳过
        if self.is_tty:
            lines.append('\033[H\033[J')  # 清屏并移动到左上角

        # 显示面板
        lines.append("┌" + "─" * 60 + "┐")
        lines.append("│" + "  Dock Pose Simulator - Control Panel".center(60) + "│")
        lines.append("├" + "─" * 60 + "┤")

        # 模块状态
        slam_icon = self._get_state_icon(data['slam_state'])
        perc_icon = self._get_state_icon(data['perc_state'])
        slam_name = data['slam_state_name']
        perc_name = data['perc_state_name']

        lines.append(f"│ SLAM Module:       [{slam_name:8s}] {slam_icon}".ljust(61) + "│")
        lines.append(f"│ Perception Module: [{perc_name:8s}] {perc_icon}".ljust(61) + "│")
        lines.append("│" + " " * 60 + "│")

        # Pose显示（使用不同精度）
        # map坐标系使用2位精度
        dock_map_str = self._format_pose(data.get('dock_map'), precision=2)
        robot_map_str = self._format_pose(data.get('robot_map'), precision=2)

        # base_link坐标系使用3位精度，更好地显示噪声差异
        dock_truth_str = self._format_pose(data.get('dock_truth'), precision=3)
        slam_dock_str = self._format_pose(data.get('slam_dock'), precision=3)
        perc_dock_str = self._format_pose(data.get('perc_dock'), precision=3)

        # 获取坐标来源标识
        source_tag = data.get('dock_source', 'config')
        source_label = "[config]" if source_tag == "config" else "[manual]"

        lines.append(f"│ Dock Pose (map):      {dock_map_str} {source_label}".ljust(61) + "│")
        lines.append(f"│ Robot Pose (map):     {robot_map_str}".ljust(61) + "│")
        lines.append("│" + " " * 60 + "│")
        lines.append(f"│ Dock Truth (base):    {dock_truth_str}".ljust(61) + "│")
        lines.append(f"│ SLAM Dock (base):     {slam_dock_str}".ljust(61) + "│")
        lines.append(f"│ Perc Dock (base):     {perc_dock_str}".ljust(61) + "│")
        lines.append("│" + " " * 60 + "│")

        # 发布状态
        slam_pub = "✓" if data.get('slam_publishing') else "✗"
        perc_pub = "✓" if data.get('perc_publishing') else "✗"
        lines.append(f"│ Publishing: SLAM{slam_pub}  Perception{perc_pub}".ljust(61) + "│")

        # 误差统计（如果有真值和噪声数据）
        dock_truth = data.get('dock_truth')
        slam_dock = data.get('slam_dock')
        perc_dock = data.get('perc_dock')

        if dock_truth and (slam_dock or perc_dock):
            lines.append("│" + " " * 60 + "│")
            lines.append("│ " + "Error vs Truth:".ljust(59) + "│")

            if slam_dock:
                slam_err = self._calculate_pose_error(dock_truth, slam_dock)
                slam_err_str = f"ΔX={slam_err[0]:+.3f}m ΔY={slam_err[1]:+.3f}m"
                lines.append(f"│   SLAM:       {slam_err_str}".ljust(61) + "│")

            if perc_dock:
                perc_err = self._calculate_pose_error(dock_truth, perc_dock)
                perc_err_str = f"ΔX={perc_err[0]:+.3f}m ΔY={perc_err[1]:+.3f}m"
                lines.append(f"│   Perception: {perc_err_str}".ljust(61) + "│")

        lines.append("└" + "─" * 60 + "┘")
        lines.append("")
        lines.append("Controls:")
        lines.append("  1/2: SLAM [PASSIVE→STANDBY→RUNNING / ←]")
        lines.append("  3/4: Perception [PASSIVE→STANDBY→RUNNING / ←]")
        lines.append("  r: Reset all to PASSIVE")
        lines.append("  i: Re-input dock pose")
        lines.append("  q: Quit")

        # 一次性输出所有内容，减少闪烁
        output = '\n'.join(lines)
        print(output, flush=True)  # flush=True 强制立即刷新缓冲区

    def _format_pose(self, pose: Optional[Pose], precision: int = 2) -> str:
        """格式化Pose显示

        Args:
            pose: 要格式化的Pose对象
            precision: 小数位数（默认2位，可设置3-4位显示高精度）

        Returns:
            格式化的字符串，如 "(  1.500,   0.500,   45.0°)"
        """
        if pose is None:
            return "N/A"

        yaw = self._get_yaw_from_quaternion(pose.orientation)

        # 根据精度动态调整字段宽度
        width = 6 + (precision - 2)  # 基础宽度6，精度每增加1位，宽度+1

        return f"({pose.position.x:{width}.{precision}f}, {pose.position.y:{width}.{precision}f}, {math.degrees(yaw):6.1f}°)"

    def _get_yaw_from_quaternion(self, quat: Quaternion) -> float:
        """从四元数获取yaw角"""
        _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        return yaw

    def _calculate_pose_error(self, truth: Pose, measured: Pose) -> Tuple[float, float, float]:
        """计算pose误差

        Args:
            truth: 真值Pose
            measured: 测量值Pose

        Returns:
            (error_x, error_y, error_yaw) 误差元组
        """
        error_x = measured.position.x - truth.position.x
        error_y = measured.position.y - truth.position.y

        truth_yaw = self._get_yaw_from_quaternion(truth.orientation)
        measured_yaw = self._get_yaw_from_quaternion(measured.orientation)
        error_yaw = measured_yaw - truth_yaw

        # 归一化yaw到[-pi, pi]
        while error_yaw > math.pi:
            error_yaw -= 2 * math.pi
        while error_yaw < -math.pi:
            error_yaw += 2 * math.pi

        return (error_x, error_y, error_yaw)

    def _get_state_icon(self, state: ModuleState) -> str:
        """获取状态图标"""
        if state == ModuleState.RUNNING:
            return "🟢"
        elif state == ModuleState.STANDBY:
            return "🟡"
        else:  # PASSIVE
            return "🔴"

    def cleanup(self):
        """清理资源"""
        self.running = False
        if self.old_settings:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            except:
                pass


# ============================================================================
# 主仿真节点
# ============================================================================

class DockPoseSimulator(Node):
    """主仿真节点"""

    def __init__(self, config_path: Optional[str] = None):
        super().__init__('dock_pose_simulator')

        self.get_logger().info("正在初始化 Dock Pose Simulator...")

        # 获取use_sim_time参数
        # 注意：use_sim_time是ROS2自动声明的全局参数，不需要手动declare
        # 直接通过get_parameter获取即可
        use_sim_time = self.get_parameter('use_sim_time').value
        self.get_logger().info(f"use_sim_time: {use_sim_time}")

        # 配置管理器
        self.config = ConfigManager(config_path)

        # 各功能模块
        self.transform_handler = TransformHandler(self, self.config)
        self.state_manager = DualModuleStateManager(
            initial_slam=self.config.get('initial_states', 'slam', default='PASSIVE'),
            initial_perception=self.config.get('initial_states', 'perception', default='PASSIVE')
        )
        self.noise_generator = NoiseGenerator(self.config)
        self.publish_manager = PublishManager(self, self.config)
        self.ui_manager = UIManager()

        # Dock真值（map坐标系）
        self.dock_pose_map = None

        # Dock坐标来源标识: "config" 或 "manual"
        self.dock_pose_source = "config"

        # 计算缓存
        self.dock_truth_base = None
        self.slam_dock_base = None
        self.perc_dock_base = None

        # 创建定时器
        self._setup_timers()

        self.get_logger().info("✓ Dock Pose Simulator 初始化完成")

    def _setup_timers(self):
        """创建定时器"""
        # 50Hz dock pose发布
        dock_pose_rate = self.config.get('frequencies', 'dock_pose_publish_rate', default=50.0)
        self.dock_pose_timer = self.create_timer(
            1.0 / dock_pose_rate,
            self._dock_pose_publish_callback
        )

        # 10Hz 状态发布
        state_rate = self.config.get('frequencies', 'state_publish_rate', default=10.0)
        self.state_timer = self.create_timer(
            1.0 / state_rate,
            self._state_publish_callback
        )

        # 5Hz UI更新
        display_rate = self.config.get('frequencies', 'display_update_rate', default=5.0)
        self.display_timer = self.create_timer(
            1.0 / display_rate,
            self._display_update_callback
        )

    def initialize(self):
        """初始化流程"""
        # 1. 从配置文件加载默认dock pose
        x = self.config.get('default_dock_pose', 'x', default=0.0)
        y = self.config.get('default_dock_pose', 'y', default=0.0)
        yaw = self.config.get('default_dock_pose', 'yaw', default=0.0)

        self.dock_pose_map = UserInputHandler.create_pose_from_xyyaw(x, y, yaw)
        self.dock_pose_source = "config"

        self.get_logger().info(f"✓ 已加载默认Dock位置: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f}°) [from config]")

        # 2. 等待里程计数据
        if not self.transform_handler.wait_for_odom(timeout=180.0):
            self.get_logger().error("里程计数据等待超时，退出")
            return False

        # 3. 启动键盘监听
        self.ui_manager.start_keyboard_listener()

        self.get_logger().info("\n" + "=" * 50)
        self.get_logger().info("✓ 初始化完成，仿真器已启动")
        self.get_logger().info("=" * 50 + "\n")

        return True

    def _dock_pose_publish_callback(self):
        """Dock Pose发布回调 (50Hz)"""
        if self.dock_pose_map is None:
            return

        # 1. 计算理论真值（无噪声）
        self.dock_truth_base = self.transform_handler.transform_dock_to_base_link(
            self.dock_pose_map
        )

        if self.dock_truth_base is None:
            return

        # 2. 添加噪声
        self.slam_dock_base = self.noise_generator.add_slam_noise(self.dock_truth_base)
        self.perc_dock_base = self.noise_generator.add_perception_noise(self.dock_truth_base)

        # 3. 发布
        self.publish_manager.publish_dock_poses(
            self.slam_dock_base,
            self.perc_dock_base,
            self.state_manager.should_publish_slam_dock_pose(),
            self.state_manager.should_publish_perception_dock_pose()
        )

    def _state_publish_callback(self):
        """状态发布回调 (10Hz)"""
        self.publish_manager.publish_module_states(
            self.state_manager.slam_state,
            self.state_manager.perception_state
        )

    def _display_update_callback(self):
        """UI更新回调 (5Hz)"""
        # 处理键盘输入
        key = self.ui_manager.get_last_key()
        if key:
            self._handle_keyboard_input(key)

        # 更新显示
        robot_pose = self.transform_handler.get_robot_pose_in_map()

        display_data = {
            'slam_state': self.state_manager.slam_state,
            'perc_state': self.state_manager.perception_state,
            'slam_state_name': self.state_manager.get_state_name(self.state_manager.slam_state),
            'perc_state_name': self.state_manager.get_state_name(self.state_manager.perception_state),
            'dock_map': self.dock_pose_map,
            'robot_map': robot_pose,
            'dock_truth': self.dock_truth_base,
            'slam_dock': self.slam_dock_base,
            'perc_dock': self.perc_dock_base,
            'slam_publishing': self.state_manager.should_publish_slam_dock_pose(),
            'perc_publishing': self.state_manager.should_publish_perception_dock_pose(),
            'dock_source': self.dock_pose_source  # 新增：坐标来源
        }

        self.ui_manager.display_status(display_data)

    def _handle_keyboard_input(self, key: str):
        """处理键盘输入"""
        if key == '1':
            self.state_manager.transition_slam_next()
        elif key == '2':
            self.state_manager.transition_slam_prev()
        elif key == '3':
            self.state_manager.transition_perception_next()
        elif key == '4':
            self.state_manager.transition_perception_prev()
        elif key == 'r' or key == 'R':
            self.state_manager.reset_all_to_passive()
            self.get_logger().info("所有模块已重置到PASSIVE")
        elif key == 'i' or key == 'I':
            self._reinput_dock_pose()
        elif key == 'q' or key == 'Q':
            self.get_logger().info("用户请求退出")
            self.ui_manager.cleanup()
            rclpy.shutdown()

    def _reinput_dock_pose(self):
        """重新输入dock pose"""
        # 暂停UI更新
        self.display_timer.cancel()

        # 恢复终端
        self.ui_manager.cleanup()

        # 获取新输入
        x, y, yaw = UserInputHandler.get_dock_pose_input()
        self.dock_pose_map = UserInputHandler.create_pose_from_xyyaw(x, y, yaw)
        self.dock_pose_source = "manual"  # 更新来源标识

        # 重新启动UI
        self.ui_manager = UIManager()
        self.ui_manager.start_keyboard_listener()

        # 恢复定时器
        display_rate = self.config.get('frequencies', 'display_update_rate', default=5.0)
        self.display_timer = self.create_timer(
            1.0 / display_rate,
            self._display_update_callback
        )

    def cleanup(self):
        """清理资源"""
        self.ui_manager.cleanup()


# ============================================================================
# 主函数
# ============================================================================

def main(args=None):
    """主函数"""
    rclpy.init(args=args)

    simulator = None

    try:
        # 创建仿真节点
        simulator = DockPoseSimulator()

        # 初始化
        if not simulator.initialize():
            return

        # 运行
        rclpy.spin(simulator)

    except KeyboardInterrupt:
        print("\n\n用户中断，正在退出...")
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if simulator:
            simulator.cleanup()
            simulator.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        print("\n✓ Dock Pose Simulator 已退出\n")


if __name__ == '__main__':
    main()
