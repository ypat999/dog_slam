#!/usr/bin/env python3
"""
GPS到地图坐标系校准节点
监听map->base_footprint的TF和/odometry/gps（UTM坐标），通过最小二乘法拟合GPS到map的TF变换
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import yaml
import os
import math
import sys
from typing import List, Tuple, Optional

try:
    from ament_index_python.packages import get_package_share_directory
    global_config_path = os.path.join(get_package_share_directory('global_config'), '../src/global_config')
    if global_config_path not in sys.path:
        sys.path.insert(0, global_config_path)
    from global_config import NAV2_DEFAULT_MAP_FILE
except ImportError:
    print("Warning: Failed to import global_config, using default calibration file path")
    NAV2_DEFAULT_MAP_FILE = "./map.yaml"


class GPSMapCalibrator(Node):
    def __init__(self):
        super().__init__('gps_map_calibrator')
        
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('published_frame', 'gps_loc')
        self.declare_parameter('gps_odom_topic', '/fix_utm')
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('num_calibration_points', 20)
        self.declare_parameter('min_distance', 10.0)
        self.declare_parameter('tf_timeout', 2.0)
        self.declare_parameter('publish_threshold', 0.01)
        self.declare_parameter('ransac_iterations', 1000)
        self.declare_parameter('ransac_threshold', 2.0)
        self.declare_parameter('ransac_min_samples', 3)
        # self.declare_parameter('mode', 'calibration')  # 模式：'calibration'标定模式，'localization'定位模式
        self.declare_parameter('mode', 'localization')
        
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.published_frame = self.get_parameter('published_frame').value
        self.gps_odom_topic = self.get_parameter('gps_odom_topic').value
        user_calibration_file = self.get_parameter('calibration_file').value
        self.num_calibration_points = self.get_parameter('num_calibration_points').value
        self.min_distance = self.get_parameter('min_distance').value
        self.tf_timeout = self.get_parameter('tf_timeout').value
        self.publish_threshold = self.get_parameter('publish_threshold').value
        self.ransac_iterations = self.get_parameter('ransac_iterations').value
        self.ransac_threshold = self.get_parameter('ransac_threshold').value
        self.ransac_min_samples = self.get_parameter('ransac_min_samples').value
        self.mode = self.get_parameter('mode').value  # 模式
        
        if user_calibration_file:
            self.calibration_file = user_calibration_file
        else:
            map_dir = os.path.dirname(NAV2_DEFAULT_MAP_FILE)
            self.calibration_file = os.path.join(map_dir, 'gps_map_calibration.yaml')
        
        self.calibration_data = []
        self.all_calibration_data = []  # 标定模式：所有历史数据
        self.calibration_batch_count = 0  # 标定模式：批次计数器
        self.last_map_position = None
        self.last_map_time = None
        self.calibration_done = False
        self.tf_transform = None
        self.last_gps_odom = None
        self.last_published_pose = None  # 上次发布的初始位姿
        
        self.tf_broadcaster = None
        self.tf_buffer = None
        self.tf_listener = None
        self.tf_timer = None
        
        # 根据模式初始化
        if self.mode == 'calibration':
            # 标定模式：不加载校准文件，直接开始收集数据
            self.get_logger().info('标定模式：开始收集校准数据')
            self.setup_tf_listener()
            self.setup_subscribers()
        elif self.mode == 'localization':
            # 定位模式：加载校准文件并开始发布TF
            if self.load_calibration():
                self.get_logger().info('定位模式：从文件加载校准参数成功')
                self.calibration_done = True
                self.setup_tf_listener()
                self.setup_tf_broadcaster()
                self.setup_subscribers()
            else:
                self.get_logger().error('定位模式：未找到校准文件，请先运行标定模式')
        else:
            self.get_logger().error(f'未知模式: {self.mode}，支持的模式：calibration, localization')
        
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        
        self.get_logger().info(f'GPS地图校准节点已启动')
        self.get_logger().info(f'map坐标系: {self.map_frame}')
        self.get_logger().info(f'base坐标系: {self.base_frame}')
        self.get_logger().info(f'发布TF坐标系: {self.published_frame}')
        self.get_logger().info(f'GPS里程计话题: {self.gps_odom_topic}')
        self.get_logger().info(f'校准文件: {self.calibration_file}')
        self.get_logger().info(f'发布距离阈值: {self.publish_threshold}m')
    
    def setup_tf_listener(self):
        from tf2_ros import Buffer, TransformListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
    
    def setup_subscribers(self):
        self.gps_odom_sub = self.create_subscription(
            Odometry,
            self.gps_odom_topic,
            self.gps_odom_callback,
            10
        )
    
    def setup_tf_broadcaster(self):
        from tf2_ros import TransformBroadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
    
    def get_map_position(self) -> Optional[Tuple[float, float]]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout)
            )
            
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            
            current_time = self.get_clock().now()
            
            if self.last_map_time is not None:
                time_diff = (current_time.nanoseconds - self.last_map_time.nanoseconds) / 1e9
                if time_diff > self.tf_timeout:
                    self.get_logger().warn(f'TF数据超时 ({time_diff:.2f}s)，重置最后GPS数据')
                    self.last_gps_odom = None
            
            self.last_map_position = (x, y)
            self.last_map_time = current_time
            
            return (x, y)
        except Exception as e:
            self.get_logger().debug(f'无法获取TF变换: {e}')
            return None
    
    def is_valid_gps_odom(self, msg: Odometry) -> bool:
        if math.isnan(msg.pose.pose.position.x) or math.isnan(msg.pose.pose.position.y):
            self.get_logger().warning('GPS里程计位置包含NaN值')
            return False
        
        return True
    
    def gps_odom_callback(self, msg: Odometry):
        self.get_logger().info(f'收到GPS里程计数据')
        if not self.is_valid_gps_odom(msg):
            return
        
        # 定位模式：只执行TF距离检测和发布
        if self.mode == 'localization':
            if self.calibration_done and self.last_gps_odom is not None:
                self.calculate_and_publish_initialpose(msg)
            self.last_gps_odom = msg
            return
        
        # 标定模式：收集数据并计算校准参数
        map_position = self.get_map_position()
        
        if map_position is None:
            self.get_logger().warning('未收到有效的TF数据，跳过GPS数据')
            return
        
        map_x, map_y = map_position
        gps_utm_x = msg.pose.pose.position.x
        gps_utm_y = msg.pose.pose.position.y
        
        # 添加到当前批次数据
        self.calibration_data.append((
            map_x,
            map_y,
            gps_utm_x,
            gps_utm_y
        ))
        
        # 检查距离过滤
        if len(self.calibration_data) > 1:
            last_x, last_y, _, _ = self.calibration_data[-2]
            distance = math.sqrt((map_x - last_x)**2 + (map_y - last_y)**2)
            if distance < self.min_distance:
                self.get_logger().debug(f'距离太近 ({distance:.2f}m)，跳过此点')
                self.calibration_data.pop()  # 移除刚才添加的点
                return
        
        # 检查批次是否完成
        if len(self.calibration_data) >= self.num_calibration_points:
            self.get_logger().info(f'批次{self.calibration_batch_count}：已收集{len(self.calibration_data)}对校准数据，开始计算变换')
            self.get_logger().info(f'添加前历史数据量：{len(self.all_calibration_data)}对')
            self.get_logger().info(f'calibration_data id: {id(self.calibration_data)}, all_calibration_data id: {id(self.all_calibration_data)}')
              
            # 将当前批次数据添加到所有历史数据
            self.all_calibration_data.extend(self.calibration_data)
              
            self.get_logger().info(f'添加后历史数据量：{len(self.all_calibration_data)}对')
            self.get_logger().info(f'all_calibration_data前3个数据: {self.all_calibration_data[:3]}')
              
            # 使用所有历史数据重新计算校准参数
            self.recalculate_calibration()
              
            # 清空当前批次数据
            self.calibration_data = []
            self.calibration_batch_count += 1
              
            self.get_logger().info(f'批次{self.calibration_batch_count}完成，总数据量：{len(self.all_calibration_data)}对')
        else:
            self.get_logger().info(f'收集校准数据 {len(self.calibration_data)}/{self.num_calibration_points}: '
                                   f'map=({map_x:.2f}, {map_y:.2f}), '
                                   f'GPS_UTM=({gps_utm_x:.2f}, {gps_utm_y:.2f})')
    
    def recalculate_calibration(self):
        """使用RANSAC算法重新计算校准参数，去除离群点"""
        self.get_logger().info(f'recalculate_calibration开始，all_calibration_data数量：{len(self.all_calibration_data)}')
        
        if len(self.all_calibration_data) < self.ransac_min_samples:
            self.get_logger().error(f'历史校准数据不足，至少需要{self.ransac_min_samples}个点')
            return
        
        map_points = np.array([[d[0], d[1]] for d in self.all_calibration_data])
        gps_utm_points = np.array([[d[2], d[3]] for d in self.all_calibration_data])
        
        self.get_logger().info(f'map_points数量：{len(map_points)}, gps_utm_points数量：{len(gps_utm_points)}')
        
        # 使用RANSAC算法进行鲁棒拟合
        best_inliers = []
        best_transform = None
        best_error = float('inf')
        
        num_points = len(self.all_calibration_data)
        
        self.get_logger().info(f'开始RANSAC迭代，迭代次数：{self.ransac_iterations}')
        
        for iteration in range(self.ransac_iterations):
            # 随机采样最小样本数
            sample_indices = np.random.choice(num_points, self.ransac_min_samples, replace=False)
            
            # 提取采样点
            sample_map = map_points[sample_indices]
            sample_gps = gps_utm_points[sample_indices]
            
            # 使用采样点计算变换
            R, t, yaw = self.calculate_transform_from_points(sample_map, sample_gps)
            
            if R is None:
                continue
            
            # 计算所有点的误差
            errors = self.calculate_transform_errors(map_points, gps_utm_points, R, t)
            
            # 统计内点（误差小于阈值的点）
            inliers = np.where(errors < self.ransac_threshold)[0]
            
            # 如果内点数量更多，更新最佳结果
            if len(inliers) > len(best_inliers):
                best_inliers = inliers
                best_transform = (R, t, yaw)
                best_error = np.mean(errors[inliers])
                
                if (iteration + 1) % 100 == 0:
                    self.get_logger().info(f'迭代{iteration + 1}: 内点数={len(inliers)}, 最佳内点数={len(best_inliers)}')
        
        if len(best_inliers) < self.ransac_min_samples:
            self.get_logger().error(f'RANSAC失败，内点数量不足：{len(best_inliers)}')
            return
        
        # 使用所有内点重新计算最终变换
        inlier_map = map_points[best_inliers]
        inlier_gps = gps_utm_points[best_inliers]
        
        self.get_logger().info(f'RANSAC完成：总点数={num_points}, 内点数={len(best_inliers)}, 离群点数={num_points - len(best_inliers)}')
        
        # 使用内点计算最终变换
        R, t, yaw = self.calculate_transform_from_points(inlier_map, inlier_gps)
        
        if R is None:
            self.get_logger().error('最终变换计算失败')
            return
        
        # 使用固定缩放因子1.0（因为都是米制单位）
        scale = 1.0
        
        self.tf_transform = {
            'translation': {
                'x': t[0],
                'y': t[1]
            },
            'rotation': {
                'yaw': yaw
            },
            'scale': scale
        }
        
        self.get_logger().info('重新计算校准完成！')
        self.get_logger().info(f'平移: x={t[0]:.6f}, y={t[1]:.6f}')
        self.get_logger().info(f'旋转: yaw={yaw:.6f} rad')
        self.get_logger().info(f'缩放: {scale:.6f}')
        
        # 验证校准结果
        self.validate_calibration(gps_utm_points, map_points)
        
        # 保存校准参数
        self.save_calibration()
        self.calibration_done = True
    
    def calculate_transform_from_points(self, map_points, gps_points):
        """使用最小二乘法计算从GPS到map的变换"""
        if len(map_points) < 2 or len(gps_points) < 2:
            return None, None, None
        
        # 计算中心点
        map_center = np.mean(map_points, axis=0)
        gps_center = np.mean(gps_points, axis=0)
        
        # 中心化坐标
        map_centered = map_points - map_center
        gps_centered = gps_points - gps_center
        
        # 计算旋转矩阵
        H = gps_centered.T @ map_centered
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        
        if np.linalg.det(R) < 0:
            Vt[1, :] *= -1
            R = Vt.T @ U.T
        
        # 计算旋转角度
        yaw = math.atan2(R[1, 0], R[0, 0])
        
        # 计算平移：t = map_center - R * gps_center
        t = map_center - R @ gps_center
        
        return R, t, yaw
    
    def calculate_transform_errors(self, map_points, gps_points, R, t):
        """计算所有点在给定变换下的误差"""
        # 应用变换：map = R * gps + t
        transformed_gps = (R @ gps_points.T).T + t
        
        # 计算误差
        errors = np.sqrt(np.sum((map_points - transformed_gps) ** 2, axis=1))
        
        return errors
    
    def validate_calibration(self, src_points, dst_points):
        """验证校准结果的合理性"""
        tx = self.tf_transform['translation']['x']
        ty = self.tf_transform['translation']['y']
        yaw = self.tf_transform['rotation']['yaw']
        scale = self.tf_transform['scale']
        
        # 检查平移值是否合理（应该在几百米范围内）
        if abs(tx) > 10000 or abs(ty) > 10000:
            self.get_logger().warning(f'平移值异常大: x={tx:.2f}, y={ty:.2f}')
        
        # 检查缩放因子是否合理（应该接近1.0）
        if abs(scale - 1.0) > 0.1:
            self.get_logger().warning(f'缩放因子异常: {scale:.6f}')
        
        # 测试变换
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        
        for i in range(min(3, len(src_points))):
            src_x, src_y = src_points[i]
            dst_x, dst_y = dst_points[i]
            
            # 应用变换：dst = R * src * scale + t
            transformed_x = (src_x * cos_yaw - src_y * sin_yaw) * scale + tx
            transformed_y = (src_x * sin_yaw + src_y * cos_yaw) * scale + ty
            
            error = math.sqrt((transformed_x - dst_x)**2 + (transformed_y - dst_y)**2)
            self.get_logger().info(f'验证点{i+1}: 误差={error:.4f}m')
    
    def calculate_and_publish_initialpose(self, current_gps_odom: Odometry):
        # 标定模式下不发布TF
        if self.mode == 'calibration':
            return
        
        if self.last_gps_odom is None or self.tf_transform is None:
            self.get_logger().warning('未完成校准或未收到GPS数据')
            return
        
        x1 = self.last_gps_odom.pose.pose.position.x
        y1 = self.last_gps_odom.pose.pose.position.y
        x2 = current_gps_odom.pose.pose.position.x
        y2 = current_gps_odom.pose.pose.position.y
        
        dx = x2 - x1
        dy = y2 - y1
        
        bearing = math.atan2(dy, dx)
        
        # 修正：使用新的变换关系
        # map = R * gps_utm + t
        # 所以方向角：yaw = bearing + yaw_rot
        yaw = bearing + self.tf_transform['rotation']['yaw']
        
        from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
        
        gps_utm_x = current_gps_odom.pose.pose.position.x
        gps_utm_y = current_gps_odom.pose.pose.position.y
        
        # 使用新的变换公式
        tx = self.tf_transform['translation']['x']
        ty = self.tf_transform['translation']['y']
        yaw_rot = self.tf_transform['rotation']['yaw']
        
        # 计算旋转矩阵
        cos_yaw = math.cos(yaw_rot)
        sin_yaw = math.sin(yaw_rot)
        
        # 应用变换：map = R * gps_utm + t
        map_x = gps_utm_x * cos_yaw - gps_utm_y * sin_yaw + tx
        map_y = gps_utm_x * sin_yaw + gps_utm_y * cos_yaw + ty
        
        # 获取base_frame在map坐标系下的位置
        base_position = self.get_map_position()
        
        # 计算gps_loc到base_frame的距离
        distance = -1.0
        should_publish = True
        if base_position is not None:
            base_x, base_y = base_position
            distance = math.sqrt((map_x - base_x)**2 + (map_y - base_y)**2)
            if distance < self.publish_threshold:
                self.get_logger().info(f'gps_loc到{self.base_frame}距离{distance:.2f}m < 阈值{self.publish_threshold}m，跳过发布')
                should_publish = False
        
        if should_publish:
            # 发布map->published_frame的TF
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = self.map_frame
            transform.child_frame_id = self.published_frame
            
            transform.transform.translation.x = map_x
            transform.transform.translation.y = map_y
            transform.transform.translation.z = 0.0
            
            transform.transform.rotation.z = math.sin(yaw / 2)
            transform.transform.rotation.w = math.cos(yaw / 2)
            
            self.tf_broadcaster.sendTransform(transform)
            self.last_published_pose = (map_x, map_y)
            
            self.get_logger().info(f'gps_loc到{self.base_frame}距离{distance:.2f}m, 发布TF: {self.map_frame}->{self.published_frame} pos=({map_x:.2f}, {map_y:.2f}), yaw={yaw:.2f}')
            
            # 暂时注释掉initialpose发布
            # initialpose = PoseWithCovarianceStamped()
            # initialpose.header.stamp = self.get_clock().now().to_msg()
            # initialpose.header.frame_id = 'map'
            # 
            # initialpose.pose.pose.position.x = map_x
            # initialpose.pose.pose.position.y = map_y
            # initialpose.pose.pose.position.z = 0.0
            # 
            # initialpose.pose.pose.orientation.z = math.sin(yaw / 2)
            # initialpose.pose.pose.orientation.w = math.cos(yaw / 2)
            # 
            # self.initialpose_pub.publish(initialpose)
            # self.last_published_pose = (map_x, map_y)
            # 
            # self.get_logger().info(f'发布初始位姿: pos=({map_x:.2f}, {map_y:.2f}), yaw={yaw:.2f}')
    
    def save_calibration(self):
        try:
            calibration_dir = os.path.dirname(self.calibration_file)
            if calibration_dir and not os.path.exists(calibration_dir):
                os.makedirs(calibration_dir)
            
            # 将numpy类型转换为Python原生类型
            tf_transform_safe = {
                'translation': {
                    'x': float(self.tf_transform['translation']['x']),
                    'y': float(self.tf_transform['translation']['y'])
                },
                'rotation': {
                    'yaw': float(self.tf_transform['rotation']['yaw'])
                },
                'scale': float(self.tf_transform['scale'])
            }
            
            with open(self.calibration_file, 'w') as f:
                yaml.dump(tf_transform_safe, f, default_flow_style=False)
            
            self.get_logger().info(f'校准参数已保存到: {self.calibration_file}')
        except Exception as e:
            self.get_logger().error(f'保存校准参数失败: {e}')
    
    def load_calibration(self) -> bool:
        try:
            if not os.path.exists(self.calibration_file):
                return False
            
            with open(self.calibration_file, 'r') as f:
                tf_transform_loaded = yaml.safe_load(f)
            
            # 处理numpy类型数据
            def convert_numpy_to_float(obj):
                if isinstance(obj, dict):
                    return {k: convert_numpy_to_float(v) for k, v in obj.items()}
                elif isinstance(obj, list):
                    return [convert_numpy_to_float(item) for item in obj]
                elif hasattr(obj, 'item'):  # numpy标量类型
                    return float(obj.item())
                else:
                    return obj
            
            self.tf_transform = convert_numpy_to_float(tf_transform_loaded)
            
            self.get_logger().info(f'从文件加载校准参数: {self.calibration_file}')
            self.get_logger().info(f'平移: x={self.tf_transform["translation"]["x"]:.6f}, y={self.tf_transform["translation"]["y"]:.6f}')
            self.get_logger().info(f'旋转: yaw={self.tf_transform["rotation"]["yaw"]:.6f} rad')
            self.get_logger().info(f'缩放: {self.tf_transform["scale"]:.6f}')
            return True
        except Exception as e:
            self.get_logger().error(f'加载校准参数失败: {e}')
            return False


def main(args=None):
    rclpy.init(args=args)
    
    try:
        calibrator = GPSMapCalibrator()
        rclpy.spin(calibrator)
    except KeyboardInterrupt:
        pass
    finally:
        calibrator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
