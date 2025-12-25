#!/usr/bin/env python3


# 订阅/map，获取二维地图，以opencv图像形式留存
# 订阅/scan，获取当前扫描，积累20帧叠加，以opencv图像形式留存
# 使用sift算法，匹配当前扫描与地图，获取位姿偏移。记得留存地图和scan的原点作为变换基准
# 发布位姿偏移到/odom
#初步判断不可用

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import numpy as np
import cv2
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf2_ros import TransformBroadcaster
import tf_transformations
from collections import deque
import math


class siftMapMatching(Node):
    """
    sift地图匹配节点
    
    功能：
    1. 订阅/map话题，获取二维地图并转换为OpenCV图像
    2. 订阅/scan话题，获取激光扫描数据，积累20帧叠加后转换为OpenCV图像
    3. 使用sift算法匹配当前扫描图像与地图图像，计算位姿偏移
    4. 发布位姿偏移到/odom话题
    """
    
    def __init__(self):
        super().__init__('sift_map_matching')
        
        # 参数设置
        self.declare_parameter('scan_buffer_size', 100)  # 扫描数据缓存大小
        self.declare_parameter('map_resolution', 0.05)  # 地图分辨率
        self.declare_parameter('scan_range_max', 50.0)  # 扫描最大范围
        self.declare_parameter('min_match_count', 10)   # 最小匹配特征点数
        
        self.scan_buffer_size = self.get_parameter('scan_buffer_size').value
        self.map_resolution = self.get_parameter('map_resolution').value
        self.scan_range_max = self.get_parameter('scan_range_max').value
        self.min_match_count = self.get_parameter('min_match_count').value
        
        # 数据存储
        self.map_image = None  # 地图图像
        self.map_origin = None  # 地图原点
        self.map_info = None  # 地图信息
        self.scan_buffer = deque(maxlen=self.scan_buffer_size)  # 扫描数据缓冲区
        self.scan_origin = None  # 扫描原点
        
        # ORB特征检测器 - 更适合几何线条
        self.orb = cv2.ORB_create(nfeatures=1000, scaleFactor=1.2, nlevels=10, edgeThreshold=15, fastThreshold=100)
        
        # 暴力匹配器 - 更适合ORB特征
        self.bf_matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        
        # 订阅者
        # 创建best effort QoS配置
        best_effort_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            best_effort_qos
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            best_effort_qos
        )
        
        # 发布者
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info('sift地图匹配节点已启动')
    
    def map_callback(self, msg):
        """处理地图数据回调"""
        try:
            # 保存地图信息
            self.map_info = msg.info
            self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
            
            # 将OccupancyGrid转换为OpenCV图像
            width = msg.info.width
            height = msg.info.height
            
            # 创建图像
            map_data = np.array(msg.data, dtype=np.int8).reshape((height, width))
            
            # 将-1(未知)转换为128，0(空闲)转换为255，100(占用)转换为0
            map_image = np.zeros((height, width), dtype=np.uint8)
            map_image[map_data == -1] = 128  # 未知区域
            map_image[map_data == 0] = 255   # 空闲区域
            map_image[map_data == 100] = 0   # 占用区域
            
            # 修正地图图像方向，确保与扫描数据坐标系一致
            # 地图通常使用世界坐标系，需要与激光雷达坐标系对齐
            self.map_image = cv2.flip(map_image, 0)  # 垂直翻转，修正Y轴方向
            
            self.get_logger().info(f'地图已更新: {width}x{height}, 原点: {self.map_origin}')
            
            # 显示地图图像
            if width > 0 and height > 0:
                # 调整图像大小以便显示
                display_map = cv2.resize(self.map_image, (800, 600)) if width > 800 or height > 600 else self.map_image
                cv2.imshow('Map', display_map)
                cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'处理地图数据时出错: {str(e)}')
    
    def scan_callback(self, msg):
        """处理激光扫描数据回调"""
        try:
            # 保存当前扫描数据
            self.scan_buffer.append(msg)
            
            # 如果缓冲区已满，开始处理
            if len(self.scan_buffer) >= self.scan_buffer_size:
                self.process_scan_data()
                
        except Exception as e:
            self.get_logger().error(f'处理扫描数据时出错: {str(e)}')
    
    def scan_to_coordinates(self, scan_msg):
        """将激光扫描数据转换为笛卡尔坐标"""
        # 获取扫描参数
        angle_min = scan_msg.angle_min
        angle_max = scan_msg.angle_max
        angle_increment = scan_msg.angle_increment
        ranges = scan_msg.ranges
        
        # 转换为笛卡尔坐标
        x_coords = []
        y_coords = []
        
        for i, r in enumerate(ranges):
            if 0 < r < self.scan_range_max:
                angle = angle_min + i * angle_increment
                # 修正坐标系：将激光雷达坐标系转换为图像坐标系
                x = r * math.cos(angle)  # 保持X轴方向不变
                y = -r * math.sin(angle) # Y轴取反，解决镜像问题
                x_coords.append(x)
                y_coords.append(y)
        
        return x_coords, y_coords
    
    def coordinates_to_image(self, scan_msg, x_min, x_max, y_min, y_max):
        """将坐标转换为图像，使用统一的归一化参数"""
        x_coords, y_coords = self.scan_to_coordinates(scan_msg)
        
        if not x_coords:
            return None
        
        # 设置图像尺寸
        img_size = 400
        scale = min(img_size / (x_max - x_min), img_size / (y_max - y_min)) * 0.8
        
        # 创建图像
        scan_image = np.zeros((img_size, img_size), dtype=np.uint8)
        
        # 绘制扫描点
        for x, y in zip(x_coords, y_coords):
            # 使用统一的归一化参数
            img_x = int((x - x_min) * scale)
            img_y = int((y - y_min) * scale)
            
            # 确保坐标在图像范围内
            if 0 <= img_x < img_size and 0 <= img_y < img_size:
                cv2.circle(scan_image, (img_x, img_y), 2, 255, -1)
        
        return scan_image
    
    def laser_scan_to_image(self, scan_msg):
        """将激光扫描数据转换为OpenCV图像（单帧，保持向后兼容）"""
        x_coords, y_coords = self.scan_to_coordinates(scan_msg)
        
        if not x_coords:
            return None
        
        # 归一化坐标到图像尺寸
        x_min, x_max = min(x_coords), max(x_coords)
        y_min, y_max = min(y_coords), max(y_coords)
        
        return self.coordinates_to_image(scan_msg, x_min, x_max, y_min, y_max)
    
    def process_scan_data(self):
        """处理累积的扫描数据"""
        if self.map_image is None:
            self.get_logger().warn('地图数据尚未准备好')
            return
        
        if len(self.scan_buffer) < self.scan_buffer_size:
            return
        
        try:
            # 第一步：收集所有扫描点的坐标，计算统一的归一化参数
            all_x_coords = []
            all_y_coords = []
            
            for scan_msg in self.scan_buffer:
                x_coords, y_coords = self.scan_to_coordinates(scan_msg)
                if x_coords and y_coords:
                    all_x_coords.extend(x_coords)
                    all_y_coords.extend(y_coords)
            
            if not all_x_coords:
                self.get_logger().warn('无法获取有效的扫描点坐标')
                return
            
            # 计算统一的归一化参数
            x_min, x_max = min(all_x_coords), max(all_x_coords)
            y_min, y_max = min(all_y_coords), max(all_y_coords)
            
            # 第二步：使用统一参数生成叠加图像
            combined_scan_image = None
            
            for scan_msg in self.scan_buffer:
                scan_img = self.coordinates_to_image(scan_msg, x_min, x_max, y_min, y_max)
                if scan_img is not None:
                    if combined_scan_image is None:
                        combined_scan_image = scan_img.copy()
                    else:
                        combined_scan_image = cv2.add(combined_scan_image, scan_img)
            
            if combined_scan_image is None:
                self.get_logger().warn('无法生成有效的扫描图像')
                return
            
            # 显示扫描图像（修正坐标系方向）
            display_scan = cv2.flip(combined_scan_image, 0)  # 垂直翻转，确保与地图方向一致
            cv2.imshow('Scan', display_scan)
            cv2.waitKey(1)
            
            # 使用ORB进行特征匹配
            pose_offset = self.orb_feature_matching(combined_scan_image)
            
            if pose_offset is not None:
                self.publish_odometry(pose_offset)
                
        except Exception as e:
            self.get_logger().error(f'处理扫描数据时出错: {str(e)}')
    
    def orb_feature_matching(self, scan_image):
        """使用ORB算法进行特征匹配"""
        try:
            # 检测ORB特征和描述符
            kp1, des1 = self.orb.detectAndCompute(scan_image, None)
            kp2, des2 = self.orb.detectAndCompute(self.map_image, None)
            
            if des1 is None or des2 is None or len(kp1) < 10 or len(kp2) < 10:
                self.get_logger().warn(f'无法检测到足够的特征点: 扫描{len(kp1) if kp1 else 0}, 地图{len(kp2) if kp2 else 0}')
                return None
            
            # 使用暴力匹配器进行特征匹配
            matches = self.bf_matcher.match(des1, des2)
            
            # 按距离排序，选择最佳匹配
            matches = sorted(matches, key=lambda x: x.distance)
            
            # 选择前N个最佳匹配
            good_matches = matches[:min(len(matches), 20)]
            
            if len(good_matches) < self.min_match_count:
                self.get_logger().warn(f'匹配点数量不足: {len(good_matches)} < {self.min_match_count}')
                return None
            
            # 提取匹配点的坐标
            src_pts = np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
            dst_pts = np.float32([kp2[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)
            
            # 使用RANSAC计算单应性矩阵
            H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
            
            if H is None:
                self.get_logger().warn('无法计算单应性矩阵')
                return None
            
            # 从单应性矩阵中提取位姿变换
            # 假设是平面运动，只考虑平移和旋转
            translation_x = H[0, 2] * self.map_resolution
            translation_y = H[1, 2] * self.map_resolution
            
            # 计算旋转角度
            rotation = math.atan2(H[1, 0], H[0, 0])
            
            self.get_logger().info(f'匹配成功: 平移({translation_x:.3f}, {translation_y:.3f}), 旋转: {rotation:.3f}rad')
            
            # 显示特征匹配结果
            self.show_feature_matches(scan_image, kp1, kp2, good_matches, H)
            
            return (translation_x, translation_y, rotation)
            
        except Exception as e:
            self.get_logger().error(f'ORB特征匹配时出错: {str(e)}')
            return None
    
    def show_feature_matches(self, scan_image, kp1, kp2, matches, H):
        """显示特征匹配结果"""
        try:
            # 创建匹配可视化图像
            img_matches = cv2.drawMatches(
                scan_image, kp1, self.map_image, kp2, matches[:20], None,
                flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
            )
            
            # 调整图像大小以便显示
            h, w = img_matches.shape[:2]
            if w > 1200 or h > 800:
                scale = min(1200/w, 800/h)
                img_matches = cv2.resize(img_matches, (int(w*scale), int(h*scale)))
            
            cv2.imshow('Feature Matches', img_matches)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().warn(f'显示特征匹配结果时出错: {str(e)}')
    
    def publish_odometry(self, pose_offset):
        """发布位姿数据"""
        try:
            dx, dy, dtheta = pose_offset
            
            # 创建Odometry消息
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = 'map'
            odom_msg.child_frame_id = 'base_link'
            
            # 设置位姿
            odom_msg.pose.pose.position.x = dx
            odom_msg.pose.pose.position.y = dy
            odom_msg.pose.pose.position.z = 0.0
            
            # 转换为四元数
            quat = tf_transformations.quaternion_from_euler(0, 0, dtheta)
            odom_msg.pose.pose.orientation.x = quat[0]
            odom_msg.pose.pose.orientation.y = quat[1]
            odom_msg.pose.pose.orientation.z = quat[2]
            odom_msg.pose.pose.orientation.w = quat[3]
            
            # 发布消息
            self.odom_pub.publish(odom_msg)
            
            # 发布TF变换
            self.publish_tf_transform(dx, dy, dtheta)
            
        except Exception as e:
            self.get_logger().error(f'发布位姿数据时出错: {str(e)}')
    
    def publish_tf_transform(self, x, y, theta):
        """发布TF变换"""
        try:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = 'base_link'
            
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            
            quat = tf_transformations.quaternion_from_euler(0, 0, theta)
            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]
            
            self.tf_broadcaster.sendTransform(t)
            
        except Exception as e:
            self.get_logger().error(f'发布TF变换时出错: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    node = siftMapMatching()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()