#!/usr/bin/env python3


# 订阅/map，获取二维地图，以opencv图像形式留存
# 订阅/scan，获取当前扫描，积累20帧叠加，以opencv图像形式留存
# 使用sift算法，匹配当前扫描与地图，获取位姿偏移。记得留存地图和scan的原点作为变换基准
# 发布位姿偏移到/odom


import rclpy
from rclpy.node import Node
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
        self.declare_parameter('scan_buffer_size', 20)  # 扫描数据缓存大小
        self.declare_parameter('map_resolution', 0.05)  # 地图分辨率
        self.declare_parameter('scan_range_max', 10.0)  # 扫描最大范围
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
        
        # sift特征检测器
        self.sift = cv2.SIFT_create()
        
        # FLANN匹配器
        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=50)
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)
        
        # 订阅者
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
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
            
            self.map_image = map_image
            self.get_logger().info(f'地图已更新: {width}x{height}, 原点: {self.map_origin}')
            
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
    
    def laser_scan_to_image(self, scan_msg):
        """将激光扫描数据转换为OpenCV图像"""
        # 获取扫描参数
        angle_min = scan_msg.angle_min
        angle_max = scan_msg.angle_max
        angle_increment = scan_msg.angle_increment
        ranges = scan_msg.ranges
        
        # 过滤无效数据
        valid_ranges = [r if r < self.scan_range_max else self.scan_range_max 
                       for r in ranges if r > 0]
        
        if not valid_ranges:
            return None
        
        # 创建极坐标到笛卡尔坐标的转换
        angles = np.arange(angle_min, angle_max + angle_increment, angle_increment)
        
        # 转换为笛卡尔坐标
        x_coords = []
        y_coords = []
        
        for i, r in enumerate(ranges):
            if 0 < r < self.scan_range_max:
                angle = angles[i] if i < len(angles) else angle_min + i * angle_increment
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                x_coords.append(x)
                y_coords.append(y)
        
        if not x_coords:
            return None
        
        # 归一化坐标到图像尺寸
        x_min, x_max = min(x_coords), max(x_coords)
        y_min, y_max = min(y_coords), max(y_coords)
        
        # 设置图像尺寸
        img_size = 400
        scale = min(img_size / (x_max - x_min), img_size / (y_max - y_min)) * 0.8
        
        # 创建图像
        scan_image = np.zeros((img_size, img_size), dtype=np.uint8)
        
        # 绘制扫描点
        for x, y in zip(x_coords, y_coords):
            img_x = int((x - x_min) * scale)
            img_y = int((y - y_min) * scale)
            if 0 <= img_x < img_size and 0 <= img_y < img_size:
                cv2.circle(scan_image, (img_x, img_y), 2, 255, -1)
        
        return scan_image
    
    def process_scan_data(self):
        """处理累积的扫描数据"""
        if self.map_image is None:
            self.get_logger().warn('地图数据尚未准备好')
            return
        
        if len(self.scan_buffer) < self.scan_buffer_size:
            return
        
        try:
            # 叠加多帧扫描数据
            combined_scan_image = None
            
            for scan_msg in self.scan_buffer:
                scan_img = self.laser_scan_to_image(scan_msg)
                if scan_img is not None:
                    if combined_scan_image is None:
                        combined_scan_image = scan_img.copy()
                    else:
                        combined_scan_image = cv2.add(combined_scan_image, scan_img)
            
            if combined_scan_image is None:
                self.get_logger().warn('无法生成有效的扫描图像')
                return
            
            # 使用sift进行特征匹配
            pose_offset = self.sift_feature_matching(combined_scan_image)
            
            if pose_offset is not None:
                self.publish_odometry(pose_offset)
                
        except Exception as e:
            self.get_logger().error(f'处理扫描数据时出错: {str(e)}')
    
    def sift_feature_matching(self, scan_image):
        """使用sift算法进行特征匹配"""
        try:
            # 检测sift特征和描述符
            kp1, des1 = self.sift.detectAndCompute(scan_image, None)
            kp2, des2 = self.sift.detectAndCompute(self.map_image, None)
            
            if des1 is None or des2 is None:
                self.get_logger().warn('无法检测到足够的特征点')
                return None
            
            # 使用FLANN进行特征匹配
            matches = self.flann.knnMatch(des1, des2, k=2)
            
            # 应用Lowe's比率测试
            good_matches = []
            for m, n in matches:
                if m.distance < 0.7 * n.distance:
                    good_matches.append(m)
            
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
            
            return (translation_x, translation_y, rotation)
            
        except Exception as e:
            self.get_logger().error(f'sift特征匹配时出错: {str(e)}')
            return None
    
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