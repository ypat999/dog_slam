#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
GPS Waypoint Navigator for nav2_dog_slam
功能：利用两点手动标定 (Calibration) 建立 GPS 到 Map 的局部坐标转换关系，并下发导航目标点。

使用方法：
1. 标定起点: ros2 run nav2_dog_slam gps_waypoint_navigator.py --calibrate pt1
2. 移动机器狗 3 米后标定终点: ros2 run nav2_dog_slam gps_waypoint_navigator.py --calibrate pt2
3. 下发导航目标: ros2 run nav2_dog_slam gps_waypoint_navigator.py --navigate --lat <纬度> --lon <经度>
"""

import os
import sys
import json
import math
import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import Buffer, TransformListener

# 检查依赖
try:
    import utm
except ImportError:
    print("Error: Missing 'utm' package. Please install it with: pip3 install utm")
    sys.exit(1)

# 配置保存路径
CONFIG_FILE = os.path.expanduser("~/.ros/gps_datum.json")

class GpsWaypointNavigator(Node):
    def __init__(self, mode, pt_id=None, target_lat=None, target_lon=None):
        super().__init__('gps_waypoint_navigator')
        self.mode = mode
        self.pt_id = pt_id
        self.target_lat = target_lat
        self.target_lon = target_lon

        self.current_gps = None
        self.current_map_pose = None
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # QoS for GPS
        gps_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )
        
        if self.mode == 'calibrate':
            self.get_logger().info(f"开启标定模式: 记录 {self.pt_id.upper()}")
            self.gps_sub = self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, gps_qos)
            self.timer = self.create_timer(1.0, self.calibration_check_timer)
        
        elif self.mode == 'navigate':
            self.get_logger().info(f"开启导航模式: 目标 GPS -> Lat: {self.target_lat}, Lon: {self.target_lon}")
            self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
            self.timer = self.create_timer(1.0, self.navigate_timer)

    def gps_callback(self, msg):
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        if getattr(msg.status, 'status', 0) < 0:
            return # 无效GPS
        self.current_gps = (msg.latitude, msg.longitude)

    def get_robot_map_pose(self):
        try:
            # 监听 base_link 在 map 中的位置
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('map', 'base_link', now, rclpy.duration.Duration(seconds=1.0))
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            return (x, y)
        except Exception as e:
            self.get_logger().debug(f"TF 获取失败: {str(e)}")
            return None

    def calibration_check_timer(self):
        """标定模式的连续检查循环"""
        self.current_map_pose = self.get_robot_map_pose()
        
        if self.current_gps is None:
            self.get_logger().info("等待有效的 GPS 信号...", throttle_duration_sec=2.0)
            return
            
        if self.current_map_pose is None:
            self.get_logger().info("等待 /map -> base_link TF 坐标关系建立...", throttle_duration_sec=2.0)
            return

        # 满足条件，记录点数据
        lat, lon = self.current_gps
        map_x, map_y = self.current_map_pose
        
        # 转换为 UTM 平面坐标
        utm_x, utm_y, utm_zone_num, utm_zone_letter = utm.from_latlon(lat, lon)
        
        point_data = {
            'gps': {'lat': lat, 'lon': lon},
            'utm': {'x': utm_x, 'y': utm_y, 'zone': utm_zone_num, 'letter': utm_zone_letter},
            'map': {'x': map_x, 'y': map_y}
        }
        
        # 读取现有 config
        config = self._load_config()
        if 'points' not in config:
            config['points'] = {}
            
        config['points'][self.pt_id] = point_data
        
        self.get_logger().info(f"成功记录 {self.pt_id.upper()}:")
        self.get_logger().info(f"  GPS: ({lat:.6f}, {lon:.6f})")
        self.get_logger().info(f"  MAP: ({map_x:.2f}, {map_y:.2f})")
        
        # 如果是 pt2，执行转换矩阵的计算
        if self.pt_id == 'pt2':
            if 'pt1' not in config['points']:
                self.get_logger().error("缺失 pt1 标定数据，请先执行 --calibrate pt1。")
            else:
                self.calculate_datum(config)
        
        # 保存并退出
        self._save_config(config)
        rclpy.shutdown()

    def calculate_datum(self, config):
        """计算 UTM(正北) 到 Map(局部) 坐标系的转换矩阵 (由于标度通常一致，这里只求 Yaw 和平移)"""
        p1 = config['points']['pt1']
        p2 = config['points']['pt2']
        
        if p1['utm']['zone'] != p2['utm']['zone']:
            self.get_logger().warn("警告：两次标定跨越了 UTM 区域，可能引入误差！")
            
        # 1. 计算两个标定点在两套坐标系下的向量
        dx_utm = p2['utm']['x'] - p1['utm']['x']
        dy_utm = p2['utm']['y'] - p1['utm']['y']
        
        dx_map = p2['map']['x'] - p1['map']['x']
        dy_map = p2['map']['y'] - p1['map']['y']
        
        dist_utm = math.hypot(dx_utm, dy_utm)
        dist_map = math.hypot(dx_map, dy_map)
        
        if dist_utm < 1.0 or dist_map < 1.0:
            self.get_logger().error("错误：两点距离太近（< 1米）。请重新标定使两点相距足够远（建议3米以上）。")
            return
            
        # 2. 计算航向偏角 Yaw Offset (Map相对于UTM转了多少度)
        yaw_utm = math.atan2(dy_utm, dx_utm)
        yaw_map = math.atan2(dy_map, dx_map)
        
        # rotation = map_angle - utm_angle
        yaw_offset = yaw_map - yaw_utm 
        
        # Normalize to [-pi, pi]
        yaw_offset = math.atan2(math.sin(yaw_offset), math.cos(yaw_offset))
        
        # 3. 将 UTM 基准平移：我们以 point1 为原点对照基准 (Datum)
        datum = {
            'pt1_utm_x': p1['utm']['x'],
            'pt1_utm_y': p1['utm']['y'],
            'pt1_map_x': p1['map']['x'],
            'pt1_map_y': p1['map']['y'],
            'yaw_offset': yaw_offset,
            'utm_zone': p1['utm']['zone'],
            'utm_letter': p1['utm']['letter'],
            'scale_factor': dist_map / dist_utm  # 记录留作参考，通常接近 1.0
        }
        
        config['datum'] = datum
        self.get_logger().info(f"\n=======================")
        self.get_logger().info(f"标定计算成功！Datum 参数：")
        self.get_logger().info(f" 偏航角补偿: {math.degrees(yaw_offset):.2f}°")
        self.get_logger().info(f" 距离校准比: {datum['scale_factor']: .4f} (越接近1越好)")
        self.get_logger().info(f"=======================\n")

    def navigate_timer(self):
        """读取 Datum，将目标 GPS 转为 Map Pose 并下发"""
        config = self._load_config()
        if 'datum' not in config:
            self.get_logger().error("找不到 Datum 坐标系统配置，请先运行标定程序 (--calibrate pt1 / pt2)。")
            rclpy.shutdown()
            return
            
        datum = config['datum']
        
        # 1. 目标 GPS 换算 UTM
        try:
            t_utm_x, t_utm_y, t_zone, t_letter = utm.from_latlon(self.target_lat, self.target_lon)
        except Exception as e:
            self.get_logger().error(f"UTM 转换失败：{str(e)}")
            rclpy.shutdown()
            return

        # 2. 计算目标 UTM 相对于 pt1 UTM 的偏移量
        dx_utm = t_utm_x - datum['pt1_utm_x']
        dy_utm = t_utm_y - datum['pt1_utm_y']
        
        # 3. 将 UTM 偏移量旋转到 Map 坐标轴方向
        theta = datum['yaw_offset']
        dx_map_rot = dx_utm * math.cos(theta) - dy_utm * math.sin(theta)
        dy_map_rot = dx_utm * math.sin(theta) + dy_utm * math.cos(theta)
        
        # 4. 加上 pt1 的 Map 原点得到最终 Map 坐标
        target_map_x = datum['pt1_map_x'] + dx_map_rot
        target_map_y = datum['pt1_map_y'] + dy_map_rot
        
        self.get_logger().info(f"目标 Map 坐标: X = {target_map_x:.2f}, Y = {target_map_y:.2f}")

        # 5. 发布 /goal_pose
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.pose.position.x = target_map_x
        msg.pose.position.y = target_map_y
        msg.pose.position.z = 0.0
        
        # 为了朝向目标，计算朝南目标点的 yaw
        curr_pose = self.get_robot_map_pose()
        if curr_pose:
            target_yaw = math.atan2(target_map_y - curr_pose[1], target_map_x - curr_pose[0])
            msg.pose.orientation.z = math.sin(target_yaw / 2.0)
            msg.pose.orientation.w = math.cos(target_yaw / 2.0)
        else:
            msg.pose.orientation.w = 1.0

        self.goal_pub.publish(msg)
        self.get_logger().info("目标点已成功下发给 Nav2。")
        
        # 发布一次即可
        rclpy.shutdown()

    def _load_config(self):
        if os.path.exists(CONFIG_FILE):
            with open(CONFIG_FILE, 'r') as f:
                return json.load(f)
        return {}

    def _save_config(self, config):
        os.makedirs(os.path.dirname(CONFIG_FILE), exist_ok=True)
        with open(CONFIG_FILE, 'w') as f:
            json.dump(config, f, indent=4)
        self.get_logger().info(f"配置已保存至 {CONFIG_FILE}")

def main(args=None):
    parser = argparse.ArgumentParser(description='GPS Waypoint Navigator & Calibrator')
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--calibrate', choices=['pt1', 'pt2'], help='标定模式：记录坐标基准点 1 或 2')
    group.add_argument('--navigate', action='store_true', help='导航模式：下发目标位置')
    
    parser.add_argument('--lat', type=float, help='(导航模式必备) 目标纬度')
    parser.add_argument('--lon', type=float, help='(导航模式必备) 目标经度')

    parsed_args, ros_args = parser.parse_known_args()

    if parsed_args.navigate and (parsed_args.lat is None or parsed_args.lon is None):
        parser.error("--navigate 需要提供 --lat 和 --lon 参数。")

    rclpy.init(args=ros_args)

    mode = 'calibrate' if parsed_args.calibrate else 'navigate'
    try:
        navigator = GpsWaypointNavigator(
            mode=mode, 
            pt_id=parsed_args.calibrate, 
            target_lat=parsed_args.lat, 
            target_lon=parsed_args.lon
        )
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
