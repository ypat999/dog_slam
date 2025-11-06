#!/usr/bin/env python3
"""
检查速度平滑器配置的脚本
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter_service import GetParameters
import time

class VelocityConfigChecker(Node):
    def __init__(self):
        super().__init__('velocity_config_checker')
        
        # 创建参数获取客户端
        self.velocity_client = self.create_client(
            GetParameters,
            '/velocity_smoother/get_parameters'
        )
        
        # 等待服务可用
        while not self.velocity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待速度平滑器服务...')
        
        self.get_logger().info('服务已连接')
        
    def check_feedback_mode(self):
        """检查当前的反馈模式配置"""
        try:
            # 获取feedback参数
            request = GetParameters.Request()
            request.names = ['feedback']
            
            future = self.velocity_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            
            if future.result() is not None:
                response = future.result()
                if response.values:
                    feedback_value = response.values[0].string_value
                    self.get_logger().info(f"当前feedback模式: {feedback_value}")
                    
                    # 提供建议
                    if feedback_value == "OPEN_LOOP":
                        self.get_logger().info("✓ 当前为开环模式，适合直线+旋转导航策略")
                    elif feedback_value == "CLOSED_LOOP":
                        self.get_logger().info("✓ 当前为闭环模式，适合精密导航")
                    else:
                        self.get_logger().warn(f"⚠ 未知的feedback模式: {feedback_value}")
                else:
                    self.get_logger().error("无法获取feedback参数值")
            else:
                self.get_logger().error("无法获取参数")
                
        except Exception as e:
            self.get_logger().error(f"检查参数时出错: {e}")
    
    def check_all_velocity_params(self):
        """检查所有速度平滑器参数"""
        try:
            # 获取关键参数
            request = GetParameters.Request()
            request.names = [
                'feedback',
                'smoothing_frequency', 
                'max_velocity',
                'min_velocity',
                'max_accel',
                'max_decel'
            ]
            
            future = self.velocity_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            
            if future.result() is not None:
                response = future.result()
                self.get_logger().info("=== 速度平滑器配置 ===")
                
                for param in response.values:
                    if param.type == 1:  # STRING
                        value = param.string_value
                    elif param.type == 2:  # INTEGER
                        value = param.integer_value
                    elif param.type == 3:  # DOUBLE
                        value = param.double_value
                    elif param.type == 4:  # BOOL
                        value = param.bool_value
                    elif param.type == 5:  # STRING_ARRAY
                        value = list(param.string_array_value)
                    elif param.type == 6:  # DOUBLE_ARRAY
                        value = list(param.double_array_value)
                    else:
                        value = "未知类型"
                    
                    self.get_logger().info(f"  {param.name}: {value}")
                    
            else:
                self.get_logger().error("无法获取参数")
                
        except Exception as e:
            self.get_logger().error(f"检查参数时出错: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        checker = VelocityConfigChecker()
        
        # 等待系统启动
        time.sleep(2)
        
        # 检查反馈模式
        checker.check_feedback_mode()
        
        # 检查所有参数
        checker.check_all_velocity_params()
        
        # 保持运行一段时间
        time.sleep(2)
        
    except KeyboardInterrupt:
        pass
    finally:
        checker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()