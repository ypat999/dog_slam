#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robots_dog_msgs.srv import LoadMap
import sys

class LoadMapTestClient(Node):
    def __init__(self):
        super().__init__('load_map_test_client')
        self.client = self.create_client(LoadMap, '/load_map_service')
        
    def send_request(self, pcd_path):
        request = LoadMap.Request()
        request.pcd_path = pcd_path
        
        self.get_logger().info(f'Sending request: {pcd_path}')
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.done():
            try:
                response = future.result()
                self.get_logger().info(f'Response: success={response.success}, message={response.message}')
                return response
            except Exception as e:
                self.get_logger().error(f'Service call failed: {e}')
                return None
        else:
            self.get_logger().error('Service call timed out')
            return None

def main():
    rclpy.init()
    
    if len(sys.argv) != 2:
        print("Usage: python3 test_load_map_service.py <pcd_file_path>")
        print("Example: python3 test_load_map_service.py /home/yuanxq/bag/localization_global_test/B2_data/map_2025_08_14_18_27_01.pcd")
        print("")
        print("Expected file structure:")
        print("  /home/yuanxq/bag/localization_global_test/B2_data/")
        print("  ├── map_2025_08_14_18_27_01.pcd")
        print("  └── map_2025_08_14_18_27_01/")
        print("      └── *.db3  (任意命名的.db3文件)")
        print("")
        print("Note: The system will automatically find any .db3 file in the derived directory.")
        print("      No need to match the exact filename with the PCD file.")
        return
    
    pcd_path = sys.argv[1]
    
    client = LoadMapTestClient()
    
    # Wait for service to be available
    while not client.client.wait_for_service(timeout_sec=1.0):
        client.get_logger().info('Service not available, waiting...')
    
    # Send request
    response = client.send_request(pcd_path)
    
    if response:
        if response.success:
            client.get_logger().info('✅ LoadMap service call successful!')
        else:
            client.get_logger().error('❌ LoadMap service call failed!')
    
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 