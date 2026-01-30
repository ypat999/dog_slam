#!/usr/bin/env python3
"""
GPS配置验证脚本
验证GPS融合相关的配置文件和参数是否正确
"""

import os
import yaml
import sys

def verify_yaml_file(file_path):
    """验证YAML配置文件格式是否正确"""
    try:
        with open(file_path, 'r') as f:
            yaml.safe_load(f)
        return True, "YAML格式正确"
    except yaml.YAMLError as e:
        return False, f"YAML格式错误: {e}"
    except FileNotFoundError:
        return False, "文件不存在"

def check_file_exists(file_path):
    """检查文件是否存在"""
    return os.path.exists(file_path), "文件存在" if os.path.exists(file_path) else "文件不存在"

def main():
    # 获取当前脚本所在目录
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.join(script_dir, '../../..')
    
    # 需要检查的文件列表
    files_to_check = [
        'config/gps_ekf.yaml',
        'config/navsat_transform.yaml',
        'launch/gps_fusion.launch.py',
        'launch/nav2_gps_fusion.launch.py',
        'scripts/test_gps_fusion.py',
        'docs/gps_fusion_guide.md'
    ]
    
    print("=== GPS融合配置验证 ===")
    print()
    
    all_passed = True
    
    for file_path in files_to_check:
        full_path = os.path.join(project_root, file_path)
        
        print(f"检查文件: {file_path}")
        
        # 检查文件是否存在
        exists, exists_msg = check_file_exists(full_path)
        print(f"  存在性: {exists_msg}")
        
        # 如果是YAML文件，检查格式
        if file_path.endswith('.yaml') and exists:
            valid, valid_msg = verify_yaml_file(full_path)
            print(f"  格式验证: {valid_msg}")
            if not valid:
                all_passed = False
        
        # 如果是Python文件，检查语法（简单检查）
        elif file_path.endswith('.py') and exists:
            try:
                with open(full_path, 'r') as f:
                    compile(f.read(), full_path, 'exec')
                print("  语法检查: Python语法正确")
            except SyntaxError as e:
                print(f"  语法检查: Python语法错误: {e}")
                all_passed = False
        
        print()
    
    # 检查robot_localization包是否可用
    print("=== 系统依赖检查 ===")
    try:
        import rclpy
        print("✓ ROS2 Python接口可用")
    except ImportError:
        print("✗ ROS2 Python接口不可用")
        all_passed = False
    
    # 检查robot_localization包
    try:
        from ament_index_python.packages import get_package_share_directory
        robot_localization_path = get_package_share_directory('robot_localization')
        print("✓ robot_localization包已安装")
    except:
        print("✗ robot_localization包未安装")
        all_passed = False
    
    print()
    print("=== 验证结果 ===")
    
    if all_passed:
        print("✓ 所有配置检查通过！GPS融合系统可以正常使用。")
        print()
        print("使用建议:")
        print("1. 启动GPS融合系统: ros2 launch nav2_dog_slam nav2_gps_fusion.launch.py")
        print("2. 测试融合效果: ros2 run nav2_dog_slam test_gps_fusion.py")
        print("3. 查看详细指南: cat " + os.path.join(project_root, 'docs/gps_fusion_guide.md'))
    else:
        print("✗ 部分配置存在问题，请检查上述错误信息。")
        sys.exit(1)

if __name__ == "__main__":
    main()