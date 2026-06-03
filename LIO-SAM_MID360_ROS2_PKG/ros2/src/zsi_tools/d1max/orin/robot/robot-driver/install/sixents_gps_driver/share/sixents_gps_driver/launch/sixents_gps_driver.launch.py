import os
import configparser
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 从环境变量读取配置文件路径，如果没有设置则使用默认路径
    config_path = os.environ.get('RTK_CONFIG_PATH')
    
    if not config_path:
        # 如果环境变量未设置，使用默认路径
        config_path = "/ota/sixents_config.ini"
        print(f"Environment variable RTK_CONFIG_PATH not set, using default path: {config_path}")
    else:
        print(f"Using config path from environment variable: {config_path}")
    
    # 默认参数
    params = {
        # debug info warn error fatal unset
        "log_level": "info",
        # 基础参数
        "rtk_port": "/dev/ttyTHS3",
        "baudrate": 460800,
        "frame_id": "gps_link",
        "topic_name": "/rtk_pvh",
        # 六分SDK参数
        "enable_sdk": False,
        "sdk_ak": "",
        "sdk_as": "",
        "sdk_auth_host": "openapi.sixents.com",
        "sdk_vrs_host": "vrs.sixents.com",
        # 是否自动获取设备信息
        "enable_auto_get_dev_info": False,
        "sdk_dev_id": "",
        "sdk_dev_type": "",
        # 是否使用CRT校验
        "enable_crt": False,
        "sdk_crt_path": "/crt/root.crt",
        # 上传GGA的频率
        "gga_interval": 1.0,
        # 是否开启SDK LOG
        "enable_sdk_log": False,
    }
    
    # 检查配置文件是否存在
    if os.path.exists(config_path):
        try:
            config = configparser.ConfigParser()
            config.read(config_path)
            
            if config.has_section('sixents_sdk'):
                # 读取SDK参数
                sdk_ak = config.get('sixents_sdk', 'sdk_ak', fallback='')
                sdk_as = config.get('sixents_sdk', 'sdk_as', fallback='')
                sdk_dev_id = config.get('sixents_sdk', 'sdk_dev_id', fallback='')
                sdk_dev_type = config.get('sixents_sdk', 'sdk_dev_type', fallback='')
                
                # 检查四个必需参数是否都存在
                if sdk_ak and sdk_as and sdk_dev_id and sdk_dev_type:
                    # 设置enable_sdk为True
                    params["enable_sdk"] = True
                    # 更新参数
                    params["sdk_ak"] = sdk_ak
                    params["sdk_as"] = sdk_as
                    params["sdk_dev_id"] = sdk_dev_id
                    params["sdk_dev_type"] = sdk_dev_type
                        
        except Exception as e:
            print(f"Error reading config file {config_path}: {e}, using defalut param")
    
    # 创建参数列表
    parameters_list = []
    for key, value in params.items():
        parameters_list.append({key: value})
    
    return LaunchDescription(
        [
            Node(
                package="sixents_gps_driver",
                executable="sixents_gps_driver",
                name="sixents_gps_driver",
                output="screen",
                emulate_tty=True,
                parameters=parameters_list,
            )
        ]
    )