# LIO-SAM_MID360_ROS2_PKG

推荐使用阿里源：
deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.aliyun.com/ros2/ubuntu/ jammy main
deb [arch=amd64] https://mirrors.aliyun.com/ubuntu/ jammy main

使用前请添加gpg key：
sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg


## 依赖项

- Ubuntu 22.04
- ROS2 Humble
- [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2)

### Livox-SDK2 安装步骤

在编译本项目之前，需要先安装Livox-SDK2：

1. 安装依赖项：
   ```bash
   sudo apt-get update
   sudo apt-get install -y git cmake g++ libboost-all-dev libpcl-dev
   ```

2. 克隆Livox-SDK2仓库：
   ```bash
   git clone https://github.com/Livox-SDK/Livox-SDK2.git
   cd Livox-SDK2
   ```

3. 编译和安装：
   ```bash
   mkdir build
   cd build
   cmake ..
   make
   sudo make install
   ```

我们需要 Livox MID360 硬件。
```bash
## LIO-SAM (ros2)
sudo apt install -y ros-humble-perception-pcl \
  	   ros-humble-pcl-msgs \
  	   ros-humble-vision-opencv \
  	   ros-humble-xacro \
	   ros-humble-vision-msgs

## LIO-SAM (gtsam)
sudo add-apt-repository ppa:borglab/gtsam-release-4.1
sudo apt install -y libgtsam-dev libgtsam-unstable-dev
```

## 构建 (首次)
删除 build/ install/ log/ 目录
运行 `build_ros2.sh` 进行首次构建。它会正确构建 Livox 包。

## 运行
```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py

ros2 launch lio_sam run.launch.py
```

## 保存点云 （不使用）
source install/setup.bash 
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap "{resolution: 0.2, destination: '/projects/LOAM/'}"
sudo apt update

## 转换为占用网格 （PNG 地图输出）
sudo apt install ros-humble-octomap ros-humble-octomap-msgs
sudo apt install ros-humble-octomap-server
#### 当 lio-sam 运行且地图构建完成后，保存地图
ros2 run nav2_map_server map_saver_cli -t /projected_map -f /home/ywj/projects/map_grid/map --fmt png

## nav2
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-rosbridge-server 
sudo apt-get update && sudo apt-get install -y ros-humble-dwb-critics ros-humble-nav2-dwb-controller ros-humble-nav2-controller ros-humble-nav2-amcl ros-humble-nav2-planner ros-humble-nav2-bt-navigator ros-humble-nav2-lifecycle-manager ros-humble-nav2-map-server ros-humble-nav2-waypoint-follower ros-humble-rosbridge-server 

## pointcloud_to_laserscan
sudo apt install ros-humble-pointcloud-to-laserscan 

## ros2topic
sudo apt install ros-humble-ros2topic=0.18.14-1jammy.20251008.030

## rqt
sudo apt install ros-humble-rqt  


## 项目说明

### 项目提供了多种启动脚本和配置：

#### 1. lio_sam.launch.py
这是LIO-SAM SLAM系统的核心启动文件，包含以下主要组件：
- **IMU预积分节点** (`imuPreintegration`)：处理IMU数据，提供IMU预积分 odometry
- **点云投影节点** (`imageProjection`)：将3D点云投影到range image和bev map，提取特征点
- **激光里程计节点** (`featureOdomatry`)：使用提取的特征点进行激光里程计计算
- **地图优化节点** (`mapOptmization`)：执行滑动窗口优化，进行回环检测和全局优化
- **静态坐标变换**：定义传感器之间的坐标关系
- **OctoMap服务器**：用于生成3D占据网格地图
- **RViz2可视化工具**：提供SLAM过程的可视化界面

#### 2. nav2.launch.py
这是Nav2导航系统的启动文件，包含以下主要组件：
- **地图服务器** (`map_server`)：加载并发布静态地图
- **AMCL定位节点** (`amcl`)：使用自适应蒙特卡洛定位算法进行机器人定位
- **Nav2核心节点组**：包括控制器、规划器、行为服务器等
- **静态坐标变换发布器**：发布map→odom和odom→base_link坐标变换
- **Rosbridge WebSocket节点**：提供Web端与ROS2通信的桥梁
- **初始位姿发布器**：发布机器人的初始位置
- **点云到激光扫描转换器**：将点云数据转换为激光扫描数据供导航使用

#### 3. lio_sam_nav2.launch.py
这是集成LIO-SAM SLAM和Nav2导航的启动文件，通过包含其他launch文件实现：
- **包含lio_sam.launch.py**：启动SLAM系统
- **延时启动nav2.launch.py**：在SLAM启动后10秒启动导航系统
- **服务调用**：延时5秒后调用/reinitialize_global_localization服务
- **Web控制界面启动**：启动Nav2的Web控制界面

#### 4. pointcloud_to_laserscan.launch.py
这是点云到激光扫描的转换启动文件，包含以下主要组件：
- **PointCloud to LaserScan节点**：将3D点云数据转换为2D激光扫描数据
- **参数配置**：设置高度过滤范围、角度范围、距离范围等参数
- **坐标变换设置**：确保数据在正确的坐标系中
- **话题重映射**：将输入点云和输出激光扫描映射到正确的主题

### Web控制界面
项目提供了一个基于Web的导航控制界面，可以通过浏览器访问并控制机器人：
- **地图显示**：实时显示机器人构建的地图
- **机器人位置标记**：显示机器人当前位置和方向
- **目标点设置**：通过点击地图设置导航目标点
- **ROS桥接**：通过rosbridge_websocket实现Web端与ROS2的通信

### 配置文件说明
项目包含多个配置文件，用于调整系统参数：
- **liosam_params.yaml**：LIO-SAM算法参数配置，包括传感器设置、特征提取阈值、回环检测等
- **nav2_params.yaml**：Nav2导航参数配置，包括代价地图、路径规划器、控制器等
- **initialpose.yaml**：初始位姿配置文件
- **robot.urdf.xacro**：机器人模型描述文件
- **rviz2.rviz**：RViz可视化配置文件


## 使用说明

### 环境准备
1. 确保已安装所有依赖项（参考依赖项部分）
2. 确保已正确构建项目（参考构建部分）
3. 确保Livox MID360雷达已连接并可被系统识别
4. 如果使用MobaXterm进行远程访问，需要先获取$DISPLAY变量值

### 启动系统
#### 方法一：使用集成启动脚本（推荐）
在MobaXterm中使用前，需要先获取$DISPLAY变量值：
```bash
# 在MobaXterm中获取DISPLAY变量值
echo $DISPLAY
```
然后修改`run_lio_sam_nav2.sh`脚本中的`export DISPLAY=localhost:10.0`行，将`localhost:10.0`替换为实际的DISPLAY值。

运行集成启动脚本：
```bash
# 进入项目根目录
cd /home/ywj/projects/git/dog_slam/LIO-SAM_MID360_ROS2_PKG

# 运行集成启动脚本
./run_lio_sam_nav2.sh
```
此脚本会自动设置环境变量、编译项目并启动LIO-SAM SLAM和Nav2导航系统。

#### 方法二：分别启动各组件
1. 启动SLAM系统：
   ```bash
   # 进入ROS2工作空间
   cd /home/ywj/projects/git/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2
   
   # 编译项目
   colcon build --packages-select lio_sam
   
   # 设置环境变量
   source install/setup.bash
   
   # 启动LIO-SAM SLAM系统
   ros2 launch lio_sam lio_sam.launch.py
   ```

2. 启动导航系统：
   ```bash
   # 在新的终端中执行以下命令
   
   # 进入ROS2工作空间
   cd /home/ywj/projects/git/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2
   
   # 设置环境变量
   source install/setup.bash
   
   # 启动Nav2导航系统
   ros2 launch lio_sam nav2.launch.py
   ```

### Web控制界面使用
系统启动后，可通过Web浏览器访问控制界面：
1. 打开浏览器，访问地址：`http://localhost:8083/nav2_web_control.html`
2. 界面功能说明：
   - **地图显示**：实时显示构建的地图
   - **机器人位置**：红色圆点表示机器人当前位置
   - **目标点设置**：在地图上点击设置导航目标点
   - **位置源切换**：可在AMCL定位和LIO-SAM里程计之间切换
   - **地图颜色配置**：可自定义地图显示颜色
   - **缩放与拖拽**：支持鼠标滚轮缩放和拖拽地图

### 地图保存与使用
#### 保存点云地图
当SLAM系统运行并完成建图后，可保存点云地图：
```bash
# 保存OctoMap点云地图
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap "{resolution: 0.05, destination: '/projects/LOAM/'}"
```

#### 转换为占用网格地图
将点云地图转换为Nav2可使用的PNG格式占用网格地图：
```bash
# 转换并保存地图
ros2 run nav2_map_server map_saver_cli -t /projected_map -f /home/ywj/projects/map_grid/map --fmt png
```

### 导航控制
#### 方法一：Web界面控制（推荐）
通过Web控制界面点击地图设置目标点，系统将自动规划路径并导航。

#### 方法二：命令行控制
使用提供的Python脚本发送导航目标：
```bash
# 发送目标点(x=1.0, y=2.0)
ros2 run lio_sam send_goal.py 1.0 2.0
```

#### 方法三：ROS2命令行工具
使用ROS2内置工具发送导航目标：
```bash
# 发送导航目标
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
```

### 系统参数调整
#### LIO-SAM参数
修改`config/liosam_params.yaml`文件可调整SLAM系统参数：
- 传感器配置：雷达和IMU参数
- 特征提取：阈值和算法参数
- 回环检测：使能开关和检测参数
- 优化设置：滑动窗口大小和优化频率

#### Nav2参数
修改`config/nav2_params.yaml`文件可调整导航系统参数：
- 代价地图：尺寸、分辨率、障碍物处理
- 路径规划：规划器类型和参数
- 控制器：运动控制算法和参数
- 行为服务器：恢复行为配置

### 故障排除
1. **无法连接雷达**：
   - 检查雷达是否正确连接
   - 检查雷达驱动是否正常安装
   - 查看雷达驱动输出日志

2. **SLAM系统无法启动**：
   - 检查依赖项是否完整安装
   - 检查构建是否成功完成
   - 查看启动日志排查错误

3. **Web界面无法访问**：
   - 检查rosbridge_websocket是否正常启动
   - 检查网络连接和端口是否开放
   - 查看浏览器控制台错误信息

4. **导航精度问题**：
   - 检查AMCL参数配置
   - 调整代价地图参数
   - 优化传感器数据质量