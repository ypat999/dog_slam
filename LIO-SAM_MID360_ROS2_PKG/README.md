# FAST-LIO2 & LIO-SAM MID360 ROS2 Package

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

### FAST-LIO2 启动方式（推荐）
```bash
# 启动Livox MID360雷达驱动
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# 启动FAST-LIO2 SLAM系统
ros2 launch fast_lio mapping.launch.py
```

### LIO-SAM 启动方式
```bash
# 启动Livox MID360雷达驱动
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# 启动LIO-SAM SLAM系统
ros2 launch lio_sam run.launch.py
```

## 保存点云 （可选）
source install/setup.bash 
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap "{resolution: 0.05, destination: '/projects/LOAM/'}"

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

## slam_toolbox
apt install ros-humble-slam-toolbox


# gazebo
sudo apt update
sudo apt install lsb-release curl gnupg

# 添加 GZ Sim GPG key
curl -sSL https://packages.osrfoundation.org/gazebo.gpg | sudo tee /etc/apt/trusted.gpg.d/gazebo.gpg > /dev/null

# 添加 Harmonic 软件源（Ubuntu 22.04 用 jammy）
echo "deb http://packages.osrfoundation.org/gz-harmonic/ubuntu `lsb_release -cs` main" | \
  sudo tee /etc/apt/sources.list.d/gz-harmonic.list > /dev/null

sudo apt update
sudo apt install mesa-vulkan-drivers vulkan-tools -y


sudo apt install \
  ros-humble-gazebo-* \          
  <!-- sudo apt install ros-humble-ros-gz
  sudo apt install gz-harmonic -->

  ros-humble-ros2-control* \    
  ros-humble-robot-state-publisher \ 
  ros-humble-joint-state-publisher \ 
  ros-humble-xacro  

#############################################
# 若Gazebo缺少基础模型，可手动克隆官方模型库
#############################################
cd ~/.gazebo/
git clone https://github.com/osrf/gazebo_models.git models


sudo chmod 777 ~/.gazebo/models
sudo chmod 777 ~/.gazebo/models/*
## 项目说明

本项目是一个集成了FAST-LIO2和LIO-SAM两种SLAM算法的ROS2软件包，专门针对Livox MID360激光雷达优化。项目提供了完整的SLAM建图、定位和导航解决方案。

### 核心SLAM算法对比

#### FAST-LIO2 (推荐使用)
- **算法特点**：基于迭代卡尔曼滤波的紧耦合激光-惯性里程计
- **优势**：计算效率高、内存占用低、实时性强、抗震性好
- **适用场景**：实时建图、高频定位、资源受限环境

#### LIO-SAM (备选方案)
- **算法特点**：基于因子图的松耦合激光-惯性里程计
- **优势**：回环检测能力强、全局优化精度高
- **适用场景**：需要高精度全局地图、回环检测的场景

### 项目提供了多种启动脚本和配置：

#### 1. fast_lio_mapping.launch.py
这是FAST-LIO2 SLAM系统的核心启动文件，包含以下主要组件：
- **IMU预处理节点**：处理IMU数据，进行预积分
- **点云预处理节点**：对激光点云进行滤波和特征提取
- **激光-惯性里程计节点**：基于迭代卡尔曼滤波的紧耦合SLAM
- **地图构建节点**：实时构建并更新3D点云地图
- **RViz2可视化工具**：提供SLAM过程的可视化界面

#### 2. lio_sam.launch.py
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
- **nav2_params.yaml**：Nav2导航参数配置，近期更新调整为差速运动模型，主要变更包括：
  - 机器人运动模型从全向移动更改为差速移动模型
  - 优化了里程计运动模型噪声参数，提高室内环境定位精度
  - 调整了更新阈值参数，提高定位稳定性
  - 更新了激光参数设置，增加最大范围至80.0
  - 优化了地图参数，减小机器人半径和膨胀半径，适配差速运动
  - 调整了速度限制和加速度限制，适配差速运动特性
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

#### FAST-LIO2 + Nav2 集成启动（推荐）
```bash
# 进入项目根目录
cd /home/ywj/projects/git/dog_slam/FAST-LIO2_LIO-SAM_MID360_ROS2_PKG

# 运行FAST-LIO2集成启动脚本
./run_fast_lio_nav2.sh
```

#### LIO-SAM + Nav2 集成启动
```bash
# 进入项目根目录
cd /home/ywj/projects/git/dog_slam/FAST-LIO2_LIO-SAM_MID360_ROS2_PKG

# 运行LIO-SAM集成启动脚本
./run_lio_sam_nav2.sh
```

#### 分别启动各组件
1. 启动FAST-LIO2 SLAM系统：
   ```bash
   # 进入ROS2工作空间
   cd /home/ywj/projects/git/dog_slam/FAST-LIO2_LIO-SAM_MID360_ROS2_PKG/ros2
   
   # 编译项目
   colcon build --packages-select fast_lio
   
   # 设置环境变量
   source install/setup.bash
   
   # 启动FAST-LIO2 SLAM系统
   ros2 launch fast_lio mapping.launch.py
   ```

2. 启动LIO-SAM SLAM系统：
   ```bash
   # 进入ROS2工作空间
   cd /home/ywj/projects/git/dog_slam/FAST-LIO2_LIO-SAM_MID360_ROS2_PKG/ros2
   
   # 编译项目
   colcon build --packages-select lio_sam
   
   # 设置环境变量
   source install/setup.bash
   
   # 启动LIO-SAM SLAM系统
   ros2 launch lio_sam lio_sam.launch.py
   ```

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

#### FAST-LIO2参数
修改`fast_lio/config/mid360.yaml`文件可调整FAST-LIO2系统参数：
- **传感器配置**：雷达类型、IMU话题、坐标系设置
- **预处理参数**：点云过滤、盲区设置、扫描线数
- **建图参数**：噪声协方差、视野范围、外参设置
- **发布参数**：路径发布、点云输出、地图保存
- **性能优化**：迭代次数、过滤尺寸、立方体尺寸

#### LIO-SAM参数
修改`lio_sam/config/liosam_params.yaml`文件可调整SLAM系统参数：
- **传感器配置**：雷达和IMU参数
- **特征提取**：阈值和算法参数
- **回环检测**：使能开关和检测参数
- **优化设置**：滑动窗口大小和优化频率

#### Nav2参数
修改`nav2_dog_slam/config/nav2_params.yaml`文件可调整导航系统参数：
- **代价地图**：尺寸、分辨率、障碍物处理
- **路径规划**：规划器类型和参数
- **控制器**：运动控制算法和参数
- **行为服务器**：恢复行为配置

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


## 后续规划
🔵 一、SLAM 方向（建图、定位、畸变）
1. 改用 FAST-LIO2

修正LIOSAM的健壮性问题，替换 LIO-SAM，降低 CPU/内存，提高实时性、稳定性、抗震性。

2. 改用 slam_toolbox（2D 定位）

实现 2D 层面的稳定定位，结合 FAST-LIO2 作为高频 odom。或者使用cartographer作为定位。

3. 建图扭曲校正（IMU / 外参 / 时间同步）

Livox 时间源统一

外参优化工具

IMU-LiDAR 时延估计

处理长直线弯曲问题

4. 大地图 + 动态地图处理

Submap

多分辨率地图

动态物体过滤（Voxel + Temporal filter）(costmap距离优化)

节点生命周期热切换

5. 尝试 3D 重定位与导航

NDT-Matching Global Localization、Correlative Scan Matching、ICP 粗匹配 + 局部优化等全局初始化方法

AMCL + 激光全局匹配（Google 的经典方案）
使用 correlative scan matcher 粗匹配得到全局位姿
用此位姿作为 AMCL initial pose
AMCL 开始精配

使用 ScanContext / ISC 重定位

Fast-LIO + Octomap → Lite3D Nav

多楼层、多工位自动返回

🟣 二、导航（Nav2 控制、路径规划）
6. 导航直行优化（更丝滑）

MPPI

Timed Elastic Band

DWB 参数调优

控制延迟补偿

小核→大核迁移

7. 各种超时问题处理

planner_server 超时

controller_server 超时

costmap update 超时

global_costmap clear 超时
重点：解决 TF extrapolation / Future TF。

8. 行为树优化

去掉无用 action 节点

增加恢复策略（recovery tree）

增加 fallback 和运行时安全保护

多任务调度（任务队列）

9. 根据现场优化机器人速度

动态加速度限制

根据通道宽度自适应速度

根据代价地图动态调整提速/减速

自动测速标定

🟢 三、硬件/算力方向（RK3588 大小核调度）
10. 大小核资源分配（核心绑定）

建议方案：

Livox → A76 大核 2/3

FAST-LIO → A76 大核

Nav2（planner/controller/BT）→ A76

Costmap、AMCL、TF → 小核

RVIZ → 小核（不在主板上跑）

11. DDS/ROS2 通信优化

更换 CycloneDDS → Zenoh（可选）

降低点云复制次数

使用 loaned messages，降低内存 alloc

12. 内存和 swap 优化

减少内存碎片造成的卡顿

TCP buffer/UDP buffer 优化

Hugepages（可能提高 10% 性能）

13.增加外部硬件定位

UWB定位：https://item.taobao.com/item.htm?ali_refid=a3_420434_1006%3A1232160183%3AN%3AiLGziN2xQg7ZXuJqeAHqJQ%3D%3D%3A4ab99fd62fd386de2c03be677202e418&ali_trackid=1_4ab99fd62fd386de2c03be677202e418&id=620357984572&mi_id=0000YbbY1Xw9iafMAT0NIu6ipqFnzlgrJYJecfyW0ii9qws&mm_sceneid=1_0_460640197_0&priceTId=2147826a17636183853591244e1805&skuId=4385110940671&spm=a21n57.1.hoverItem.5&utparam=%7B%22aplus_abtest%22%3A%228cd4a81de99827ec9db8e14c446b5f5a%22%7D&xxc=ad_ztc

https://item.taobao.com/item.htm?abbucket=17&id=817535595979&mi_id=00005kq1AfphR1j35XPX7K1RZXiRU4URGapu3kJ2vsuOpfk&ns=1&priceTId=213e093b17636229488816365e0ec9&skuId=5517778360700&spm=a21n57.1.hoverItem.4&utparam=%7B%22aplus_abtest%22%3A%226cb8814cb1cd26812cdf681e1acf2029%22%7D&xxc=taobaoSearch

蓝牙信标RSSI
蓝牙寻向AOA
wifi
RFID
声学定位

🟡 四、系统稳定性（可靠性 + 安全）
14. 完整的 TF 延迟/漂移监控

TF future/past extrapolation

滤除不合规 TF

每秒监控 TF 延迟

15. 自动恢复策略

SLAM 崩溃自动重启

Nav2 崩溃自动恢复

Livox 数据延迟自动切换

16. 远程调试 / OTA 合规化

集成日志（syslog + ros2 log）

OTA 分区

崩溃快照 dump

🟠 五、应用层（地图管理、流程优化）
17. 地图服务化（Map server / 多地图管理）

一个仓库多个 map

多楼层 map

地图存档/回滚/更新

18. 自定义任务框架（更可靠的任务执行）

队列化调度

并发任务

安全区限制

19. 支持“边走边更新地图”

lifelong SLAM / online mapping

对轻微环境变化实时适应

无需停机建新地图

🟤 六、扩展能力（未来可选）
20. 引入 YOLO / RT-DETR 做障碍动态识别

人体检测

动态屏蔽 + costmap integration

21. 碰撞预测模型（Collision Prediction RNN/MLP）

在动作执行前提前预判潜在风险。

22. 整体架构组件化（提升可维护性）

SLAM、Nav、Sensor、App 四大模块解耦

使用 docker 化管理（可选）

提高可移植性