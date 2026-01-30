#!/bin/bash

# =============================================================================
# LIO-SAM MID360 ROS2 全新开发板环境部署脚本
# 包含完整的导航和建图服务安装
# =============================================================================

set -e

echo "=================================================="
echo "  LIO-SAM MID360 ROS2 开发板环境部署脚本"
echo "=================================================="

# 检查是否以root权限运行
if [ "$EUID" -ne 0 ]; then 
    echo "错误: 请使用sudo或以root权限运行此脚本"
    exit 1
fi

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "项目根目录: $PROJECT_ROOT"
echo "当前用户: $(whoami)"
echo ""

# =============================================================================
# 第一步：系统环境检查和配置
# =============================================================================

echo "=== 第一步：系统环境检查和配置 ==="

# 检查操作系统版本
if [ ! -f /etc/os-release ]; then
    echo "错误: 无法检测操作系统版本"
    exit 1
fi

source /etc/os-release
if [ "$VERSION_CODENAME" != "jammy" ] || [ "$VERSION_ID" != "22.04" ]; then
    echo "警告: 当前系统为 $PRETTY_NAME"
    echo "推荐使用 Ubuntu 22.04 (Jammy Jellyfish)"
    read -p "是否继续? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# 检查磁盘空间
echo "检查磁盘空间..."
AVAILABLE_SPACE=$(df / | awk 'NR==2 {print $4}')
if [ $AVAILABLE_SPACE -lt 10485760 ]; then  # 小于10GB
    echo "警告: 磁盘空间不足，建议至少10GB可用空间"
    read -p "是否继续? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# 更新软件包列表
echo "更新软件包列表..."
apt-get update

# 安装基础依赖
echo "安装基础依赖包..."
apt-get install -y \
    curl \
    wget \
    git \
    build-essential \
    cmake \
    pkg-config \
    python3-pip \
    python3-venv \
    python3-dev \
    python3-setuptools \
    software-properties-common \
    lsb-release \
    gnupg2 \
    udev \
    usbutils

# =============================================================================
# 第二步：ROS2 Humble 安装
# =============================================================================

echo ""
echo "=== 第二步：ROS2 Humble 安装 ==="

# 检查是否已安装ROS2
if [ -f "/opt/ros/humble/setup.bash" ]; then
    echo "ROS2 Humble 已安装，跳过安装步骤"
else
    echo "安装ROS2 Humble..."
    
    # 设置locale
    apt-get install -y locales
    locale-gen en_US en_US.UTF-8
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    
    # 添加ROS2仓库
    apt-get install -y curl gnupg2 lsb-release
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    
    # 使用阿里云源
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.aliyun.com/ros2/ubuntu/ $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    # 更新并安装ROS2
    apt-get update
    apt-get install -y ros-humble-desktop
    
    # 安装ROS2开发工具
    apt-get install -y \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool
    
    # 初始化rosdep
    rosdep init
    rosdep update
    
    echo "ROS2 Humble 安装完成"
fi

# =============================================================================
# 第三步：Livox-SDK2 安装
# =============================================================================

echo ""
echo "=== 第三步：Livox-SDK2 安装 ==="

LIVOX_SDK_DIR="$PROJECT_ROOT/../Livox-SDK2"

if [ -d "$LIVOX_SDK_DIR" ] && [ -f "$LIVOX_SDK_DIR/build/livox_sdk_shared.so" ]; then
    echo "Livox-SDK2 已安装，跳过安装步骤"
else
    echo "安装Livox-SDK2依赖..."
    apt-get install -y libboost-all-dev libpcl-dev
    
    # 克隆Livox-SDK2仓库
    if [ ! -d "$LIVOX_SDK_DIR" ]; then
        echo "克隆Livox-SDK2仓库..."
        cd "$(dirname "$LIVOX_SDK_DIR")"
        git clone https://github.com/Livox-SDK/Livox-SDK2.git
    fi
    
    # 编译和安装
    echo "编译Livox-SDK2..."
    cd "$LIVOX_SDK_DIR"
    mkdir -p build
    cd build
    cmake ..
    make -j$(nproc)
    make install
    
    echo "Livox-SDK2 安装完成"
fi

# =============================================================================
# 第四步：项目依赖包安装
# =============================================================================

echo ""
echo "=== 第四步：项目依赖包安装 ==="

# 安装ROS2导航相关包
echo "安装ROS2导航相关包..."
apt-get install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-rosbridge-server \
    ros-humble-octomap \
    ros-humble-octomap-msgs \
    ros-humble-octomap-server \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-slam-toolbox \
    ros-humble-rqt \
    ros-humble-ros2topic

# 安装LIO-SAM特定依赖
echo "安装LIO-SAM特定依赖..."
apt-get install -y \
    ros-humble-perception-pcl \
    ros-humble-pcl-msgs \
    ros-humble-vision-opencv \
    ros-humble-xacro \
    ros-humble-vision-msgs

# 安装GTSAM
add-apt-repository -y ppa:borglab/gtsam-release-4.1
apt-get update
apt-get install -y libgtsam-dev libgtsam-unstable-dev

# =============================================================================
# 第五步：Gazebo仿真环境安装（可选）
# =============================================================================

echo ""
echo "=== 第五步：Gazebo仿真环境安装（可选） ==="

read -p "是否安装Gazebo仿真环境? (y/N): " -n 1 -r
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo
    echo "安装Gazebo相关包..."
    
    # # 添加GZ Sim GPG key
    # curl -sSL https://packages.osrfoundation.org/gazebo.gpg | tee /etc/apt/trusted.gpg.d/gazebo.gpg > /dev/null
    
    # # 添加 Harmonic 软件源
    # echo "deb http://packages.osrfoundation.org/gz-harmonic/ubuntu $(lsb_release -cs) main" | \
    #     tee /etc/apt/sources.list.d/gz-harmonic.list > /dev/null
    
    apt-get update
    apt-get install -y mesa-vulkan-drivers vulkan-tools
    
    apt-get install -y \
        ros-humble-gazebo-* \
        ros-humble-ros2-control* \
        ros-humble-robot-state-publisher \
        ros-humble-joint-state-publisher \
        ros-humble-xacro
    
    # 下载Gazebo模型
    echo "下载Gazebo模型..."
    mkdir -p /home/$(logname)/.gazebo/
    cd /home/$(logname)/.gazebo/
    if [ ! -d "models" ]; then
        git clone https://github.com/osrf/gazebo_models.git models
        chmod 777 models
        chmod 777 models/*
    fi
    
    echo "Gazebo仿真环境安装完成"
else
    echo
    echo "跳过Gazebo安装"
fi

# =============================================================================
# 第六步：项目构建
# =============================================================================

echo ""
echo "=== 第六步：项目构建 ==="

cd "$PROJECT_ROOT"

# 检查并构建Livox驱动
if [ -f "ros2/src/livox_ros_driver2/build.sh" ]; then
    echo "构建Livox ROS2驱动..."
    cd ros2/src/livox_ros_driver2
    ./build.sh humble
    cd "$PROJECT_ROOT"
else
    echo "错误: 未找到Livox驱动构建脚本"
    exit 1
fi

# 构建ROS2工作空间
echo "构建ROS2工作空间..."
cd "$PROJECT_ROOT/ros2"

# 安装项目依赖
rosdep install --from-paths src --ignore-src -r -y

# 构建所有包
colcon build --symlink-install --parallel-workers $(nproc)

# 验证构建结果
if [ $? -eq 0 ]; then
    echo "项目构建成功"
else
    echo "错误: 项目构建失败"
    exit 1
fi

# =============================================================================
# 第七步：系统服务安装
# =============================================================================

echo ""
echo "=== 第七步：系统服务安装 ==="

# 检查现有的service文件是否存在
if [ ! -f "$SCRIPT_DIR/lio_nav2_unified.service" ]; then
    echo "错误：找不到导航服务文件 lio_nav2_unified.service"
    exit 1
fi

# 检查现有的service文件是否存在
if [ ! -f "$SCRIPT_DIR/lio_sam_nav2.service" ]; then
    echo "错误：找不到导航服务文件 lio_nav2_unified.service"
    exit 1
fi

if [ ! -f "$SCRIPT_DIR/lio_sam_buildmap.service" ]; then
    echo "错误：找不到手动建图服务文件 lio_sam_buildmap.service"
    exit 1
fi

if [ ! -f "$SCRIPT_DIR/auto_buildmap.service" ]; then
    echo "错误：找不到自动建图服务文件 auto_buildmap.service"
    exit 1
fi

# 复制现有的service文件到系统目录
cp "$SCRIPT_DIR/lio_nav2_unified.service" /etc/systemd/system/
cp "$SCRIPT_DIR/lio_sam_buildmap.service" /etc/systemd/system/
cp "$SCRIPT_DIR/lio_sam_buildmap.service" /etc/systemd/system/
cp "$SCRIPT_DIR/auto_buildmap.service" /etc/systemd/system/

# 设置服务文件权限
chmod 644 /etc/systemd/system/lio_nav2_unified.service
chmod 644 /etc/systemd/system/lio_sam_buildmap.service
chmod 644 /etc/systemd/system/lio_sam_buildmap.service
chmod 644 /etc/systemd/system/auto_buildmap.service

# 添加用户到必要组
usermod -aG dialout $(logname)
usermod -aG audio $(logname)

# 重新加载systemd并启用服务
systemctl daemon-reload
systemctl enable lio_sam_nav2.service
# systemctl enable lio_nav2_unified.service
# systemctl enable lio_sam_buildmap.service
# systemctl enable auto_buildmap.service

echo "服务安装完成:"
echo "  - 导航服务: lio_nav2_unified.service"
echo "  - 手动建图服务: lio_sam_buildmap.service"
echo "  - 自动建图服务: auto_buildmap.service"
echo ""
echo "启动命令:"
echo "  sudo systemctl start lio_nav2_unified"
echo "  sudo systemctl start lio_sam_buildmap"
echo "  sudo systemctl start auto_buildmap"
echo ""
echo "查看状态:"
echo "  sudo systemctl status lio_nav2_unified"
echo "  sudo systemctl status lio_sam_buildmap"
echo "  sudo systemctl status auto_buildmap"

# =============================================================================
# 第八步：环境配置和权限设置
# =============================================================================

echo ""
echo "=== 第八步：环境配置和权限设置 ==="

# 创建环境配置脚本
ENV_SCRIPT="/home/$(logname)/.lio_nav2_env.sh"

cat > "$ENV_SCRIPT" << 'EOF'
#!/bin/bash
# LIO-SAM MID360 ROS2 环境配置脚本

export ROS_DOMAIN_ID=27
export ROS_LOCALHOST_ONLY=0
export MANUAL_BUILD_MAP=False
export AUTO_BUILD_MAP=False

# 加载ROS2环境
source /opt/ros/humble/setup.bash

# 加载项目环境
if [ -f "$HOME/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/install/setup.bash" ]; then
    source "$HOME/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/install/setup.bash"
fi

# 设置Python路径
export PYTHONPATH="$PYTHONPATH:$HOME/.local/lib/python3.10/site-packages"

echo "LIO-SAM MID360 ROS2 环境已加载"
echo "建图模式配置:"
echo "  MANUAL_BUILD_MAP=$MANUAL_BUILD_MAP (手动建图)"
echo "  AUTO_BUILD_MAP=$AUTO_BUILD_MAP (自动探索建图)"
EOF

chmod +x "$ENV_SCRIPT"

# 添加到bashrc
BASHRC_FILE="/home/$(logname)/.bashrc"
if ! grep -q "lio_nav2_env.sh" "$BASHRC_FILE"; then
    echo "" >> "$BASHRC_FILE"
    echo "# LIO-SAM MID360 ROS2 环境配置" >> "$BASHRC_FILE"
    echo "source $ENV_SCRIPT" >> "$BASHRC_FILE"
fi

# 设置udev规则（如果使用真实Livox设备）
UDEV_RULE_FILE="/etc/udev/rules.d/99-livox.rules"
if [ ! -f "$UDEV_RULE_FILE" ]; then
    cat > "$UDEV_RULE_FILE" << 'EOF'
# Livox MID360 设备规则
SUBSYSTEM=="usb", ATTR{idVendor}=="2d95", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="0483", MODE="0666"
EOF
    udevadm control --reload-rules
    udevadm trigger
fi

# =============================================================================
# 第九步：验证安装
# =============================================================================

echo ""
echo "=== 第九步：验证安装 ==="

# 切换到普通用户执行验证
sudo -u $(logname) bash << 'EOF'
cd "$PROJECT_ROOT/ros2"

# 验证ROS2环境
if ! source /opt/ros/humble/setup.bash; then
    echo "错误: ROS2环境加载失败"
    exit 1
fi

# 验证项目环境
if ! source install/setup.bash; then
    echo "错误: 项目环境加载失败"
    exit 1
fi

# 验证关键包
if ! ros2 pkg list | grep -q "fast_lio"; then
    echo "错误: fast_lio包未找到"
    exit 1
fi

if ! ros2 pkg list | grep -q "lio_sam"; then
    echo "错误: lio_sam包未找到"
    exit 1
fi

if ! ros2 pkg list | grep -q "nav2_dog_slam"; then
    echo "错误: nav2_dog_slam包未找到"
    exit 1
fi

echo "安装验证通过"
EOF

# =============================================================================
# 完成信息
# =============================================================================

echo ""
echo "=================================================="
echo "  部署完成！"
echo "=================================================="
echo ""
echo "重要信息："
echo "1. 导航系统服务已安装并启用：lio_nav2_unified.service"
echo "2. 建图服务已安装（默认禁用）："
echo "   - 手动建图服务: lio_buildmap.service"
echo "   - 自动探索建图服务: lio_auto_buildmap.service"
echo "3. 环境配置已添加到 ~/.bashrc"
echo "4. 重启后导航服务将自动启动"
echo ""
echo "常用命令："
echo "导航服务："
echo "- 启动导航: sudo systemctl start lio_nav2_unified"
echo "- 停止导航: sudo systemctl stop lio_nav2_unified"
echo "- 查看状态: sudo systemctl status lio_nav2_unified"
echo "- 查看日志: sudo journalctl -u lio_nav2_unified -f"
echo ""
echo "建图服务："
echo "- 启动手动建图: sudo systemctl start lio_buildmap"
echo "- 启动自动探索建图: sudo systemctl start lio_auto_buildmap"
echo "- 停止建图: sudo systemctl stop lio_buildmap lio_auto_buildmap"
echo "- 查看日志: sudo journalctl -u lio_buildmap -f"
echo ""
echo "手动启动方式："
echo "1. 打开新终端"
echo "2. 执行: source ~/.lio_nav2_env.sh"
echo "3. 执行以下命令之一："
echo "   - 导航模式: ros2 launch nav2_dog_slam lio_nav2_unified.launch.py"
echo "   - 手动建图: MANUAL_BUILD_MAP=True ros2 launch nav2_dog_slam lio_nav2_unified.launch.py"
echo "   - 自动建图: AUTO_BUILD_MAP=True ros2 launch nav2_dog_slam lio_nav2_unified.launch.py"
echo ""
echo "下一步操作："
echo "1. 重启系统以应用所有更改"
echo "2. 连接Livox MID360设备（如使用真实设备）"
echo "3. 启动导航服务进行测试"
echo "4. 需要建图时启用相应建图服务"
echo ""

read -p "是否立即重启系统? (y/N): " -n 1 -r
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo
    echo "系统将在5秒后重启..."
    sleep 5
    reboot
fi

echo ""
echo "部署脚本执行完成！"