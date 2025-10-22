#!/bin/bash
# ROS2 幽灵节点清理脚本
# 用于清理残留的节点信息

echo "=== ROS2 幽灵节点清理脚本 ==="
echo "时间: $(date)"

# 函数：安全执行命令
safe_execute() {
    local cmd="$1"
    local description="$2"
    echo "执行: $description"
    if eval "$cmd"; then
        echo "✓ $description 完成"
    else
        echo "✗ $description 失败"
    fi
}

# 函数：检查ROS2环境
check_ros2_env() {
    if ! command -v ros2 &> /dev/null; then
        echo "错误: ROS2 未安装或环境未配置"
        exit 1
    fi
    
    if [ -z "$ROS_DISTRO" ]; then
        echo "警告: ROS_DISTRO 未设置，尝试自动检测..."
        source /opt/ros/*/setup.bash 2>/dev/null || true
    fi
}

# 函数：获取幽灵节点列表
get_ghost_nodes() {
    local nodes_file="/tmp/ros2_nodes_before_cleanup.txt"
    ros2 node list 2>/dev/null | grep -E "(algo_manager|livox_lidar|static_transform)" > "$nodes_file" 2>/dev/null
    echo "$nodes_file"
}

# 函数：获取运行中的相关进程
get_running_processes() {
    ps aux | grep -E "(algo_manager|livox_lidar|static_transform)" | grep -v grep | awk '{print $2, $11}'
}

# 主清理流程
main_cleanup() {
    echo "1. 检查ROS2环境..."
    check_ros2_env
    
    echo "2. 获取当前节点状态..."
    nodes_file=$(get_ghost_nodes)
    node_count=$(wc -l < "$nodes_file" 2>/dev/null || echo "0")
    
    if [ "$node_count" -eq 0 ]; then
        echo "未发现需要清理的幽灵节点"
        rm -f "$nodes_file"
        return 0
    fi
    
    echo "发现 $node_count 个需要检查的节点:"
    cat "$nodes_file"
    
    echo "3. 获取运行中的进程..."
    get_running_processes
    
    echo "4. 执行清理步骤..."
    
    # 停止ROS2守护进程
    safe_execute "ros2 daemon stop" "停止ROS2守护进程"
    sleep 2
    
    # 强制结束可能的残留进程
    safe_execute "killall -9 algo_manager_node 2>/dev/null || true" "强制结束algo_manager_node进程"
    safe_execute "killall -9 livox_lidar_publisher 2>/dev/null || true" "强制结束livox_lidar_publisher进程"
    safe_execute "killall -9 static_transform_publisher 2>/dev/null || true" "强制结束static_transform_publisher进程"
    sleep 1
    
    # 清理ROS2缓存
    safe_execute "rm -rf ~/.ros/log/*" "清理ROS2日志文件"
    safe_execute "rm -rf /tmp/ros2*" "清理ROS2临时文件"
    
    # 重启ROS2守护进程
    safe_execute "ros2 daemon start" "重启ROS2守护进程"
    sleep 2
    
    echo "5. 验证清理结果..."
    final_nodes=$(ros2 node list 2>/dev/null | grep -E "(algo_manager|livox_lidar|static_transform)" | wc -l)
    
    if [ "$final_nodes" -eq 0 ]; then
        echo "✓ 清理成功！所有幽灵节点已清除"
        return 0
    else
        echo "⚠ 仍有 $final_nodes 个节点存在，可能需要深度清理"
        echo "建议执行深度清理步骤..."
        return 1
    fi
}

# 深度清理
deep_cleanup() {
    echo "执行深度清理..."
    
    # 清理所有ROS2缓存
    safe_execute "sudo rm -rf /tmp/ros2*" "清理系统级ROS2临时文件"
    safe_execute "rm -rf ~/.ros/" "清理用户ROS2配置"
    
    # 重新source环境
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    fi
    
    # 如果有本地工作空间，也source
    if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
        source $HOME/ros2_ws/install/setup.bash
    fi
    
    # 重启守护进程
    safe_execute "ros2 daemon stop" "停止守护进程"
    sleep 1
    safe_execute "ros2 daemon start" "重启守护进程"
    
    echo "深度清理完成"
}

# 创建监控脚本
create_monitor_script() {
    local monitor_script="$HOME/.ros/monitor_ghost_nodes.sh"
    
    cat > "$monitor_script" << 'EOF'
#!/bin/bash
# 幽灵节点监控脚本

log_file="$HOME/.ros/ghost_nodes.log"
echo "$(date): 开始监控幽灵节点" >> "$log_file"

# 获取节点列表
nodes=$(ros2 node list 2>/dev/null | grep -E "(algo_manager|livox_lidar|static_transform)")

for node in $nodes; do
    # 检查是否有对应的运行进程
    node_name=$(echo $node | sed 's/^\///')
    if ! pgrep -f "$node_name" > /dev/null; then
        echo "$(date): 发现幽灵节点: $node" >> "$log_file"
    fi
done

# 统计幽灵节点数量
ghost_count=$(echo "$nodes" | wc -l)
if [ "$ghost_count" -gt 5 ]; then
    echo "$(date): 警告：检测到 $ghost_count 个幽灵节点" >> "$log_file"
fi
EOF
    
    chmod +x "$monitor_script"
    echo "监控脚本已创建: $monitor_script"
    echo "可以添加到crontab定期执行:"
    echo "*/5 * * * * $monitor_script"
}

# 主程序
main() {
    # 检查参数
    case "${1:-}" in
        --deep)
            main_cleanup
            if [ $? -ne 0 ]; then
                deep_cleanup
            fi
            ;;
        --monitor)
            create_monitor_script
            ;;
        --help)
            echo "用法: $0 [--deep] [--monitor] [--help]"
            echo "  --deep    执行深度清理"
            echo "  --monitor 创建监控脚本"
            echo "  --help    显示帮助信息"
            ;;
        *)
            main_cleanup
            ;;
    esac
    
    # 清理临时文件
    rm -f /tmp/ros2_nodes_before_cleanup.txt
    
    echo "=== 清理脚本执行完成 ==="
}

# 如果直接执行脚本
if [ "${BASH_SOURCE[0]}" == "${0}" ]; then
    main "$@"
fi