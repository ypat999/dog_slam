#!/bin/bash

# ROS2节点彻底清理脚本
# 使用方法: ./kill_node.sh <node_name>
# 例如: ./kill_node.sh arc_state_machine

set -e

# 检查参数
if [ $# -ne 1 ]; then
    echo "Usage: $0 <node_name>"
    echo "Example: $0 arc_state_machine"
    exit 1
fi

NODE_NAME="$1"
SCRIPT_NAME="kill_node.sh"

echo "🔍 正在查找与 '$NODE_NAME' 相关的进程..."

# 获取所有相关进程，但排除脚本自身
get_target_pids() {
    ps aux | grep "$NODE_NAME" | grep -v grep | grep -v "$SCRIPT_NAME" | awk '{print $2}' || true
}

# 第一步：查找进程
PIDS=$(get_target_pids)

if [ -z "$PIDS" ]; then
    echo "✅ 未找到与 '$NODE_NAME' 相关的进程"

    # 检查ROS2节点状态
    if command -v ros2 > /dev/null 2>&1; then
        echo "🔍 验证ROS2节点状态..."
        if [ -f "/home/zy/JSZR/jszr_workspace/install/setup.bash" ]; then
            source "/home/zy/JSZR/jszr_workspace/install/setup.bash"
        fi
        NODE_COUNT=$(ros2 node list 2>/dev/null | grep "$NODE_NAME" | wc -l 2>/dev/null || echo "0")
        if [ "$NODE_COUNT" -eq 0 ]; then
            echo "✅ ROS2节点已完全清理"
        else
            echo "⚠️  发现 $NODE_COUNT 个ROS2节点仍在运行"
        fi
    fi
    echo "🎯 节点清理完成！"
    exit 0
fi

echo "📋 发现以下进程："
echo "$PIDS" | while read pid; do
    if [ ! -z "$pid" ]; then
        ps -p "$pid" -o pid,ppid,cmd --no-headers 2>/dev/null || true
    fi
done
echo

# 第二步：优雅终止进程 (SIGTERM)
echo "🔄 尝试优雅终止进程 (SIGTERM)..."
echo "$PIDS" | xargs -r kill -TERM 2>/dev/null || true
sleep 2

# 第三步：检查残留进程并强制终止 (SIGKILL)
REMAINING_PIDS=$(get_target_pids)
if [ ! -z "$REMAINING_PIDS" ]; then
    echo "⚠️  仍有进程残留，执行强制终止 (SIGKILL)..."
    echo "$REMAINING_PIDS" | xargs -r kill -9 2>/dev/null || true
    sleep 1
fi

# 第四步：清理launch相关进程
LAUNCH_PIDS=$(ps aux | grep "ros2.*launch.*$NODE_NAME" | grep -v grep | grep -v "$SCRIPT_NAME" | awk '{print $2}' || true)
if [ ! -z "$LAUNCH_PIDS" ]; then
    echo "🚀 清理launch相关进程..."
    echo "$LAUNCH_PIDS" | xargs -r kill -9 2>/dev/null || true
fi

# 第五步：最终验证
FINAL_CHECK=$(get_target_pids)
if [ -z "$FINAL_CHECK" ]; then
    echo "✅ 所有与 '$NODE_NAME' 相关的进程已彻底清理完成"

    # 检查ROS2节点状态
    if command -v ros2 > /dev/null 2>&1; then
        echo "🔍 验证ROS2节点状态..."
        if [ -f "/home/zy/JSZR/jszr_workspace/install/setup.bash" ]; then
            source "/home/zy/JSZR/jszr_workspace/install/setup.bash"
        fi
        NODE_COUNT=$(ros2 node list 2>/dev/null | grep "$NODE_NAME" | wc -l 2>/dev/null || echo "0")
        if [ "$NODE_COUNT" -eq 0 ]; then
            echo "✅ ROS2节点已完全清理"
        else
            echo "⚠️  发现 $NODE_COUNT 个ROS2节点仍在运行"
        fi
    fi
else
    echo "❌ 仍有进程残留："
    echo "$FINAL_CHECK" | while read pid; do
        if [ ! -z "$pid" ]; then
            ps -p "$pid" -o pid,ppid,cmd --no-headers 2>/dev/null || true
        fi
    done
    exit 1
fi

echo "🎯 节点清理完成！"