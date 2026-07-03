#!/bin/bash

# 设备配置使用示例脚本
# 演示如何在实际脚本中调用 parse_device_json.sh

set -e

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PARSE_SCRIPT="$SCRIPT_DIR/parse_device_json.sh"

# ============================================
# 方法 1: 逐个读取字段
# ============================================
echo "=== 方法 1: 逐个读取字段 ==="

BOARD=$(${PARSE_SCRIPT} get board)
DEVICE_TYPE=$(${PARSE_SCRIPT} get type)
SN=$(${PARSE_SCRIPT} get sn)
HW_VER=$(${PARSE_SCRIPT} get hw_ver)
CUSTOM=$(${PARSE_SCRIPT} get custom)
TAG=$(${PARSE_SCRIPT} get tag)

echo "主板: $BOARD"
echo "设备类型: $DEVICE_TYPE"
echo "序列号: $SN"
echo "硬件版本: $HW_VER"
echo "自定义: $CUSTOM"
echo "标签: $TAG"
echo ""

# ============================================
# 方法 2: 一次导入所有字段为环境变量
# ============================================
echo "=== 方法 2: 导入所有环境变量 ==="

source <(${PARSE_SCRIPT} export)

echo "DEVICE_BOARD=$DEVICE_BOARD"
echo "DEVICE_TYPE=$DEVICE_TYPE"
echo "DEVICE_SN=$DEVICE_SN"
echo "DEVICE_HW_VER=$DEVICE_HW_VER"
echo "DEVICE_CUSTOM=$DEVICE_CUSTOM"
echo "DEVICE_TAG=$DEVICE_TAG"
echo ""

# ============================================
# 方法 3: 使用设备信息进行条件判断
# ============================================
echo "=== 方法 3: 条件判断 ==="

if [[ "$DEVICE_BOARD" == "RK3588S" ]]; then
    echo "✓ 检测到 RK3588S 主板"
else
    echo "✗ 未检测到 RK3588S 主板"
fi

case "$DEVICE_TYPE" in
    "ZSM-1F")
        echo "✓ 设备类型识别: ZSM-1F"
        ;;
    *)
        echo "✗ 未知的设备类型: $DEVICE_TYPE"
        ;;
esac

echo ""

# ============================================
# 方法 4: 实际应用示例 - 生成配置信息
# ============================================
echo "=== 方法 4: 生成配置信息 ==="

cat > /tmp/device_info.conf <<EOF
# 自动生成的设备配置信息
# 生成时间: $(date '+%Y-%m-%d %H:%M:%S')

DEVICE_BOARD="$DEVICE_BOARD"
DEVICE_TYPE="$DEVICE_TYPE"
DEVICE_SN="$DEVICE_SN"
DEVICE_HW_VER="$DEVICE_HW_VER"
DEVICE_CUSTOM="$DEVICE_CUSTOM"
DEVICE_TAG="$DEVICE_TAG"
EOF

echo "✓ 配置文件已生成: /tmp/device_info.conf"
cat /tmp/device_info.conf
echo ""

# ============================================
# 方法 5: 打印所有设备信息
# ============================================
echo "=== 方法 5: 打印所有设备信息 ==="
${PARSE_SCRIPT} print
echo ""

# ============================================
# 方法 6: 创建格式化的设备标签
# ============================================
echo "=== 方法 6: 创建设备标签 ==="

# 清理 null 值
CUSTOM_CLEAN="${DEVICE_CUSTOM//null/}"
TAG_CLEAN="${DEVICE_TAG//null/}"

DEVICE_LABEL="${DEVICE_TYPE}_${DEVICE_BOARD}_${DEVICE_SN}"
DEVICE_LABEL_FULL="${DEVICE_TYPE}_${DEVICE_BOARD}_HW${DEVICE_HW_VER}_${DEVICE_SN}"

echo "简化标签: $DEVICE_LABEL"
echo "完整标签: $DEVICE_LABEL_FULL"
echo ""

# ============================================
# 方法 7: 日志记录示例
# ============================================
echo "=== 方法 7: 日志记录示例 ==="

LOG_MESSAGE="[$(date '+%Y-%m-%d %H:%M:%S')] 设备信息 - 类型: $DEVICE_TYPE, 序列号: $DEVICE_SN, 硬件版本: $DEVICE_HW_VER"
echo "$LOG_MESSAGE"
echo ""

echo "✓ 所有演示完成！"
