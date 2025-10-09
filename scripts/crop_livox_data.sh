#!/bin/bash
# 裁切LiDAR数据脚本 - 默认裁切前两分钟数据

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CROP_SCRIPT="$SCRIPT_DIR/crop_bag_data.py"

# 默认参数
INPUT_PATH="/public/dataset/robot/livox_record/"
CROP_TIME=120  # 默认裁切120秒（2分钟）

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        -t|--time)
            CROP_TIME="$2"
            shift 2
            ;;
        -o|--output)
            OUTPUT_PATH="$2"
            shift 2
            ;;
        -h|--help)
            echo "用法: $0 [选项]"
            echo "选项:"
            echo "  -t, --time <秒数>    裁切时间（秒），默认120秒"
            echo "  -o, --output <路径>  输出路径"
            echo "  -h, --help          显示帮助信息"
            exit 0
            ;;
        *)
            echo "未知参数: $1"
            echo "使用 -h 或 --help 查看帮助"
            exit 1
            ;;
    esac
done

# 自动生成输出路径
if [ -z "$OUTPUT_PATH" ]; then
    OUTPUT_PATH="/tmp/livox_record_cropped_${CROP_TIME}s/"
fi

echo "开始裁切LiDAR数据..."
echo "输入路径: $INPUT_PATH"
echo "裁切时间: ${CROP_TIME}秒"
echo "输出路径: $OUTPUT_PATH"

# 检查输入路径是否存在
if [ ! -d "$INPUT_PATH" ] && [ ! -f "$INPUT_PATH" ]; then
    echo "错误：输入路径不存在: $INPUT_PATH"
    exit 1
fi

# 检查裁切脚本是否存在
if [ ! -f "$CROP_SCRIPT" ]; then
    echo "错误：找不到裁切脚本: $CROP_SCRIPT"
    exit 1
fi

# 执行裁切
echo "执行裁切..."
python3 "$CROP_SCRIPT" "$INPUT_PATH" -t "$CROP_TIME" -o "$OUTPUT_PATH"

if [ $? -eq 0 ]; then
    echo "裁切完成！"
    echo "输出文件位于: $OUTPUT_PATH"
else
    echo "裁切失败！"
    exit 1
fi