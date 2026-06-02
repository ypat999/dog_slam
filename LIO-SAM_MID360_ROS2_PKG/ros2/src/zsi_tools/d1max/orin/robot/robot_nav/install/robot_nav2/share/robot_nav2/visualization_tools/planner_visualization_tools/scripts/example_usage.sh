#!/bin/bash
# Navigo 可视化工具使用示例

echo "Navigo 离线分析可视化工具使用示例"
echo "======================================"

# 设置变量
DATA_DIR="/tmp/navigo_debug_data"
TOOLS_DIR="/tmp/navigo_debug_data/visualization_tools"
OUTPUT_DIR="./analysis_results"

echo "数据目录: $DATA_DIR"
echo "工具目录: $TOOLS_DIR"
echo "输出目录: $OUTPUT_DIR"

# 检查Python环境
if ! command -v python3 &> /dev/null; then
    echo "❌ 错误：未找到Python3，请先安装Python3"
    exit 1
fi

# 检查依赖
echo ""
echo "📦 检查Python依赖..."
python3 -c "import matplotlib, numpy, scipy" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "⚠️  缺少Python依赖，正在安装..."
    pip3 install matplotlib numpy scipy
fi

# 创建输出目录
mkdir -p "$OUTPUT_DIR"

echo ""
echo "🔍 搜索数据文件..."

# 查找JSON文件
FRONTEND_FILES=$(find "$DATA_DIR" -name "*frontend*.json" 2>/dev/null | head -3)
CORRIDOR_FILES=$(find "$DATA_DIR" -name "*corridor*.json" 2>/dev/null | head -3)
BACKEND_FILES=$(find "$DATA_DIR" -name "*backend*.json" 2>/dev/null | head -3)

echo "找到文件："
echo "  前端搜索: $(echo "$FRONTEND_FILES" | wc -l) 个"
echo "  走廊生成: $(echo "$CORRIDOR_FILES" | wc -l) 个"
echo "  后端优化: $(echo "$BACKEND_FILES" | wc -l) 个"

# 分析前端搜索文件
if [ -n "$FRONTEND_FILES" ]; then
    echo ""
    echo "📈 分析前端搜索数据..."
    for file in $FRONTEND_FILES; do
        if [ -f "$file" ]; then
            echo "  处理: $(basename "$file")"
            python3 "$TOOLS_DIR/visualize_frontend_search.py" "$file" \
                --save "$OUTPUT_DIR/$(basename "${file%.json}")_analysis.png" \
                --no-plot 2>/dev/null
            if [ $? -eq 0 ]; then
                echo "    ✅ 分析完成"
            else
                echo "    ❌ 分析失败"
            fi
        fi
    done
else
    echo "⚠️  未找到前端搜索数据文件"
fi

# 分析走廊生成文件
if [ -n "$CORRIDOR_FILES" ]; then
    echo ""
    echo "🛣️  分析走廊生成数据..."
    for file in $CORRIDOR_FILES; do
        if [ -f "$file" ]; then
            echo "  处理: $(basename "$file")"
            python3 "$TOOLS_DIR/visualize_corridor.py" "$file" \
                --save "$OUTPUT_DIR/$(basename "${file%.json}")_analysis.png" \
                --no-plot 2>/dev/null
            if [ $? -eq 0 ]; then
                echo "    ✅ 分析完成"
            else
                echo "    ❌ 分析失败"
            fi
        fi
    done
else
    echo "⚠️  未找到走廊生成数据文件"
fi

# 分析后端优化文件
if [ -n "$BACKEND_FILES" ]; then
    echo ""
    echo "⚙️  分析后端优化数据..."
    for file in $BACKEND_FILES; do
        if [ -f "$file" ]; then
            echo "  处理: $(basename "$file")"
            python3 "$TOOLS_DIR/visualize_backend_optimization.py" "$file" \
                --save "$OUTPUT_DIR/$(basename "${file%.json}")_analysis.png" \
                --no-plot 2>/dev/null
            if [ $? -eq 0 ]; then
                echo "    ✅ 分析完成"
            else
                echo "    ❌ 分析失败"
            fi
        fi
    done
else
    echo "⚠️  未找到后端优化数据文件"
fi

# 运行批量分析
echo ""
echo "📊 运行批量分析..."
python3 "$TOOLS_DIR/batch_analysis.py" "$DATA_DIR" \
    --output-dir "$OUTPUT_DIR" \
    --no-plots \
    --report-file "batch_analysis_report.txt" 2>/dev/null

if [ $? -eq 0 ]; then
    echo "✅ 批量分析完成"
else
    echo "❌ 批量分析失败"
fi

# 显示结果
echo ""
echo "📁 分析结果："
if [ -d "$OUTPUT_DIR" ]; then
    echo "输出目录: $OUTPUT_DIR"
    echo "生成的文件："
    ls -la "$OUTPUT_DIR" 2>/dev/null | grep -E '\.(png|txt|csv)$' | while read -r line; do
        echo "  $line"
    done
else
    echo "未生成输出文件"
fi

echo ""
echo "🎉 示例运行完成！"
echo ""
echo "💡 使用提示："
echo "1. 运行Navigo规划器生成JSON数据文件"
echo "2. 使用单独工具分析特定模块："
echo "   python3 visualize_frontend_search.py your_file.json"
echo "   python3 visualize_corridor.py your_file.json"
echo "   python3 visualize_backend_optimization.py your_file.json"
echo "3. 使用批量工具分析所有数据："
echo "   python3 batch_analysis.py $DATA_DIR"
echo "4. 查看README.md获取详细使用说明"