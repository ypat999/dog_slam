#!/bin/bash
#
# Navigo 一键分析脚本 (Wrapper for navigo_analyze.py)
#
# 注意：此脚本已更新为调用统一的navigo_analyze.py工具
# 保留此脚本以保持向后兼容性
#
# 新用法推荐：直接使用 navigo_analyze.py
#   python3 navigo_analyze.py --latest
#   python3 navigo_analyze.py --session 20241218_15
#   python3 navigo_analyze.py --list
#

# 设置脚本目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TOOLS_BASE_DIR="$(dirname "$SCRIPT_DIR")"
NAVIGO_ANALYZE="$TOOLS_BASE_DIR/navigo_analyze.py"

echo "🚀 Navigo 一键数据分析工具 (v2.0)"
echo "================================"
echo ""
echo "⚠️  注意: 此脚本现在使用新的统一CLI工具"
echo "💡 推荐直接使用: python3 navigo_analyze.py --latest"
echo ""

# 检查Python环境
if ! command -v python3 &> /dev/null; then
    echo "❌ 错误: 找不到 python3"
    exit 1
fi

# 检查新工具
if [[ ! -f "$NAVIGO_ANALYZE" ]]; then
    echo "❌ 错误: 找不到 navigo_analyze.py: $NAVIGO_ANALYZE"
    echo "💡 请确保核心模块已正确安装"
    exit 1
fi

# 解析命令行参数并转换为新CLI格式
NAVIGO_ARGS=()
CLEANUP_FLAG=false
SESSION_ID=""

while [[ $# -gt 0 ]]; do
    case $1 in
        -y|--yes)
            # 新CLI默认不需要确认
            shift
            ;;
        --clean)
            CLEANUP_FLAG=true
            shift
            ;;
        --report)
            # 报告功能暂不支持，跳过
            echo "⚠️  警告: --report 选项在新版本中暂不支持"
            shift
            ;;
        --report-format)
            # 跳过报告格式参数
            shift 2
            ;;
        --session)
            SESSION_ID="$2"
            shift 2
            ;;
        -h|--help)
            # 显示新的帮助信息
            python3 "$NAVIGO_ANALYZE" --help
            exit 0
            ;;
        *)
            echo "❌ 未知选项: $1"
            echo "使用 -h 或 --help 查看帮助"
            exit 1
            ;;
    esac
done

# 构建新CLI命令
if [[ -n "$SESSION_ID" ]]; then
    NAVIGO_ARGS+=(--session "$SESSION_ID")
else
    NAVIGO_ARGS+=(--latest)
fi

if [[ "$CLEANUP_FLAG" == "true" ]]; then
    NAVIGO_ARGS+=(--auto-cleanup)
fi

# 运行新CLI工具
echo "📊 启动分析..."
echo "执行命令: python3 navigo_analyze.py ${NAVIGO_ARGS[*]}"
echo ""

python3 "$NAVIGO_ANALYZE" "${NAVIGO_ARGS[@]}"

exit_code=$?

if [[ $exit_code -eq 0 ]]; then
    echo ""
    echo "✅ 分析完成！"
    echo ""
    echo "💡 新功能提示:"
    echo "   - 使用 'python3 navigo_analyze.py --list' 查看所有会话"
    echo "   - 使用 'python3 navigo_analyze.py --info' 查看系统信息"
    echo "   - 使用 'python3 navigo_analyze.py --cleanup --days 1' 清理旧数据"
else
    echo ""
    echo "❌ 分析过程中出现错误 (退出码: $exit_code)"
fi

exit $exit_code
