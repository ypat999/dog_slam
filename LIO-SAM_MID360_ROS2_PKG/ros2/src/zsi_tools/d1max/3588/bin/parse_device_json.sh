#!/bin/bash

# 设备配置JSON文件解析脚本
# 用法：
#   1. 获取单个字段: parse_device_json.sh get <field_name>
#   2. 导出所有字段为环境变量: source <(parse_device_json.sh export)
#   3. 打印所有字段和值: parse_device_json.sh print
#   4. 获取JSON文件路径: parse_device_json.sh --json-file

# 获取脚本所在目录
# SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# JSON_FILE="${SCRIPT_DIR}/device.json"

JSON_FILE="/userdata/config/device.json"

# 检查JSON文件是否存在
if [[ ! -f "$JSON_FILE" ]]; then
    echo "错误: 找不到JSON文件: $JSON_FILE" >&2
    exit 1
fi

# ============================================
# 检测可用的JSON解析工具
# ============================================
PARSER=""

if command -v jq &> /dev/null; then
    PARSER="jq"
elif command -v python3 &> /dev/null; then
    PARSER="python3"
elif command -v python &> /dev/null; then
    PARSER="python"
else
    PARSER="bash"
fi

# ============================================
# 基于Bash的JSON解析函数（不需要外部工具）
# ============================================
parse_json_bash() {
    local field=$1
    local json_file=$2
    
    # 使用grep和sed提取字段值
    # 格式: "field": "value" 或 "field": value (不带引号)
    grep -o "\"$field\"[[:space:]]*:[[:space:]]*[^,}]*" "$json_file" | \
    grep -o ":[[:space:]]*[^,}]*" | \
    sed 's/:[[:space:]]*"\([^"]*\)".*/\1/' | \
    sed 's/:[[:space:]]*\([^,}]*\).*/\1/' | \
    sed 's/^[[:space:]]*//;s/[[:space:]]*$//'
}

# ============================================
# 基于Python的JSON解析函数
# ============================================
parse_json_python() {
    local field=$1
    local json_file=$2
    local python_cmd=${PARSER}
    
    $python_cmd -c "
import json
with open('$json_file', 'r') as f:
    data = json.load(f)
    print(data.get('$field', ''))
" 2>/dev/null || echo ""
}

# ============================================
# 基于JQ的JSON解析函数
# ============================================
parse_json_jq() {
    local field=$1
    local json_file=$2
    
    jq -r ".${field}" "$json_file" 2>/dev/null || echo ""
}

# ============================================
# 获取单个字段值 - 自动选择解析方式
# ============================================
get_field() {
    local field=$1
    
    case "$PARSER" in
        jq)
            parse_json_jq "$field" "$JSON_FILE"
            ;;
        python3|python)
            parse_json_python "$field" "$JSON_FILE"
            ;;
        bash)
            parse_json_bash "$field" "$JSON_FILE"
            ;;
    esac
}

# ============================================
# 导出所有字段为环境变量
# ============================================
export_all() {
    case "$PARSER" in
        jq)
            jq -r 'to_entries | .[] | "export DEVICE_\(.key | ascii_upcase)=\"\(.value)\""' "$JSON_FILE"
            ;;
        python3|python)
            $PARSER -c "
import json
with open('$JSON_FILE', 'r') as f:
    data = json.load(f)
    for key, value in data.items():
        print(f'export DEVICE_{key.upper()}=\"{value}\"')
" 2>/dev/null
            ;;
        bash)
            # 手动提取所有字段
            grep -o '"[^"]*"[[:space:]]*:[[:space:]]*[^,}]*' "$JSON_FILE" | while read line; do
                key=$(echo "$line" | grep -o '"[^"]*"' | head -1 | sed 's/"//g')
                value=$(echo "$line" | grep -o ':[[:space:]]*[^,}]*' | sed 's/:[[:space:]]*"\([^"]*\)".*/\1/' | sed 's/:[[:space:]]*\([^,}]*\).*/\1/' | sed 's/^[[:space:]]*//;s/[[:space:]]*$//')
                echo "export DEVICE_${key^^}=\"$value\""
            done
            ;;
    esac
}

# ============================================
# 打印所有字段
# ============================================
print_all() {
    case "$PARSER" in
        jq)
            jq -r 'to_entries[] | "\(.key): \(.value)"' "$JSON_FILE"
            ;;
        python3|python)
            $PARSER -c "
import json
with open('$JSON_FILE', 'r') as f:
    data = json.load(f)
    for key, value in data.items():
        print(f'{key}: {value}')
" 2>/dev/null
            ;;
        bash)
            grep -o '"[^"]*"[[:space:]]*:[[:space:]]*[^,}]*' "$JSON_FILE" | while read line; do
                key=$(echo "$line" | grep -o '"[^"]*"' | head -1 | sed 's/"//g')
                value=$(echo "$line" | grep -o ':[[:space:]]*[^,}]*' | sed 's/:[[:space:]]*"\([^"]*\)".*/\1/' | sed 's/:[[:space:]]*\([^,}]*\).*/\1/' | sed 's/^[[:space:]]*//;s/[[:space:]]*$//')
                echo "$key: $value"
            done
            ;;
    esac
}

# 主程序
case "${1:---help}" in
    get)
        if [[ -z "$2" ]]; then
            echo "用法: $0 get <field_name>" >&2
            echo "示例: $0 get board" >&2
            exit 1
        fi
        get_field "$2"
        ;;
    export)
        export_all
        ;;
    print)
        print_all
        ;;
    --json-file)
        echo "$JSON_FILE"
        ;;
    --parser)
        echo "当前使用的解析器: $PARSER"
        ;;
    --help|-h)
        cat <<EOF
设备配置JSON文件解析脚本

用法：
  $0 get <field>          - 获取指定字段的值
  $0 export               - 导出所有字段为环境变量
  $0 print                - 打印所有字段和值
  $0 --json-file          - 显示JSON文件路径
  $0 --parser             - 显示当前使用的解析器
  $0 --help               - 显示此帮助信息

支持的解析器（优先级顺序）：
  1. jq        - 最快，功能最全
  2. python3   - 安装Python 3.x
  3. python    - 安装Python 2.x
  4. bash      - 纯bash实现，无需外部依赖

示例：
  # 获取单个字段
  $0 get board
  $0 get sn

  # 导出所有字段为环境变量（在脚本中使用）
  source <($0 export)
  echo \$DEVICE_BOARD
  echo \$DEVICE_SN

  # 打印所有字段
  $0 print

  # 查看当前使用的解析器
  $0 --parser

  # 在其他脚本中使用
  BOARD=\$($0 get board)
  SN=\$($0 get sn)
  echo "设备: \$BOARD, 序列号: \$SN"

现有字段列表:
  board   - 主板型号
  custom  - 自定义字段
  hw_ver  - 硬件版本
  sn      - 序列号
  tag     - 标签
  type    - 设备类型
EOF
        ;;
    *)
        echo "未知的命令: $1" >&2
        echo "使用 '$0 --help' 查看帮助信息" >&2
        exit 1
        ;;
esac
