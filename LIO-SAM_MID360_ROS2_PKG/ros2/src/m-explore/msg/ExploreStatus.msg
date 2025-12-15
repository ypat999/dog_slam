# 探索状态消息定义

# 探索状态枚举
uint8 STATE_STARTING = 0      # 启动中
uint8 STATE_EXPLORING = 1     # 探索中  
uint8 STATE_COMPLETED = 2     # 探索完成
uint8 STATE_STOPPED = 3       # 已停止

# 消息头
std_msgs/Header header

# 当前状态
uint8 state

# 状态描述
string state_description

# 已探索的前沿数量
uint32 frontiers_explored

# 剩余的前沿数量
uint32 frontiers_remaining

# 探索进度百分比 (0-100)
float32 progress_percentage

# 是否正在返回初始位置
bool returning_to_init