# 控制模式
uint32 value

# 趴下
uint32 LIE_DOWN = 0

# 站立
uint32 STAND_UP = 1

# 站低
uint32 STAND_LOW = 2

#爬高台
uint32 CLIMB = 3
#廋身
uint32 SLIM = 4
#步态
uint32 GAIT =5 


#紧急停止
uint32 EMERGENCY_STOP = 8
#紧急停止恢复
uint32 EMERGENCY_STOP_RECOVER = 9

#锁定
uint32 LOCKED = 10


#设置模式 通用模式
uint32 MODE_GENERAL = 11

#设置模式 本地模式
uint32 MODE_IN_PLACE = 12

#设置模式 登阶模式
uint32 MODE_STAIR =13


#左右探头 
uint32 LOOK_LEFT = 16
uint32 LOOK_RIGHT = 17
uint32 HEAD_LEFT = 18
uint32 HEAD_RIGHT = 19
uint32 HEAD_LOOK_RECOVER = 20

#高低站姿 
#低站姿
uint32 STANCE_LOW = 25
#高站姿
uint32 STANCE_HIGH = 26 
#恢复站姿
uint32 STANCE_RECOVER = 27 

#左右翻滚  
#左翻滚
uint32 ROLL_LEFT = 30
#右翻滚
uint32 ROLL_RIGHT = 31
#恢复
uint32 ROLL_RECOVER = 32

## 中狗拓展
#前补光灯
uint32 FRONT_LIGHT_OPEN = 40
uint32 FRONT_LIGHT_CLOSE = 41

#后补光灯
uint32 BACK_LIGHT_OPEN = 42
uint32 BACK_LIGHT_CLOSE = 43

#调整头尾方向
uint32 REVERSER_HEAD_TAIL = 50

#开始充电
uint32 START_CHARGING = 60
#结束充电
uint32 EXIT_CHARGING = 61
#开始对准
uint32 START_ARCALIGN_COARSE = 62 

#设置速度 低速  中速  高速
uint32 LOW_SPEED = 100

uint32 MEDIUM_SPEED = 101

uint32 HIGH_SPEED = 102
