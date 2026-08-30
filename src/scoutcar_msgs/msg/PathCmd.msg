# 单跳路径指令：从 start 走到 goal，到达 goal 后执行 action
# 对应协议帧 FF 02 [start][goal][action] DD；整条路线 = 多条 PathCmd 依次下发
# 语义：action 是"到达终点后执行的动作"
uint8 STOP    = 0     # 停车
uint8 STRAIGHT = 1    # 直行
uint8 LEFT    = 2     # 左转
uint8 RIGHT   = 3     # 右转
uint8 UTURN   = 4     # 掉头

uint8 start
uint8 goal
uint8 action
