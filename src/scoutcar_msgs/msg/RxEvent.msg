# 下位机回传事件
# 常量值 = 串口协议字节，serial_node 解包后直接映射，不另做翻译层
uint8 START    = 170   # 0xAA 启动任务
uint8 ARRIVED  = 221   # 0xDD 到达目标点，开始转向
uint8 TURN_FINISHED = 238  # 0xEE 转向结束
uint8 OBSTACLE = 204   # 0xCC 障碍：立即重规划 + 掉头
uint8 STRAIGHT = 255   # 0xFF 直行心跳（直行状态下持续发送；路口直行 = DD 立即 EE 再回 FF 流）

uint8 event
