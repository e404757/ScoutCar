# 下位机回传事件
# 常量值 = 串口协议字节，serial_node 
uint8 START    = 170   # 0xAA 启动任务
uint8 STOP = 187   
uint8 CAM_TURNED= 204   
uint8 ARRIVED  = 221   # 0xDD 到达目标点，开始转向
uint8 TURN_FINISHED = 238  # 0xEE 转向结束
#uint8 HEARTED = 255   # 0xFF 心跳


uint8 event
