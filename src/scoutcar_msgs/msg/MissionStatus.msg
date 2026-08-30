# 任务状态/调试信息（对应调试帧 FF 01 [状态] [固定剩余] [随机剩余] DD 的扩展）
# mission_node 发布；serial_node 订阅发调试帧；web/telemetry 订阅显示任务进度
uint8 READY = 170   # 0xAA
uint8 ERROR = 187   # 0xBB

uint8 status            # READY / ERROR
uint8 fixed_remain      # 剩余固定任务点
uint8 random_remain     # 剩余随机侦查点
int32 current_node      # 小车当前位置（格点）
int32 seg_index         # 下一条待发路段下标
int32 state             # 0=等待 1=运行 2=完成
