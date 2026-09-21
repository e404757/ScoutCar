std_msgs/Header header
uint32 revision


uint8 SYSTEM_UNKNOWN=0
uint8 SYSTEM_READY=170
uint8 SYSTEM_ERROR=187
uint8 system_status


uint8 WAIT_START=0
uint8 DRIVING=1 
uint8 FINDING_BTP=2 
uint8 TURNING= 3 
uint8 DETECTING= 4
uint8 FINISHED= 5
uint8 mission_state


# 任务剩余数量
uint8 fixed_remaining
uint8 random_remaining


# 当前正在行驶的路段；不在路段时设为 -1
int32 segment_start
int32 segment_goal

# 当前正在执行的路段下标；无有效路段时设为 -1
int32 segment_index

uint8 DIRECTION_NONE=0
uint8 DIRECTION_AHEAD=1
uint8 DIRECTION_LEFT=2
uint8 DIRECTION_RIGHT=3
uint8 DIRECTION_UTURN=4

# 两个摄像头分别记录目标姿态
uint8 front_camera_pose
uint8 turn_camera_pose

# 当前路段终点的到达后动作
uint8 arrival_action

# 最近一次侦察结果
bool recon_result_valid
uint8 recon_left_result
uint8 recon_right_result
