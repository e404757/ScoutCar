# 整帧道路走廊拟合结果；控制端只使用 valid/deviation
std_msgs/Header header
bool valid
int16 deviation
uint16 road_center
uint16 scan_y
int16 left
int16 right

# Web 调参诊断：均为算法本帧实际采用的数据，不由 Web 推测。
string preview_mode
float32 selected_pt
uint16 width
uint16 min_width
uint16 max_width
string width_status
string boundary_source

# 算法诊断与控制门控分离：任务离线时算法结果仍可在 Web 正常显示。
bool algorithm_valid
int16 algorithm_deviation
bool control_ready
