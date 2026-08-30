# 整帧道路走廊拟合结果；控制端只使用 valid/deviation
std_msgs/Header header
bool valid
int16 deviation
uint16 road_center
uint16 scan_y
int16 raw_left
int16 raw_right
uint16 fit_top_y
uint16 fit_bottom_y
bool left_reconstructed
bool right_reconstructed
int16[] raw_left_boundary
int16[] raw_right_boundary
int16[] fitted_left_boundary
int16[] fitted_right_boundary
uint8[] left_repaired
uint8[] right_repaired

# 真正用于控制的鲁棒中心直线，x = center_slope*y + center_intercept。
int16[] center_boundary
float32 center_slope
float32 center_intercept
float32 center_confidence
uint16 center_candidate_count
uint16 center_inlier_count
bool center_predicted
