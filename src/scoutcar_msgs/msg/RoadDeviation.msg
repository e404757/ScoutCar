uint8 STATUS_OK=0
uint8 STATUS_NO_SEGMENTATION=1
uint8 STATUS_TRACKING_FAILED=2
uint8 STATUS_INFERENCE_ERROR=3

std_msgs/Header header
uint8 status
int16 deviation
uint16 road_center
uint16 scan_y
int16 left
int16 right

int32 front_reference_x
int32 turn_reference_x


uint16 min_width
uint16 max_width
string boundary_source
