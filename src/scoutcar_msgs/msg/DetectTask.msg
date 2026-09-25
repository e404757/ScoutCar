uint8 START = 170
uint8 END = 187
# 感知自动侦察超时退出，与网页手动 END 区分：手动 END 会把转向相机留在转弯方向，
# 自动退出需要两路相机都回正并直接恢复巡航
uint8 AUTO_END = 188

uint8 status
uint8 left_result
uint8 right_result