# 航行模式信息
# 话题 navimode_data
int32 stop		# 0正常，1刹车
int32 control_mode	# 0挡位控制，1转速控制，2航速控制
string path		# 目标航路
int32 autopilot		# 0无，1自动舵模式
float64 heading_target	# 目标艏向
int32 berthing		# 0无，1靠泊，2离泊
float64 berth_heading	# 泊位朝向
float64 berth_lon	# 泊位经度
float64 berth_lat	# 泊位纬度

# 话题 berth_path
string berth_path	# 靠泊路径

# 话题 identify_completed_trigger
bool identify_completed	# 辨识完成
