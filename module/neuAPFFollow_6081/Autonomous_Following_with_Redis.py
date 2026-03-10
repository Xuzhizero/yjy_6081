import math
import redis
import time
import math
import ip_transform
import numpy as np
import ip_bezier_path 
import ip_is_intersecting_line_segment
import logging
import os
from datetime import datetime
from ca_main import PathPlanningScript
import threading

avoid_script = None
avoid_thread = None

has_deleted_lpath = False     # 是否已经删除过路径
target_lost = False           # 是否处于目标丢失状态

# target_V = 40 # 对应8节
# # default_V = 100 # 巡航速度，对应17.5节
# turn_V = 60 # 掉头/绕尾速度，对应11.4节
# slow_V = 20

"""判断碰撞风险时不用加入目标船，和其他障碍船存在碰撞风险需要避碰时要考虑目标船"""
"""在能维持舵效的情况下，最低航速能达到多少"""

class TargetLostError(Exception):
    pass

# ===== 日志路径 =====
desktop_path = os.path.join(os.path.expanduser("~"), "桌面")
log_dir = os.path.join(desktop_path, "program_logs")
os.makedirs(log_dir, exist_ok=True)

log_file = os.path.join(
    log_dir,
    datetime.now().strftime("%Y-%m-%d") + ".log"
)

# ===== 强制重置 root logger =====
logger = logging.getLogger()
logger.setLevel(logging.INFO)

# ⚠️ 关键：清空已有 handler（否则 basicConfig 永远没用）
logger.handlers.clear()

# 文件日志
file_handler = logging.FileHandler(log_file, encoding="utf-8")
file_handler.setLevel(logging.INFO)

# 终端日志（强烈建议）
console_handler = logging.StreamHandler()
console_handler.setLevel(logging.INFO)

formatter = logging.Formatter(
    "%(asctime)s [%(levelname)s] %(message)s",
    "%Y-%m-%d %H:%M:%S"
)

file_handler.setFormatter(formatter)
console_handler.setFormatter(formatter)

logger.addHandler(file_handler)
logger.addHandler(console_handler)

logger.info("日志系统初始化完成")

def speed_to_rpm(speed):
        ###########################################
        # 6081实船
        print("speed",speed)
        if speed == 0:
            return 0
        elif speed == 1:
            return 3.0864
        elif speed == 2:  
            return 4.1152
        elif speed == 3:
            return 6.1728
        elif speed == 4:
            return 7.2016
        elif speed == 5:
            return 9.2592
        elif speed == 6:
            return 11.3168
        else:
            return 0 
        
        # # 2D海图模拟器
        # if speed == 0:
        #     return 0
        # elif speed == -1:
        #     return 450
        # elif speed <= 1:
        #     return 500
        # elif speed <= 2:
        #     return 600
        # elif speed <= 3:
        #     return 700
        # elif speed <= 4:
        #     return 800
        # elif speed > 4:
        #     return 850
        # else:
        #     return 0 
def write_speed_control_mode(redis_conn, mode_str):
   # 将路径字符串发送到Redis
   redis_conn.hset("Navi", "speed_control_mode", mode_str)

def calculate_angle(A, B, C):
    """
    计算以正北为0°，顺时针方向的线段AB和线段AC的夹角。
    如果夹角大于180°，返回最小夹角（即补角）。

    参数:
        A (tuple): 点A的坐标 (X1, Y1)
        B (tuple): 点B的坐标 (X2, Y2)
        C (tuple): 点C的坐标 (X3, Y3)
speed_to_rpm
    返回:
        float: 夹角度数 (0° - 180°)
    """

    # 解包坐标
    X1, Y1 = A
    X2, Y2 = B
    X3, Y3 = C

    # 计算向量AB和AC的方位角
    theta_AB = math.degrees(math.atan2(X2 - X1, Y2 - Y1))
    theta_AC = math.degrees(math.atan2(X3 - X1, Y3 - Y1))

    # 计算顺时针夹角
    angle = (theta_AC - theta_AB) % 360
    if angle < 0:
        angle += 360  # 确保为正值

    # 如果夹角大于180°，取补角
    final_angle = min(angle, 360 - angle)
    return final_angle
def bearing_from_north_clockwise_xy(A, B):
    # A, B: (x, y) -> x 向东，y 向北
    x1, y1 = A
    x2, y2 = B
    dx = x2 - x1   # 东向分量
    dy = y2 - y1   # 北向分量
    # 下面的 atan2(dx, dy) —— 注意参数顺序（dx, dy）而不是 (dy, dx)
    # 这样返回的角度就是以正北为 0，顺时针为正
    theta = math.degrees(math.atan2(dx, dy))
    bearing = (theta + 360) % 360
    return bearing
def calculate_foot_of_perpendicular(A, B, P):
    """
    计算点P到线段AB的垂点（垂足）坐标
    :param A: 线段起点，元组形式 (x1, y1)
    :param B: 线段终点，元组形式 (x2, y2)
    :param P: 目标点，元组形式 (x3, y3)
    :return: 垂点坐标 (x_q, y_q)
    """
    # 解包坐标
    x1, y1 = A
    x2, y2 = B
    x3, y3 = P
    
    # 计算向量AB的分量
    dx_AB = x2 - x1
    dy_AB = y2 - y1
    
    # 处理特殊情况：A和B是同一个点（线段长度为0）
    if dx_AB == 0 and dy_AB == 0:
        return (x1, y1)
    
    # 计算向量AP的分量
    dx_AP = x3 - x1
    dy_AP = y3 - y1
    
    # 计算点积 AP · AB
    dot_product = dx_AP * dx_AB + dy_AP * dy_AB
    # 计算|AB|²
    len_sq_AB = dx_AB **2 + dy_AB** 2
    
    # 计算投影参数t
    t = dot_product / len_sq_AB
    
    # 限制t在[0, 1]范围内（线段约束）
    t = max(0.0, min(t, 1.0))
    
    # 计算垂点坐标
    x_q = x1 + t * dx_AB
    y_q = y1 + t * dy_AB
    
    return (round(x_q, 4), round(y_q, 4))  # 保留4位小数，避免浮点冗余

def bearing_from_north_clockwise(A, B):
    """
    A, B: (x, y)
    返回值：角度（0~360），正北为0°，顺时针为正
    """
    dx = B[0] - A[0]   # 东向
    dy = B[1] - A[1]   # 北向

    angle = math.degrees(math.atan2(dx, dy)) % 360
    return angle

def filter_path_by_distance(path, min_dist=25.0):
    """
    path: [(x, y), (x, y), ...]
    min_dist: 最小相邻点距离（米）
    
    保证第一个点和最后一个点必保留
    """
    if len(path) <= 1:
        return path

    filtered_path = [path[0]]  # 第一个点必保留
    last_kept = path[0]

    for pt in path[1:-1]:  # 遍历中间点，不包含最后一个点
        dx = pt[0] - last_kept[0]
        dy = pt[1] - last_kept[1]
        dist = math.hypot(dx, dy)
        if dist >= min_dist:
            filtered_path.append(pt)
            last_kept = pt

    # 最后一个点必保留
    filtered_path.append(path[-1])

    return filtered_path

def calculate_angle_between_AB_AC(A, B, C, return_degree=True):
    """
    计算以A为顶点，线段AB和AC组成的夹角
    :param A: 顶点坐标，元组 (x_A, y_A)
    :param B: 点B坐标，元组 (x_B, y_B)
    :param C: 点C坐标，元组 (x_C, y_C)
    :param return_degree: 是否返回角度（True=角度，False=弧度）
    :return: 夹角值（角度/弧度），无意义时返回None
    """
    # 解包坐标
    x_A, y_A = A
    x_B, y_B = B
    x_C, y_C = C
    
    # 计算向量AB和AC的分量
    vec_AB_x = x_B - x_A
    vec_AB_y = y_B - y_A
    vec_AC_x = x_C - x_A
    vec_AC_y = y_C - y_A
    
    # 计算向量点积
    dot_product = vec_AB_x * vec_AC_x + vec_AB_y * vec_AC_y
    
    # 计算向量模长
    len_AB = math.hypot(vec_AB_x, vec_AB_y)  # 等价于sqrt(x²+y²)，更高效
    len_AC = math.hypot(vec_AC_x, vec_AC_y)
    
    # 处理边界情况：AB或AC长度为0（A与B/C重合）
    if len_AB == 0 or len_AC == 0:
        print("错误：点A与B/C重合，无法计算夹角！")
        return None
    
    # 计算余弦值，并约束在[-1, 1]（避免浮点误差导致超出范围）
    cos_theta = dot_product / (len_AB * len_AC)
    cos_theta = max(min(cos_theta, 1.0), -1.0)
    
    # 计算夹角（弧度）
    theta_rad = math.acos(cos_theta)
    
    # 转换为角度（可选）
    if return_degree:
        theta_deg = math.degrees(theta_rad)
        return round(theta_deg, 2)  # 保留2位小数
    else:
        return round(theta_rad, 4)  # 弧度保留4位小数

def xy_to_latlon(Lon1, Lat1, x1, y1, x2, y2):
    """
    根据已知的A点的经纬度和两点的平面坐标，推导B点的经纬度。

    :param Lon1: A点的经度
    :param Lat1: A点的纬度
    :param x1: A点的x平面坐标（米）
    :param y1: A点的y平面坐标（米）
    :param x2: B点的x平面坐标（米）
    :param y2: B点的y平面坐标（米）
    :return: B点的经纬度 (Lon2, Lat2)
    """
    # 地球半径，单位为米
    R = 6375100

    # 计算 A 点和 B 点之间的平面坐标差值
    dx = x2 - x1
    dy = y2 - y1

    # 将 A 点的经纬度从度转换为弧度
    Lon1_rad = math.radians(Lon1)
    Lat1_rad = math.radians(Lat1)

    # 计算 B 点的纬度增量
    dLat = dy / R  # 纬度差值，以弧度为单位
    Lat2_rad = Lat1_rad + dLat  # B 点的纬度（弧度）

    # 计算 B 点的经度增量
    # 考虑到纬度不同，经度的差异会不同，因此需要乘以 cos(Lat1)
    dLon = dx / (R * math.cos(Lat1_rad))  # 经度差值，以弧度为单位
    Lon2_rad = Lon1_rad + dLon  # B 点的经度（弧度）

    # 将 B 点的经纬度从弧度转换为度
    Lon2 = math.degrees(Lon2_rad)
    Lat2 = math.degrees(Lat2_rad)

    return Lon2, Lat2
def has_collision_risk(obstacles):
    for obs in obstacles:
        x = obs.get("x")
        y = obs.get("y")
        dcpa = obs.get("dcpa")
        tcpa = obs.get("tcpa")

        if x is None or y is None:
            continue

        if dcpa < 400 and 0 < tcpa <= 5:
            return True

    return False

redis_conn = None
# gp_index = 0

def init_redis():
    host = '127.0.0.1'
    port = 6379
    db =  0
    password =  None

    redis_conn = redis.Redis(host=host, port=port, db=db, password=password)

    try:
        redis_conn.ping()  
        print("Redis connection established successfully.")
    except redis.ConnectionError as e:
        print(f"Error connecting to Redis: {e}")
    return redis_conn

# 建立 Redis 连接
redis_conn = init_redis()

if redis_conn:
    try:
        while True:
            try:
                # ================== 跟随模式状态 ==================
                status_bytes = redis_conn.hget("Follow_Mode", "Status")
                status = status_bytes.decode('utf-8')
                print("status", status)
                if status_bytes is None:
                    print("⚠️ Follow_Mode:Status 不存在，等待中")
                    time.sleep(1)
                    continue
                # ================== 跟随过程避碰状态 ==================
                Auto_CA_sw_bytes = redis_conn.hget("UI", "Auto_CA_sw")
                Auto_CA_sw = Auto_CA_sw_bytes.decode('utf-8')
                print("Auto_CA_sw", Auto_CA_sw)
                if status_bytes is None:
                    print("⚠️ UI:Auto_CA_sw 不存在，等待中")
                    time.sleep(1)
                    continue

                # ================== 本船状态 ==================
                value_own_Lat = redis_conn.hget("IMU", "Lat")
                value_own_Lon = redis_conn.hget("IMU", "Lon")
                value_own_heading = redis_conn.hget("IMU", "heading")
                value_own_speed = redis_conn.hget("IMU", "speed")

                if value_own_heading:
                    Heading = float(value_own_heading)
                    if Heading < 0:
                        Heading += 360
                else:
                    raise RuntimeError("IMU heading 缺失")

                print("============================")

                # ================== 锁定目标 ==================
                lockedID = redis_conn.hget("Follow_Mode", "lockedID")
                if lockedID is None:
                    print("⚠️ 尚未锁定目标船，等待中")
                    # write_speed_control_mode(redis_conn, 0) # 新加0305
                    if avoid_script is not None:
                        print("⛔ 跟随目标已取消，强制退出避碰模式") # 这种做法是否合理？
                        avoid_script.stop()
                        avoid_script = None
                        avoid_thread = None
                    if not has_deleted_lpath:
                        redis_conn.hdel("Navi", "LPath")
                        has_deleted_lpath = True
                    time.sleep(1)
                    continue
                lockedID = int(lockedID.decode("utf-8"))
                has_deleted_lpath = False
                print("lockedid", lockedID)

                key_target = f"data:{lockedID}"
                print(redis_conn.exists(key_target))

                # ★★★★★ 目标是否存在（硬校验）★★★★★
                if redis_conn.exists(key_target) == 0:
                    raise TargetLostError(
                        f"目标船 {key_target} 在运行过程中突然从 Redis 消失"
                    )

                # ★★★★★ 目标从丢失状态恢复 ★★★★★
                if target_lost:
                    print("✅ 目标船重新出现，恢复 FOLLOW")
                    target_lost = False
                    has_deleted_lpath = False

                # ================== 所有船只 ==================
                obstacles = []   # 用来存所有障碍船
                keys = redis_conn.keys("data:*")
                keys = list(keys)
                print("keys", keys)

                if len(keys) == 0:
                    raise TargetLostError("Redis 中不存在任何 data:* 船舶数据")

                # ================== 目标船信息 ==================
                target_lat = None
                target_lon = None

                for key in keys:
                    key_str = key.decode("utf-8")
                    sid = int(key_str.split(":")[1])
                    if sid == lockedID:
                        print("id", lockedID)
                        print("key_str", key_str)

                        target_lat = redis_conn.hget(key_str, "latitude")
                        target_lon = redis_conn.hget(key_str, "longitude")
                        target_distance = redis_conn.hget(key_str, "distance") # 新加0305
                        target_heading = redis_conn.hget(key_str, "direction")
                        target_speed = redis_conn.hget(key_str, "speed")
                        speed_knot = float(target_speed.decode('utf-8'))
                        KNOT_TO_MS = 0.514444
                        target_V = speed_knot * KNOT_TO_MS # 目标船速度
                        turn_V = 3 # 掉头/绕尾速度
                        slow_V = 1 # 等待速度
                        if any(v is None for v in [target_lat, target_lon, target_heading, target_speed]):
                           raise TargetLostError(f"{key_str} 字段不完整，视为目标丢失")
                        target_distance = float(target_distance.decode("utf-8")) # 新加0305
                        target_Heading = float(target_heading.decode("utf-8"))
                        target_speed = float(target_speed.decode("utf-8"))
                        target_x, target_y = ip_transform.geodistance(
                            value_own_Lon.decode('utf-8'),
                            value_own_Lat.decode('utf-8'),
                            target_lon,
                            target_lat
                        )
                        print("目标船坐标",(target_x,target_y))
                    else:  # 排除追踪船
                        obstacle_lat = redis_conn.hget(key_str, "latitude")
                        obstacle_lon = redis_conn.hget(key_str, "longitude")
                        obstacle_heading = redis_conn.hget(key_str, "direction")
                        obstacle_speed = redis_conn.hget(key_str, "speed")
                        if any(v is None for v in [obstacle_lat, obstacle_lon, obstacle_heading, obstacle_speed]):
                           raise TargetLostError(f"{key_str} 字段不完整，视为目标丢失")
                        obstacle_Heading = float(obstacle_heading.decode("utf-8"))
                        obstacle_speed = float(obstacle_speed.decode("utf-8"))
                        obstacle_x, obstacle_y = ip_transform.geodistance(
                            value_own_Lon.decode('utf-8'),
                            value_own_Lat.decode('utf-8'),
                            obstacle_lon,
                            obstacle_lat
                        )
                        tcpa = redis_conn.hget(key_str, "cpTime")
                        tcpa=float(tcpa.decode("utf-8")) 
                        dcpa = redis_conn.hget(key_str, "cpDistance")
                        dcpa=float(dcpa.decode("utf-8"))
                        dcpa=dcpa*1852

                        obstacles.append({
                            "id": sid,
                            "x": obstacle_x,
                            "y": obstacle_y,
                            "dcpa": dcpa,
                            "tcpa": tcpa
                        })
                        print(f"障碍船 {sid} | 坐标=({obstacle_x:.1f}, {obstacle_y:.1f}) "
                f"| DCPA={dcpa:.1f} m | TCPA={tcpa:.1f} s")

                if target_lat is None or target_lon is None:
                    raise TargetLostError("未在 data:* 中找到目标船")

                Speed = redis_conn.hget("Navi", "TargetSpeed")
                Speed = int(Speed.decode('utf-8'))
                print("speed",Speed)
                default_V = speed_to_rpm(Speed) 
                # ================== 跟随逻辑 ==================
                if status == "following":
                    cog_os = Heading
                    sog_os = float(value_own_speed.decode('utf-8'))
                    x_os, y_os = 0, 0
                    cog_ts = target_Heading
                    sog_ts = target_speed
                    x_ts = target_x
                    y_ts = target_y

                    follow_distance = float(redis_conn.hget("Follow_Mode", "follow_distance").decode("utf-8"))
                    follow_Heading = float(redis_conn.hget("Follow_Mode", "follow_angle").decode("utf-8"))
                    Heading_rad = math.radians(follow_Heading)
                    Escort_distance_x = follow_distance * math.sin(Heading_rad) # 该等于0的时候不等于0，导致后面判断出问题
                    if abs(Escort_distance_x) < 1e-6:
                        Escort_distance_x = 0.0
                    Escort_distance_y = follow_distance * math.cos(Heading_rad)

                    Tug_State = [0, 0, Heading, sog_os]
                    Target_State = [x_ts, y_ts, cog_ts, sog_ts, 50]
                    u_turn_r=100
                    heading_rad = math.radians(Target_State[2])
                    target_heading_rad = np.deg2rad(Target_State[2])

                    Escort_Pos = [
                        Target_State[0] + math.cos(heading_rad) * Escort_distance_x + Escort_distance_y * np.sin(heading_rad),
                        Target_State[1] - math.sin(heading_rad) * Escort_distance_x + Escort_distance_y * np.cos(heading_rad)
                    ]

                    result = ip_is_intersecting_line_segment.is_intersecting_line_segment(
                        Tug_State[0], Tug_State[1],
                        Escort_Pos[0], Escort_Pos[1],
                        Target_State[0] - 10000 * np.sin(target_heading_rad),
                        Target_State[1] - 10000 * np.cos(target_heading_rad),
                        Target_State[0] + 10000 * np.sin(target_heading_rad),
                        Target_State[1] + 10000 * np.cos(target_heading_rad)
                    )

                    risk = has_collision_risk(obstacles) # 判断与障碍船是否存在碰撞风险
                    print("碰撞风险",risk)

                    if not result: 
                        print("进入掉头模式")
                        # 预测提前
                        l_xy = (Target_State[0] - 10000 * np.sin(target_heading_rad)),(Target_State[1] - 10000 * np.cos(target_heading_rad))
                        f_xy = (Target_State[0] + 10000 * np.sin(target_heading_rad)),(Target_State[1] + 10000 * np.cos(target_heading_rad))
                        target_xy = (Target_State[0],Target_State[1])
                        own_xy = (0,0)
                        own_targeg_heading = calculate_angle_between_AB_AC(target_xy, own_xy, f_xy, return_degree=True)
                        print("夹角",own_targeg_heading)
                        foot_xy = calculate_foot_of_perpendicular(f_xy,l_xy,own_xy) # 本船到目标航线方向线段的垂足
                        dist_to_foot = math.sqrt((foot_xy[0] - Target_State[0]) ** 2 + (foot_xy[1] - Target_State[1]) ** 2) # 目标距离垂足的距离
                        own_dist_to_foot = math.sqrt((foot_xy[0] - 0) ** 2 + (foot_xy[1] - 0) ** 2) # 本船距离垂足的距离
                        if own_targeg_heading >= 90 or Escort_distance_x == 0:
                            Escort_distance_y1 = Escort_distance_y 
                        else:
                            Escort_distance_y1 = Escort_distance_y + max(dist_to_foot,own_dist_to_foot) + 70.7
                        D = 200  # 正值表示在目标船正横右D(m)为起点，负值表示在目标船正横左D(m为起点)
                        d = 50
                        R = u_turn_r  # 拖轮最小转弯半径
                        R1 = R

                        if Escort_distance_x < 0:
                            R = -R
                        if Escort_distance_x == 0:
                            R = R

                        Right_Target1 = [
                            (Target_State[0] + Escort_distance_x * np.cos(heading_rad) + Escort_distance_y1 * np.sin(
                                heading_rad)) + np.cos(heading_rad) * 2 * R,
                            (Target_State[1] - Escort_distance_x * np.sin(heading_rad) + Escort_distance_y1 * np.cos(
                                heading_rad)) - np.sin(heading_rad) * 2 * R]  # 第一个追踪点

                        Right_Target2 = [
                            (Target_State[0] + Escort_distance_x * np.cos(heading_rad) + Escort_distance_y1 * np.sin(
                                heading_rad)) + np.cos(heading_rad) * R - R1 * np.sin(heading_rad),
                            (Target_State[1] - Escort_distance_x * np.sin(heading_rad) + Escort_distance_y1 * np.cos(
                                heading_rad)) - np.sin(heading_rad) * R - R1 * np.cos(heading_rad)]  # 第二个追踪点     大问题：右侧相向跟随时路径点不对

                        Right_Target3 = [
                            Target_State[0] + math.cos(heading_rad) * (Escort_distance_x) + Escort_distance_y1 * np.sin(heading_rad),
                            Target_State[1] - math.sin(heading_rad) * (Escort_distance_x) + Escort_distance_y1 * np.cos(heading_rad)]  # 第三个追踪点
                        
                        Right_Target_goal = [
                            Target_State[0] + math.cos(heading_rad) * (Escort_distance_x) + Escort_distance_y * np.sin(heading_rad),
                            Target_State[1] - math.sin(heading_rad) * (Escort_distance_x) + Escort_distance_y * np.cos(heading_rad)]  # 伴航点

                        Center_Point = [
                            (Target_State[0] + Escort_distance_x * np.cos(heading_rad) + Escort_distance_y1 * np.sin(
                                heading_rad)) + np.cos(heading_rad) * R,
                            (Target_State[1] - Escort_distance_x * np.sin(heading_rad) + Escort_distance_y1 * np.cos(
                                heading_rad)) - np.sin(heading_rad) * R]  # 转弯圆心

                        Distance_to_Target1 = math.sqrt(
                            np.square(Right_Target1[0] - x_os) + np.square(Right_Target1[1] - y_os))  # 拖轮与第一个点的距离
                        Distance_to_Target2 = math.sqrt(
                            np.square(Right_Target2[0] - x_os) + np.square(Right_Target2[1] - y_os))  # 拖轮与第二个点的距离
                        Distance_to_Target3 = math.sqrt(
                            np.square(Right_Target3[0] - x_os) + np.square(Right_Target3[1] - y_os))  # 拖轮与第三个点的距离
                        Distance_to_Target_goal = math.sqrt(
                            np.square(Right_Target_goal[0] - x_os) + np.square(Right_Target_goal[1] - y_os))
                        Point0 = [0,0]
                        Point1 = [Right_Target1[0], Right_Target1[1]]
                        Point2 = [Right_Target2[0], Right_Target2[1]]
                        Point3 = [Right_Target3[0], Right_Target3[1]]
                        Point4 = [Right_Target3[0] + 6 * d * math.sin(heading_rad),  
                                Right_Target3[1] + 6 * d * math.cos(heading_rad)]
                        Point5 = [Right_Target3[0] + 12 * d * math.sin(heading_rad),
                                Right_Target3[1] + 12 * d * math.cos(heading_rad)]
                        Point6 = [Right_Target_goal[0] + 18 * d * math.sin(heading_rad),
                                Right_Target_goal[1] + 18 * d * math.cos(heading_rad)]

                        # 假设四个点的坐标为 A(x, y), B(x1, y1), C(x2, y2), D(x3, y3)
                        # 计算向量的表示
                        vector_AB = (Right_Target1[0] - x_os, Right_Target1[1] - y_os)
                        vector_AC = (Right_Target2[0] - x_os, Right_Target2[1] - y_os)
                        vector_AD = (Right_Target3[0] - x_os, Right_Target3[1] - y_os)

                        # 计算叉乘结果
                        cross_product_AC_AB = vector_AC[0] * vector_AB[1] - vector_AC[1] * vector_AB[0]
                        cross_product_AC_AD = vector_AC[0] * vector_AD[1] - vector_AC[1] * vector_AD[0]
                        
                        if Escort_distance_x < 0:
                            cross_product_AC_AB = -cross_product_AC_AB
                            cross_product_AC_AD = -cross_product_AC_AD
                        if Target_State[2] < (cog_os - 180):
                            Target_State[2] = Target_State[2] + 360
                        if cog_os < (Target_State[2] - 180):
                            Target_State[2] = Target_State[2] - 360

                        A=(Right_Target1[0], Right_Target1[1])
                        B=(0, 0)
                        C=(Right_Target_goal[0], Right_Target_goal[1])
                        DT_2_nei_angle = calculate_angle(A, B, C)

                        result_to_Right_Target2 = ip_is_intersecting_line_segment.is_intersecting_line_segment(
                        Tug_State[0], Tug_State[1],
                        Escort_Pos[0], Escort_Pos[1],
                        Right_Target2[0] - 10000 * np.sin(target_heading_rad),
                        Right_Target2[1] - 10000 * np.cos(target_heading_rad),
                        Right_Target2[0] + 10000 * np.sin(target_heading_rad),
                        Right_Target2[1] + 10000 * np.cos(target_heading_rad))

                        if cross_product_AC_AB > 0 and cross_product_AC_AD < 0 and Distance_to_Target3 > 20 and abs(Target_State[2] - cog_os) > 100: # 追踪第一个点
                            plan_path_points=[Point0,Point1,Point2,Point3,Point4,Point5]
                            calculated_latlon_lists = []
                            for i in range(0, len(plan_path_points)):
                                x2, y2 = plan_path_points[i]  # 获取路径点列表中的 (x2, y2)
                                Lon1 = float(value_own_Lon.decode('utf-8'))
                                Lat1 = float(value_own_Lat.decode('utf-8'))
                                x1 = 0
                                y1 = 0
                                Lon2, Lat2 = xy_to_latlon(Lon1, Lat1, x1, y1, x2, y2)
                                calculated_latlon_lists.append([Lon2, Lat2])
                                D_list=len(calculated_latlon_lists)
                            calculated_latlon_lists=','.join(','.join(map(str,pair))for pair in calculated_latlon_lists)#为了符合Redis的输出格式
                            value_str = f"$GP,{D_list},{calculated_latlon_lists}"
                            redis_conn.hset("Navi", "GPath",value_str)
                            if risk and Auto_CA_sw == "1":
                                if avoid_script is None:
                                    print("⚠️ 进入避碰模式")
                                    Planner_V = default_V
                                    avoid_script = PathPlanningScript(Planner_V )
                                    avoid_thread = threading.Thread(
                                        target=avoid_script.run,
                                        daemon=True)
                                    avoid_thread.start()     
                            else:
                                if avoid_script is not None:
                                    print("✅ 风险解除，退出避碰")
                                    avoid_script.stop()
                                    avoid_script = None
                                    avoid_thread = None
                                print("追踪掉头第一个点", Point1)
                                print("与掉头第一个点的距离", Distance_to_Target1)
                                waypoint = [Point1,Point2, Point3, Point4, Point5]
                                Planner_V = default_V
                        
                            A=(Right_Target1[0], Right_Target1[1])
                            B=(0, 0)
                            C=(Right_Target_goal[0], Right_Target_goal[1])
                            angle = calculate_angle(A, B, C)
                            if angle < 90:
                                calculated_latlon_lists = []
                                plan_path_points=[Point0,Point3,Point4,Point5]
                                # 这里可以把路径发往redis，让避碰直接从redis上取
                                for i in range(0, len(plan_path_points)):
                                    x2, y2 = plan_path_points[i]  # 获取路径点列表中的 (x2, y2)
                                    Lon1 = float(value_own_Lon.decode('utf-8'))
                                    Lat1 = float(value_own_Lat.decode('utf-8'))
                                    x1 = 0
                                    y1 = 0
                                    Lon2, Lat2 = xy_to_latlon(Lon1, Lat1, x1, y1, x2, y2)
                                    calculated_latlon_lists.append([Lon2, Lat2])
                                    D_list=len(calculated_latlon_lists)
                                calculated_latlon_lists=','.join(','.join(map(str,pair))for pair in calculated_latlon_lists)#为了符合Redis的输出格式
                                value_str = f"$GP,{D_list},{calculated_latlon_lists}"
                                redis_conn.hset("Navi", "GPath",value_str)
                                if risk and Auto_CA_sw == "1":
                                    if avoid_script is None:
                                        print("⚠️ 进入避碰模式")
                                        if Distance_to_Target2 > 100:
                                            Planner_V=default_V
                                        else:
                                            Planner_V=turn_V
                                        avoid_script = PathPlanningScript(Planner_V)
                                        avoid_thread = threading.Thread(
                                            target=avoid_script.run,
                                            daemon=True)
                                        avoid_thread.start()
                                else:
                                    if avoid_script is not None:
                                        print("✅ 风险解除，退出避碰")
                                        avoid_script.stop()
                                        avoid_script = None
                                        avoid_thread = None
                                    waypoint = [Point3, Point4, Point5]
                                    if Distance_to_Target2 > 100:
                                            Planner_V=default_V
                                    else:
                                            Planner_V=turn_V
                                    print("当前与跟踪船横距太近,直接追踪掉头第三个点,当前航速为",Planner_V) # 本来是第二个点，改为第三个点

                        elif cross_product_AC_AB < 0 and Distance_to_Target2 > 10 and abs(Target_State[2] - cog_os) > 50:
                            plan_path_points=[Point0,Point2,Point3,Point4,Point5]
                            calculated_latlon_lists = []
                            # 这里可以把路径发往redis，让避碰直接从redis上取
                            for i in range(0, len(plan_path_points)):
                                x2, y2 = plan_path_points[i]  # 获取路径点列表中的 (x2, y2)
                                Lon1 = float(value_own_Lon.decode('utf-8'))
                                Lat1 = float(value_own_Lat.decode('utf-8'))
                                x1 = 0
                                y1 = 0
                                Lon2, Lat2 = xy_to_latlon(Lon1, Lat1, x1, y1, x2, y2)
                                calculated_latlon_lists.append([Lon2, Lat2])
                                D_list=len(calculated_latlon_lists)
                            calculated_latlon_lists=','.join(','.join(map(str,pair))for pair in calculated_latlon_lists)#为了符合Redis的输出格式
                            value_str = f"$GP,{D_list},{calculated_latlon_lists}"
                            redis_conn.hset("Navi", "GPath",value_str)
                            if risk and Auto_CA_sw == "1":
                                if avoid_script is None:
                                    print("⚠️ 进入避碰模式")
                                    if Distance_to_Target2 < 100:
                                        Planner_V = turn_V
                                    else:
                                        Planner_V = default_V
                                    avoid_script = PathPlanningScript(Planner_V)
                                    avoid_thread = threading.Thread(
                                        target=avoid_script.run,
                                        daemon=True)
                                    avoid_thread.start()
                            else:
                                if avoid_script is not None:
                                    print("✅ 风险解除，退出避碰")
                                    avoid_script.stop()
                                    avoid_script = None
                                    avoid_thread = None
                                print("当前进入低速转向模式，单舵归零，另一个舵保持，追踪掉头第二个点：", Point2)
                                waypoint = [Point2, Point3,Point4, Point5]
                                if Distance_to_Target2 < 100:
                                    Planner_V = turn_V
                                else:
                                    Planner_V = default_V
                        # 新加逻辑
                        elif cross_product_AC_AB > 0 and  DT_2_nei_angle < 45 and result_to_Right_Target2: # 还需加上本船与跟随点不在同一侧的判断
                            plan_path_points= [Point0, Point3, Point4, Point5]
                            calculated_latlon_lists = []
                            # 这里可以把路径发往redis，让避碰直接从redis上取
                            for i in range(0, len(plan_path_points)):
                                x2, y2 = plan_path_points[i]  # 获取路径点列表中的 (x2, y2)
                                Lon1 = float(value_own_Lon.decode('utf-8'))
                                Lat1 = float(value_own_Lat.decode('utf-8'))
                                x1 = 0
                                y1 = 0
                                Lon2, Lat2 = xy_to_latlon(Lon1, Lat1, x1, y1, x2, y2)
                                calculated_latlon_lists.append([Lon2, Lat2])
                                D_list=len(calculated_latlon_lists)
                            calculated_latlon_lists=','.join(','.join(map(str,pair))for pair in calculated_latlon_lists)#为了符合Redis的输出格式
                            value_str = f"$GP,{D_list},{calculated_latlon_lists}"
                            redis_conn.hset("Navi", "GPath",value_str)
                            if risk and Auto_CA_sw == "1":
                                if avoid_script is None:
                                    print("⚠️ 进入避碰模式")
                                    if Distance_to_Target2 > 200:
                                        Planner_V = default_V
                                    else:
                                        Planner_V = turn_V
                                    avoid_script = PathPlanningScript(Planner_V)
                                    avoid_thread = threading.Thread(
                                        target=avoid_script.run, 
                                        daemon=True)
                                    avoid_thread.start()
                            else:
                                if avoid_script is not None:
                                    print("✅ 风险解除，退出避碰")
                                    avoid_script.stop()
                                    avoid_script = None
                                    avoid_thread = None
                                waypoint = [Point3, Point4, Point5]
                                if Distance_to_Target2 > 200:
                                    Planner_V = default_V
                                else:
                                    Planner_V = turn_V
                                print("当前追踪掉头第二个路径点11",Point2)
                                print("当前追踪掉头第二个路径点,当前航速为",Planner_V)
                            

                        elif cross_product_AC_AB > 0 and cross_product_AC_AD < 0 and Distance_to_Target3 < 130 and abs(Target_State[2] - cog_os) > 100:
                            plan_path_points=[Point0,Point2,Point3,Point4,Point5]
                            calculated_latlon_lists = []
                            # 这里可以把路径发往redis，让避碰直接从redis上取
                            for i in range(0, len(plan_path_points)):
                                x2, y2 = plan_path_points[i]  # 获取路径点列表中的 (x2, y2)
                                Lon1 = float(value_own_Lon.decode('utf-8'))
                                Lat1 = float(value_own_Lat.decode('utf-8'))
                                x1 = 0
                                y1 = 0
                                Lon2, Lat2 = xy_to_latlon(Lon1, Lat1, x1, y1, x2, y2)
                                calculated_latlon_lists.append([Lon2, Lat2])
                                D_list=len(calculated_latlon_lists)
                            calculated_latlon_lists=','.join(','.join(map(str,pair))for pair in calculated_latlon_lists)#为了符合Redis的输出格式
                            value_str = f"$GP,{D_list},{calculated_latlon_lists}"
                            redis_conn.hset("Navi", "GPath",value_str)
                            if risk and Auto_CA_sw == "1":
                                if avoid_script is None:
                                    print("⚠️ 进入避碰模式")
                                    Planner_V = turn_V
                                    avoid_script = PathPlanningScript(Planner_V)
                                    avoid_thread = threading.Thread(
                                        target=avoid_script.run,
                                        daemon=True)
                                    avoid_thread.start()
                            else:
                                if avoid_script is not None:
                                    print("✅ 风险解除，退出避碰")
                                    avoid_script.stop()
                                    avoid_script = None
                                    avoid_thread = None
                                print("追踪掉头第二个点", Point2)
                                print("与掉头第二个点的距离", Distance_to_Target2)
                                waypoint=[Point2, Point3,Point4, Point5]
                                Planner_V = turn_V
                        elif cross_product_AC_AD > 0 or Distance_to_Target2 < 10 or abs(Target_State[2] - cog_os) <= 50:
                            judge = math.atan2(x_os - Right_Target_goal[0], y_os - Right_Target_goal[1]) * 180 / math.pi % 360
                            if judge < (Target_State[2] - 180):
                                judge = judge + 360
                            if Target_State[2] < (judge - 180):
                                judge = judge - 360
                            # if abs(judge - Target_State[2]) > 90 and Distance_to_Target_goal > 200:  # 从后方追近，且距离较远时加速，距离近时再减速，方位判断属于第三区域时就追踪第三个点  
                            A=(Right_Target2[0], Right_Target2[1])
                            B=(0, 0)
                            C=(Right_Target3[0], Right_Target3[1])
                            angle = calculate_angle(A, B, C)  # 计算AB和AC夹角
                            print("距离", Distance_to_Target_goal)
                            if angle > 90 and  Distance_to_Target_goal > 200:  # 从后方追近，且距离较远时加速，距离近时再减速，方位判断属于第三区域时就追踪第三个点
                                if Distance_to_Target3 > 800:
                                    plan_path_points=[Point0,Point3,Point4]
                                else:
                                    plan_path_points = [Point0,Point5,Point6]
                                calculated_latlon_lists = []
                                # 这里可以把路径发往redis，让避碰直接从redis上取
                                calculated_latlon_lists = []
                                for i in range(0, len(plan_path_points)):
                                    x2, y2 = plan_path_points[i]  # 获取路径点列表中的 (x2, y2)
                                    Lon1 = float(value_own_Lon.decode('utf-8'))
                                    Lat1 = float(value_own_Lat.decode('utf-8'))
                                    x1 = 0
                                    y1 = 0
                                    Lon2, Lat2 = xy_to_latlon(Lon1, Lat1, x1, y1, x2, y2)
                                    calculated_latlon_lists.append([Lon2, Lat2])
                                    D_list=len(calculated_latlon_lists)
                                calculated_latlon_lists=','.join(','.join(map(str,pair))for pair in calculated_latlon_lists)#为了符合Redis的输出格式
                                value_str = f"$GP,{D_list},{calculated_latlon_lists}"
                                redis_conn.hset("Navi", "GPath",value_str)
                                if risk and Auto_CA_sw == "1":
                                    if avoid_script is None:
                                        print("⚠️ 进入避碰模式")
                                        Planner_V=default_V
                                        avoid_script = PathPlanningScript(Planner_V)
                                        avoid_thread = threading.Thread(
                                            target=avoid_script.run,
                                            daemon=True)
                                        avoid_thread.start()
                                else:
                                    if avoid_script is not None:
                                        print("✅ 风险解除，退出避碰")
                                        avoid_script.stop()
                                        avoid_script = None
                                        avoid_thread = None
                                    Planner_V=default_V
                                    print("当前开始追踪掉头第三个关键路径点,当前速度为:", Planner_V)
                                    print("此时从目标船远距离追进不减速,追踪掉头第三个点", Point3)
                                    waypoint = [Point3, Point4, Point5]
                            elif abs(Target_State[2] - cog_os) > 30:
                                if Distance_to_Target3 > 800:
                                    plan_path_points=[Point0,Point3,Point4]
                                else:
                                    plan_path_points = [Point0,Point5,Point6]
                                calculated_latlon_lists = []
                                # 这里可以把路径发往redis，让避碰直接从redis上取
                                for i in range(0, len(plan_path_points)):
                                    x2, y2 = plan_path_points[i]  # 获取路径点列表中的 (x2, y2)
                                    Lon1 = float(value_own_Lon.decode('utf-8'))
                                    Lat1 = float(value_own_Lat.decode('utf-8'))
                                    x1 = 0
                                    y1 = 0
                                    Lon2, Lat2 = xy_to_latlon(Lon1, Lat1, x1, y1, x2, y2)
                                    calculated_latlon_lists.append([Lon2, Lat2])
                                    D_list=len(calculated_latlon_lists)
                                calculated_latlon_lists=','.join(','.join(map(str,pair))for pair in calculated_latlon_lists)#为了符合Redis的输出格式
                                value_str = f"$GP,{D_list},{calculated_latlon_lists}"
                                redis_conn.hset("Navi", "GPath",value_str)
                                if risk and Auto_CA_sw == "1":
                                    if avoid_script is None:
                                        print("⚠️ 进入避碰模式")
                                        if Distance_to_Target3 > 200:
                                            Planner_V = default_V
                                        else:
                                            Planner_V = turn_V
                                        avoid_script = PathPlanningScript(Planner_V)
                                        avoid_thread = threading.Thread(
                                            target=avoid_script.run,
                                            daemon=True)
                                        avoid_thread.start()
                                else:
                                    if avoid_script is not None:
                                        print("✅ 风险解除，退出避碰")
                                        avoid_script.stop()
                                        avoid_script = None
                                        avoid_thread = None
                                    print("当前进入低速转向模式，单舵归零，另一个舵保持，追踪掉头第三个点：", Point3)
                                    if Distance_to_Target3 > 200:
                                        Planner_V = default_V
                                    else:
                                        Planner_V = turn_V
                                    print("当前开始追踪掉头第三个关键路径点,当前速度为:", Planner_V)
                                    waypoint = [Point3, Point4, Point5]

                            elif 0 < abs(judge - Target_State[2]) <= 90: # 是否要加一个本船
                                if Distance_to_Target3 > 800:
                                    plan_path_points=[Point0,Point3,Point4]
                                else:
                                    plan_path_points = [Point0,Point5,Point6]
                                calculated_latlon_lists = []
                                # 这里可以把路径发往redis，让避碰直接从redis上取
                                for i in range(0, len(plan_path_points)):
                                    x2, y2 = plan_path_points[i]  # 获取路径点列表中的 (x2, y2)
                                    Lon1 = float(value_own_Lon.decode('utf-8'))
                                    Lat1 = float(value_own_Lat.decode('utf-8'))
                                    x1 = 0
                                    y1 = 0
                                    Lon2, Lat2 = xy_to_latlon(Lon1, Lat1, x1, y1, x2, y2)
                                    calculated_latlon_lists.append([Lon2, Lat2])
                                    D_list=len(calculated_latlon_lists)
                                calculated_latlon_lists=','.join(','.join(map(str,pair))for pair in calculated_latlon_lists)#为了符合Redis的输出格式
                                value_str = f"$GP,{D_list},{calculated_latlon_lists}"
                                redis_conn.hset("Navi", "GPath",value_str)
                                if risk and Auto_CA_sw == "1":
                                    if avoid_script is None:
                                        print("⚠️ 进入避碰模式")
                                        Planner_V = slow_V
                                        avoid_script = PathPlanningScript(Planner_V)
                                        avoid_thread = threading.Thread(
                                            target=avoid_script.run,
                                            daemon=True)
                                        avoid_thread.start()
                                else:
                                    if avoid_script is not None:
                                        print("✅ 风险解除，退出避碰")
                                        avoid_script.stop()
                                        avoid_script = None
                                        avoid_thread = None
                                    print("当前超过目标点，进入逐渐调整航速、航向阶段，追踪掉头第三个点：", Point4)
                                    Planner_V = slow_V
                                    print("当前速度",Planner_V)
                                    waypoint = [Point4, Point5, Point6]
                            elif Distance_to_Target_goal < 100:
                                if Distance_to_Target3 > 800:
                                    plan_path_points=[Point0,Point3,Point4]
                                else:
                                    plan_path_points = [Point0,Point5,Point6]
                                calculated_latlon_lists = []
                                # 这里可以把路径发往redis，让避碰直接从redis上取
                                for i in range(0, len(plan_path_points)):
                                    x2, y2 = plan_path_points[i]  # 获取路径点列表中的 (x2, y2)
                                    Lon1 = float(value_own_Lon.decode('utf-8'))
                                    Lat1 = float(value_own_Lat.decode('utf-8'))
                                    x1 = 0
                                    y1 = 0
                                    Lon2, Lat2 = xy_to_latlon(Lon1, Lat1, x1, y1, x2, y2)
                                    calculated_latlon_lists.append([Lon2, Lat2])
                                    D_list=len(calculated_latlon_lists)
                                calculated_latlon_lists=','.join(','.join(map(str,pair))for pair in calculated_latlon_lists)#为了符合Redis的输出格式
                                value_str = f"$GP,{D_list},{calculated_latlon_lists}"
                                redis_conn.hset("Navi", "GPath",value_str)
                                if risk and Auto_CA_sw == "1":
                                    if avoid_script is None:
                                        print("⚠️ 进入避碰模式")
                                        Planner_V = target_V
                                        avoid_script = PathPlanningScript(Planner_V)
                                        avoid_thread = threading.Thread(
                                            target=avoid_script.run,
                                            daemon=True)
                                        avoid_thread.start()
                                else:
                                    if avoid_script is not None:
                                        print("✅ 风险解除，退出避碰")
                                        avoid_script.stop()
                                        avoid_script = None
                                        avoid_thread = None
                                    print("当前进入调整航速、航向阶段，追踪掉头第三个点：", Point3)
                                    escort_stage = 4
                                    Planner_V = target_V
                                    # A = (Target_State[0] + Escort_distance_y * np.sin(heading_rad),
                                    #      Target_State[1] + Escort_distance_y * np.cos(heading_rad))
                                    # B = (Target_State[0], Target_State[1])
                                    # C = (Tug_State[0], Tug_State[1])
                                    # angle = calculate_angle(A, B, C)
                                    # print("夹角度数为", angle)
                                    # Planner_V = (angle / 90) * 520
                                    print("当前开始追踪掉头第三个关键路径点,当前速度为:", Planner_V)
                                    waypoint = [Point3, Point4, Point5]
                            else:
                                if Distance_to_Target3 > 800:
                                    plan_path_points=[Point0,Point3,Point4]
                                else:
                                    plan_path_points = [Point0,Point5,Point6]
                                calculated_latlon_lists = []
                                # 这里可以把路径发往redis，让避碰直接从redis上取
                                for i in range(0, len(plan_path_points)):
                                    x2, y2 = plan_path_points[i]  # 获取路径点列表中的 (x2, y2)
                                    Lon1 = float(value_own_Lon.decode('utf-8'))
                                    Lat1 = float(value_own_Lat.decode('utf-8'))
                                    x1 = 0
                                    y1 = 0
                                    Lon2, Lat2 = xy_to_latlon(Lon1, Lat1, x1, y1, x2, y2)
                                    calculated_latlon_lists.append([Lon2, Lat2])
                                    D_list=len(calculated_latlon_lists)
                                calculated_latlon_lists=','.join(','.join(map(str,pair))for pair in calculated_latlon_lists)#为了符合Redis的输出格式
                                value_str = f"$GP,{D_list},{calculated_latlon_lists}"
                                redis_conn.hset("Navi", "GPath",value_str)
                                if risk and Auto_CA_sw == "1":
                                    if avoid_script is None:
                                        print("⚠️ 进入避碰模式")
                                        Planner_V = default_V
                                        avoid_script = PathPlanningScript(Planner_V)
                                        avoid_thread = threading.Thread(
                                            target=avoid_script.run,
                                            daemon=True)
                                        avoid_thread.start()
                                else:
                                    if avoid_script is not None:
                                        print("✅ 风险解除，退出避碰")
                                        avoid_script.stop()
                                        avoid_script = None
                                        avoid_thread = None
                                    print("当前进入正向追击、逐渐调整航向阶段，追踪掉头第三个点：", Point3)
                                    Planner_V = default_V
                                    print("当前开始追踪掉头第三个关键路径点,当前速度为:", Planner_V)
                                    waypoint = [Point3, Point4, Point5]
                        else:
                            if Distance_to_Target3 > 800:
                                    plan_path_points=[Point0,Point3,Point4]
                            else:
                                plan_path_points = [Point0,Point5,Point6]
                            calculated_latlon_lists = []
                            # 这里可以把路径发往redis，让避碰直接从redis上取
                            for i in range(0, len(plan_path_points)):
                                x2, y2 = plan_path_points[i]  # 获取路径点列表中的 (x2, y2)
                                Lon1 = float(value_own_Lon.decode('utf-8'))
                                Lat1 = float(value_own_Lat.decode('utf-8'))
                                x1 = 0
                                y1 = 0
                                Lon2, Lat2 = xy_to_latlon(Lon1, Lat1, x1, y1, x2, y2)
                                calculated_latlon_lists.append([Lon2, Lat2])
                                D_list=len(calculated_latlon_lists)
                            calculated_latlon_lists=','.join(','.join(map(str,pair))for pair in calculated_latlon_lists)#为了符合Redis的输出格式
                            value_str = f"$GP,{D_list},{calculated_latlon_lists}"
                            redis_conn.hset("Navi", "GPath",value_str)
                            if risk and Auto_CA_sw == "1":
                                if avoid_script is None:
                                    print("⚠️ 进入避碰模式")
                                    Planner_V = default_V
                                    avoid_script = PathPlanningScript(Planner_V)
                                    avoid_thread = threading.Thread(
                                        target=avoid_script.run,
                                        daemon=True)
                                    avoid_thread.start()
                            else:
                                if avoid_script is not None:
                                    print("✅ 风险解除，退出避碰")
                                    avoid_script.stop()
                                    avoid_script = None
                                    avoid_thread = None
                                print("当前进入调整航速、航向阶段，追踪掉头第三个点：", Point3)
                                Planner_V = default_V
                                print("当前开始追踪掉头第三个关键路径点,当前速度为:", Planner_V)
                                waypoint = [Point3, Point4, Point5]
                        if risk == False or Auto_CA_sw == "0":
                            Own_Pos = [0, 0]
                            plan_path_points = [Own_Pos]
                            for i in range(1, len(waypoint)+1):
                                plan_path_points.append(waypoint[i-1])
                            print('规划成功,当前路径点序列：', plan_path_points)

                    if result : # 进入绕尾模式
                        print("进入绕尾模式")
                        if -Escort_distance_x > 0:
                            D = 200
                            D1 = 200
                            B1 = 30
                        else:
                            D = -200
                            D1 = 200
                            B1 = -50

                        tug_Left_Heading = Target_State[2]  # 和大船航向保持一致
                        heading_rad = math.radians(tug_Left_Heading)
                        Left_TugPosG1 = [Target_State[0] + np.cos(heading_rad) * (D),
                                            Target_State[1] - np.sin(heading_rad) * (D)]  # 第一个追踪点

                        Left_TugPosG2 = [
                            Target_State[0] - math.sin(heading_rad) * (D1),
                            Target_State[1] - math.cos(heading_rad) * (D1)]  # 第二个追踪点

                        # Left_TugPosG3 = [Target_State_x - math.cos(heading_rad) * (D),
                        #                  Target_State_y + math.sin(heading_rad) * (D)]  # 第三个追踪点

                        target_heading_rad = np.deg2rad(Target_State[2])
                        Left_TugPosG3 = [Target_State[0] + Escort_distance_y * np.sin(target_heading_rad) +
                                        Escort_distance_x * np.cos(target_heading_rad),
                                        Target_State[1] + Escort_distance_y * np.cos(target_heading_rad) -
                                        Escort_distance_x * np.sin(target_heading_rad)]
                        
                        d = 50
                        waypoint = []
                        # print('拖轮当前坐标',x_os,y_os)
                        tug_Left_Heading = Target_State[2]  # 和大船航向保持一致
                        heading_rad = math.radians(tug_Left_Heading)
                        if -Escort_distance_x > 0:
                            D = 200
                            D1 = 200
                            B1 = 30
                        else:
                            D = -200
                            D1 = 200
                            B1 = -50

                        Left_TugPosG1 = [Target_State[0] + np.cos(heading_rad) * (D),
                                        Target_State[1] - np.sin(heading_rad) * (D)]  # 第一个追踪点
                        Left_TugPosG2 = [
                            Target_State[0] - math.sin(heading_rad) * (D1),
                            Target_State[1] - math.cos(heading_rad) * (D1)]  # 第二个追踪点

                        # Left_TugPosG3 = [Target_State_x - math.cos(heading_rad) * (D),
                        #                  Target_State_y + math.sin(heading_rad) * (D)]  # 第三个追踪点

                        target_heading_rad = np.deg2rad(Target_State[2])
                        Left_TugPosG3 = [Target_State[0] + Escort_distance_y * np.sin(target_heading_rad) +
                                    Escort_distance_x * np.cos(target_heading_rad),
                                    Target_State[1] + Escort_distance_y * np.cos(target_heading_rad) -
                                    Escort_distance_x * np.sin(target_heading_rad)]


                        Distance_to_Target1 = math.sqrt(np.square(Left_TugPosG1[0] - x_os) + np.square(Left_TugPosG1[1] - y_os))  # 与第一个点的距离
                        Distance_to_Target2 = math.sqrt(np.square(Left_TugPosG2[0] - x_os) + np.square(Left_TugPosG2[1] - y_os))  # 与第二个点的距离
                        Distance_to_Target3 = math.sqrt(np.square(Left_TugPosG3[0] - x_os) + np.square(Left_TugPosG3[1] - y_os))  # 与第三个点的距离

                        Point0 = [0, 0]
                        Point1 = [Left_TugPosG1[0], Left_TugPosG1[1]]
                        Point2 = [Left_TugPosG2[0], Left_TugPosG2[1]]
                        Point3 = [Left_TugPosG3[0], Left_TugPosG3[1]]
                        Point4 = [Left_TugPosG3[0] + 6 * d * math.sin(heading_rad), Left_TugPosG3[1] + 6 * d * math.cos(heading_rad)]
                        Point5 = [Left_TugPosG3[0] + 12 * d * math.sin(heading_rad), Left_TugPosG3[1] + 12 * d * math.cos(heading_rad)]
                        Point6 = [Left_TugPosG3[0] + 18 * d * math.sin(heading_rad), Left_TugPosG3[1] + 18 * d * math.cos(heading_rad)]
                        Distance_to_Target_goal = math.sqrt(
                            np.square(Left_TugPosG3[0] - x_os) + np.square(Left_TugPosG3[1] - y_os))
                        # 假设四个点的坐标为 A(x, y), B(x1, y1), C(x2, y2), D(x3, y3)
                        # 计算向量的表示
                        vector_AB = (Left_TugPosG1[0] - x_os, Left_TugPosG1[1] - y_os)
                        vector_AC = (Left_TugPosG2[0] - x_os, Left_TugPosG2[1] - y_os)
                        vector_AD = (Left_TugPosG3[0] - x_os, Left_TugPosG3[1] - y_os)

                        # 计算叉乘结果
                        cross_product_AC_AB = vector_AC[0] * vector_AB[1] - vector_AC[1] * vector_AB[0]
                        # print('ACxAB',cross_product_AC_AB)
                        cross_product_AC_AD = vector_AC[0] * vector_AD[1] - vector_AC[1] * vector_AD[0]
                        # print('ACxAD',cross_product_AC_AD)
                        if D < 0:
                            cross_product_AC_AD = -cross_product_AC_AD
                            cross_product_AC_AB = -cross_product_AC_AB

                        if Target_State[2] < (cog_os - 180):
                            Target_State[2] = Target_State[2] + 360
                        if cog_os < (Target_State[2] - 180):
                            Target_State[2] = Target_State[2] - 360
                        print("abs(Target_State[2] - cog_os)",abs(Target_State[2] - cog_os))
                        print("cross_product_AC_AB",cross_product_AC_AB)

                        A=(Left_TugPosG1[0], Left_TugPosG1[1])
                        B=(0, 0)
                        C=(Left_TugPosG3[0], Left_TugPosG3[1])
                        angle_2_nei = calculate_angle(A, B, C)

                        result_to_Left_TugPosG2 = ip_is_intersecting_line_segment.is_intersecting_line_segment(
                        Tug_State[0], Tug_State[1],
                        Escort_Pos[0], Escort_Pos[1],
                        Left_TugPosG2[0] - 10000 * np.sin(target_heading_rad),
                        Left_TugPosG2[1] - 10000 * np.cos(target_heading_rad),
                        Left_TugPosG2[0] + 10000 * np.sin(target_heading_rad),
                        Left_TugPosG2[1] + 10000 * np.cos(target_heading_rad))
                        # 判断线段AC的位置情况
                        if cross_product_AC_AB > 0 and cross_product_AC_AD < 0 and Distance_to_Target3 > 20 and abs(Target_State[2]-cog_os)>100:
                            plan_path_points= [Point0, Point1, Point2, Point3, Point4, Point5]
                            calculated_latlon_lists = []
                            # 这里可以把路径发往redis，让避碰直接从redis上取
                            for i in range(0, len(plan_path_points)):
                                x2, y2 = plan_path_points[i]  # 获取路径点列表中的 (x2, y2)
                                Lon1 = float(value_own_Lon.decode('utf-8'))
                                Lat1 = float(value_own_Lat.decode('utf-8'))
                                x1 = 0
                                y1 = 0
                                Lon2, Lat2 = xy_to_latlon(Lon1, Lat1, x1, y1, x2, y2)
                                calculated_latlon_lists.append([Lon2, Lat2])
                                D_list=len(calculated_latlon_lists)
                            calculated_latlon_lists=','.join(','.join(map(str,pair))for pair in calculated_latlon_lists)#为了符合Redis的输出格式
                            value_str = f"$GP,{D_list},{calculated_latlon_lists}"
                            redis_conn.hset("Navi", "GPath",value_str)
                            if risk and Auto_CA_sw == "1":
                                if avoid_script is None:
                                    print("⚠️ 进入避碰模式")
                                    Planner_V = default_V
                                    avoid_script = PathPlanningScript(Planner_V)
                                    avoid_thread = threading.Thread(
                                        target=avoid_script.run,
                                        daemon=True)
                                    avoid_thread.start()
                            else:
                                if avoid_script is not None:
                                    print("✅ 风险解除，退出避碰")
                                    avoid_script.stop()
                                    avoid_script = None
                                    avoid_thread = None
                                waypoint = [Point1, Point2, Point3, Point4, Point5]
                                Planner_V = default_V
                                print("当前追踪绕尾第一个路径点1",Point1)
                                print("当前追踪绕尾第一个路径点,当前航速为",Planner_V)
                            A=(Left_TugPosG1[0], Left_TugPosG1[1])
                            B=(0, 0)
                            C=(Left_TugPosG3[0], Left_TugPosG3[1])
                            angle = calculate_angle(A, B, C)  # 计算AB和AC夹角
                            if angle < 90: # 解决第一区域半圆内部问题
                                calculated_latlon_lists = []
                                plan_path_points=[Point0,Point2,Point3,Point4,Point5]
                                # 这里可以把路径发往redis，让避碰直接从redis上取
                                for i in range(0, len(plan_path_points)):
                                    x2, y2 = plan_path_points[i]  # 获取路径点列表中的 (x2, y2)
                                    Lon1 = float(value_own_Lon.decode('utf-8'))
                                    Lat1 = float(value_own_Lat.decode('utf-8'))
                                    x1 = 0
                                    y1 = 0
                                    Lon2, Lat2 = xy_to_latlon(Lon1, Lat1, x1, y1, x2, y2)
                                    calculated_latlon_lists.append([Lon2, Lat2])
                                    D_list=len(calculated_latlon_lists)
                                calculated_latlon_lists=','.join(','.join(map(str,pair))for pair in calculated_latlon_lists)#为了符合Redis的输出格式
                                value_str = f"$GP,{D_list},{calculated_latlon_lists}"
                                redis_conn.hset("Navi", "GPath",value_str)
                                if risk and Auto_CA_sw == "1":
                                    if avoid_script is None:
                                        print("⚠️ 进入避碰模式")
                                        if Distance_to_Target2 > 100:
                                                Planner_V=default_V
                                        else:
                                                Planner_V=turn_V
                                        avoid_script = PathPlanningScript(Planner_V)
                                        avoid_thread = threading.Thread(
                                            target=avoid_script.run,
                                            daemon=True)
                                        avoid_thread.start()
                                else:
                                    if avoid_script is not None:
                                        print("✅ 风险解除，退出避碰")
                                        avoid_script.stop()
                                        avoid_script = None
                                        avoid_thread = None
                                    waypoint = [Point2,Point3, Point4, Point5]
                                    if Distance_to_Target2 > 100:
                                            Planner_V=default_V
                                    else:
                                            Planner_V=turn_V
                                    print("当前与跟踪船横距太近,直接追踪绕尾第二个点,当前航速为",Planner_V) # 本来是第二个点，改为第三个点

                        elif cross_product_AC_AB < 0 and Distance_to_Target2 > 10 and abs(Target_State[2] - cog_os)> 50: 
                            plan_path_points= [Point0, Point2, Point3, Point4, Point5]
                            calculated_latlon_lists = []
                            # 这里可以把路径发往redis，让避碰直接从redis上取
                            for i in range(0, len(plan_path_points)):
                                x2, y2 = plan_path_points[i]  # 获取路径点列表中的 (x2, y2)
                                Lon1 = float(value_own_Lon.decode('utf-8'))
                                Lat1 = float(value_own_Lat.decode('utf-8'))
                                x1 = 0
                                y1 = 0
                                Lon2, Lat2 = xy_to_latlon(Lon1, Lat1, x1, y1, x2, y2)
                                calculated_latlon_lists.append([Lon2, Lat2])
                                D_list=len(calculated_latlon_lists)
                            calculated_latlon_lists=','.join(','.join(map(str,pair))for pair in calculated_latlon_lists)#为了符合Redis的输出格式
                            value_str = f"$GP,{D_list},{calculated_latlon_lists}"
                            redis_conn.hset("Navi", "GPath",value_str)
                            if risk and Auto_CA_sw == "1":
                                if avoid_script is None:
                                    print("⚠️ 进入避碰模式")
                                    if Distance_to_Target2 > 200:
                                        Planner_V = default_V
                                    else:
                                        Planner_V = turn_V
                                    avoid_script = PathPlanningScript(Planner_V)
                                    avoid_thread = threading.Thread(
                                        target=avoid_script.run,
                                        daemon=True)
                                    avoid_thread.start()
                            else:
                                if avoid_script is not None:
                                    print("✅ 风险解除，退出避碰")
                                    avoid_script.stop()
                                    avoid_script = None
                                    avoid_thread = None
                                waypoint = [Point2, Point3, Point4, Point5]
                                if Distance_to_Target2 > 200:
                                    Planner_V = default_V
                                else:
                                    Planner_V = turn_V
                                print("当前追踪绕尾第二个路径点",Point2)
                                print("当前追踪绕尾第二个路径点,当前航速为",Planner_V)

                        # 新加逻辑 解决第二区域半圆内部问题
                        elif cross_product_AC_AB > 0 and  angle_2_nei < 45 and result_to_Left_TugPosG2: 
                            plan_path_points= [Point0, Point2, Point3, Point4, Point5]
                            calculated_latlon_lists = []
                            # 这里可以把路径发往redis，让避碰直接从redis上取
                            for i in range(0, len(plan_path_points)):
                                x2, y2 = plan_path_points[i]  # 获取路径点列表中的 (x2, y2)
                                Lon1 = float(value_own_Lon.decode('utf-8'))
                                Lat1 = float(value_own_Lat.decode('utf-8'))
                                x1 = 0
                                y1 = 0
                                Lon2, Lat2 = xy_to_latlon(Lon1, Lat1, x1, y1, x2, y2)
                                calculated_latlon_lists.append([Lon2, Lat2])
                                D_list=len(calculated_latlon_lists)
                            calculated_latlon_lists=','.join(','.join(map(str,pair))for pair in calculated_latlon_lists)#为了符合Redis的输出格式
                            value_str = f"$GP,{D_list},{calculated_latlon_lists}"
                            redis_conn.hset("Navi", "GPath",value_str)
                            if risk and Auto_CA_sw == "1":
                                if avoid_script is None:
                                    print("⚠️ 进入避碰模式")
                                    if Distance_to_Target2 > 200:
                                        Planner_V = default_V
                                    else:
                                        Planner_V = turn_V
                                    avoid_script = PathPlanningScript(Planner_V)
                                    avoid_thread = threading.Thread(
                                        target=avoid_script.run,
                                        daemon=True)
                                    avoid_thread.start()
                            else:
                                if avoid_script is not None:
                                    print("✅ 风险解除，退出避碰")
                                    avoid_script.stop()
                                    avoid_script = None
                                    avoid_thread = None
                                waypoint = [Point2, Point3, Point4, Point5]
                                if Distance_to_Target2 > 200:
                                    Planner_V = default_V
                                else:
                                    Planner_V = turn_V
                                print("当前追踪绕尾第二个路径点11",Point2)
                                print("当前追踪绕尾第二个路径点,当前航速为",Planner_V)

                        elif cross_product_AC_AD > 0 or Distance_to_Target2 < 10 or abs(Target_State[2] - cog_os) <= 50:
                            judge = math.atan2(x_os - Left_TugPosG3[0], y_os - Left_TugPosG3[1]) * 180 / math.pi % 360
                            if judge < (Target_State[2] - 180):
                                judge = judge + 360
                            if Target_State[2] < (judge - 180):
                                judge = judge - 360
                            # if abs(judge - Target_State[2]) > 90 and Distance_to_Target3 > 200:  # 从后方追近，且距离较远时加速，距离近时再减速，方位判断属于第三区域时就追踪第三个点
                            A=(Left_TugPosG2[0], Left_TugPosG2[1])
                            B=(0, 0)
                            C=(Left_TugPosG3[0], Left_TugPosG3[1])
                            angle = calculate_angle(A, B, C)  # 计算AB和AC夹角
                            if angle > 90 and Distance_to_Target3 > 200:  # 从后方追近，且距离较远时加速，距离近时再减速，方位判断属于第三区域时就追踪第三个点
                                if Distance_to_Target3 > 800:
                                    plan_path_points=[Point0,Point3,Point4]
                                else:
                                    plan_path_points = [Point0,Point5,Point6]
                                calculated_latlon_lists = []
                                # 这里可以把路径发往redis，让避碰直接从redis上取
                                for i in range(0, len(plan_path_points)):
                                    x2, y2 = plan_path_points[i]  # 获取路径点列表中的 (x2, y2)
                                    Lon1 = float(value_own_Lon.decode('utf-8'))
                                    Lat1 = float(value_own_Lat.decode('utf-8'))
                                    x1 = 0
                                    y1 = 0
                                    Lon2, Lat2 = xy_to_latlon(Lon1, Lat1, x1, y1, x2, y2)
                                    calculated_latlon_lists.append([Lon2, Lat2])
                                    D_list=len(calculated_latlon_lists)
                                calculated_latlon_lists=','.join(','.join(map(str,pair))for pair in calculated_latlon_lists)#为了符合Redis的输出格式
                                value_str = f"$GP,{D_list},{calculated_latlon_lists}"
                                redis_conn.hset("Navi", "GPath",value_str)
                                if risk and Auto_CA_sw == "1":
                                    if avoid_script is None:
                                        print("⚠️ 进入避碰模式")
                                        Planner_V=default_V
                                        avoid_script = PathPlanningScript(Planner_V)
                                        avoid_thread = threading.Thread(
                                            target=avoid_script.run,
                                            daemon=True)
                                        avoid_thread.start()
                                else:
                                    if avoid_script is not None:
                                        print("✅ 风险解除，退出避碰")
                                        avoid_script.stop()
                                        avoid_script = None
                                        avoid_thread = None
                                    Planner_V=default_V
                                    print("当前开始追踪绕尾第三个关键路径点,当前速度为:", Planner_V)
                                    print("此时从目标船远距离追进不减速,追踪绕尾第三个点", Point3)
                                    waypoint = [Point3, Point4, Point5]
                            elif abs(Target_State[2] - cog_os) > 30:
                                if Distance_to_Target3 > 800:
                                    plan_path_points=[Point0,Point3,Point4]
                                else:
                                    plan_path_points = [Point0,Point5,Point6]
                                calculated_latlon_lists = []
                                # 这里可以把路径发往redis，让避碰直接从redis上取
                                for i in range(0, len(plan_path_points)):
                                    x2, y2 = plan_path_points[i]  # 获取路径点列表中的 (x2, y2)
                                    Lon1 = float(value_own_Lon.decode('utf-8'))
                                    Lat1 = float(value_own_Lat.decode('utf-8'))
                                    x1 = 0
                                    y1 = 0
                                    Lon2, Lat2 = xy_to_latlon(Lon1, Lat1, x1, y1, x2, y2)
                                    calculated_latlon_lists.append([Lon2, Lat2])
                                    D_list=len(calculated_latlon_lists)
                                calculated_latlon_lists=','.join(','.join(map(str,pair))for pair in calculated_latlon_lists)#为了符合Redis的输出格式
                                value_str = f"$GP,{D_list},{calculated_latlon_lists}"
                                redis_conn.hset("Navi", "GPath",value_str)
                                if risk and Auto_CA_sw == "1":
                                    if avoid_script is None:
                                        print("⚠️ 进入避碰模式")
                                        if Distance_to_Target3 > 200:
                                            Planner_V = default_V
                                        else:
                                            Planner_V = turn_V
                                        avoid_script = PathPlanningScript(Planner_V)
                                        avoid_thread = threading.Thread(
                                            target=avoid_script.run,
                                            daemon=True)
                                        avoid_thread.start()
                                else:
                                    if avoid_script is not None:
                                        print("✅ 风险解除，退出避碰")
                                        avoid_script.stop()
                                        avoid_script = None
                                        avoid_thread = None
                                    print("当前进入低速转向模式，单舵归零，另一个舵保持，追踪绕尾第三个点：", Point3)
                                    if Distance_to_Target3 > 200:
                                        Planner_V = default_V
                                    else:
                                        Planner_V = turn_V
                                    print("当前开始追踪绕尾第三个关键路径点,当前速度为:", Planner_V)
                                    waypoint = [Point3, Point4, Point5]
                                
                            elif 0 < abs(judge - Target_State[2]) <= 90:
                                if Distance_to_Target3 > 800:
                                    plan_path_points=[Point0,Point3,Point4]
                                else:
                                    plan_path_points = [Point0,Point5,Point6]
                                calculated_latlon_lists = []
                                # 这里可以把路径发往redis，让避碰直接从redis上取
                                for i in range(0, len(plan_path_points)):
                                    x2, y2 = plan_path_points[i]  # 获取路径点列表中的 (x2, y2)
                                    Lon1 = float(value_own_Lon.decode('utf-8'))
                                    Lat1 = float(value_own_Lat.decode('utf-8'))
                                    x1 = 0
                                    y1 = 0
                                    Lon2, Lat2 = xy_to_latlon(Lon1, Lat1, x1, y1, x2, y2)
                                    calculated_latlon_lists.append([Lon2, Lat2])
                                    D_list=len(calculated_latlon_lists)
                                calculated_latlon_lists=','.join(','.join(map(str,pair))for pair in calculated_latlon_lists)#为了符合Redis的输出格式
                                value_str = f"$GP,{D_list},{calculated_latlon_lists}"
                                redis_conn.hset("Navi", "GPath",value_str)
                                if risk and Auto_CA_sw == "1":
                                    if avoid_script is None:
                                        print("⚠️ 进入避碰模式")
                                        Planner_V = slow_V
                                        avoid_script = PathPlanningScript(Planner_V)
                                        avoid_thread = threading.Thread(
                                            target=avoid_script.run,
                                            daemon=True)
                                        avoid_thread.start()
                                else:
                                    if avoid_script is not None:
                                        print("✅ 风险解除，退出避碰")
                                        avoid_script.stop()
                                        avoid_script = None
                                        avoid_thread = None
                                    print("当前超过目标点，进入逐渐调整航速、航向阶段，追踪绕尾第三个点：", Point4)
                                    Planner_V = slow_V
                                    print("当前速度",Planner_V)
                                    waypoint = [Point4, Point5, Point6]
                            elif Distance_to_Target3 < 100:
                                if Distance_to_Target3 > 800:
                                    plan_path_points=[Point0,Point3,Point4]
                                else:
                                    plan_path_points = [Point0,Point5,Point6]
                                calculated_latlon_lists = []
                                # 这里可以把路径发往redis，让避碰直接从redis上取
                                for i in range(0, len(plan_path_points)):
                                    x2, y2 = plan_path_points[i]  # 获取路径点列表中的 (x2, y2)
                                    Lon1 = float(value_own_Lon.decode('utf-8'))
                                    Lat1 = float(value_own_Lat.decode('utf-8'))
                                    x1 = 0
                                    y1 = 0
                                    Lon2, Lat2 = xy_to_latlon(Lon1, Lat1, x1, y1, x2, y2)
                                    calculated_latlon_lists.append([Lon2, Lat2])
                                    D_list=len(calculated_latlon_lists)
                                calculated_latlon_lists=','.join(','.join(map(str,pair))for pair in calculated_latlon_lists)#为了符合Redis的输出格式
                                value_str = f"$GP,{D_list},{calculated_latlon_lists}"
                                redis_conn.hset("Navi", "GPath",value_str)
                                if risk and Auto_CA_sw == "1":
                                    if avoid_script is None:
                                        print("⚠️ 进入避碰模式")
                                        Planner_V = target_V
                                        avoid_script = PathPlanningScript(Planner_V)
                                        avoid_thread = threading.Thread(
                                            target=avoid_script.run,
                                            daemon=True)
                                        avoid_thread.start()
                                else:
                                    if avoid_script is not None:
                                        print("✅ 风险解除，退出避碰")
                                        avoid_script.stop()
                                        avoid_script = None
                                        avoid_thread = None
                                    print("当前进入调整航速、航向阶段，追踪绕尾第三个点：", Point3)
                                    Planner_V = target_V
                                    print("当前开始追踪绕尾第三个关键路径点,当前速度为:", Planner_V)
                                    waypoint = [Point3, Point4, Point5]
                            else:
                                if Distance_to_Target3 > 800:
                                    plan_path_points=[Point0,Point3,Point4]
                                else:
                                    plan_path_points = [Point0,Point5,Point6]
                                calculated_latlon_lists = []
                                # 这里可以把路径发往redis，让避碰直接从redis上取
                                for i in range(0, len(plan_path_points)):
                                    x2, y2 = plan_path_points[i]  # 获取路径点列表中的 (x2, y2)
                                    Lon1 = float(value_own_Lon.decode('utf-8'))
                                    Lat1 = float(value_own_Lat.decode('utf-8'))
                                    x1 = 0
                                    y1 = 0
                                    Lon2, Lat2 = xy_to_latlon(Lon1, Lat1, x1, y1, x2, y2)
                                    calculated_latlon_lists.append([Lon2, Lat2])
                                    D_list=len(calculated_latlon_lists)
                                calculated_latlon_lists=','.join(','.join(map(str,pair))for pair in calculated_latlon_lists)#为了符合Redis的输出格式
                                value_str = f"$GP,{D_list},{calculated_latlon_lists}"
                                redis_conn.hset("Navi", "GPath",value_str)
                                if risk and Auto_CA_sw == "1":
                                    if avoid_script is None:
                                        print("⚠️ 进入避碰模式")
                                        Planner_V = default_V
                                        avoid_script = PathPlanningScript(Planner_V)
                                        avoid_thread = threading.Thread(
                                            target=avoid_script.run,
                                            daemon=True)
                                        avoid_thread.start()
                                else:
                                    if avoid_script is not None:
                                        print("✅ 风险解除，退出避碰")
                                        avoid_script.stop()
                                        avoid_script = None
                                        avoid_thread = None
                                    Planner_V = default_V
                                    print("当前开始追踪绕尾第三个关键路径点,当前速度为:", Planner_V)
                                    waypoint = [Point3, Point4, Point5]
                        else:
                            if Distance_to_Target3 > 800:
                                    plan_path_points=[Point0,Point3,Point4]
                            else:
                                plan_path_points = [Point0,Point5,Point6]
                            calculated_latlon_lists = []
                            # 这里可以把路径发往redis，让避碰直接从redis上取
                            for i in range(0, len(plan_path_points)):
                                x2, y2 = plan_path_points[i]  # 获取路径点列表中的 (x2, y2)
                                Lon1 = float(value_own_Lon.decode('utf-8'))
                                Lat1 = float(value_own_Lat.decode('utf-8'))
                                x1 = 0
                                y1 = 0
                                Lon2, Lat2 = xy_to_latlon(Lon1, Lat1, x1, y1, x2, y2)
                                calculated_latlon_lists.append([Lon2, Lat2])
                                D_list=len(calculated_latlon_lists)
                            calculated_latlon_lists=','.join(','.join(map(str,pair))for pair in calculated_latlon_lists)#为了符合Redis的输出格式
                            value_str = f"$GP,{D_list},{calculated_latlon_lists}"
                            redis_conn.hset("Navi", "GPath",value_str)
                            if risk and Auto_CA_sw == "1":
                                if avoid_script is None:
                                    print("⚠️ 进入避碰模式")
                                    Planner_V = default_V
                                    avoid_script = PathPlanningScript(Planner_V)
                                    avoid_thread = threading.Thread(
                                        target=avoid_script.run,
                                        daemon=True)
                                    avoid_thread.start()
                            else:
                                if avoid_script is not None:
                                    print("✅ 风险解除，退出避碰")
                                    avoid_script.stop()
                                    avoid_script = None
                                    avoid_thread = None
                            
                                print("当前进入调整航速、航向阶段，追踪绕尾第三个点1：", Point3)
                                Planner_V = default_V
                                print("当前开始追踪绕尾第三个关键路径点,当前速度为:", Planner_V)
                                waypoint = [Point3, Point4, Point5]
                                
                        if risk == False or Auto_CA_sw == "0":
                            Own_Pos = [0, 0]
                            plan_path_points = [Own_Pos]
                            for i in range(1, len(waypoint)+1):
                                plan_path_points.append(waypoint[i-1])
                            print('规划成功,当前路径点序列：', plan_path_points)
                    own = (Tug_State[0],Tug_State[1])
                    angle = bearing_from_north_clockwise(own,plan_path_points[1])
                    if abs(Tug_State[2]-angle) > 10: # 本船航向与线段航向差小于
                        # print("angle",(abs(Tug_State[2]-angle)))
                        theta_point = bearing_from_north_clockwise_xy(plan_path_points[1],  plan_path_points[2])
                        theta_point = (90 - theta_point) % 360 # 正北方向角度转为正东方向角度
                        theta_own = (90 - Heading) % 360 # 正北方向角度转为正东方向角度
                        sx = 0.0  
                        sy = 0.0 
                        s_yaw = np.radians(theta_own) 
                    
                        gx = plan_path_points[1][0]  
                        gy = plan_path_points[1][1]  
                        g_yaw = np.radians(theta_point) 
                        offset = 5.0

                        path, control_points = ip_bezier_path.calc_4points_bezier_path(
                            sx, sy, s_yaw, gx, gy, g_yaw, offset)
                        plan_path_points_Bezier = list(zip(path.T[0], path.T[1])) 
                        # plan_path_points_Bezier = list(zip(path.T[0][:10], path.T[1][:10]))
                        plan_path_points_Bezier = filter_path_by_distance(plan_path_points_Bezier, min_dist=25.0)
                        if len(plan_path_points) > 4:
                            plan_path_points_Bezier.extend([Point2, Point3, Point4, Point5])
                            plan_path_points = plan_path_points_Bezier
                        else:
                            plan_path_points_Bezier.extend([Point4, Point5])
                            plan_path_points = plan_path_points_Bezier
                    else:
                        plan_path_points = plan_path_points
                    calculated_latlon_lists = []
                    if risk == False or Auto_CA_sw == "0":
                        write_speed_control_mode(redis_conn,2)
                        for i in range(0, len(plan_path_points)):
                            x2, y2 = plan_path_points[i]  # 获取路径点列表中的 (x2, y2)
                            Lon1 = float(value_own_Lon.decode('utf-8'))
                            Lat1 = float(value_own_Lat.decode('utf-8'))
                            x1 = 0
                            y1 = 0
                            Lon2, Lat2 = xy_to_latlon(Lon1, Lat1, x1, y1, x2, y2)
                            calculated_latlon_lists.append([Lon2, Lat2])
                            D_list=len(calculated_latlon_lists)
                        calculated_latlon_lists=','.join(','.join(map(str,pair))for pair in calculated_latlon_lists)#为了符合Redis的输出格式
                        if target_distance < 80: # 距离过近直接停船  # 新加0305
                            value_str = f"$LP,{0},{D_list},{calculated_latlon_lists}"
                            redis_conn.hset("Navi", "LPath",value_str)
                        else:
                            value_str = f"$LP,{Planner_V},{D_list},{calculated_latlon_lists}"
                            redis_conn.hset("Navi", "LPath",value_str)
                    else:
                        write_speed_control_mode(redis_conn,2)
                    time.sleep(2)
                else:
                    if not has_deleted_lpath:
                        redis_conn.hdel("Navi", "LPath")
                        # write_speed_control_mode(redis_conn, 0) # 新加0305
                        has_deleted_lpath = True
                        print("自主跟随模式未启动,等待中...")
            except TargetLostError as e:
                if not target_lost:
                    logging.warning(f"TargetLostError: {e}")
                    print("⚠️ 目标船丢失，进入 LOST 状态：", e)
                    if avoid_script is not None:
                        print("⛔ 强制停止避碰线程")
                        avoid_script.stop()
                        avoid_script = None
                        avoid_thread = None
                    redis_conn.hdel("Navi", "LPath")
                    # write_speed_control_mode(redis_conn, 0) # 新加0305
                    has_deleted_lpath = True
                    target_lost = True
                time.sleep(1)
                continue
            except Exception as e:
                logging.exception(
                    f"未捕获异常 | status={status if 'status' in locals() else 'N/A'} "
                    f"| lockedID={lockedID if 'lockedID' in locals() else 'N/A'}")
                print("❌ 程序发生未预期异常：", e)
                # write_speed_control_mode(redis_conn, 0) # 新加0305
                redis_conn.hdel("Navi", "LPath")    
                time.sleep(1)
                continue
    except KeyboardInterrupt:
        print("🛑 程序被用户手动停止 (Ctrl+C)")
        # write_speed_control_mode(redis_conn, 0) # 新加0305
        redis_conn.hdel("Navi", "LPath")
else:
    print("Redis未连接，无法获取数据")










