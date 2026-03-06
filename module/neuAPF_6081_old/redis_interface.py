
import copy
import time
import redis
import socket
import warnings
import threading
import numpy as np
import pandas as pd

from ownship import Ship
from algo_utility import convert_dxy_to_lonlat, convert_abs_xy_to_lonlat, convert_lonlat_to_rel_xy, closest_point_on_line
from algo_utility import convert_lonlat_to_abs_xy, cal_distance, norm, find_current_segment_index, perpendicular_distance
from para import unit_to_meter, redis_host, udp_targeting_host



def init_redis():
    host = redis_host
    port = 6379
    db =  0
    password =  None

    # 建立 Redis 连接
    redis_conn = redis.Redis(host=host, port=port, db=db, password=password)

    # 可以在此处进行一些连接测试或其他初始化操作
    try:
        redis_conn.ping()  # 测试连接是否正常
        print("Redis connection established successfully.")
    except redis.ConnectionError as e:
        print(f"Error connecting to Redis: {e}")
    return redis_conn

def get_or_default(redis_conn, key, field, default='0'):
    # 从 Redis 获取字段值
    value = redis_conn.hget(key, field)

    # 检查值是否为 None 或空字符串，如果是，则返回默认值
    if value is None or value.decode('utf-8') == '':
        return default

    # 返回获取的值，解码为字符串
    return value.decode('utf-8')


def get_default_speed(redis_conn, default='2'):
    sim_RPM = get_USVSIM_rpm(redis_conn)
    if sim_RPM is not None:  
        if sim_RPM==450:
            default_speed = 2
        elif 450<sim_RPM<=600:
            default_speed = 4
        elif 600<sim_RPM<=700:
            default_speed = 6
        elif 700<sim_RPM:
            default_speed = 8
        else:
            default_speed = 0
    else:
        default_speed = float(get_or_default(redis_conn, "Navi", "TargetSpeed", default=default)) * 2
    return default_speed

def get_USVSIM_rpm(redis_conn, default=None):
    sim_RPM = get_or_default(redis_conn, "USV_SIM", "TargetRPM", default=default)
    if sim_RPM is None:
        return None
    return float(sim_RPM)

def get_sim_speed(redis_conn, default='1'):
    return float(get_or_default(redis_conn, "Navi", "Sim_speed_scaler", default=default))

def read_target_data(redis_conn, ownship, u2m=unit_to_meter):
    # 获取所有键以"data:"为前缀的键
    keys = redis_conn.keys("data:*")
    # 创建空的DataFrame
    df = pd.DataFrame(columns=['t_idx','lon', 'lat', 'speed', 'heading','dcpa', 'tcpa', 'azimuth', 'distance', 'alarm'])
    for key in keys:
        # 从Redis中获取键对应的值
        value = redis_conn.hgetall(key)
        # 解码字节字符串为Unicode字符串
        value = {k.decode(): v.decode() for k, v in value.items()}
        # if(len(value) == 10):
            # 将数据逐行添加到DataFrame
        if "speed" not in value or float(value["speed"]) < 1.5:
            value["speed"] = str(0.1)
        if "longitude" in value:
            df.loc[len(df)] = [
                int(key.decode().split(":")[1]),
                float(value["longitude"]),
                float(value["latitude"]),
                float(value["speed"])*0.5144444/u2m,
                float(value["direction"]),
                float(value["cpDistance"]),
                float(value["cpTime"]),
                float(value["azimuth"]),
                float(value["distance"]),      
                int(value["Alarmstufe"])
                ]
            
    df = df.loc[~((df['lon'] == 1.0) & (df['lat'] == 1.0))]

    if not df.empty:
        # 应用函数并将结果保存到新的列
        df['x'], df['y'] = zip(*df.apply(lambda row: convert_lonlat_to_rel_xy((row['lon'],row['lat']), ownship.start_lonlat, u2m), axis=1))
        # 计算新的列 u 和 v
        df['u'] = df['speed'] * np.sin(np.radians(df['heading']))
        df['v'] = df['speed'] * np.cos(np.radians(df['heading']))
        # 删除指定列
        # df.drop(columns=['speed', 'heading'], inplace=True)
        pd.set_option('float_format', '{:.10f}'.format)
        # Ensure t_idx column is of type int
        df['t_idx'] = df['t_idx'].astype(int)
    # print(df.head(20))
    # remove row whose longitude and latitude are 1  
    return df


def read_global_path(redis_conn, u2m=unit_to_meter):
    gp_lonlat_str = get_or_default(redis_conn, "Navi", "GPath", default='$GP,0')
    gp_list = gp_lonlat_str.split(',')
    gp_list = [x for x in gp_list if x is not None and x !='']
    head = gp_list[0]
    path = []
    if head == '$GP' and len(gp_list[2:])==int(gp_list[1])*2: 
        for i in range(2, len(gp_list), 2):
            # path.append(convert_lonlat_to_abs_xy((float(gp_list[i]), float(gp_list[i + 1])), u2m))
            path.append((float(gp_list[i]), float(gp_list[i + 1])))
    return path


def read_ownship(redis_conn, u2m=unit_to_meter):
    own_speed = float(get_or_default(redis_conn, "IMU", "speed", default='0'))
    own_lonlat_lon = float(get_or_default(redis_conn, "IMU", "Lon", default='0'))
    own_lonlat_lat = float(get_or_default(redis_conn, "IMU", "Lat", default='0'))
    own_heading = float(get_or_default(redis_conn, "IMU", "angle", default='0'))
    own_lonlat = (own_lonlat_lon, own_lonlat_lat)
    ownship = Ship(own_pos_lonlat=own_lonlat, abs_speed=own_speed, heading=own_heading, u2m=u2m)
    return ownship

def read_status_bytes(redis_conn):
    status_bytes = redis_conn.hget("Auto_Navi_Mode", "status")
    status = status_bytes.decode('utf-8')
    return status

def read_Auto_CA_sw(redis_conn):
    auto_ca_sw_bytes = redis_conn.hget("UI", "Auto_CA_sw")
    Auto_CA_sw = auto_ca_sw_bytes.decode('utf-8')
    Auto_CA_sw = int(Auto_CA_sw)
    return Auto_CA_sw

def write_local_path(redis_conn, path_str):
    # 将路径字符串发送到Redis
    redis_conn.hset("Navi", "LPath", path_str)

# send udp message to port 5000
def send_udp_message(message, ip=udp_targeting_host, port=9090):
    # 创建套接字
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        # 发送消息
        s.sendto(message.encode(), (ip, port))

def set_stop(redis_conn):
    if redis_conn.hget("Navi", "State").decode("utf-8") == "1":
        redis_conn.hset("MotorCtrl", "Stop", 1)
        redis_conn.expire("MotorCtrl", 30)

def reset_stop(redis_conn):
    if redis_conn.hget("Navi", "State").decode("utf-8") == "0":
        redis_conn.hdel("MotorCtrl", "Stop")

def get_navi_state(redis_conn):
    navi_state = redis_conn.hget("Navi", "State")
    if navi_state is None:
        return 0
    return int(navi_state.decode("utf-8"))

def get_rc_state(redis_conn):
    rc_state = redis_conn.hget("Navi", "RC")
    if rc_state is None:
        return 0
    return int(rc_state.decode("utf-8"))