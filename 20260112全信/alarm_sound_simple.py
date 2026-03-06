import os
import sys
import time
import redis
import signal
import pygame
import pyproj
import numpy as np
import tkinter as tk
from threading import Thread
from algo_utility import find_current_segment_index, closest_point_on_line, convert_lonlat_to_abs_xy, perpendicular_distance

redis_conn = None
terminate_flag = False

def init_redis():
    global redis_conn
    host = 'localhost'
    port = 6379
    db =  0
    password = None
    redis_conn = redis.Redis(host=host, port=port, db=db, password=password)
    try:
        redis_conn.ping()
        print("Redis connection established successfully.")
    except redis.ConnectionError as e:
        print(f"Error connecting to Redis: {e}")

def get_or_default(redis_conn, key, field, default='0'):
    # 从 Redis 获取字段值
    value = redis_conn.hget(key, field)

    # 检查值是否为 None 或空字符串，如果是，则返回默认值
    if value is None or value.decode('utf-8') == '':
        return default

    # 返回获取的值，解码为字符串
    return value.decode('utf-8')


mymixer = pygame.mixer
# mymixer.pre_init(devicename='directsound')
mymixer.init()
init_redis()

def signal_handler(sig=None, frame=None):
    global terminate_flag
    print('Signal received, terminating...')
    terminate_flag = True
    flush_alarm_ctrl(redis_conn)
    sys.exit(0)

# Register for both SIGINT and SIGTERM
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

def flush_alarm_ctrl(redis_conn):
    key = "Alarmctrl"
    redis_conn.delete(key)

def get_alarm_ctrl(redis_conn):
    lplen, t_tcpa, rel_x, rel_y, t_speed, t_heading, t_idx, alarm_sw = None, None, None, None, None, None, None, 1
    if redis_conn.exists('Alarmctrl'):
        lplen = int(get_or_default(redis_conn, "Alarmctrl", "LPlen", default='0'))
        t_tcpa = int(float(get_or_default(redis_conn, "Alarmctrl", "t_tcpa", default='10000')))
        rel_x = float(get_or_default(redis_conn, "Alarmctrl", "rel_x", default='10000'))
        rel_y = float(get_or_default(redis_conn, "Alarmctrl", "rel_7", default='10000'))
        t_speed = float(get_or_default(redis_conn, "Alarmctrl", "t_speed", default='0'))
        t_heading = float(get_or_default(redis_conn, "Alarmctrl", "t_heading", default='0'))
        t_idx = int(get_or_default(redis_conn, "Alarmctrl", "t_idx", default='-1'))
        alarm_sw = int(get_or_default(redis_conn, "Alarmctrl", "alarm_sw", default='1'))
    return lplen, t_tcpa, rel_x, rel_y, t_speed, t_heading, t_idx, alarm_sw

def parse_path(gp_lonlat_str):
    # $GP,1,122.123,22.123,122.124,22.124,122.125,22.125,122.126,22.126
    gp_list = gp_lonlat_str.decode('utf-8').split(',')
    head = gp_list[0]
    if head == '$GP'and len(gp_list[2:])==int(gp_list[1])*2:
        path = []
        for i in range(2, len(gp_list), 2):
            path.append((float(gp_list[i]), float(gp_list[i + 1])))
    elif head == '$LP' and len(gp_list[3:])==int(gp_list[2])*2:
        path = []
        for i in range(3, len(gp_list), 2):
            path.append((float(gp_list[i]), float(gp_list[i + 1])))

def get_gp_lp(redis_conn):
    lp, gp = None, None
    if redis_conn.exists('Navi'):
        lp_str = redis_conn.hget("Navi", "LPath")
        gp_str = redis_conn.hget("Navi", "GPath")
        if lp_str and gp_str:
            lp = parse_path(redis_conn.hget("Navi", "LPath"))
            gp = parse_path(redis_conn.hget("Navi", "GPath"))
    return lp, gp

def read_hp_target_data(redis_conn):
    keys = redis_conn.keys("data:*")
    highest_alarmstufe = -1
    min_value = None
    min_tcpa = 999999999
  
    for key in keys:
        value = redis_conn.hgetall(key)
        value = {k.decode(): v.decode() for k, v in value.items()}
        
        if "Alarmstufe" in value and int(value["Alarmstufe"]) > 0:          
            if len(value["Alarmstufe"])==5:
                alarmstufe = int(value["Alarmstufe"], 2)
            else:
                alarmstufe = int(value["Alarmstufe"])
            tcpa = float(value["cpTime"])
            
            if (alarmstufe > highest_alarmstufe) or (alarmstufe == highest_alarmstufe and tcpa < min_tcpa):
                highest_alarmstufe = alarmstufe
                min_tcpa = tcpa
                min_value = value
                min_value["target_idx"] = int(key.decode().split(":")[1])

    return min_value

def get_ownship(redis_conn):
    speed, own_lon, own_lat, heading = None, None, None, None
    speed = redis_conn.hget("IMU", "speed")
    own_lon = redis_conn.hget("IMU", "Lon")
    own_lat = redis_conn.hget("IMU", "Lat")
    heading =  redis_conn.hget("IMU", "heading")
    if speed:
        speed = float(speed)
    if own_lon:
        own_lon = float(own_lon)
    if own_lat:
        own_lat = float(own_lat)
    if heading:
        heading = float(heading)
    return speed, own_lon, own_lat, heading

def rel_heading2dir(own_heading, target_heading, angle_tolerance=15):
    target_heading = (target_heading + 360)%360
    own_heading = (own_heading + 360)%360
    angle = (target_heading - own_heading + 360)%360
    if 360-angle_tolerance<=angle<=360 or 0<=angle<=0+angle_tolerance:
        return 'Front'
    elif 90-angle_tolerance <= angle <= 90+angle_tolerance:
        return 'Right'
    elif 180-angle_tolerance <= angle <= 180+angle_tolerance:
        return 'Rear'
    elif 270-angle_tolerance <= angle <= 270+angle_tolerance:
        return 'Left'
    elif 0+angle_tolerance < angle < 90-angle_tolerance:
        return 'RFront'
    elif 90+angle_tolerance < angle < 180-angle_tolerance:
        return 'RRear'
    elif 180+angle_tolerance < angle < 270-angle_tolerance:
        return 'LRear'
    elif 270+angle_tolerance < angle < 360-angle_tolerance:
        return 'LFront'
    
def rel_xy2dir(rel_x, rel_y, angle_tolerance=15):
    angle = (np.degrees(np.arctan2(rel_x, rel_y))+360)%360
    if 360-angle_tolerance<=angle<=360 or 0<=angle<=0+angle_tolerance:
        return 'N'
    elif 90-angle_tolerance <= angle <= 90+angle_tolerance:
        return 'E'
    elif 180-angle_tolerance <= angle <= 180+angle_tolerance:
        return 'S'
    elif 270-angle_tolerance <= angle <= 270+angle_tolerance:
        return 'W'
    elif 0+angle_tolerance < angle < 90-angle_tolerance:
        return 'EN'
    elif 90+angle_tolerance < angle < 180-angle_tolerance:
        return 'ES'
    elif 180+angle_tolerance < angle < 270-angle_tolerance:
        return 'WS'
    elif 270+angle_tolerance < angle < 360-angle_tolerance:
        return 'WN'

def convert_lonlat_to_abs_xy(pos):
    lon, lat = pos
    Proj = pyproj.Proj(proj='utm', zone=50, ellps='WGS84', preserve_units=True)
    x, y = Proj(lon, lat)
    return x, y

def play_audio(fname, waitms=1000):
    if fname==None:
        return
    # get root directory
    root = os.path.dirname(os.path.abspath(__file__))

    fname = root+f"/audio/{fname}.mp3"
    sound = pygame.mixer.Sound(fname)
    sound.play()
    pygame.time.wait(int(sound.get_length() * waitms))

def play_alarm_w_CA(own_heading, rel_t_direction, t_tcpa, lplen, counter):
    situ = rel_heading2dir(own_heading, rel_t_direction)
    if 10>=t_tcpa>=7 and counter%10==0:
        play_audio(situ)
        play_audio(f"{t_tcpa}m")
    elif 7>t_tcpa>=5 and counter%10==0:
        play_audio(situ)
        play_autoCA_sound(lplen)
    elif 5>t_tcpa>=0 and counter%1==0:
        play_danger_sound()
        play_autoCA_sound(lplen)
    else:
        return False
    return True

def play_alarm_no_CA(own_heading, t_heading, t_tcpa, counter):
    situ = rel_heading2dir(own_heading, t_heading)
    if 10>=t_tcpa>=5 and counter%10==0:
        play_audio(situ)
        play_audio(f"{t_tcpa}m")
    elif 5>t_tcpa>=0 and counter%1==0:
        play_audio(situ)
        play_danger_sound()

def play_alarm_no_CA_simple(own_heading, t_heading, t_tcpa, counter, alarm_level):
    situ = rel_heading2dir(own_heading, t_heading)
    if alarm_level==1 and counter%10==0:
        play_audio(situ)
        play_audio(f"{t_tcpa}m")
    elif alarm_level==2 and counter%1==0:
        # play_audio(situ)
        play_danger_sound()
    else:
        return False
    return True

def play_autoCA_sound(lplen):    
    if lplen==5:
        play_audio("CAReady")
    elif 1<lplen<5:
        play_audio("CAInAct")  

def play_danger_sound():
    play_audio("warning-sound")
    # play_audio("danger")

def play_autoCA_sound_curv(situ):
    if situ == "CAReady":
        play_audio("CAReady")
    elif situ == "CAInAct":
        play_audio("CAInAct")

def play_sys_error_sound(counter):
    if counter%10==0:
        error_gps = int(get_or_default(redis_conn, "LOST", "GPS", default='1'))
        # error_radar = int(get_or_default(redis_conn, "LOST", "lada", default='1'))
        error_motor = int(get_or_default(redis_conn, "LOST", "dongli", default='1'))
        error_power = int(get_or_default(redis_conn, "LOST", "dianli", default='1'))
        if error_gps==0:
            play_audio("error_gps")
        # if error_radar==0:
        #     play_audio("error_radar")
        if error_motor==0:
            play_audio("error_motor")
        if error_power==0:
            play_audio("error_power")


def check_path(lp_lonlat, gp_lonlat, cur_pos_lonlat):
    gp_xy = [convert_lonlat_to_abs_xy(xy, 1) for xy in gp_lonlat]
    lp_xy = [convert_lonlat_to_abs_xy(xy, 1) for xy in lp_lonlat]
    cur_pos_xy = convert_lonlat_to_abs_xy(cur_pos_lonlat, 1)
    pos = find_current_segment_index(cur_pos_xy, gp_lonlat, 150)
    if pos<=0 or pos==len(gp_xy) or len(lp_lonlat)<=1:
        return None
    gp_seg_start = gp_xy[pos - 1]
    gp_seg_end = gp_xy[pos]
    pos_on_line = closest_point_on_line(cur_pos_xy, gp_seg_start, gp_seg_end)

    dists_to_global_path = [
        perpendicular_distance(xy, gp_seg_start, gp_seg_end)
        for xy in lp_xy
    ]

    total_avg = sum(dists_to_global_path) / len(dists_to_global_path)
    epsilon = 20

    third_length = len(dists_to_global_path) // 3

    first_avg = sum(dists_to_global_path[:third_length]) / third_length
    middle_avg = sum(dists_to_global_path[third_length:2 * third_length]) / third_length
    last_avg = sum(dists_to_global_path[2 * third_length:]) / third_length
    if (first_avg<middle_avg or first_avg<last_avg) and first_avg<=epsilon:
        return "CAReady"
    elif (first_avg>epsilon) and (first_avg<middle_avg or middle_avg<last_avg):
        return "CAInAct"
    else:
        return None

def play_alarm_w_CA_curv(own_heading, t_heading, t_tcpa, lp, gp, cur_lonlat, counter):
    situ = rel_heading2dir(own_heading, t_heading)
    path_situ = check_path(lp, gp, cur_lonlat)
    if 10>=t_tcpa>=7 and counter%10==0:
        play_audio(situ)
        play_audio(f"{t_tcpa}m")
    elif 7>t_tcpa>=5 and counter%10==0:
        play_audio(situ)
        play_autoCA_sound_curv(path_situ)
    elif 5>t_tcpa>=0 and counter%1==0:
        play_danger_sound()
        play_autoCA_sound_curv(path_situ)
        
def hp_target_to_redis(redis_conn, min_value, own_heading):
    if min_value is not None:
        for field, value in min_value.items():
            if field == "azimuth":
                azimuth = (float(value) - own_heading + 360)%360
                value = azimuth
            redis_conn.hset('hp_target', field, value)
            redis_conn.expire('hp_target', 10)  # 设置过期时间为10秒
    else:
        redis_conn.delete('hp_target')

def run_alarm():
    global terminate_flag, mymixer
    counter = 0
    min_count = 0
    print("Alarm sound is running...")
    
    while not terminate_flag:  # Changed from while True to respect the terminate flag
        lplen, t_tcpa, rel_x, rel_y, t_speed, t_heading, t_idx, alarm_sw = get_alarm_ctrl(redis_conn)
        own_speed, own_lon, own_lat, own_heading = get_ownship(redis_conn)
        # lp, gp = get_gp_lp(redis_conn)
        coli_alarm = False
        # if lplen is not None and lplen>0 and t_tcpa is not None:
        #     rel_t_direction = (np.degrees(np.arctan2(rel_x, rel_y))+360)%360
        #     print("Target_idx: ", t_idx)
        #     print("TCPA: ", t_tcpa)
        #     print("relative direction: ", (rel_t_direction - own_heading + 360)%360)
        #     coli_alarm = play_alarm_w_CA(own_heading, rel_t_direction, t_tcpa, lplen, counter)
        # elif lp and gp:
        #     rel_t_direction = (np.degrees(np.arctan2(rel_x, rel_y))+360)%360
        #     play_alarm_w_CA_curv(own_heading, rel_t_direction, t_tcpa, lp, gp, (own_lon, own_lat), counter)
    
        # else:
        min_value = read_hp_target_data(redis_conn)
        hp_target_to_redis(redis_conn, min_value, own_heading)
        if min_value and own_lon and own_lat and alarm_sw:
            t_tcpa = int(float(min_value["cpTime"]))
            t_lon = float(min_value["longitude"])
            t_lat = float(min_value["latitude"])
            alarm_level = int(min_value["Alarmstufe"])
            if len(min_value["Alarmstufe"])==5:
                alarm_level = int(min_value["Alarmstufe"], 2)
            else:
                alarm_level = int(min_value["Alarmstufe"])
            x0, y0 = convert_lonlat_to_abs_xy((own_lon, own_lat))
            x1, y1 = convert_lonlat_to_abs_xy((t_lon, t_lat))
            rel_x = x1 - x0
            rel_y = y1 - y0
            rel_t_direction = (np.degrees(np.arctan2(rel_x, rel_y))+360)%360
            print("Target_idx: ", min_value["target_idx"])
            print("TCPA: ", min_value["cpTime"])
            print("relative direction: ", (rel_t_direction - own_heading + 360)%360)
            # play_alarm_no_CA(own_heading, rel_t_direction, t_tcpa, counter)
            coli_alarm = play_alarm_no_CA_simple(own_heading, rel_t_direction, t_tcpa, counter, alarm_level)
        
        if not coli_alarm:
            play_sys_error_sound(counter)

        counter = (counter + 1)%60
        if counter%60==0:
            min_count += 1
        # if min_count%5==0:
        #     mymixer.init()
        # print("errrrrr")
        time.sleep(0.5)

if __name__=="__main__":
    run_alarm()