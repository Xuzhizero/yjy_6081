import os
import sys
import time
import redis
import json
import signal
import threading
import numpy as np
import tkinter as tk
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
from matplotlib.patches import Ellipse
from algo_utility import convert_lonlat_to_abs_xy, khachiyan_algorithm

# insert root directory into python module search path
sys.path.insert(1, os.getcwd())

from ModifyAndDump import get_or_default

# Define global variables
g_polygons = None
g_elipses = None
poly_num = 0

terminate_flag = False

def signal_handler(sig, frame):
    global terminate_flag
    print('You pressed Ctrl+C!')
    terminate_flag = True
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

def init_redis():
    global redis_conn
    host = 'localhost'
    port = 6379
    db = '0'
    password = None
    # 建立 Redis 连接
    redis_conn = redis.Redis(host=host, port=port, db=db, password=password)
    # 可以在此处进行一些连接测试或其他初始化操作
    try:
        redis_conn.ping()  # 测试连接是否正常
        print("Redis connection established successfully.")
    except redis.ConnectionError as e:
        print(f"Error connecting to Redis: {e}")

init_redis()


# read a config file and extract the file path of read and write
def read_config_file(config_file="config.json"):
    script_dir = os.path.dirname(__file__)  # Get the directory of the script
    abs_config_file = os.path.join(script_dir, config_file)  # Construct the absolute path
    with open(abs_config_file, 'r', encoding="utf-8") as file:  
        config = json.load(file)
        read_path = config['read_path']
        write_path = config['write_path']
    return read_path, write_path

# get ploygons from the file, each line is a ploygon, remove the frist digit of a line which is the number of points, return a list of a 2d array as a list of plolygon coordinates
def get_polygon_points(lines):
    polygons = []
    for line in lines:
        points = line.strip(",\n").split(",")
        polygon = [convert_lonlat_to_abs_xy((float(points[i]), float(points[i+1])), 1) for i in range(1, len(points), 2)]
        polygons.append(polygon)
        print(polygons)
    return polygons

def is_point_in_ellipse_with_A_c(point, A, c):
    diff = point - c
    return np.dot(diff.T, np.dot(A, diff)) <= 1

def is_point_in_polygon(point, polygon):
    x, y = point
    n = len(polygon)
    inside = False
    p1x, p1y = polygon[0]
    for i in range(n + 1):
        p2x, p2y = polygon[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y
    return inside

def get_ploy_encl_elipses(polygons, poly_num):
    poly_path, index_path = read_config_file()
    with open(poly_path, 'r', encoding="utf-8") as file:
        p_lines = [line.strip() for line in file.readlines() if line.strip()]
    if (polygons!=None and len(p_lines)!=0) and len(p_lines)==poly_num:
        return polygons, poly_num
    polygons = get_polygon_points(p_lines)
    # get json lines from index_path
    return polygons, len(p_lines)

def update_polygons_and_elipses():
    global g_polygons, poly_num
    while True:
        g_polygons, poly_num = get_ploy_encl_elipses(g_polygons, poly_num)
        time.sleep(60) 

def motor_ctrl():
    def confirm_action():
        redis_conn.hset("MotorCtrl", "Stop", 0)
        window.destroy()

    while True:
        motor_ctrl = redis_conn.hget("MotorCtrl", "Stop")
        if motor_ctrl and motor_ctrl.decode("utf-8") == "1":
            print("MotorCtrl is 1")
            window = tk.Tk()
            window.title("Motor Control")
            window.geometry("200x100") 
            window.attributes("-topmost", True)
            label = tk.Label(window, text="已紧急停船")
            label.pack(pady=10)
            confirm_button = tk.Button(window, text="恢复", command=confirm_action)
            confirm_button.pack(pady=10)
            window.mainloop()
            
  

def main():
    global g_polygons, poly_num
    g_polygons, poly_num = get_ploy_encl_elipses(g_polygons, poly_num)
    while True:
        lon = get_or_default(redis_conn, "IMU", 'Lon', default='116.3975')
        lat = get_or_default(redis_conn, "IMU", 'Lat', default='39.9085')
        lon = float(lon)
        lat = float(lat)
        point = convert_lonlat_to_abs_xy((lon, lat), 1)
        for i, polygon in enumerate(g_polygons):
            # if (A_.size==0 and c_.size==0) and is_point_in_ellipse_with_A_c(point, A_, c_):
            if is_point_in_polygon(point, polygon):
                redis_conn.hset("Alarmctrl","alarm_sw", 0)
                print("Point is inside polygon and inside ellipse")
                break
        else:
            redis_conn.hset("Alarmctrl","alarm_sw", 1)
            print("Point is outside polygon or outside all zone")
        time.sleep(1)

if __name__=="__main__":
   # Create and start a thread
    thread = threading.Thread(target=update_polygons_and_elipses)
    thread.daemon = True  # Optional: make the thread a daemon so it exits when the main program does
    thread2 = threading.Thread(target=motor_ctrl)
    thread2.daemon = True
    thread.start()
    thread2.start()
    main()
