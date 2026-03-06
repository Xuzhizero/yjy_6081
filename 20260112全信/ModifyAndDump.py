import json
import math
import os
import socket
import struct

import pandas as pd
from pyproj import Geod
import redis
import time
from datetime import datetime, timedelta,timezone
from alarm import Alarm
from algo_utility import convert_lonlat_to_abs_xy
from datetime import datetime

pre_file_path = os.path.dirname(__file__)
alarm_instance = Alarm(domain_scale=3)

now_os_speed = 4.4
now_os_abs_xy = (0, 0)
now_os_course = 0
now_os_heading = 0
Duo_move_state =0
pre_Duo=0

FENCE_POLYGONS = []  # list of { 'bbox': (minx, miny, maxx, maxy), 'coords': [(lon, lat), ...] }

def save_to_file(data,channel_name,nowtime_name, max_file_size):
    filename = f"{nowtime_name}.txt"
    file_path = os.path.join(pre_file_path,"dump_folder",channel_name, filename)

    #print(file_path)
    # 如果文件路径不存在，则创建路径
    directory = os.path.dirname(file_path)
    if not os.path.exists(directory):
        os.makedirs(directory)

    # 如果文件不存在，则创建空文件
    if not os.path.isfile(file_path):
        with open(file_path, 'w', encoding='utf-8'):
            pass  # 空语句，仅用于创建文件

    # 写入数据到文件
    with open(file_path, 'a', encoding='utf-8') as f:
        if(type(data)==str):
            f.write(str(time.time())+"    "+str(data) + '\n')
        elif(type(data)==bytes):
            hex_representation = ''.join(f'{byte:02x}' for byte in data)
            f.write(str(time.time()) + "    " + str(hex_representation) + '\n')
        f.flush()
        os.fsync(f.fileno())
    # 检查文件大小
    if os.path.exists(file_path):
        current_size = os.path.getsize(file_path)
    else:
        current_size = 0

    # 如果文件大小超过最大限制，则新建文件
    if current_size > max_file_size:
        nowtime_name = datetime.now().strftime("%Y%m%d_%H%M%S")

    return nowtime_name



# # 连接本地Redis数据库
# r = redis.Redis(host='localhost', port=6379)
#融合数据缓存
def fusion_data_toredis(data,redis_conn):
    #data = data.decode('utf-8')
    dict_data = json.loads(data)

    target_id   = dict_data['TarID']
    longitude   = dict_data['Lon']
    latitude    = dict_data['Lat']
    speed       = dict_data['Speed']
    direction   = dict_data['Course']

    add_data(redis_conn,target_id, longitude, latitude, speed, direction)
    return data

def gp_from_redis(data, redis_conn):
    gpstr = get_or_default(redis_conn, "Navi", "GPath", '$GP,0')
    return gpstr.encode('utf-8')

def lp_from_redis(data, redis_conn):
    lpstr = get_or_default(redis_conn, "Navi", "LPath", '$LP,0,0')
    return lpstr.encode('utf-8')

def GPath_toredis(data,redis_conn):
    data_list = data.split(',')
    if(data_list[0]=="$GP"):
        redis_conn.hset("Navi","GPath", data)
    elif(data_list[0] == "$LP"):
        redis_conn.hset("Navi", "LPath", data)
    return data

def NaviState_toredis(data,redis_conn):
    data_list = data.split(',')
    redis_conn.hset("Navi","State", data_list[1])
    redis_conn.hset("Navi","TargetDuo", '0')


def NaviDUO_CTRL(data,redis_conn):
    data_list = data.split(',')

    redis_conn.hset("Navi","TargetDuo", float(data_list[1]))
 
def get_or_default(redis_conn, key, field, default='0'):
    # 从 Redis 获取字段值
    value = redis_conn.hget(key, field)

    # 检查值是否为 None 或空字符串，如果是，则返回默认值
    if value is None or value.decode('utf-8') == '':
        return default

    # 返回获取的值，解码为字符串
    return value.decode('utf-8')

def ArduinoDuoCTRL(data,redis_conn):



    new_list = [''] * 2  # 新建一个和data_list长度相同的字符串数组，默认空字符串


    new_list[0] = "#DUO"

    # 从 Redis 获取字段值
    value = get_or_default(redis_conn, "Navi", "State", '0')
    if(value == "1"):
        data = get_or_default(redis_conn, "Navi", "TargetDuo", '!')
        new_list[1] = data

        return ((','.join(new_list))+"\n").encode('utf-8')
    else:
        return "NOP\n".encode('utf-8')
def DuoCTRLTest(data,redis_conn):
    global pre_Duo
    UDP_IP1 = "193.0.1.88"
    # 创建UDP套接字
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if(pre_Duo == 0):
        # 右打舵
        # hex_string = "04 0F 00 10 00 02 01 02 5E AA"
        hex_string = "04 0F 00 10 00 02 01 08 DE AD"
        byte_sequence = bytes.fromhex(hex_string)
        sock.sendto(byte_sequence, (UDP_IP1, 3004))
        pre_Duo = 1
    elif pre_Duo == 1:
        # 左打舵
        # hex_string = "04 0F 00 10 00 02 01 01 1E AB"
        hex_string = "04 0F 00 10 00 02 01 04 DE A8"
        byte_sequence = bytes.fromhex(hex_string)
        sock.sendto(byte_sequence, (UDP_IP1, 3004))
        pre_Duo = 0
    time.sleep(1)
    # 停打舵
    # hex_string = "04 0F 00 10 00 02 01 00 DF 6B"
    hex_string = "04 0F 00 12 00 02 01 00 A6 AB"
    byte_sequence = bytes.fromhex(hex_string)
    sock.sendto(byte_sequence, (UDP_IP1, 3004))

def PLC_DuoCTRL(data,redis_conn):
    UDP_IP1 = "193.0.1.99"

    # 创建UDP套接字
    #sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # 从 Redis 获取字段值
    #value = get_or_default(redis_conn, "Navi", "State", '0')
    value = '1'

    # 检查值是否为 None 或空字符串，如果是，则返回默认值
    if value == '0':

        return None
    if value == '1':

        TargetDuo = get_or_default(redis_conn, "Navi", "TargetDuo", '!')
        NowDuo = get_or_default(redis_conn, "Navi", "NowDuo", '!')
        if(TargetDuo!='!' and NowDuo!='!'):
            byte_sequence = struct.pack(">ff",float(TargetDuo),float(NowDuo))
            hex_string = ' '.join(f'{byte:02X}'for byte in byte_sequence)
            print("HEX:")
            print(hex_string)
            return  byte_sequence
        else:
             return None
        #sock.sendto(byte_sequence, (UDP_IP1, 8500))

def calculate_checksum(byte_array):
    print(byte_array)
    print("AAA")
    checksum = sum(byte_array) & 0xFF  # 计算校验和
    return bytes([checksum])

def append_checksum(byte_array):

    checksum = calculate_checksum(byte_array)  # 计算校验和

    byte_array = byte_array+checksum  # 将校验和添加到字节数组末尾
    return byte_array

def DuoCTRL6081_(data,redis_conn):

    
    value = '1'

    # 检查值是否为 None 或空字符串，如果是，则返回默认值
    if value == '0':

        return None
    if value == '1':
        print("DuoCTRL6081_")
        TargetDuo = get_or_default(redis_conn, "Navi", "TargetDuo", '!')
        print(TargetDuo)
        if(TargetDuo!='!' ):
            try:
                print(TargetDuo)
                byte_array = (32767 + int(float(TargetDuo)) * 10).to_bytes(2, byteorder='big')
                
                bytearray= bytes([0x10, 0x01, 0x6f, 0x41, 0x21, 0x00])+byte_array+  bytes([0x10, 0x03] )
                print(bytearray)

                checksum = sum(bytearray) & 0xFF 
                result_byte_array = bytearray + bytes([checksum])
                # result_byte_array = append_checksum(bytearray)
                
                print(result_byte_array)
                return result_byte_array
            
            except Exception as e:
                print({e})
                return None

        else:
             return None


def PLC_zhuan_suCTRL(data,redis_conn):
    UDP_IP1 = "193.0.1.99"


    TargetU = int(get_or_default(redis_conn, "Navi", "TargetU", '0'))
    Gear = int(get_or_default(redis_conn, "Navi", "Gear", '0'))

    byte_array1 = TargetU.to_bytes(4,byteorder='big')
    byte_array2 = Gear.to_bytes(4,byteorder='big')

    xx= byte_array2+byte_array1

   
    hex_string = ' '.join(f'{byte:02X}'for byte in xx)
    #print("HEXxx:")
    print(hex_string)
    return  xx
    #sock.sendto(byte_sequence, (UDP_IP1, 8500))



def calculate_nmea_crc(sentence):
    """
    计算NMEA0183协议的CRC校验码（8位）
    :param sentence: 要计算的字符串（不含末尾的*XX）
    :return: 两位十六进制的校验码字符串（大写）
    """
    # 初始化CRC值为0
    crc = 0
    # 遍历字符串中的每个字符（从$后开始，到最后一个字符）
    for char in sentence:
        # 跳过$符号（NMEA校验不包含$）
        if char == '$':
            continue
        # 按位异或计算CRC
        crc ^= ord(char)
    # 返回两位十六进制格式（02x），并转为大写
    return f"*{crc:02X}"

def zhuansuCTRL6081_(data,redis_conn):
    print("zhuansuCTRL6081_")


    TargetDuo = get_or_default(redis_conn, "Navi", "TargetDuo", '0')
    print(TargetDuo)

    TargetDuo=int(float(TargetDuo))


    TargetU = int(get_or_default(redis_conn, "Navi", "TargetU", '0'))
    datax = f"$NAVICTL,1,{TargetU},{TargetU},{TargetDuo},{TargetDuo},0"
    datax = datax +calculate_nmea_crc(datax) + "\x0D\x0A"




    return  datax.encode("utf-8")
    #sock.sendto(byte_sequence, (UDP_IP1, 8500))


def PLC_ADzhuan_suCTRL(data,redis_conn):

    Target_Left_zhuansu = int(get_or_default(redis_conn, "Navi", "left_zhuansu", '0'))
    Target_Right_zhuansu = int(get_or_default(redis_conn, "Navi", "right_zhuansu", '0'))

    # 转成整数，如果字段不存在则默认 0
    left = Target_Left_zhuansu
    right = Target_Right_zhuansu

    print("PLC_ADzhuan_suCTRL")
    print("Redis读取值：", left, right)
    # 限幅处理（-100 ~ 100）
    if not -100 <= left <= 100:
        left = 0
    if not -100 <= right <= 100:
        right = 0

    # 将 int 转为 2 字节有符号整型（大端序）并转 hex
    left_bytes = struct.pack(">h", left)   # big-endian signed short
    right_bytes = struct.pack(">h", right)

    # 格式化为 PLC 惯用的 hex 输出
    hex_output = " ".join(f"{b:02X}" for b in (left_bytes + right_bytes))

    print("Redis读取值：", left, right)
    print("PLC输出16进制：", hex_output)


    return  left_bytes + right_bytes
    #sock.sendto(byte_sequence, (UDP_IP1, 8500))


def plc_rudder_feed(data,redis_conn):

    print("plc_rudder_feed")
    print(data)

    byte_array = data
    # print(byte_array[0])
    # B4 04 14 01 B3 00 25 00 2A 00 29 01 45 00 03 00 00 00 00 00 01 00 00 71 C2
    档位 = int.from_bytes(byte_array[0:4], byteorder='big')
    weizhi = int.from_bytes(byte_array[4:8], byteorder='big')

    print("weizhi"+str(weizhi))
    print("档位"+str(档位))

    redis_conn.hset("Navi","NowU", weizhi)
    redis_conn.hset("Navi","NowGear", 档位)


def 五元组(data,redis_conn):

    print("plc_rudder_feed")
    print(data)

    byte_array = data
    # print(byte_array[0])
    # B4 04 14 01 B3 00 25 00 2A 00 29 01 45 00 03 00 00 00 00 00 01 00 00 71 C2
    手自动 = int.from_bytes(byte_array[0:1], byteorder='big')
    左上限 = int.from_bytes(byte_array[1:2], byteorder='big')
    左下限 = int.from_bytes(byte_array[2:3], byteorder='big')
    右上限 = int.from_bytes(byte_array[3:4], byteorder='big')
    右下限 = int.from_bytes(byte_array[4:5], byteorder='big')
    print(手自动,左上限,左下限,右上限,右下限)




    redis_conn.hset("wuyuanzu:shouzidong","wuyuanzu:shouzidong", 手自动)
    redis_conn.expire("wuyuanzu:shouzidong", 5)
    redis_conn.hset("wuyuanzu:LU","wuyuanzu:LU", 左上限)
    redis_conn.expire("wuyuanzu:LU", 5)
    redis_conn.hset("wuyuanzu:LD","wuyuanzu:LD", 左下限)
    redis_conn.expire("wuyuanzu:LD", 5)
    redis_conn.hset("wuyuanzu:RU","wuyuanzu:RU", 右上限)
    redis_conn.expire("wuyuanzu:RU", 5)
    redis_conn.hset("wuyuanzu:RD","wuyuanzu:RD", 右下限)
    redis_conn.expire("wuyuanzu:RD", 5)
    
    print("##@@@@@@@@########")


    # redis_conn.hset("Navi","NowU", weizhi)
    # redis_conn.hset("Navi","NowGear", 档位)







def ADzhuansu(data,redis_conn):



    # 左手柄位置（前两字节，高字节在前）
    left_bytes = data[0:2]
    left_pos = int.from_bytes(left_bytes, byteorder='big', signed=True)

    # 右手柄位置（第3-4字节）
    right_bytes = data[2:4]
    right_pos = int.from_bytes(right_bytes, byteorder='big', signed=True)

    # 车钟手自动状态（第5字节）
    car_auto = data[4]

    # 舵机手自动状态（第6字节）
    rudder_auto = data[5]


    redis_conn.hset("Navi","now_left_pos", left_pos)
    redis_conn.hset("Navi","now_right_pos", right_pos)
    redis_conn.hset("Navi","car_auto", car_auto)
    redis_conn.hset("Navi","rudder_auto", rudder_auto)
    
    print(f"左手柄位置: {left_pos}")
    print(f"右手柄位置: {right_pos}")
    print(f"车钟手自动状态: {car_auto}")
    print(f"舵机手自动状态: {rudder_auto}")




def DuoCTRL(data,redis_conn):
    global  Duo_move_state
    global  pre_Duo

    print("##############################")
    UDP_IP1 = "193.0.1.88"

    # 创建UDP套接字
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # 从 Redis 获取字段值
    value = get_or_default(redis_conn,"Navi","State",'0')
    wucha = 2
    #TargetDuo = float(get_or_default(redis_conn, "Navi", "TargetDuo", '!'))
    # if(TargetDuo == 0):
    #     wucha= 1
    # 检查值是否为 None 或空字符串，如果是，则返回默认值
    if value == '0':
        # 停打舵
        # hex_string = "04 0F 00 10 00 02 01 00 DF 6B"

        #
        # hex_string = "04 0F 00 12 00 02 01 00 A6 AB"
        # byte_sequence = bytes.fromhex(hex_string)
        # sock.sendto(byte_sequence, (UDP_IP1, 3004))
        # Duo_move_state = 0
        return
    if value == '1':
        NowDuo =float( get_or_default(redis_conn,"Navi","NowDuo",'!'))
        TargetDuo = float(get_or_default(redis_conn, "Navi", "TargetDuo", '!'))
        # if(abs(NowDuo-TargetDuo)>=5):
        #     sectime = abs(NowDuo - TargetDuo) / 4.4
        # elif(abs(NowDuo-TargetDuo)>=1):
        #     sectime = abs(NowDuo - TargetDuo) / 8.8
        # else:
        #     sectime = abs(NowDuo - TargetDuo) / 12.0

        # print(value + "#################################value")
        print("#1#######################"+str(NowDuo)+" "+str(TargetDuo))

        if NowDuo !="!" and TargetDuo !='':
            if(NowDuo < TargetDuo -wucha):
                if(Duo_move_state == 0 or Duo_move_state == 2):
                    # 右打舵
                    # hex_string = "04 0F 00 10 00 02 01 02 5E AA"
                    hex_string = "04 0F 00 10 00 02 01 08 DE AD"
                    byte_sequence = bytes.fromhex(hex_string)
                    sock.sendto(byte_sequence, (UDP_IP1, 3004))
                    Duo_move_state = 2
                    # time.sleep(sectime)
                    #
                    # hex_string = "04 0F 00 12 00 02 01 00 A6 AB"
                    # byte_sequence = bytes.fromhex(hex_string)
                    # sock.sendto(byte_sequence, (UDP_IP1, 3004))
                    # Duo_move_state = 0



                elif(Duo_move_state == 1):
                    # 停打舵
                    hex_string = "04 0F 00 12 00 02 01 00 A6 AB"
                    byte_sequence = bytes.fromhex(hex_string)
                    sock.sendto(byte_sequence, (UDP_IP1, 3004))
                    Duo_move_state = 0


            elif(NowDuo > TargetDuo +wucha):
                if(Duo_move_state == 0 or Duo_move_state == 1):
                    # 左打舵
                    # hex_string = "04 0F 00 10 00 02 01 01 1E AB"
                    hex_string = "04 0F 00 10 00 02 01 04 DE A8"
                    byte_sequence = bytes.fromhex(hex_string)
                    sock.sendto(byte_sequence, (UDP_IP1, 3004))
                    Duo_move_state = 1

                    # time.sleep(sectime)
                    #
                    # hex_string = "04 0F 00 12 00 02 01 00 A6 AB"
                    # byte_sequence = bytes.fromhex(hex_string)
                    # sock.sendto(byte_sequence, (UDP_IP1, 3004))
                    # Duo_move_state = 0

                elif(Duo_move_state == 2):
                    # 停打舵
                    # hex_string = "04 0F 00 10 00 02 01 00 DF 6B"
                    hex_string = "04 0F 00 12 00 02 01 00 A6 AB"
                    byte_sequence = bytes.fromhex(hex_string)
                    sock.sendto(byte_sequence, (UDP_IP1, 3004))
                    Duo_move_state = 0
            else:
                # 停打舵
                # hex_string = "04 0F 00 10 00 02 01 00 DF 6B"
                hex_string = "04 0F 00 12 00 02 01 00 A6 AB"
                byte_sequence = bytes.fromhex(hex_string)
                sock.sendto(byte_sequence, (UDP_IP1, 3004))
                Duo_move_state = 0


    pre_Duo = TargetDuo



cur_Rud= 0
init_flag = 0

def left_turn(sock, ip, dur):
    # 左打舵
    hex_string = "04 0F 00 10 00 02 01 04 DE A8"
    # hex_string = "04 0F 00 10 00 02 01 01 1E AB"
    byte_sequence = bytes.fromhex(hex_string)
    sock.sendto(byte_sequence, (ip, 3004))
    time.sleep(dur)
    hex_string = "04 0F 00 12 00 02 01 00 A6 AB"
    # hex_string = "04 0F 00 10 00 02 01 00 DF 6B"
    byte_sequence = bytes.fromhex(hex_string)
    sock.sendto(byte_sequence, (ip, 3004))

def right_turn(sock, ip, dur):
    # 右打舵
    hex_string = "04 0F 00 10 00 02 01 08 DE AD"
    # hex_string = "04 0F 00 10 00 02 01 02 5E AA"
    byte_sequence = bytes.fromhex(hex_string)
    sock.sendto(byte_sequence, (ip, 3004))
    time.sleep(dur)
    hex_string = "04 0F 00 12 00 02 01 00 A6 AB"
    # hex_string = "04 0F 00 10 00 02 01 00 DF 6B"
    byte_sequence = bytes.fromhex(hex_string)
    sock.sendto(byte_sequence, (ip, 3004))

def rudder_ctrl(data,redis_conn):
    global cur_Rud, init_flag
    UDP_IP1 = "193.0.1.88"
    value = get_or_default(redis_conn, "Navi", "State", '0')
    # 创建UDP套接字
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    interval = 4.5
    NowRud = float(get_or_default(redis_conn, "Navi", "NowDuo", '!'))

    zhuansu =get_or_default(redis_conn, "engine_parameters", "zhuan_su", '!')
    TargetRud = float(get_or_default(redis_conn, "Navi", "TargetDuo", '!'))
    #TargetRud = int (TargetRud_f/     (float(zhuansu)/100.0)   ) if float(zhuansu)!= 0 else 0
    if value == '0':
        init_flag = 0
        return
    elif value == '1':
        if init_flag == 0:
            init_rudder(redis_conn)
            cur_Rud = 0
        # return
        else:
            if cur_Rud < TargetRud:
                # 右打舵
                right_turn(sock, UDP_IP1, 1)
                cur_Rud += 1

            elif cur_Rud > TargetRud:
                # 左打舵
                left_turn(sock, UDP_IP1, 1)
                cur_Rud -= 1

            rudder_ctrl_zero(redis_conn)

def rudder_ctrl_zero(redis_conn, flag = None):
    global cur_Rud, init_flag
    UDP_IP1 = "193.0.1.88"

    # 创建UDP套接字
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if cur_Rud ==0:
        RTNowRud = float(get_or_default(redis_conn, "Navi", "NowDuo", '!'))
        if RTNowRud > 0.5:
            # 左打舵
            left_turn(sock, UDP_IP1, 0.1)
        elif RTNowRud < -0.5:
            # 右打舵
            right_turn(sock, UDP_IP1, 0.1)

        elif -0.5<=RTNowRud <=0.5 and flag == 0:
            init_flag = 1

        time.sleep(0.4)

def init_rudder(redis_conn):
    global init_flag

    UDP_IP1 = "193.0.1.88"
    # 创建UDP套接字
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    NowRud = float(get_or_default(redis_conn, "Navi", "NowDuo", '!'))
    if NowRud > 3:
        # 左打舵
        left_turn(sock, UDP_IP1, 1)
    elif NowRud < -3:
        # 右打舵
        right_turn(sock, UDP_IP1, 1)
    else:
        rudder_ctrl_zero(redis_conn, init_flag)

g_lon =0
g_lat =0
def PathSend(data,redis_conn):

    data = get_or_default(redis_conn, "Navi", "LPath", '!')
    print("LocalPath" + data)
    if data == '!':
        data = get_or_default(redis_conn, "Navi", "GPath", '!')
        return data.encode('utf-8')
    else:
        # 按逗号分隔字符串

        data_list = data.split(',')
        data_list[1] = str(int(len(data_list)/2-1))
        return ','.join(data_list).encode('utf-8')

def PathSend_to_show(data,redis_conn):

    data = get_or_default(redis_conn, "Navi", "LPath", '!')
    lon = get_or_default(redis_conn, "IMU", "Lon", '!')
    lat = get_or_default(redis_conn, "IMU", "Lat", '!')
    lon = float(lon)
    lat = float(lat)
    print("LocalPath" + data)
    if data == '!':
        data = get_or_default(redis_conn, "Navi", "GPath", '!')
        return data.encode('utf-8')
    else:
        # 按逗号分隔字符串

        data_list = data.split(',')
        if len(data_list) == 2 or data_list[2] != '':
            data_list = data_list[:2] +[str(lon),str(lat)] + data_list[2:]
            # data_list[2] = str(lon)
            # data_list[3] = str(lat)
        data_list[1] = str(int(len(data_list)/2-1))
        return ','.join(data_list).encode('utf-8')

def new_PathSend(data,redis_conn):
    data = get_or_default(redis_conn, "Navi", "LPath", '!')
    print("LocalPath" + data)
    if data == '!':
        data = get_or_default(redis_conn, "Navi", "GPath", '!')
        if data == '!':
            return "$GP,0"
        datalist = data.split(',')
        datalist = list(filter(None, datalist))
        outst = ','.join(datalist[1:])
        outst = "$GP,4,"+outst
        return outst.encode('utf-8')
    return data.encode('utf-8')

def new_PathSend_to_show(data,redis_conn):
    data = get_or_default(redis_conn, "Navi", "LPath", '!')
    print("LocalPath" + data)
    if data == '!':
        data = get_or_default(redis_conn, "Navi", "GPath", '!')
        if data == '!':
            return "$GP,0"
        return data.encode('utf-8')
    datalist = data.split(',')
    datalist = list(filter(None, datalist))
    if (len(datalist)-1)%2 == 0:
        datalist = datalist[0:1]+ datalist[2:]
        return ','.join(datalist).encode('utf-8')
    return data.encode('utf-8')


def calculate_polyline_distance_and_time(data_str, speed_kmh):
    """
    计算折线总距离和预计驾驶时间
    
    参数:
    data_str -- 输入字符串，格式如 "2,lon1,lat1,lon2,lat2" 或 "3,lon1,lat1,lon2,lat2,lon3,lat3"
    speed_kmh -- 速度（千米/小时）
    
    返回:
    (总距离米, 驾驶时间秒)
    """
    parts = data_str.split(',')
    try:
        num_points = int(parts[2])
        coordinates = list(map(float, parts[3:]))
        
        # 验证数据完整性
        if len(coordinates) != num_points * 2:
            raise ValueError(f"坐标数量不匹配：声明有{num_points}个点，但提供了{len(coordinates)/2}个点的数据")
        
        geod = Geod(ellps="WGS84")
        total_distance = 0.0
        
        # 计算每段距离
        for i in range(num_points - 1):
            lon1, lat1 = coordinates[i*2], coordinates[i*2+1]
            lon2, lat2 = coordinates[(i+1)*2], coordinates[(i+1)*2+1]
            _, _, dist = geod.inv(lon1, lat1, lon2, lat2)
            total_distance += dist
        
        # 计算时间
        speed_ms = speed_kmh   # 转换为米/秒
        travel_time = total_distance / speed_ms
        
        return total_distance, travel_time
        
    except (ValueError, IndexError) as e:
        raise ValueError(f"输入数据格式错误: {e}")


def lp_calc_time(data,redis_conn):
    data = get_or_default(redis_conn, "Navi", "LPath", '!')
    SPEED = get_or_default(redis_conn, "IMU", "speed", '!')
    print("LocalPath" + data)
    if data == '!':
        return 
    
    distance, time = calculate_polyline_distance_and_time(data, float(SPEED))

    print(f"折线总长: {distance:.2f}米")
    print(f"驾驶时间: {time:.2f}秒 | {time/60:.2f}分钟 | {time/3600:.2f}小时")
    redis_conn.hset("IMU", "cost_time", '{:.2f}'.format(time/3600))
    redis_conn.hset("Navi", "cost_dis", '{:.2f}'.format(distance))

    return


def USV_ctrl_to_redis(data,redis_conn):
    data_list = data.split(',')
    if(data_list[0]=="$NAVISIM"):
        redis_conn.hset("USV_SIM", "TargetRPM", data_list[2])
        redis_conn.expire("USV_SIM", 30)
    return data.encode('utf-8')

def USV控制转发(data,redis_conn):
    print("USV控制转发" )

    data_list = data.split(',')

    print(data_list[4])

    redis_conn.hset("Navi","TargetDuo", float(data_list[4]))

def IMU_fix_USV(data,redis_conn):
    data_list = data.split(',')


    # if(float(data_list[6])==40.0  and float(data_list[7])==116.0):
    #     return None


    #data_list[3]  = str( (float (data_list[3]) +90 ) %360)
    data_list[3]  = str( float (data_list[3]) )


    new_list = [''] * 22  # 新建一个和data_list长度相同的字符串数组，默认空字符串

    # 假设你想从data_list取一部分内容并赋值给new_list
    # 举个例子，取前3个元素并赋值
    new_list[0] = "$NAVIDAT"
    new_list[1] =  str(time.time())
    new_list[2] = data_list[6]#LAT
    new_list[3] = data_list[7]#LON

    east_speed = float( data_list[9])
    north_speed = float( data_list[10])
    # 计算水平速度（东向速度和北向速度的平方和开方）
    horizontal_speed = math.sqrt(east_speed ** 2 + north_speed ** 2)

    angle = math.degrees(math.atan2(east_speed, north_speed))

    # 确保角度在0到360度之间
    if angle < 0:
        angle += 360




    new_list[4] = str(horizontal_speed)#speed
    new_list[5] = str(angle)#COG
    new_list[6] = data_list[8]#height

    new_list[7] = data_list[9]#dong
    new_list[8] = data_list[10]#bei
    new_list[9] = data_list[11]#tian

    new_list[13] = data_list[3]#heading

    new_list[14] = data_list[4]#pitch
    new_list[15] = data_list[5]#roll


    # 最后将new_list按逗号分割组成字符串
    result = ','.join(new_list)

    print(result)

    # if(strcmp(items[0].s, "$NAVIDAT") != 0)
    #     return -1;
    # if(num < 20) return -1;
    #
    # MYSHIP_INFO info;
    # info.gps_time = atoi(items[1].s);
    # info.lat = atof(items[2].s);
    # info.lon = atof(items[3].s);
    # info.speed = atof(items[4].s);
    # info.COG = atof(items[5].s);
    # info.height = atof(items[6].s);
    # info.east_speed = atof(items[7].s);
    # info.north_speed = atof(items[8].s);
    # info.vertical_speed = atof(items[9].s);
    # info.x_speed = atof(items[10].s);
    # info.y_speed = atof(items[11].s);
    # info.z_speed = atof(items[12].s);
    # info.heading = atof(items[13].s);
    # info.pitch = atof(items[14].s);
    # info.roll = atof(items[15].s);
    # info.x_ENU_rate = atof(items[16].s);
    # info.y_ENU_rate = atof(items[17].s);
    # info.z_ENU_rate = atof(items[18].s);
    # info.x_Oxbybzb_rate = atof(items[19].s);
    # info.y_Oxbybzb_rate = atof(items[20].s);
    # info.z_Oxbybzb_rate = atof(items[21].s);
    # m_pUI->naviDat_OK(info);


    return result

def IMU_fix(data,redis_conn):


    #data = "$RATTM,140,748881789199581324,25.4,210.0,130.5,15.3,0,24.516265008631695,118.22852733943805,123.4,27.4,43,60,1,1,0,Stella;AIS,XINJIEAN,143255580,,15,,121.0,0,,10,33,10,1,0,0,,2023/6/19 15:39:32*5F"
    #        $RATTM, 8992584, 22.154211780083802, 83808.80171563302, 26.838413812913657, 152.62775307496568, 281.22104, 14.324516, 0, 30.681089984145917, 122.45148029953938, 248.7, 14.8, 39, 50, 0, 1, 0, AIS,, 412408580,, 15, 0, 2900, 2024 / 0

    # 按逗号分隔字符串
    data_list = data.split(',')

    # if(float(data_list[6])==40.0  and float(data_list[7])==116.0):
    #     return None



    #data_list[3]  = str( (float (data_list[3]) +90 ) %360)
    data_list[3]  = str( float (data_list[3])  )

    return ','.join(data_list)


def convert_to_decimal(degree, minute, direction):
    # 计算纬度或经度的小数表示
    decimal = float(degree) + float(minute) / 60.0
    if direction in ['S', 'W']:  # 南纬或西经是负值
        decimal = -decimal
    return decimal


# 解析GPGGA和GPHDT消息
def parse_gpgga(message):
    global now_os_abs_xy
    parts = message.strip().split(',')
    if len(parts) < 15:
        return None

    time = parts[1]
    latitude = parts[2]
    latitude_dir = parts[3]
    longitude = parts[4]
    longitude_dir = parts[5]

    # 解析经纬度（假设是度分格式，注意去除多余的0）
    lat_deg = latitude[:2]  # 纬度的前两位是度数
    lat_min = latitude[2:]  # 后面的部分是分钟数
    lon_deg = longitude[:3]  # 经度的前三位是度数
    lon_min = longitude[3:]  # 后面的部分是分钟数

    lat_decimal = convert_to_decimal(lat_deg, lat_min, latitude_dir)
    lon_decimal = convert_to_decimal(lon_deg, lon_min, longitude_dir)
    now_os_abs_xy = convert_lonlat_to_abs_xy((lon_decimal, lat_decimal), 1)

    return {
        "Time": time,
        "Latitude": lat_decimal,
        "Longitude": lon_decimal
    }


def parse_gphdt(message):
    global now_os_heading
    parts = message.strip().split(',')
    if len(parts) < 2:
        return None

    heading = parts[1]
    now_os_heading = float(heading)

    return {
        "Heading": heading
    }




def parse_gpvtg(message):
    global now_os_speed, now_os_course
    parts = message.strip().split(',')
    if len(parts) < 9:
        return None

    true_course = parts[1]  # 真实航向（真北）
    mag_course = parts[3]  # 磁航向（磁北）
    speed_kmh = parts[7]  # 速度（公里/小时）
    speed_knots = parts[5]  # 速度（节）
    now_os_speed = float(parts[5])*0.514444
    now_os_course = float(parts[1])

    return {
        "TrueCourse": true_course,
        "MagCourse": mag_course,
        "Speed_Kmh": speed_kmh,
        "Speed_Knots": speed_knots
    }
def parse_gpsxt(sentence):
    if not sentence.startswith("$GPSXT"):
        raise ValueError("Not a GPSXT sentence")

    # 去除前缀 `$GPSXT,` 和末尾的校验和部分
    body = sentence.split("*")[0]
    fields = body.split(',')

    # 提取字段
    timestamp = fields[1]
    longitude = float(fields[2])         # 经度
    latitude = float(fields[3])          # 纬度
    heading = float(fields[5])           # 首向角
    course = float(fields[7])            # 航向角
    speed = float(fields[8])             # 速度大小

    return {

        "Latitude": latitude,
        "Longitude": longitude,
        "Heading": heading,
        "course": course,
        "speed": speed
    }


# 测试解析
gpvtg_message = "$GPVTG,272.746,T,273.008,M,9.219,N,17.038,K*37"
gpvtg_data = parse_gpvtg(gpvtg_message)

print("GPVTG Parsed Data:", gpvtg_data)

def IMU_check_and_toredis_YYDH(data,redis_conn):

        # 按逗号分隔字符串
        data_list = data.split(',')
        print("$GPSXT" + data_list[0] )
        if data_list[0] == "$GPSXT":
            gpsxt_data = parse_gpsxt(data)

            redis_conn.hset("IMU", "Lon", gpsxt_data["Longitude"])
            redis_conn.hset("IMU", "Lat", gpsxt_data["Latitude"])
            redis_conn.hset("IMU", "heading", gpsxt_data["Heading"])
            redis_conn.hset("IMU", "speed", gpsxt_data["speed"])
            redis_conn.hset("IMU", "angle", gpsxt_data["course"])
            redis_conn.expire("IMU", 5)



def IMU_check_and_toredis_UniStrong(data,redis_conn):

        # 按逗号分隔字符串
        data_list = data.split(',')
        print("$GPVVV" + data_list[0] )
        if data_list[0] == "$GNGGA":
            gpgga_data = parse_gpgga(data)
            redis_conn.hset("IMU", "unix_time", gpgga_data["Time"])
            redis_conn.hset("IMU", "Lon", gpgga_data["Longitude"])
            redis_conn.hset("IMU", "Lat", gpgga_data["Latitude"])
            redis_conn.expire("IMU", 5)
# $GNTHS,283.7410,A*12

        elif data_list[0] == "$GNTHS":
            gphdt_data = parse_gphdt(data)
            redis_conn.hset("IMU", "heading", gphdt_data["Heading"])
            redis_conn.expire("IMU", 5)
        elif data_list[0] == "$GNVTG":
            gpvtg_data = parse_gpvtg(data)
            print(gpvtg_data)
            redis_conn.hset("IMU", "angle", float(gpvtg_data["TrueCourse"]))
            redis_conn.hset("IMU", "speed", float(gpvtg_data["Speed_Kmh"])/3.6)
            redis_conn.hset("IMU", "Speed_Kmh", gpvtg_data["Speed_Kmh"])
            redis_conn.hset("IMU", "Speed_Knots", gpvtg_data["Speed_Knots"])
            redis_conn.expire("IMU", 5)



def send_udp_messageXX(message):
    # 1. 配置参数
    # 本地绑定的端口
    local_port = 20012
    # 目标服务器IP和端口
    target_ip = "193.0.1.81"
    target_port = 9999
    # 要发送的字符串报文

    try:
        # 2. 创建UDP套接字 (SOCK_DGRAM 表示UDP协议)
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # 3. 绑定本地端口
        # 第一个参数为空字符串表示绑定到所有可用的本地IP地址
        udp_socket.bind(("", local_port))
        print(f"成功绑定本地端口: {local_port}")

        # 4. 将字符串编码为字节流（UDP传输必须是字节类型）
        send_data = message.encode("utf-8")

        # 5. 发送UDP报文到目标地址
        udp_socket.sendto(send_data, (target_ip, target_port))
        print(f"成功发送报文到 {target_ip}:{target_port}")
        print(f"发送的内容: {message}")

    except Exception as e:
        print(f"发送过程中出现错误: {e}")
    finally:
        # 6. 关闭套接字
        udp_socket.close()
        print("套接字已关闭")

def IMU_check_and_toredis_UniStrong_X(data,redis_conn):

        # 配置目标服务器信息
        TARGET_IP = "193.0.1.81"
        TARGET_PORT = 19999

        # 创建UDP socket对象
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        print("SFAFAF"+data)
        data_lines = data.splitlines()

        # 遍历每一行数据进行处理
        for line in data_lines:
            # 去除行首尾的空白字符（如空格、制表符），避免空行或无效数据干扰
            line_stripped = line.strip()
            if not line_stripped:  # 跳过空行
                continue



            send_udp_messageXX(line)

            
            # 按逗号分隔当前行的字符串
            data_list = line_stripped.split(',')
            # 增加边界检查：避免分割后无数据导致索引越界
            if len(data_list) == 0:
                continue
            
            print(f"处理数据行: ${data_list[0]}")
            
            if data_list[0] == "$GNGGA":
                try:
                    gpgga_data = parse_gpgga(line_stripped)
                    redis_conn.hset("IMU", "unix_time", gpgga_data["Time"])
                    redis_conn.hset("IMU", "Lon", gpgga_data["Longitude"])
                    redis_conn.hset("IMU", "Lat", gpgga_data["Latitude"])
                    redis_conn.expire("IMU", 5)
                except Exception as e:
                    print(f"解析$GNGGA数据失败: {e}")
            
            elif data_list[0] == "$GNTHS":
                try:
                    gphdt_data = parse_gphdt(line_stripped)
                    redis_conn.hset("IMU", "heading", gphdt_data["Heading"])
                    redis_conn.expire("IMU", 5)
                except Exception as e:
                    print(f"解析$GNTHS数据失败: {e}")
            
            elif data_list[0] == "$GNVTG":
                try:
                    gpvtg_data = parse_gpvtg(line_stripped)
                    print(f"$GNVTG解析结果: {gpvtg_data}")
                    redis_conn.hset("IMU", "angle", float(gpvtg_data["TrueCourse"]))
                    redis_conn.hset("IMU", "speed", float(gpvtg_data["Speed_Kmh"])/3.6)
                    redis_conn.hset("IMU", "Speed_Kmh", gpvtg_data["Speed_Kmh"])
                    redis_conn.hset("IMU", "Speed_Knots", gpvtg_data["Speed_Knots"])
                    redis_conn.expire("IMU", 5)
                except Exception as e:
                    print(f"解析$GNVTG数据失败: {e}")

            
        # 关闭socket连接
        udp_socket.close()
def IMU_check_and_toredis_UniStrong1(data,redis_conn):
    for line in data.split('\r\n'):
        # 按逗号分隔字符串
        data_list = line.split(',')
        print("$GPVVV" + data_list[0] )
        if data_list[0] == "$GPGGA":
            gpgga_data = parse_gpgga(line)
            redis_conn.hset("IMU", "unix_time", gpgga_data["Time"])
            redis_conn.hset("IMU", "Lon", gpgga_data["Longitude"])
            redis_conn.hset("IMU", "Lat", gpgga_data["Latitude"])
            redis_conn.expire("IMU", 5)

        elif data_list[0] == "$GPHDT":
            gphdt_data = parse_gphdt(line)
            redis_conn.hset("IMU", "heading", gphdt_data["Heading"])
            redis_conn.expire("IMU", 5)
        elif data_list[0] == "$GPVTG":
            gpvtg_data = parse_gpvtg(line)
            print(gpvtg_data)
            redis_conn.hset("IMU", "angle", gpvtg_data["TrueCourse"])
            redis_conn.hset("IMU", "speed", float(gpvtg_data["Speed_Kmh"])/3.6)
            redis_conn.hset("IMU", "Speed_Kmh", gpvtg_data["Speed_Kmh"])
            redis_conn.hset("IMU", "Speed_Knots", gpvtg_data["Speed_Knots"])
            redis_conn.expire("IMU", 5)




def IMU_check_and_toredis(data,redis_conn):
    global g_lon
    global g_lat
    global now_os_speed
    global now_os_abs_xy
    global now_os_course
    global now_os_heading

    #data = "$RATTM,140,748881789199581324,25.4,210.0,130.5,15.3,0,24.516265008631695,118.22852733943805,123.4,27.4,43,60,1,1,0,Stella;AIS,XINJIEAN,143255580,,15,,121.0,0,,10,33,10,1,0,0,,2023/6/19 15:39:32*5F"
    #        $RATTM, 8992584, 22.154211780083802, 83808.80171563302, 26.838413812913657, 152.62775307496568, 281.22104, 14.324516, 0, 30.681089984145917, 122.45148029953938, 248.7, 14.8, 39, 50, 0, 1, 0, AIS,, 412408580,, 15, 0, 2900, 2024 / 0

    # 按逗号分隔字符串
    data_list = data.split(',')

    if (float(data_list[6]) == 40.0 and float(data_list[7]) == 116.0):
        return None

    # GPS 纪元的起始日期
    gps_epoch = datetime(1980, 1, 6)

    # 每周的秒数
    seconds_per_week = 7 * 24 * 60 * 60

    # 计算总秒数
    total_seconds = int(data_list[1]) * seconds_per_week + float( data_list[2])

    # 计算 GPS 时间
    gps_time = gps_epoch + timedelta(seconds=total_seconds)

    # 转换为 Unix 时间戳
    unix_time = int((gps_time - datetime(1970, 1, 1)).total_seconds())

    east_speed = float( data_list[9])
    north_speed = float( data_list[10])
    # 计算水平速度（东向速度和北向速度的平方和开方）
    horizontal_speed = math.sqrt(east_speed ** 2 + north_speed ** 2)
    now_os_speed = horizontal_speed
    lon = float(data_list[7])
    lat = float(data_list[6])
    now_os_abs_xy = convert_lonlat_to_abs_xy((lon, lat), 1)

    angle = math.degrees(math.atan2(east_speed, north_speed))
    now_os_course = angle
    
    heading = float(data_list[3])
    now_os_heading = heading
    # 确保角度在0到360度之间
    if angle < 0:
        angle += 360
    if heading < 0:
        heading += 360

    data_list[1]  = str(unix_time)
    data_list[2]  = str(angle)  #COG
    data_list[18] = str(horizontal_speed)  # speed
    #data_list[3]  = str( (float (data_list[3]) +90 ) %360)
    data_list[3]  = str( float (data_list[3])  )



    redis_conn.hset("IMU","unix_time", unix_time)
    redis_conn.hset("IMU","Lon",  data_list[7])
    redis_conn.hset("IMU","Lat",  data_list[6])
    redis_conn.hset("IMU","angle", angle)
    redis_conn.hset("IMU","horizontal_speed", horizontal_speed)
    redis_conn.hset("IMU","heading", heading)
    redis_conn.hset("IMU","speed",  data_list[18])
    redis_conn.hset("IMU","height",  data_list[8])




    return ','.join(data_list).encode('utf-8')


def decimal_to_dms(degree):
    # 提取度的整数部分
    degrees = int(degree)
    # 小数部分乘以60得到分钟
    minutes = (degree - degrees) * 60
    # 保留四位小数
    return f"{degrees:02d}{minutes:06.4f}"



def generate_gprmc_date():
    """生成GPRMC协议中的日期字段（格式：DDMMYY）"""
    # 获取当前本地时间
    now = datetime.utcnow()
    
    # 格式化日期为GPRMC日期格式：DDMMYY
    date_str = now.strftime("%d%m%y")
    
    return date_str


# GPS报文生成函数
def generate_gps_sentence(lat, lon, timestamp,redis_conn):
    heading = get_or_default(redis_conn, "IMU", "heading", '0')
    angle = get_or_default(redis_conn, "IMU", "angle", '0')
    speed = get_or_default(redis_conn, "IMU", "speed", '0')
    speed = format(float(speed)/0.5144444,'.1f')

    gprmc_date = generate_gprmc_date()

    # 生成 GPRMC
    gprmc = f"$GPRMC,{timestamp},A,{lat},N,{lon},E,{speed},{angle},{gprmc_date},004.9,W,A"
    gprmc_checksum = calculate_checksum(gprmc)
    gprmc = f"{gprmc}*{gprmc_checksum}"

    # 生成 GPVTG
    gpvtg = f"$GPVTG,0.00,T,,M,0.05,N,0.1,K,A"
    gpvtg_checksum = calculate_checksum(gpvtg)
    gpvtg = f"{gpvtg}*{gpvtg_checksum}"

    # 生成 GPGGA
    gpgga = f"$GPGGA,{timestamp}.000,{lat},N,{lon},E,1,09,0.9,0.4,M,0.0,M,,0000"
    gpgga_checksum = calculate_checksum(gpgga)
    gpgga = f"{gpgga}*{gpgga_checksum}"

    # 生成 GPGSA
    gpgsa = "$GPGSA,A,3,02,04,07,08,09,16,21,27,30,,,,1.5,0.9,1.2"
    gpgsa_checksum = calculate_checksum(gpgsa)
    gpgsa = f"{gpgsa}*{gpgsa_checksum}"

    hehdt = f"$HEHDT,{heading},T"
    hehdt_checksum = calculate_checksum(hehdt)
    hehdt = f"{hehdt}*{hehdt_checksum}"

    # 返回所有报文
    return gprmc, gpvtg, gpgga, gpgsa,hehdt
# 计算校验和
def calculate_checksum(sentence):
    sentence = sentence.strip('$').split('*')[0]
    checksum = 0
    for char in sentence:
        checksum ^= ord(char)
    return f"{checksum:02X}"

# 当前时间戳
timestamp = "074315"  # 根据需要修改为实际的时间戳


def get_gps_week_and_sow():
    # GPS 历元时间 (1980-01-06 00:00:00 UTC)
    gps_epoch = datetime(1980, 1, 6, 0, 0, 0, tzinfo=timezone.utc)
    current_time = datetime.now(timezone.utc)  # 当前 UTC 时间
    
    # 计算时间差
    delta = current_time - gps_epoch
    total_seconds = delta.total_seconds()
    
    # 计算 GPS 周和周内秒
    gps_week = int(total_seconds // (7 * 86400))  # 1 周 = 7 天 × 86400 秒
    gps_sow = total_seconds % (7 * 86400)         # 当前周内的秒数
    
    return gps_week, gps_sow


def IMU_toBDFPDL(data,redis_conn):
    # $BDFPDL,2370,127259.495,195.90981,2.15864,-0.35153,29.19243055507,122.17518774472,27.7410,-1.6146,-3.7892,0.0766,4.1196,-0.709,0.767,-0.035,0.113,0.225,9.767,28,28,16,0,48*04
    # f"$BDFPDL,2370,127259.495,{heading},2.15864,-0.35153,{lat},{lon},27.7410,-1.6146,-3.7892,0.0766,{speed},-0.709,0.767,-0.035,0.113,0.225,9.767,28,28,16,0,48*04\r\n"
    gps_week, gps_sow = get_gps_week_and_sow()
    global timestamp
    lon = get_or_default(redis_conn, "IMU", "Lon", '0')
    lat = get_or_default(redis_conn, "IMU", "Lat", '0')
    speed = get_or_default(redis_conn, "IMU", "speed", '0')
    # height = get_or_default(redis_conn, "IMU", "height", '0')
    angle = get_or_default(redis_conn, "IMU", "angle", '0')
    heading = get_or_default(redis_conn, "IMU", "heading", '0')
    v_n = float(speed) * math.cos(math.radians(float(angle)))
    v_e = float(speed) * math.sin(math.radians(float(angle)))

    # data=f"$BDFPDL,{timestamp},363053.460,{heading},-1.51142,1.27360,{lat},{lon},{height},{v_e},{v_n},0,{speed},0.020,-0.096,-0.006,0.202,-0.286,9.811,27,28,16,50,48*25\r\n"
    data =    f"$BDFPDL,{gps_week},{gps_sow},{heading},2.15864,-0.35153,{lat},{lon},27.7410,-1.6146,-3.7892,0.0766,{speed},-0.709,0.767,-0.035,0.113,0.225,9.767,28,28,16,0,48*04\r\n"

    # 更新时间戳
    timestamp = str(int(timestamp) + 1) # 每次更新时间戳
    print("taaa"+data)
    return data.encode('utf-8')

def IMU_toUSV(data,redis_conn):

    new_list = [''] * 22  # 新建一个和data_list长度相同的字符串数组，默认空字符串
    lon = get_or_default(redis_conn, "IMU", "Lon", '0')
    lat = get_or_default(redis_conn, "IMU", "Lat", '0')
    speed = get_or_default(redis_conn, "IMU", "speed", '0')
    height = get_or_default(redis_conn, "IMU", "height", '0')
    angle = get_or_default(redis_conn, "IMU", "angle", '0')
    heading = get_or_default(redis_conn, "IMU", "heading", '0')

    # 假设你想从data_list取一部分内容并赋值给new_list
    # 举个例子，取前3个元素并赋值
    new_list[0] = "$NAVIDAT"
    new_list[1] = str(time.time())
    new_list[2] = lat  # LAT
    new_list[3] = lon # LON



    new_list[4] = speed  # speed
    new_list[5] = angle  # COG
    new_list[6] = height  # height

    # new_list[7] = data_list[9]  # dong
    # new_list[8] = data_list[10]  # bei
    # new_list[9] = data_list[11]  # tian

    new_list[13] = heading  # heading

    # new_list[14] = data_list[4]  # pitch
    # new_list[15] = data_list[5]  # roll

    # 最后将new_list按逗号分割组成字符串
    result = ','.join(new_list)
    print("XAXA"+result)


    #$NAVIDAT, 1739417482.9440384, 29.94610436742, 122.30601878969, 0.009007774419910837, 67.13549187810035, 20.1988, 0.0083, 0.0035, -0.0068,, , , 78.28851, -1.39286, 1.32869,, , , , ,
    return  result.encode('utf-8')


def IMU_to大连雷达(data,redis_conn):
    global timestamp


    lon = get_or_default(redis_conn, "IMU", "Lon", '0')
    lat = get_or_default(redis_conn, "IMU", "Lat", '0')



    latitude = decimal_to_dms(float(lat))
    longitude = decimal_to_dms(float(lon))

    # 生成所有报文
    gprmc, gpvtg, gpgga, gpgsa ,hehdt = generate_gps_sentence(latitude, longitude, timestamp,redis_conn)
    print(gprmc + "\r\n", gpvtg + "\r\n", gpgga + "\r\n", gpgsa + "\r\n")

    # sen = gprmc + "\r\n" + gpvtg + "\r\n" + gpgga + "\r\n" + gpgsa + "\r\n"
    sen = gprmc + "\r\n" + hehdt +"\r\n"
    # sen = "$HEHDT,158.2,T*21"+ "\r\n"
    # 更新时间戳
    # timestamp = str(int(timestamp) + 1).zfill(6)  # 每次更新时间戳
    print("SENX"+sen)

    return sen.encode('utf-8')



def generate_gprmc_timestamp():
    """生成GPRMC协议中的timestamp字段（格式：HHMMSS.sss）"""
    # 获取当前本地时间
    now = datetime.utcnow()
    
    # 格式化时间为GPRMC timestamp格式
    # HHMMSS.sss 其中前6位是时分秒，后面是三位毫秒
    timestamp = now.strftime("%H%M%S.%f")[:-3]  # 取前3位毫秒
    
    return timestamp


def IMU_toSimradar(data,redis_conn):
    global timestamp
    gprmc_ts = generate_gprmc_timestamp()

    lon = get_or_default(redis_conn, "IMU", "Lon", '0')
    lat = get_or_default(redis_conn, "IMU", "Lat", '0')



    latitude = decimal_to_dms(float(lat))
    longitude = decimal_to_dms(float(lon))

    # 生成所有报文
    gprmc, gpvtg, gpgga, gpgsa ,hehdt = generate_gps_sentence(latitude, longitude, gprmc_ts,redis_conn)
    print(gprmc + "\r\n", gpvtg + "\r\n", gpgga + "\r\n", gpgsa + "\r\n")

    # sen = gprmc + "\r\n" + gpvtg + "\r\n" + gpgga + "\r\n" + gpgsa + "\r\n"
    sen = gprmc + "\r\n" 
    # sen = "$HEHDT,158.2,T*21"+ "\r\n"
    # 更新时间戳
    # timestamp = str(int(timestamp) + 1).zfill(6)  # 每次更新时间戳
    print("SENX"+sen)

    return sen.encode('utf-8')

def IMU_toSimradar1(data,redis_conn):
    global timestamp

    gprmc_ts = generate_gprmc_timestamp()

    lon = get_or_default(redis_conn, "IMU", "Lon", '0')
    lat = get_or_default(redis_conn, "IMU", "Lat", '0')



    latitude = decimal_to_dms(float(lat))
    longitude = decimal_to_dms(float(lon))

    # 生成所有报文
    gprmc, gpvtg, gpgga, gpgsa ,hehdt = generate_gps_sentence(latitude, longitude, gprmc_ts,redis_conn)
    print(gprmc + "\r\n", gpvtg + "\r\n", gpgga + "\r\n", gpgsa + "\r\n")

    # sen = gprmc + "\r\n" + gpvtg + "\r\n" + gpgga + "\r\n" + gpgsa + "\r\n"
    sen =  hehdt +"\r\n"
    # sen = "$HEHDT,158.2,T*21"+ "\r\n"
    # 更新时间戳
    # timestamp = str(int(timestamp) + 1).zfill(6)  # 每次更新时间戳
    print("SENX"+sen)

    return sen.encode('utf-8')

def IMU_from_redis(data,redis_conn):
    print("$$$$$$$$$$$$$$$$$$$$$$$$$$@@@@@@@@@")


    print("$BDFPDL")
   # print( redis_conn.hget("IMU", "unix_time"))

    print(get_or_default(redis_conn, "IMU", "unix_time", '0'))
    print(get_or_default(redis_conn, "IMU", "Lon", '0'))
    print(get_or_default(redis_conn, "IMU", "Lat", '0'))
    print(get_or_default(redis_conn, "IMU", "angle", '0'))
    print(get_or_default(redis_conn, "IMU", "horizontal_speed", '0'))
    print(get_or_default(redis_conn, "IMU", "heading", '0'))
    print(get_or_default(redis_conn, "IMU", "speed", '0'))
    print(get_or_default(redis_conn, "IMU", "height", '0'))

    data_list=[]
    data_list.append("$BDFPDL")
    data_list.append(get_or_default(redis_conn, "IMU", "unix_time", '0'))
    data_list.append(get_or_default(redis_conn, "IMU", "Lon", '0'))
    data_list.append(get_or_default(redis_conn, "IMU", "Lat", '0'))
    data_list.append(get_or_default(redis_conn, "IMU", "angle", '0'))
    data_list.append(get_or_default(redis_conn, "IMU", "horizontal_speed", '0'))
    data_list.append(get_or_default(redis_conn, "IMU", "heading", '0'))
    data_list.append(get_or_default(redis_conn, "IMU", "speed", '0'))
    data_list.append(get_or_default(redis_conn, "IMU", "height", '0'))

    print("################"+ ','.join(data_list))
    return ','.join(data_list).encode("utf-8")

def yuchuan_Object_toredis1(data,redis_conn):

    return data

def yuchuan_Object_toredis(data,redis_conn):
    #data = "$RATTM,140,748881789199581324,25.4,210.0,130.5,15.3,0,24.516265008631695,118.22852733943805,123.4,27.4,43,60,1,1,0,Stella;AIS,XINJIEAN,143255580,,15,,121.0,0,,10,33,10,1,0,0,,2023/6/19 15:39:32*5F"
    #        $RATTM, 8992584, 22.154211780083802, 83808.80171563302, 26.838413812913657, 152.62775307496568, 281.22104, 14.324516, 0, 30.681089984145917, 122.45148029953938, 248.7, 14.8, 39, 50, 0, 1, 0, AIS,, 412408580,, 15, 0, 2900, 2024 / 0


    # 按逗号分隔字符串
    data_list = data.split(',')
    # print("$RATX")
    # print(data_list[0])
    # if(data_list[0]!="$RATTM"):
    #     return

    target_id       = data_list[1]
    longitude       = data_list[10]
    latitude        = data_list[9]
    speed           = data_list[12]
    direction       = data_list[11]

    size            = data_list[13]
    cpDistance      = data_list[4]
    cpTime          = data_list[5]
    relativeCourse  = data_list[6]
    relativeSpeed   = data_list[7]
    MMSI            = data_list[20]
    # ownheading = float(get_or_default(redis_conn, "IMU", "heading", '0'))
    # azimuth = (float(data_list[2]) - ownheading +360)%360
    azimuth = (float(data_list[2])+360)%360
    distance = float(data_list[3])
    f_lon = float(longitude)
    f_lat = float(latitude)
    f_course = float(direction)
    f_dcpa = float(cpDistance)
    f_tcpa = float(cpTime)
    f_r_course = float(relativeCourse)
    f_size = float(size)
    f_speed = float(speed) * 0.514444
    # $RATTM,668,313.40266688275744,276.24354387519867,0.02594786618754531,1.7986915876328895,161.29132000000004,0,0,29.90921687236794,122.30967968714037,127.34899999999993,3.88768,29,0,1,0,0,Stella;AIS,XINJIEAN,143255580,0,0,0,0,2023/6/19 15:39:32*5F
    # ownship ais
    if(int(MMSI) == 412422414):
        return
    if(distance <= 50):
        return
    alarm_sw = get_or_default(redis_conn, "Alarmctrl", "alarm_sw", '1')
    if now_os_speed < 1:
        own_course = now_os_heading
    else:
        own_course = now_os_course
    if(alarm_sw == '1'):
        if len(data_list)<=26:
            alarm = alarm_instance.cal_alarm(f_dcpa, f_tcpa,  (f_lon, f_lat), azimuth, own_course, distance, f_course, now_os_speed, f_speed, now_os_abs_xy)     
            cur_time_stamp = datetime.now().timestamp()
            if 0 < alarm < 3:
                if not redis_conn.hget(f"data:{target_id}", "alarmtimestamp"):
                    redis_conn.hset(f"data:{target_id}", "alarmtimestamp", cur_time_stamp)
                    alarm = 0
                elif cur_time_stamp - float(redis_conn.hget(f"data:{target_id}", "alarmtimestamp")) < 3:
                    alarm = 0
                    
            elif alarm == 0 and redis_conn.hget(f"data:{target_id}", "alarmtimestamp"):
                redis_conn.hdel(f"data:{target_id}", "alarmtimestamp")
                
            elif alarm == 3:
                alarm = 2
                # if redis_conn.hget("Navi", "State").decode("utf-8") == "1":
                #     # redis_conn.hset("Navi", "State", 0)
                #     redis_conn.hset("MotorCtrl", "Stop", 1)
                #     redis_conn.expire("MotorCtrl", 180)

        else:
            alarm = data_list[26]
    else:
        alarm=0

    add_data(redis_conn,target_id, longitude,
             latitude, speed, direction,alarm,size=size,
             cpDistance=cpDistance,cpTime=cpTime,
             relativeCourse=relativeCourse,
             relativeSpeed=relativeSpeed, 
             azimuth=azimuth,
             distance=distance)
    return data

def YJ6081_Object_toredis(data,redis_conn):
    #data = "$RATTM,140,748881789199581324,25.4,210.0,130.5,15.3,0,24.516265008631695,118.22852733943805,123.4,27.4,43,60,1,1,0,Stella;AIS,XINJIEAN,143255580,,15,,121.0,0,,10,33,10,1,0,0,,2023/6/19 15:39:32*5F"
    #        $RATTM, 8992584, 22.154211780083802, 83808.80171563302, 26.838413812913657, 152.62775307496568, 281.22104, 14.324516, 0, 30.681089984145917, 122.45148029953938, 248.7, 14.8, 39, 50, 0, 1, 0, AIS,, 412408580,, 15, 0, 2900, 2024 / 0


    print(data)
    # 按逗号分隔字符串
    data_list = data.split(',')
    # print("$RATX")
    # print(data_list[0])
    # if(data_list[0]!="$RATTM"):
    #     return

    target_id       = data_list[1]
    longitude       = data_list[10]
    latitude        = data_list[9]
    speed           = data_list[12]
    direction       = data_list[11]

    size            = data_list[13]
    cpDistance      = data_list[4]
    cpTime          = data_list[5]
    relativeCourse  = data_list[6]
    relativeSpeed   = data_list[7]
    MMSI            = data_list[20]
    # ownheading = float(get_or_default(redis_conn, "IMU", "heading", '0'))
    # azimuth = (float(data_list[2]) - ownheading +360)%360
    azimuth = (float(data_list[2])+360)%360
    distance = float(data_list[3])
    f_lon = float(longitude)
    f_lat = float(latitude)
    f_course = float(direction)
    f_dcpa = float(cpDistance)
    f_tcpa = float(cpTime)
    f_r_course = float(relativeCourse)
    f_size = float(size)
    f_speed = float(speed) * 0.514444
    # $RATTM,668,313.40266688275744,276.24354387519867,0.02594786618754531,1.7986915876328895,161.29132000000004,0,0,29.90921687236794,122.30967968714037,127.34899999999993,3.88768,29,0,1,0,0,Stella;AIS,XINJIEAN,143255580,0,0,0,0,2023/6/19 15:39:32*5F
    # ownship ais
    
    if(int(MMSI) == 412422414):
        return
    if(distance <= 50):
        return
    print("YJ6081_Object_toredis")

    
    alarm_sw = get_or_default(redis_conn, "Alarmctrl", "alarm_sw", '1')
    
    if now_os_speed < 1:
        own_course = now_os_heading
    else:
        own_course = now_os_course
    if(alarm_sw == '1'):
        if len(data_list)<=26:
            alarm = alarm_instance.cal_alarm(f_dcpa, f_tcpa,  (f_lon, f_lat), azimuth, own_course, distance, f_course, now_os_speed, f_speed, now_os_abs_xy)     
            cur_time_stamp = datetime.now().timestamp()
            if 0 < alarm < 3:
                if not redis_conn.hget(f"data:{target_id}", "alarmtimestamp"):
                    redis_conn.hset(f"data:{target_id}", "alarmtimestamp", cur_time_stamp)
                    alarm = 0
                elif cur_time_stamp - float(redis_conn.hget(f"data:{target_id}", "alarmtimestamp")) < 3:
                    alarm = 0
                    
            elif alarm == 0 and redis_conn.hget(f"data:{target_id}", "alarmtimestamp"):
                redis_conn.hdel(f"data:{target_id}", "alarmtimestamp")
                
            elif alarm == 3:
                alarm = 2
                # if redis_conn.hget("Navi", "State").decode("utf-8") == "1":
                #     # redis_conn.hset("Navi", "State", 0)
                #     redis_conn.hset("MotorCtrl", "Stop", 1)
                #     redis_conn.expire("MotorCtrl", 180)

        else:
            alarm = data_list[26]
    else:
        alarm=0

    print(target_id, longitude,
             latitude, speed, direction,alarm,size,
             cpDistance,cpTime,
             relativeCourse,
             relativeSpeed, 
             azimuth,
             distance)
    add_data(redis_conn,target_id, longitude,
             latitude, speed, direction,alarm,size=size,
             cpDistance=cpDistance,cpTime=cpTime,
             relativeCourse=relativeCourse,
             relativeSpeed=relativeSpeed, 
             azimuth=azimuth,
             distance=distance)
    return data

    

def liaowuer_yuchuan_Object_toredis(data,redis_conn):

# $LRTTM,16,0.6,134.8,T,0.5,223.3,T,0.6,,N,,T,,094309.43,A*28

    #data = "$RATTM,140,748881789199581324,25.4,210.0,130.5,15.3,0,24.516265008631695,118.22852733943805,123.4,27.4,43,60,1,1,0,Stella;AIS,XINJIEAN,143255580,,15,,121.0,0,,10,33,10,1,0,0,,2023/6/19 15:39:32*5F"
    #        $RATTM, 8992584, 22.154211780083802, 83808.80171563302, 26.838413812913657, 152.62775307496568, 281.22104, 14.324516, 0, 30.681089984145917, 122.45148029953938, 248.7, 14.8, 39, 50, 0, 1, 0, AIS,, 412408580,, 15, 0, 2900, 2024 / 0

 

    # 按逗号分隔字符串
    data_list = data.split(',')
    # print("$RATX")
    # print(data_list[0])
    # if(data_list[0]!="$RATTM"):
    #     return

    target_id       = data_list[1]

    # ?longitude       = data_list[10]
    # ?latitude        = data_list[9]
    
    distance = float(data_list[2])*1852.0 #海里转米
    speed           = data_list[5]  # 节转节
    direction       = data_list[6] #目标船对地航向

    size            = 25  # 无size
    # 'cpa_distance': float(fields[8]),  # 海里
    # 'tcpa_time': float(fields[9]),     # 分钟
    cpDistance      = data_list[8]
    cpTime          = data_list[9]

    relativeCourse  = "10"
    relativeSpeed   = "2"
    MMSI            = "123456"
    # ownheading = float(get_or_default(redis_conn, "IMU", "heading", '0'))
    # azimuth = (float(data_list[2]) - ownheading +360)%360
    azimuth = (float(data_list[3])+360)%360   #bearing
    
    ownlon = float(get_or_default(redis_conn, "IMU", "Lon", '0'))
    ownlat = float(get_or_default(redis_conn, "IMU", "Lat", '0'))

    dx = distance*math.sin(math.radians(azimuth))
    dy = distance*math.cos(math.radians(azimuth))
    longitude,latitude = convert_dxy_to_lonlat((dx,dy)
    , (ownlon,ownlat), u2m=1)


    f_lon = float(longitude)
    f_lat = float(latitude)
    f_course = float(direction)
    f_dcpa = float(cpDistance)
    f_tcpa = float(cpTime)
    f_r_course = float(relativeCourse)
    f_size = float(size)
    f_speed = float(speed) * 0.514444
    # $RATTM,668,313.40266688275744,276.24354387519867,0.02594786618754531,1.7986915876328895,161.29132000000004,0,0,29.90921687236794,122.30967968714037,127.34899999999993,3.88768,29,0,1,0,0,Stella;AIS,XINJIEAN,143255580,0,0,0,0,2023/6/19 15:39:32*5F
    # ownship ais
    if(int(MMSI) == 412422414):
        return
    alarm_sw = get_or_default(redis_conn, "Alarmctrl", "alarm_sw", '1')
    if now_os_speed < 1:
        own_course = now_os_heading
    else:
        own_course = now_os_course
    if(alarm_sw == '1'):
        if len(data_list)<=26:
            alarm = alarm_instance.cal_alarm(f_dcpa, f_tcpa,  (f_lon, f_lat), azimuth, own_course, distance, f_course, now_os_speed, f_speed, now_os_abs_xy)     
            cur_time_stamp = datetime.now().timestamp()
            if 0 < alarm < 3:
                if not redis_conn.hget(f"data:{target_id}", "alarmtimestamp"):
                    redis_conn.hset(f"data:{target_id}", "alarmtimestamp", cur_time_stamp)
                    alarm = 0
                elif cur_time_stamp - float(redis_conn.hget(f"data:{target_id}", "alarmtimestamp")) < 3:
                    alarm = 0
                    
            elif alarm == 0 and redis_conn.hget(f"data:{target_id}", "alarmtimestamp"):
                redis_conn.hdel(f"data:{target_id}", "alarmtimestamp")
                
            elif alarm == 3:
                alarm = 2
                if redis_conn.hget("Navi", "State").decode("utf-8") == "1":
                    # redis_conn.hset("Navi", "State", 0)
                    redis_conn.hset("MotorCtrl", "Stop", 1)
                    redis_conn.expire("MotorCtrl", 30)

        else:
            alarm = data_list[26]
    else:
        alarm=0

    add_data(redis_conn,target_id, longitude,
             latitude, speed, direction,alarm,size=size,
             cpDistance=cpDistance,cpTime=cpTime,
             relativeCourse=relativeCourse,
             relativeSpeed=relativeSpeed, 
             azimuth=azimuth,
             distance=distance)
    return data

def gewen_bianhuan(data,redis_conn):

    # 去掉前两个字节（切片操作）
    processed_bytes = data[2:]

    # 转换为字符串（默认使用utf-8编码）
    result_string = processed_bytes.decode()

    # print("@#$%@")  # 输出: cdefgh


     # $LRTTM,16,0.6,134.8,T,0.5,223.3,T,0.6,,N,,T,,094309.43,A*28

    tmp_data = "$RATTM,2816,255.8393618083678,3347.413276451677,1.8015757522746456,1.3548683172007336,255.21001164111001,4.532194997044762,2,29.909709181488008,122.25697442768949,165.6,1.6,119,0,1,0,0,Stella,,0,,0,0,0,2025/02/12 14:39:24*4D"
    data_list = result_string.split(',')
    target_id = data_list[1]

    # ?longitude       = data_list[10]
    # ?latitude        = data_list[9]

    distance = float(data_list[2]) * 1852.0  # 海里转米
    speed = data_list[5]  # 节转节
    direction = data_list[6]  # 目标船对地航向

    size = "25"  # 无size
    # 'cpa_distance': float(fields[8]),  # 海里
    # 'tcpa_time': float(fields[9]),     # 分钟
    cpDistance = data_list[8]
    cpTime = data_list[9]
    if(cpTime== ''):
        cpTime ='999'

    relativeCourse = "10"
    relativeSpeed = "2"
    MMSI = "123456"
    # ownheading = float(get_or_default(redis_conn, "IMU", "heading", '0'))
    # azimuth = (float(data_list[2]) - ownheading +360)%360
    azimuth = (float(data_list[3]) + 360) % 360  # bearing
    print("LRTTM")
    ownlon = float(get_or_default(redis_conn, "IMU", "Lon", '0'))
    ownlat = float(get_or_default(redis_conn, "IMU", "Lat", '0'))
    #
    # dx = distance*math.sin(math.radians(azimuth))
    # dy = distance*math.cos(math.radians(azimuth))
    # longitude,latitude = convert_dxy_to_lonlat((dx,dy)
    # , (ownlon,ownlat), u2m=1)
    #

    geod = Geod(ellps="WGS84")

    longitude, latitude, back_az = geod.fwd(ownlon, ownlat, azimuth, distance)

    # f_lon = float(longitude)
    # f_lat = float(latitude)
    # f_course = float(direction)
    # f_dcpa = float(cpDistance)
    # f_tcpa = float(cpTime)
    # f_r_course = float(relativeCourse)
    # f_size = float(size)
    # f_speed = float(speed) * 0.514444


    # $RATTM,668,313.40266688275744,276.24354387519867,0.02594786618754531,1.7986915876328895,161.29132000000004,0,0,29.90921687236794,122.30967968714037,127.34899999999993,3.88768,29,0,1,0,0,Stella;AIS,XINJIEAN,143255580,0,0,0,0,2023/6/19 15:39:32*5F
    # ownship ais


    tmp_data_list = tmp_data.split(',')

    # alarm = get_or_default(redis_conn, f"data:{target_id}", "Alarmstufe", '0')
    # tmp_data_list.append(alarm)

    tmp_data_list[1] = target_id
    tmp_data_list[10] = str(longitude)
    tmp_data_list[9] = str(latitude)
    tmp_data_list[12] = speed
    tmp_data_list[11] = direction

    tmp_data_list[13] = size
    tmp_data_list[4] =  cpDistance
    tmp_data_list[5] = cpTime
    tmp_data_list[6]  = relativeCourse
    tmp_data_list[7] = relativeSpeed
    tmp_data_list[20] = MMSI
    tmp_data_list[2] = str(azimuth)
    tmp_data_list[3] = str(distance)

    print(','.join(tmp_data_list).encode('utf-8'))
    return ','.join(tmp_data_list).encode('utf-8')

def does_nothing_bianhuan(data):
    print("dddddddddddddddddddddddddddddddddddddddddddd",data)
    return data

def liaowuer_bianhuan(data,redis_conn):
    # $LRTTM,16,0.6,134.8,T,0.5,223.3,T,0.6,,N,,T,,094309.43,A*28

    tmp_data = "$RATTM,2816,255.8393618083678,3347.413276451677,1.8015757522746456,1.3548683172007336,255.21001164111001,4.532194997044762,2,29.909709181488008,122.25697442768949,165.6,1.6,119,0,1,0,0,Stella,,0,,0,0,0,2025/02/12 14:39:24*4D"
    data_list = data.split(',')
    target_id = data_list[1]

    # ?longitude       = data_list[10]
    # ?latitude        = data_list[9]

    distance = float(data_list[2]) * 1852.0  # 海里转米
    speed = data_list[5]  # 节转节
    direction = data_list[6]  # 目标船对地航向

    size = "25"  # 无size
    # 'cpa_distance': float(fields[8]),  # 海里
    # 'tcpa_time': float(fields[9]),     # 分钟
    cpDistance = data_list[8]
    cpTime = data_list[9]
    if(cpTime== ''):
        cpTime ='999'

    relativeCourse = "10"
    relativeSpeed = "2"
    MMSI = "123456"
    # ownheading = float(get_or_default(redis_conn, "IMU", "heading", '0'))
    # azimuth = (float(data_list[2]) - ownheading +360)%360
    azimuth = (float(data_list[3]) + 360) % 360  # bearing
    print("LRTTM")
    ownlon = float(get_or_default(redis_conn, "IMU", "Lon", '0'))
    ownlat = float(get_or_default(redis_conn, "IMU", "Lat", '0'))
    #
    # dx = distance*math.sin(math.radians(azimuth))
    # dy = distance*math.cos(math.radians(azimuth))
    # longitude,latitude = convert_dxy_to_lonlat((dx,dy)
    # , (ownlon,ownlat), u2m=1)
    #

    geod = Geod(ellps="WGS84")

    longitude, latitude, back_az = geod.fwd(ownlon, ownlat, azimuth, distance)

    # f_lon = float(longitude)
    # f_lat = float(latitude)
    # f_course = float(direction)
    # f_dcpa = float(cpDistance)
    # f_tcpa = float(cpTime)
    # f_r_course = float(relativeCourse)
    # f_size = float(size)
    # f_speed = float(speed) * 0.514444


    # $RATTM,668,313.40266688275744,276.24354387519867,0.02594786618754531,1.7986915876328895,161.29132000000004,0,0,29.90921687236794,122.30967968714037,127.34899999999993,3.88768,29,0,1,0,0,Stella;AIS,XINJIEAN,143255580,0,0,0,0,2023/6/19 15:39:32*5F
    # ownship ais


    tmp_data_list = tmp_data.split(',')

    # alarm = get_or_default(redis_conn, f"data:{target_id}", "Alarmstufe", '0')
    # tmp_data_list.append(alarm)

    tmp_data_list[1] = target_id
    tmp_data_list[10] = str(longitude)
    tmp_data_list[9] = str(latitude)
    tmp_data_list[12] = speed
    tmp_data_list[11] = direction

    tmp_data_list[13] = size
    tmp_data_list[4] =  cpDistance
    tmp_data_list[5] = cpTime
    tmp_data_list[6]  = relativeCourse
    tmp_data_list[7] = relativeSpeed
    tmp_data_list[20] = MMSI
    tmp_data_list[2] = str(azimuth)
    tmp_data_list[3] = str(distance)

    print(','.join(tmp_data_list).encode('utf-8'))
    return ','.join(tmp_data_list).encode('utf-8')

def yuchuan_Object_toUSV(data,redis_conn):
    #data = "$RATTM,140,748881789199581324,25.4,210.0,130.5,15.3,0,24.516265008631695,118.22852733943805,123.4,27.4,43,60,1,1,0,Stella;AIS,XINJIEAN,143255580,,15,,121.0,0,,10,33,10,1,0,0,,2023/6/19 15:39:32*5F"
    #        $RATTM, 8992584, 22.154211780083802, 83808.80171563302, 26.838413812913657, 152.62775307496568, 281.22104, 14.324516, 0, 30.681089984145917, 122.45148029953938, 248.7, 14.8, 39, 50, 0, 1, 0, AIS,, 412408580,, 15, 0, 2900, 2024 / 0

    # 按逗号分隔字符串
    data_list = data.split(',')
    target_id       = data_list[1]
    longitude       = data_list[10]
    latitude        = data_list[9]
    speed           = data_list[12]
    direction       = data_list[11]

    distance = data_list[3]
    print(float(distance))
    print("########%%%%%%%%%%")


    if(float(distance)>2*1852):
        return

    State           = data_list[8]
    vesselType      = data_list[14]
    radarType       = data_list[15]
    aisType         = data_list[16]
    beidouType      = data_list[17]
    sourceNames     = data_list[18]
    aisName         = data_list[19]

    MMSI            = data_list[20]

    bTerminalID     = data_list[21]
    # NavigationStatus            = data_list[22]
    # RateofTurn            = data_list[23]
    # Heading            = data_list[24]
    recordTime            = data_list[25]


    size            = data_list[13]
    cpDistance      = data_list[4]
    cpTime          = data_list[5]
    relativeCourse  = data_list[6]
    relativeSpeed   = data_list[7]

    azimuth = float(data_list[2])
    distance = float(data_list[3])
    f_lon = float(longitude)
    f_lat = float(latitude)
    f_course = float(direction)
    f_dcpa = float(cpDistance)
    f_tcpa = float(cpTime)
    f_r_course = float(relativeCourse)
    f_size = float(size)
    f_speed = float(speed) * 0.514444

    alarm_sw = get_or_default(redis_conn, "Alarmctrl", "alarm_sw", '1')
    # if(alarm_sw == '1'):
    #     if len(data_list)<=26:
    #         alarm = alarm_instance.cal_alarm(f_dcpa, f_tcpa,  (f_lon, f_lat), azimuth, f_r_course, distance, f_course, now_os_speed, f_speed, now_os_abs_xy)
    #         if alarm == 3:
    #             alarm = 2
    #             if redis_conn.hget("Navi", "State").decode("utf-8") == "1":
    #                 redis_conn.hset("Navi", "State", 0)
    #                 redis_conn.hset("MotorCtrl", "Stop", 1)
    #                 redis_conn.expire("MotorCtrl", 180)
    #     else:
    #         alarm = data_list[26]
    # else:
    #     alarm=0

    if(alarm_sw == '1'):
        if len(data_list)<=26:
            alarm = get_or_default(redis_conn, f"data:{target_id}", "Alarmstufe", '0')
        else:
            alarm = data_list[26]
    else:
        alarm=0

    print(alarm)
    print("alarm")
    data_dict = {
        "TarID": int(target_id),
        "UniqueID": 749590828170608781,
        "State": int(State),
        "Lat": float(latitude),
        "Lon": float(longitude),
        "Course": float(direction),
        "Speed": float(speed),
        "Size": int(size),
        "vesselType": float(vesselType),
        "radarType": float(radarType),
        "aisType": float(aisType),
        "beidouType":float( beidouType),
        "sourceNames": sourceNames,
        "aisName": aisName,
        "MMSI": int(MMSI),
        "bTerminalID": bTerminalID,
        "recordTime": recordTime,
        "alarm":int(alarm),
        "dcpa": f_dcpa * 1852,
        "tcpa": f_tcpa, 
    }

    print("yuchuan_Object_toUSV")


    json_string = json.dumps(data_dict)
    print(json_string)
    return json_string


def yuchuan_Object_toWebMap(data,redis_conn):


    data_list = data.split(',')

    distance = data_list[3]
    if(float(distance)<=50.0):
        return ""
    target_id       = data_list[1]
    alarm = get_or_default(redis_conn, f"data:{target_id}", "Alarmstufe", '0')
    data_list.append(alarm)
    return ','.join(data_list).encode('utf-8')

def liaowuer_yuchuan_Object_toWebMap(data,redis_conn):
    data_list = data.split(',')
    target_id       = data_list[1]
    alarm = get_or_default(redis_conn, f"data:{target_id}", "Alarmstufe", '0')
    data_list.append(alarm)
    return ','.join(data_list).encode('utf-8')


def GPS_data_toredis(data,redis_conn):
    byte_array =data
    FromProgram = int.from_bytes(byte_array[0:2], byteorder='little')
    ProgramID = int.from_bytes(byte_array[2:4], byteorder='little')
    MessageType = int.from_bytes(byte_array[4:6], byteorder='little')
    Number = int.from_bytes(byte_array[6:10], byteorder='little')
    Nbytes = int.from_bytes(byte_array[10:12], byteorder='little')

    print(FromProgram)
    print(ProgramID)
    print(MessageType)
    print(Number)
    print(Nbytes)
    content_array = byte_array[12:12 + Nbytes]
    if MessageType == 664:

        # timestamp = int.from_bytes(content_array[0:4], byteorder='little')
        # # 使用datetime模块将时间戳转换为datetime对象
        # dt = datetime.fromtimestamp(timestamp)
        # # 使用strftime函数将datetime对象格式化为可读的时间字符串
        # formatted_time = dt.strftime('%Y-%m-%d %H:%M:%S')
        # add_Time(formatted_time)
        # # 打印结果
        # print(formatted_time)

        # print(int.from_bytes(content_array[4:8], byteorder='little'))
        # print(int.from_bytes(content_array[8:9], byteorder='little'))
        # print(int.from_bytes(content_array[9:10], byteorder='little'))
        # print(int.from_bytes(content_array[10:12], byteorder='little'))
        Lat = struct.unpack('<d', content_array[12:20])[0]
        print("雷达纬度", Lat)

        Lon = struct.unpack('<d', content_array[20:28])[0]
        print("雷达经度", Lon)
        add_GPS(redis_conn,Lon,Lat)
    elif MessageType == 665:
        wspeed = struct.unpack('<d', content_array[12:20])[0]
        print("速度节", wspeed)
        add_SPEED(redis_conn,wspeed)

    # data = data.decode('utf-8')
    # print(f"Received data from {addr}: {data}")
    # dict_data = json.loads(data)
    # print(dict_data['TarID'])
    #
    # # 示例用法
    # target_id = dict_data['TarID']
    # longitude = dict_data['Lon']
    # latitude =  dict_data['Lat']
    # speed =  dict_data['Speed']
    # direction =  dict_data['Course']
    #
    # add_data(target_id, longitude, latitude, speed, direction)


def Course_data_toredis(data, redis_conn):
    # 接收数据
    byte_array = data
    # print(byte_array[0])

    # FromProgram = int.from_bytes(byte_array[0:2], byteorder='little')
    # ProgramID = int.from_bytes(byte_array[2:4], byteorder='little')
    # MessageType = int.from_bytes(byte_array[4:6], byteorder='little')
    # Number = int.from_bytes(byte_array[6:10], byteorder='little')
    Nbytes = int.from_bytes(byte_array[10:12], byteorder='little')

    # print(FromProgram)
    # print(ProgramID)
    # print(MessageType)
    # print(Number)
    # print(Nbytes)
    content_array = byte_array[12:12 + Nbytes]

    Course = struct.unpack('<d', content_array[0:8])[0]
    print("Course", Course)
    add_Course(redis_conn,Course)


def T_fusion_from_redis(data,redis_conn):
    byte_data = get_byte_array(redis_conn)
    return  byte_data


def AIS_send(data,redis_conn):
    return data


def duo_arduino_fankui(data,redis_conn):

    data_list = data.split(',')
    redis_conn.hset("Navi","Arduino_NowDuo", data_list[1])



def duo_fankuiX(data,redis_conn):

    data = data.lstrip('#')
    data = data.rstrip('*\r\n')
    result = float(data)
    print("SSSSXX"+str(result))
    redis_conn.hset("Navi","NowDuo", result)

def duo_fankui6081(data,redis_conn):
    print("duo_fankui6081")
    print(data)
    if data[0]==160:
        print("shoudong")
        flag1=0
    #elif data[0]==176:
        #print("suidong")

    elif data[0]==192:
        print("zidong")
        flag1=1

    flag2= struct.unpack('>h',data[2:4])[0]/10.0
    flag3= struct.unpack('>h',data[4:6])[0]/10.0


    redis_conn.hset("Navi","NowDuo", flag2)
    redis_conn.hset("Navi","NowDuoL", flag2)
    redis_conn.hset("Navi","NowDuoR", flag3)

def zhuansu_fankui_QJ(data,redis_conn):
    data =data.replace('*',",")

    print("zhuansu_fankui_QJ")
    data_list = data.split(',')
    lzhuan =    float(data_list[6])
    rzhuan =    float(data_list[7])
    redis_conn.hset("engine_parameters","zhuan_su", lzhuan)
    redis_conn.hset("engine_parameters","l_zhuan_su", lzhuan)
    redis_conn.hset("engine_parameters","r_zhuan_su", rzhuan)


    redis_conn.hset("Navi","now_left_pos", lzhuan)
    redis_conn.hset("Navi","now_right_pos", rzhuan)

def duo_fankui(data,redis_conn):

    byte_array = data
    # print(byte_array[0])
    # B4 04 14 01 B3 00 25 00 2A 00 29 01 45 00 03 00 00 00 00 00 01 00 00 71 C2
    duo = int.from_bytes(byte_array[3:5], byteorder='big')

    (duo - 32767) /32768 *100 # -1 ~ 1


    redis_conn.hset("Navi","NowDuo", str((duo - 32767) /32768 *100))




def engine_6606(data,redis_conn):


    byte_array = data
    # print(byte_array[0])
    # B4 04 14 01 B3 00 25 00 2A 00 29 01 45 00 03 00 00 00 00 00 01 00 00 71 C2
    you_ya = int.from_bytes(byte_array[5:7], byteorder='big')/100.0 #, byteorder='little'
    you_wen = int.from_bytes(byte_array[9:11], byteorder='big')
    zhuan_su = int.from_bytes(byte_array[3:5], byteorder='big')
    shui_wen = int.from_bytes(byte_array[7:9], byteorder='big')
    print(you_ya,you_wen,zhuan_su,shui_wen)

    # 将数据存储到Redis中，假设我们使用哈希表存储
    redis_conn.hmset('engine_parameters', {
        'you_ya': you_ya,
        'you_wen': you_wen,
        'zhuan_su': zhuan_su,
        'shui_wen': shui_wen
    })

    # 从Redis中读取并打印数据以验证
    #stored_data = redis_conn.hgetall('engine_parameters')
    #print(stored_data)
def DataToUI(data,redis_conn):
    # 从Redis中读取并打印数据以验证
    stored_data = redis_conn.hgetall('engine_parameters')


    # 将字节数据转换为字典
    decoded_data = {key.decode('utf-8'): value.decode('utf-8') for key, value in stored_data.items()}
    print(stored_data)
    return json.dumps(decoded_data).encode('utf-8')

def DataToUI_control(data,redis_conn):

    # 从Redis中读取并打印数据以验证
    stored_data = redis_conn.hgetall('Navi')


    # 将字节数据转换为字典
    decoded_data = {key.decode('utf-8'): value.decode('utf-8') for key, value in stored_data.items()}
    print(stored_data)
    return json.dumps(decoded_data).encode('utf-8')
def send_data(sock,modified_data, output_address):
    try:
        sock.sendto(modified_data, output_address)
        #logging.info("通道: "+channel_name+" Data sent to  "+str(output_address))
        print(f"Data sent to {output_address}")
    except OSError as e:
        #logging.info("通道: {channel_name} \r{output_address}:{e}")
        print(f"Error sending data: {e}")



def E_6607(data, redis_conn):
    print(len(data))
    print("###############7777777#########################")
    return  data


    # 06 03 03 FC 00 02 05 C8 线电压AB相       00 00 FF FF
    # 06 03 04 1A 00 02 E5 4B 总功率
    # 06 03 04 32 00 02 65 43 三相综合频率

    # 07 03 03 FC 00 02 04 19 线电压AB相
    # 07 03 04 1A 00 02 E4 9A 总功率
    # 07 03 04 32 00 02 64 92 三相综合频率



# 07 03 03 FC 00 02 04 19 线电压AB相
# 07 03 04 1A 00 03 25 5A 总功率
# 07 03 04 32 00 04 E4 90 三相综合频率
# 07 03 13 88 00 06 41 00 A、B、C相电压总谐波百分比


def query6607(data, redis_conn):


    UDP_IP2 = "193.0.1.89"
    hex_string = "07 03 03 E8 00 28 C5 C2"
    byte_sequence = bytes.fromhex(hex_string)
    #sock.sendto(byte_sequence, (UDP_IP2, 3005))
    print("query6607query6607query6607query6607query6607query6607query6607")

    data  =byte_sequence
    return  data

def query6607_1(data, redis_conn):
    # 创建UDP套接字
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    UDP_IP2 = "193.0.1.89"
    hex_string = "07 03 03 E8 00 28 C5 C2"  # 电压电流
    byte_sequence = bytes.fromhex(hex_string)
    # sock.sendto(byte_sequence, (UDP_IP2, 3005))
    send_data(sock, byte_sequence, (UDP_IP2, 3007))

    # time.sleep(0.2)
    # hex_string = "07 03 04 1A 00 03 25 5A"  # 线电压AB相
    # byte_sequence = bytes.fromhex(hex_string)
    # send_data(sock, byte_sequence, (UDP_IP2, 3007))
    # time.sleep(0.2)
    # hex_string = "07 03 04 32 00 04 E4 90"  # 线电压AB相
    # byte_sequence = bytes.fromhex(hex_string)
    # send_data(sock, byte_sequence, (UDP_IP2, 3007))

    # 关闭套接字1
    sock.close()

    return data
def query6607_2(data, redis_conn):
    # 创建UDP套接字
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    UDP_IP2 = "193.0.1.89"
    hex_string = "07 03 04 1A 00 03 25 5A"  # 线电压AB相
    byte_sequence = bytes.fromhex(hex_string)
    # sock.sendto(byte_sequence, (UDP_IP2, 3005))

    # 关闭套接字1
    sock.close()
    data = byte_sequence
    return data
def query6607_3(data, redis_conn):
    # 创建UDP套接字
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    UDP_IP2 = "193.0.1.89"
    hex_string = "07 03 04 32 00 04 E4 90"  # 线电压AB相
    byte_sequence = bytes.fromhex(hex_string)
    # sock.sendto(byte_sequence, (UDP_IP2, 3005))

    # 关闭套接字1
    sock.close()
    data = byte_sequence
    return data

def query6608_1(data, redis_conn):
    # 创建UDP套接字
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    UDP_IP2 = "193.0.1.89"
    hex_string = "06 03 03 FC 00 02 05 C8" #线电压AB相
    byte_sequence = bytes.fromhex(hex_string)
    #sock.sendto(byte_sequence, (UDP_IP2, 3005))

    # 关闭套接字1
    sock.close()
    data  =byte_sequence
    return  data

# def query6608_2(data, redis_conn):
#
# def query6608_3(data, redis_conn):


def query6605(data, redis_conn):


    UDP_IP2 = "193.0.1.89"
    hex_string = "05 04 00 00 00 01 30 4E"
    byte_sequence = bytes.fromhex(hex_string)
    #sock.sendto(byte_sequence, (UDP_IP2, 3005))


    data  =byte_sequence
    return  data
def query6606(data, redis_conn):

    # 创建UDP套接字
    #sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # #询问柴油机状态
    hex_string = "B4 04 00 00 00 0A 6A 68"
    byte_sequence = bytes.fromhex(hex_string)
    #sock.sendto(byte_sequence, (UDP_IP2, 3005))

    # 关闭套接字1
    #sock.close()
    data  =byte_sequence
    return  data

def total_query(data,redis_conn):
    # 设置UDP服务器的IP地址和端口号
    UDP_IP1 = "193.0.1.88"
    UDP_IP2 = "193.0.1.89"

    return  "05 04 00 00 00 01 30 4E"
    # 创建UDP套接字
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # #询问柴油机状态
    hex_string = "B4 04 00 00 00 0A 6A 68"
    byte_sequence = bytes.fromhex(hex_string)
    sock.sendto(byte_sequence, (UDP_IP2, 3006))

    #询问舵角
    hex_string = "05 04 00 00 00 01 30 4E"
    byte_sequence = bytes.fromhex(hex_string)
    sock.sendto(byte_sequence, (UDP_IP2, 3005))

    hex_string = "B4 04 00 00 00 0A 6A 68"
    byte_sequence = bytes.fromhex(hex_string)
    sock.sendto(byte_sequence, (UDP_IP1, 3003))

    hex_string = "B4 04 00 00 00 0A 6A 68"
    byte_sequence = bytes.fromhex(hex_string)
    sock.sendto(byte_sequence, (UDP_IP1, 3004))

    hex_string = "05 04 00 00 00 01 30 4E"
    byte_sequence = bytes.fromhex(hex_string)
    sock.sendto(byte_sequence, (UDP_IP2, 3005))

    hex_string = "B4 04 00 00 00 0A 6A 68"
    byte_sequence = bytes.fromhex(hex_string)
    sock.sendto(byte_sequence, (UDP_IP2, 3006))

    hex_string = "B4 04 00 00 00 0A 6A 68"
    byte_sequence = bytes.fromhex(hex_string)
    sock.sendto(byte_sequence, (UDP_IP2, 3007))

    hex_string = "B4 04 00 00 00 0A 6A 68"
    byte_sequence = bytes.fromhex(hex_string)
    sock.sendto(byte_sequence, (UDP_IP2, 3008))



    # 关闭套接字1
    sock.close()



def get_byte_array(redis_conn):
    byte_array = bytes()
    byte_array += bytes([0x4A, 0x59])
    byte_array += bytes([0x27, 0x04])


    # print(struct.pack('<I', counter))
    counter = 0
    byte_array += struct.pack('<I', counter)


    # 获取当前时间的秒数和微秒数
    seconds = int(time.time())  # 从1970年1月1日0:0:0开始到现在的秒数
    microseconds = int(time.time() * 1000000) % 1000000  # 微秒数

    # 将秒数和微秒数转换为字节数组
    packet_header_secs = struct.pack('<I', seconds)
    packet_header_usecs = struct.pack('<I', microseconds)


    # # 打印字节数组
    # print("PacketHeader_Secs:", packet_header_secs)
    # print("PacketHeader_Usecs:", packet_header_usecs)

    byte_array += packet_header_secs + packet_header_usecs

    # print(len(byte_array ))#16

    df_data = read_data(redis_conn)
    df_data = df_data.head(1000)
    byte_array += struct.pack('<H', 28* len(df_data))


    # print(struct.pack('<H', 28* len(df_data)))

    byte_array += bytes([0x03])
    byte_array += bytes([0x04, 0x04]) #senderID

    # radHeadup = int(bytes([0x00, 0x00]))
    byte_array += bytes([0x00, 0x00])
    # radHeadup = int(float(read_Course())*100)


    #Lon ,Lat = read_GPS(redis_conn)
    Lon  =122.2
    Lat = 30.3
    if(Lon == None or Lat == None):
        return bytes()
    print(Lon,Lat)

    byte_array += struct.pack('<d', float(Lat))
    byte_array += struct.pack('<d', float(Lon))
    byte_array += struct.pack('<f', float(5.0))
    byte_array += bytes([0x00, 0x00])



    for index, row in df_data.iterrows():
        RadarInfo = bytes()
        # 使用struct模块将整数转换为字节数组 4 8 8 4 4
        RadarInfo +=  struct.pack('<I', int(row['target_id']))

        RadarInfo += struct.pack('<d', float(row['Lat']))
        RadarInfo += struct.pack('<d', float(row['Lon']))

        RadarInfo += struct.pack('<f', float(row['speed']))
        RadarInfo += struct.pack('<f', float(row['Course']))


        RadarInfo += struct.pack('<I', int(row['Alarmstufe']))

        byte_array += RadarInfo


    print(28* len(df_data))
    # print(len(RadarInfo))

    print(byte_array)

    return byte_array

def add_data(
        redis_conn,target_id, longitude,
        latitude, speed, direction,警告=0,
        size = -1,cpDistance =-1,cpTime = -1,
        relativeCourse = -1,relativeSpeed = -1, azimuth = -1, distance = -1, 
):
    # 设置键的名称为target_id
    key = f"data:{target_id}"
    timestamp = datetime.now().timestamp()
    # 设置值为经纬度、速度和方向的字典f
    value = {
        "longitude": longitude,
        "latitude": latitude,
        "speed": speed,
        "direction": direction,
        "Alarmstufe": 警告,
        "size": size,
        "cpDistance": cpDistance,
        "cpTime": cpTime,
        "relativeCourse": relativeCourse,
        "relativeSpeed": relativeSpeed,
        "azimuth": azimuth,
        "distance": distance,
        "timestamp": timestamp
    }
    # exists = redis_conn.hexists(key, "Alarmstufe")
    # if (exists):
    #     Alarmstufe_value = redis_conn.hget(key, "Alarmstufe")
    #
    #     # 将数据添加到Redis并设置过期时间为60秒
    # print(key, value)
    redis_conn.hmset(key, value)
    redis_conn.expire(key, 60)
    #
    #     redis_conn.hset(key, "Alarmstufe", Alarmstufe_value)
    # else:
    #     # 将数据添加到Redis并设置过期时间为60秒
    #     redis_conn.hmset(key, value)
    #     redis_conn.expire(key, 60)
    #     # 初始化
    #     redis_conn.hset(key, "Alarmstufe", 0)


def add_GPS(redis_conn, longitude, latitude):
    # 设置键的名称为target_id
    key = f"dataX:GPS"

    # 设置值为经纬度、速度和方向的字典
    value = {
        "Lon": longitude,
        "Lat": latitude,
    }

    # 将数据添加到Redis并设置过期时间为60秒
    redis_conn.hmset(key, value)
    redis_conn.expire(key, 60)

def add_SPEED(redis_conn, speed):
    # 设置键的名称为target_id
    key = f"dataX:SPEED"

    # 设置值为经纬度、速度和方向的字典
    value = {
        "Speed": speed,
    }

    # 将数据添加到Redis并设置过期时间为60秒
    redis_conn.hmset(key, value)
    redis_conn.expire(key, 60)
def add_Course(redis_conn, Course):
    # 设置键的名称为target_id
    key = f"dataX:Course"

    # 设置值为经纬度、速度和方向的字典
    value = {
        "Course": Course,
    }

    # 将数据添加到Redis并设置过期时间为60秒
    redis_conn.hmset(key, value)
    redis_conn.expire(key, 60)


def add_Time(redis_conn, Time):
    # 设置键的名称为target_id
    key = f"dataX:Time"

    # 设置值为经纬度、速度和方向的字典
    value = {
        "Time": Time,
    }

    # 将数据添加到Redis并设置过期时间为60秒
    redis_conn.hmset(key, value)
    redis_conn.expire(key, 60)


def add_fankui(redis_conn,mode,duishuisudu, shuisheng,leftr,rightr):
    # 设置键的名称为target_id
    key = f"dataX:fankui"

    # 设置值为经纬度、速度和方向的字典
    value = {
        "mode": mode,
        "duishuisudu": duishuisudu,
        "shuisheng": shuisheng,
        "leftr": leftr,
        "rightr": rightr,

    }

    # 将数据添加到Redis并设置过期时间为60秒
    redis_conn.hmset(key, value)
    redis_conn.expire(key, 60)


def read_fankui(redis_conn):
    # 获取所有以 "dataX:GPS" 为前缀的键
    keys = redis_conn.keys("dataX:fankui")

    # 检查是否存在符合条件的键
    if not keys:
        print("No zhuansu data available.")
        return None, None

    # 获取所有键以"target_id"为前缀的键
    key = keys[0]
    # 从Redis中获取键对应的值
    value = redis_conn.hgetall(key)
    # 解码字节字符串为Unicode字符串
    value = {k.decode(): v.decode() for k, v in value.items()}
    print("leftr:", value["leftr"])
    print("rightr:", value["rightr"])

    return value["mode"], value["duishuisudu"],value["shuisheng"], value["leftr"], value["rightr"]

def read_data(redis_conn):
    # 获取所有键以"data:"为前缀的键
    keys = redis_conn.keys("data:*")

    # 创建空的DataFrame
    df = pd.DataFrame(columns=["target_id", "Lon", "Lat", "speed", "Course", "Alarmstufe"])

    for key in keys:
        # 从Redis中获取键对应的值
        value = redis_conn.hgetall(key)

        # 解码字节字符串为Unicode字符串
        value = {k.decode(): v.decode() for k, v in value.items()}

        if(len(value) >= 5 ):
            # 将数据逐行添加到DataFrame
            df.loc[len(df)] = [
                key.decode().split(":")[1],
                float(value["longitude"]),
                float(value["latitude"]),
                float(value["speed"]),
                float(value["direction"]),
                float(value["Alarmstufe"])
            ]
    return df
def read_GPS(redis_conn):
    # 获取所有以 "dataX:GPS" 为前缀的键
    keys = redis_conn.keys("dataX:GPS")

    # 检查是否存在符合条件的键
    if not keys:
        print("No GPS data available.")
        return None, None

    # 获取所有键以"target_id"为前缀的键
    key = keys[0]
    # 从Redis中获取键对应的值
    value = redis_conn.hgetall(key)
    # 解码字节字符串为Unicode字符串
    value = {k.decode(): v.decode() for k, v in value.items()}
    print("Longitude:", value["Lon"])
    print("Latitude:", value["Lat"])
    print("---")
    return  value["Lon"], value["Lat"]
def read_SPEED(redis_conn):
    # 获取所有键以"target_id"为前缀的键
    keys = redis_conn.keys("dataX:SPEED")

    # 检查是否存在符合条件的键
    if not keys:
        print("No SPEED data available.")
        return None, None

    key= keys[0]
    # 从Redis中获取键对应的值
    value = redis_conn.hgetall(key)
    # 解码字节字符串为Unicode字符串
    value = {k.decode(): v.decode() for k, v in value.items()}
    print("SPEED:", value["Speed"])

    print("---")
    return   value["Speed"]

def read_Course(redis_conn):
    # 获取所有键以"target_id"为前缀的键
    key = redis_conn.keys("dataX:Course")[0]
    # 从Redis中获取键对应的值
    value = redis_conn.hgetall(key)
    # 解码字节字符串为Unicode字符串
    value = {k.decode(): v.decode() for k, v in value.items()}
    print("Course:", value["Course"])

    print("---")
    return   value["Course"]
def read_Time(redis_conn):
    # 获取所有键以"target_id"为前缀的键
    key = redis_conn.keys("dataX:Time")[0]
    # 从Redis中获取键对应的值
    value = redis_conn.hgetall(key)
    # 解码字节字符串为Unicode字符串
    value = {k.decode(): v.decode() for k, v in value.items()}
    print("Time:", value["Time"])

    print("---")
    return   value["Time"]




def _parse_polygon_str(poly_item):
    """Accept polygon as CSV string "lon,lat,..." or flat list/tuple [lon1,lat1,lon2,lat2,...]."""
    try:
        coords = []
        if isinstance(poly_item, str):
            parts = [p.strip() for p in poly_item.split(',') if p.strip() != '']
            if len(parts) < 6 or (len(parts) % 2) != 0:
                return None
            it = iter(parts)
            for lon_s, lat_s in zip(it, it):
                lon = float(lon_s)
                lat = float(lat_s)
                coords.append((lon, lat))
        elif isinstance(poly_item, (list, tuple)):
            flat = list(poly_item)
            if len(flat) < 6 or (len(flat) % 2) != 0:
                return None
            for i in range(0, len(flat), 2):
                lon = float(flat[i])
                lat = float(flat[i + 1])
                coords.append((lon, lat))
        else:
            return None
        if len(coords) < 3:
            return None
        minx = min(p[0] for p in coords)
        maxx = max(p[0] for p in coords)
        miny = min(p[1] for p in coords)
        maxy = max(p[1] for p in coords)
        return {'bbox': (minx, miny, maxx, maxy), 'coords': coords}
    except Exception:
        return None


def _point_on_segment(px, py, x1, y1, x2, y2, eps=1e-12):
    """Return True if point P lies on segment (x1,y1)-(x2,y2)."""
    # Bounding box check
    if (min(x1, x2) - eps) <= px <= (max(x1, x2) + eps) and (min(y1, y2) - eps) <= py <= (max(y1, y2) + eps):
        # Cross product close to 0 -> collinear
        dx1, dy1 = x2 - x1, y2 - y1
        dxp, dyp = px - x1, py - y1
        cross = dx1 * dyp - dy1 * dxp
        if abs(cross) <= eps:
            return True
    return False



def _point_in_polygon(lon, lat, polygon):
    """Ray casting algorithm for point-in-polygon.
    - polygon: list of (lon, lat)
    - returns True if inside or on edge
    """
    if not polygon or len(polygon) < 3:
        return True
    inside = False
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        # On-edge check
        if _point_on_segment(lon, lat, x1, y1, x2, y2):
            return True
        # Ray casting: check if edge crosses the horizontal ray to the right of point
        if ((y1 > lat) != (y2 > lat)):
            xinters = (x2 - x1) * (lat - y1) / (y2 - y1 + 0.0) + x1
            if lon < xinters:
                inside = not inside
    return inside


def is_point_on_land(lon, lat):
    # Fast reject if no polygons
    with FENCE_LOCK:
        polygons_snapshot = FENCE_POLYGONS[:] if FENCE_POLYGONS else []
    if not polygons_snapshot:
        return False
    for poly in polygons_snapshot:
        minx, miny, maxx, maxy = poly['bbox']
        if lon < minx or lon > maxx or lat < miny or lat > maxy:
            continue
        if _point_in_polygon(lon, lat, poly['coords']):
            return True
    return False


def refresh_fence_loop(redis_conn, data):
    global FENCE_POLYGONS
    try:
        raw = redis_conn.hget("Fence", "s57_fence")
        if raw:
            try:
                s = raw.decode('utf-8') if isinstance(raw, (bytes, bytearray)) else str(raw)
                payload = json.loads(s)
                poly_items = payload.get('expanded_land_polygons') or []
                new_polys = []
                for poly_item in poly_items:
                    parsed = _parse_polygon_str(poly_item)
                    if parsed:
                        new_polys.append(parsed)
                FENCE_POLYGONS = new_polys
            except Exception:
                print("Fence111")# Ignore malformed payloads and keep previous cache
                pass
            
            redis_conn.hset("Fence", "FENCE_POLYGONS", FENCE_POLYGONS)
    except Exception:
        # Ignore Redis errors, retry next cycle
        print("Fence000")
        pass
    return ""