import os
import socket
import yaml
import threading
import logging
import threading

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(threadName)s - %(message)s')

from datetime import datetime

import importlib
import time
import redis

# 全局变量，用于存储 Redis 连接
redis_conn = None
global pre_file_path
pre_file_path = os.path.dirname(__file__)

def send_udp_message(message, output_address):
    # 创建一个UDP套接字
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        # 发送消息到指定的IP和端口
        udp_socket.sendto(message, output_address)
        print(f"Message sent to {output_address}")
    except Exception as e:
        print(f"Error sending message: {e}")
    finally:
        # 关闭套接字
        udp_socket.close()


# 初始化 Redis 连接
def init_redis():
    global redis_conn
    redis_config = config.get('redis', {})
    host = redis_config.get('host', 'localhost')
    port = redis_config.get('port', 6379)
    db = redis_config.get('db', 0)
    password = redis_config.get('password', None)

    # 建立 Redis 连接
    redis_conn = redis.Redis(host=host, port=port, db=db, password=password)

    # 可以在此处进行一些连接测试或其他初始化操作
    try:
        redis_conn.ping()  # 测试连接是否正常
        print("Redis connection established successfully.")
    except redis.ConnectionError as e:
        print(f"Error connecting to Redis: {e}")
    redis_conn.hset("Navi", "State", "1")
    redis_conn.hset("Navi", "TargetDuo", "0")
    redis_conn.hset("Navi", "TargetSpeed", "0")


def save_to_error(data):
    filename = f"error.txt"
    file_path = os.path.join(pre_file_path,"log", filename)

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


    return True


# 函数用于保存数据到文件，并按固定大小分割文件
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
    # 检查文件大小
    if os.path.exists(file_path):
        current_size = os.path.getsize(file_path)
    else:
        current_size = 0

    # 如果文件大小超过最大限制，则新建文件
    if current_size > max_file_size:
        nowtime_name = datetime.now().strftime("%Y%m%d_%H%M%S")

    return nowtime_name

# 全局变量用于存储每个通道的数据速率
channel_data_rates = {}

# 全局变量用于存储每个通道的累计数据量和上次统计时间
channel_data_accumulators = {}

# 全局存储定时器对象的字典
timers = {}

file_path = os.path.join(os.path.dirname(__file__),"config.yaml")
print(file_path)
# 读取配置文件
with open(file_path, 'r', encoding='utf-8') as file:
    config = yaml.safe_load(file)


# 记录每个channel的最后读数据时间
channel_last_read_time = {}
timeout_threshold = 10  # 超时时间阈值（秒）



# 初始化 Redis 连接
init_redis()

# 动态加载默认数据修改函数
# default_modify_func_path = config['udp']['modify_functions']['default']
# default_module_name, default_func_name = default_modify_func_path.rsplit('.', 1)
# default_modify_data = getattr(importlib.import_module(default_module_name), default_func_name)

# # MySQL 连接
# mysql_config = config['mysql']
# db = pymysql.connect(host=mysql_config['host'],
#                      user=mysql_config['user'],
#                      password=mysql_config['password'],
#                      database=mysql_config['database'])

# 保存数据到 MySQL
# def save_to_mysql(table, data):
#     cursor = db.cursor()
#     timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
#     sql = f"INSERT INTO {table} (timestamp, data) VALUES (%s, %s)"
#     cursor.execute(sql, (timestamp, data))
#     db.commit()

# 处理 UDP 数据
def handle_udp_data(sock, channel):
    current_thread = threading.current_thread()
    #print(f"Current thread name: {current_thread.name}")

    global channel_data_rates, channel_data_accumulators
    nowtime_name = datetime.now().strftime("%Y%m%d_%H%M%S")



    while True:
        try:
            data, addr = sock.recvfrom(1024)  # 接收数据
            data_size = len(data)
            print(data)

            # 更新当前channel的最后读数据时间
            channel_last_read_time[channel] = time.time()
   
            if(channel=="渔船融合雷达数据"):
                redis_conn.hset("LOST", "lada", '1')
            if (channel == "IMU数据" or channel == "IMU数据UniStrong"):
                redis_conn.hset("LOST", "GPS", '1')
            # 初始化累计器和计时器
            if channel not in channel_data_accumulators:
                channel_data_accumulators[channel] = {'total_data': 0, 'start_time': time.time()}
            accumulator = channel_data_accumulators[channel]

            # 累加数据量
            accumulator['total_data'] += data_size

            # 检查是否超过1秒，如果是则计算速率并显示
            current_time = time.time()
            if current_time - accumulator['start_time'] >= 1:
                data_rate = accumulator['total_data'] / (current_time - accumulator['start_time'])
                channel_data_rates[channel] = data_rate  # 存储速率
                accumulator['total_data'] = 0  # 重置累计数据量
                accumulator['start_time'] = current_time  # 更新起始时间

                # 显示速率到命令行
                print(f"\r{channel}: {data_rate:.2f} bytes/sec", end="", flush=True)

            # 处理数据转发
            for channel_info in config['udp']['channels']:
                if channel_info['name'] == channel:
                    if (channel_info['HEX']):
                        try:
                            data = data.decode('utf-8')
                        except Exception as e:
                            print(f"Error sending data: {e}")

                    #是否需要保存
                    if (channel_info['file_save']):
                        nowtime_name = save_to_file(data, channel, nowtime_name, channel_info['max_file_size'])


                    #流量转发
                    for rule in channel_info['forward_rules']:
                        if rule.get('modify'):
                            #modify_func_path = config['udp']['modify_functions'].get(channel, default_modify_func_path)
                            module_name, func_name = "ModifyAndDump", rule['modify']
                            modify_data = getattr(importlib.import_module(module_name), func_name)
                            modified_data = modify_data(data,redis_conn)

                        else:
                            modified_data = data

                        if(rule.get('output_ip') and rule.get('output_port')):
                            output_address = (rule['output_ip'], rule['output_port'])

                            if isinstance(modified_data, str) :
                                modified_data = modified_data.encode('utf-8')  # 或者使用适当的编码方式
                            if  modify_data!="":
                                #send_data(sock,modified_data, output_address)
                                send_udp_message(modified_data, output_address)

                        #sock.sendto(modified_data, output_address)  # 转发数据
                        # for table in mysql_config['tables']:
                        #     save_to_mysql(table['name'], modified_data.decode('utf-8'))  # 保存数据到 MySQL
        except Exception as e:
            print(f"Channel Error : {e}")
# 处理 UDP 数据
def handle_timer(sock, channel_name,interval):
    global channel_data_rates, channel_data_accumulators
    # nowtime_name = datetime.now().strftime("%Y%m%d_%H%M%S")
    data=""

    while True:

        # 处理数据转发
        for channel_info in config['udp']['Timers']:
            if channel_info['name'] == channel_name:

                # 流量转发
                for rule in channel_info['forward_rules']:
                    if rule.get('modify'):
                        # modify_func_path = config['udp']['modify_functions'].get(channel, default_modify_func_path)
                        module_name, func_name = "ModifyAndDump", rule['modify']
                        modify_data = getattr(importlib.import_module(module_name), func_name)
                        modified_data = modify_data(data, redis_conn)
                    else:
                        modified_data = data

                    if (modified_data != "" and rule.get('output_ip') and rule.get('output_port')):
                        output_address = (rule['output_ip'], rule['output_port'])
                        if(data !=None):
                            send_data(sock, modified_data, output_address,channel_name)

        # 延时
        time.sleep(interval/1000.0)
# 示例函数，发送数据到指定地址和端口
def send_data(sock,modified_data, output_address,channel_name):
    try:
        sock.sendto(modified_data, output_address)
        logging.info("通道: "+channel_name+" Data sent to  "+str(output_address))
        #print(f"Data sent to {output_address}")
    except Exception as e:
        logging.info("通道: {channel_name} \r{output_address}:{e}")
        #print(f"Error sending data: {e}")


# 创建 UDP socket 并绑定端口
def create_udp_socket(channel_name,port):

    while True:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind(("0.0.0.0", port))
            logging.info(channel_name+"@"+str(port).ljust(6)+" 绑定成功")
            #print(f"\r{port}: 绑定成功1")
            return sock
        except Exception as e:
            if e.errno == 10048:  # WinError 10048: 端口已被占用
                logging.info(channel_name+" "+str(port)+"is already in use. Trying next port...")
                time.sleep(2)
            else:
                raise  # 其他错误，抛出异常
                time.sleep(2)



    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", port))
    return sock


# 定时器管理函数
def start_timer(name, interval, func_name):
    # 根据函数名从配置中获取函数路径
    func_path = config['timers'][func_name]
    module_name, func_name = func_path.rsplit('.', 1)

    # 动态载入函数对象
    func = getattr(importlib.import_module(module_name), func_name)

    # # 创建定时器，参数为间隔时间和要执行的函数
    # timer = threading.Timer(interval, lambda: execute_and_store_result(name, func))
    # timers[name] = timer  # 将定时器对象存储到字典中
    # timer.start()
def check_channel_timeout():
    """ 定时检查channel的最后读数据时间，超时则打印警告 """
    while True:

        print("#####check")

        print(channel_last_read_time)

        current_time = time.time()
        for channel, last_time in list(channel_last_read_time.items()):
            if current_time - last_time > timeout_threshold:
                print(f"Warning: Channel {channel} timeout, last read {current_time - last_time:.2f} seconds ago")
                save_to_error(f"Warning: Channel {channel} timeout, last read {current_time - last_time:.2f} seconds ago")
                if(channel=="渔船融合雷达数据"):
                    redis_conn.hset("LOST", "lada", '0')
                if (channel == "IMU数据" or channel == "IMU数据UniStrong"):
                    redis_conn.hset("LOST", "GPS", '0')

        time.sleep(5)  # 每5秒检查一次

if __name__ == "__main__":

    # 启动线程处理每个通道的数据
    for channel_info in config['udp']['channels']:
        name = "{:<20}".format(channel_info['name'])
        channel_name = channel_info['name']
        if channel_name =="Example":
            continue
        if channel_info['enable'] == False:
            continue

        if channel_info.get('continue', False):
            # 更新当前channel的最后读数据时间
            channel_last_read_time[channel_name] = time.time()
            print(channel_name)
        input_port = channel_info['input_port']
        sock = create_udp_socket(channel_name,input_port)


        threading.Thread(target=handle_udp_data, args=(sock, channel_name),name=name).start()

    # 启动定时器
    for channel_info in config['udp']['Timers']:
        name = "{:<20}".format(channel_info['name'])
        channel_name = channel_info['name']
        if channel_name =="Example":
            continue
        input_port = channel_info['input_port']
        sock = create_udp_socket(channel_name,input_port)

        interval = channel_info.get('interval', None)
        threading.Thread(target=handle_timer, args=(sock, channel_name,interval),name=name).start()
        # start_timer(channel_name, interval, "example_function")


    # 启动超时检测线程
    timeout_thread = threading.Thread(target=check_channel_timeout)
    timeout_thread.start()