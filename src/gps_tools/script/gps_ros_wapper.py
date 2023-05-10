#!/usr/bin/env python3

import sys

from gps.GpsSerial import *
from gps.NtripClient import *

### 需要配置的参数
# GPS串口
serial_port = "/dev/ttyUSB0"
baud_rate = 115200
# 千寻的网址
socket_ip = "rtk.ntrip.qxwz.com"
socket_port = 8002
# 千寻账号
username = "username"
password = "password"

rospy.init_node('gps_node', anonymous=True)
pub_gps = rospy.Publisher('/gps/raw_data', NavSatFix, queue_size=1)

ser_gps = serial.Serial(serial_port, baud_rate, timeout=0)
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

try:
    client_socket.connect((socket_ip, socket_port))
    print("\033[1;32m QX: socket连接成功 \033[0m")
except Exception as e:
    print("\033[95m QX: socket连接失败 \033[0m")

# 账号 + 密码
qx_para = QXParams(username, password)
ntrip_client = NtripClient(qx_para, ser_gps, client_socket)
gps_serial = SerialPortToGPS(client_socket, ser_gps, pub_gps)

flag_connect_net = ntrip_client.init_socket()

if not flag_connect_net:
    print("千寻无法登录")
    sys.exit()
else:
    ntrip_client.run()
    gps_serial.run()

rospy.spin()
