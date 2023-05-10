#!/usr/bin/env python3

import time
import threading
import serial
from socket import socket
import rospy
from sensor_msgs.msg import NavSatFix
from . import GPSTool
class SerialPortToGPS:
    def __init__(self, client_socket: socket, ser: serial.Serial, pub_gps: rospy.Publisher):
        self.thread_read = None
        self.flag_start = False
        self.serial = ser
        self.client_socket = client_socket
        self.count = 0
        self.result = None
        self.lock = threading.Lock()
        self.pub_gps = pub_gps

    def __read_thread(self):
        try:
            while self.flag_start:
                buffer = self.serial.read(1024)
                if len(buffer) > 0:
                    ros_msg = NavSatFix()
                    ros_msg.header.stamp = rospy.Time.now()
                    str_buffer = buffer.decode("ascii").strip()
                    self.lock.acquire()
                    self.result = GPSTool.data_extraction(str_buffer)
                    temp = self.result
                    self.lock.release()
                    ros_msg.header.frame_id = 'earth'
                    ros_msg.latitude = temp[0].lat
                    ros_msg.altitude = temp[0].alt
                    ros_msg.longitude = temp[0].lon
                    # ros_msg.status.status =
                    self.pub_gps.publish(ros_msg)
                    self.__send_data_to_server(str_buffer)
                time.sleep(0.05)
        except Exception as e:
            print(f"GPS->read_thread: 读取串口数据异常：{str(e)}")

    def __format(self,data):
        result = ""
        aa = data.split("\r\n")
        for s in aa:
            if s.startswith("$GPGGA"):
                result = s + "\r\n"
                break
        return result

    def __send_data_to_server(self, data):
        if self.count > 10:
            data_send = self.__format(data)
            self.client_socket.send(data_send)
            self.count = 0
        self.count = self.count + 1

    def run(self):
        self.flag_start = True
        self.thread_read = threading.Thread(target=self.__read_thread, daemon=True)
        self.thread_read.start()

    def stop(self):
        self.flag_start = False
        if self.thread_read.is_alive():
            self.thread_read.join()
        self.serial.close()

    def get_data(self):
        self.lock.acquire()
        out_put = self.result
        self.lock.release()
        return out_put

if __name__ == "__main__":
    print(help(SerialPortToGPS))