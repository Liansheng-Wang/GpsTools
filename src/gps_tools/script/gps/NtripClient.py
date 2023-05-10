#!/usr/bin/env python3

import socket
import base64
import threading
import time
import serial


class QXParams:
    def __init__(self, username: str, password: str, ip = "rtk.ntrip.qxwz.com",
                port = 8002, mount_point = "RTCM32_GGB", is_net = True):
        self.username = username
        self.password = password
        self.ip = ip
        self.port = port
        self.mount_point = mount_point
        self.is_net = is_net                    # todo: 网络基站和地面基站的标志

class NtripClient:
    def __init__(self, qx_para: QXParams, ser: serial.Serial(), client_socket: socket.socket):
        self.thread_run = None
        self.flag_run = False
        self.ser = ser
        self.client_socket = client_socket
        self.count = 0
        self.respond : str
        self.server_address = (qx_para.ip, qx_para.port)
        self.user = base64.b64encode((qx_para.username + ":" + qx_para.password).encode('ascii')).decode('ascii')
        self.socket_user_data = "GET "+ qx_para.mount_point \
            + " HTTP/1.0\r\n" + \
            "User-Agent: NTRIP GNSSInternetRadio/1.4.10\r\n" + \
            "Accept: */*\r\n" + \
            "Connection: close\r\n" + \
            "Authorization: Basic " + \
            self.user + "\r\n" + \
            "\r\n"

    def init_socket(self):
        self.client_socket.send(self.socket_user_data.encode('ascii'))
        wait_respond = 0
        while wait_respond < 50:
            try:
                data = self.client_socket.recv(1024)
                respond = data.decode('utf-8').strip()
                if "ICY 200 OK" in respond or "SOURCETABLE 200 OK" in respond:
                    print("QX: 账号登录成功")
                    return True
            except Exception as e:
                print("QX: 账号登录连接失败")
                print(e)
                return False
            wait_respond = wait_respond + 1
        return False

    def run(self):
        self.flag_run = True
        self.thread_run = threading.Thread(target=self.__run_thread, daemon=True)
        self.thread_run.start()

    def stop(self):
        self.flag_run = False
        if self.thread_run.is_alive():
            self.thread_run.join()
        self.client_socket.close()

    def __run_thread(self):
        self.flag_run = True
        try:
            while self.flag_run:
                try:
                    data = self.client_socket.recv(1024)
                    if not data:
                        continue
                    self.ser.write(data)
                except Exception as e:
                    print(e)
                time.sleep(0.05)
        except Exception as e:
            print(f"QX->run_thread: 拉取网络数据异常：{str(e)}")
        finally:
            self.client_socket.close()

if __name__ == "__main__":
    print(help(NtripClient))

