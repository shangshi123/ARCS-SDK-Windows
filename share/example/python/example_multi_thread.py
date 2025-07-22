#! /usr/bin/env python
# coding=utf-8

"""
Stress Test
readTest(): 10 threads continuously get TCP offset
connectTest(): 10 threads continuously connect and disconnect
"""

import pyaubo_sdk
import time
import threading

robot_ip = "127.0.0.1"  # Server IP address
robot_port = 30004  # Port number
M_PI = 3.14159265358979323846
robot_rpc_client = pyaubo_sdk.RpcClient()


def readThread():
    while True:
        try:
            robot_rpc_client.setRequestTimeout(5)
            robot_name = robot_rpc_client.getRobotNames()[0]
            impl = robot_rpc_client.getRobotInterface(robot_name)
            impl.getRobotConfig().getTcpOffset()
            print(".", end=' ',flush=True)
        except pyaubo_sdk.AuboException as e:
            print(threading.get_ident(), ":", e)
        time.sleep(0.005)


def readTest():
    robot_rpc_client.connect(robot_ip, robot_port)
    robot_rpc_client.login("aubo", "123456")
    threads = []
    for i in range(10):
        threads.append(threading.Thread(target=readThread))
    for thread in threads:
        thread.start()
        thread.join()


def connectThread():
    while True:
        try:
            robot_rpc_client.setRequestTimeout(10)
            robot_rpc_client.connect(robot_ip, robot_port)
            robot_rpc_client.login("aubo", "123456")
            time.sleep(0.007)
            robot_rpc_client.logout()
            robot_rpc_client.disconnect()
            time.sleep(0.007)
        except pyaubo_sdk.AuboException as e:
            print(threading.get_ident(), ":", e)


def connectTest():
    threads = []
    for i in range(10):
        threads.append(threading.Thread(target=connectThread))
    for thread in threads:
        thread.start()
        thread.join()


if __name__ == '__main__':
    # 10 threads continuously
    readTest()
    # 10 threads continuously connect and disconnect
    # connectTest()
