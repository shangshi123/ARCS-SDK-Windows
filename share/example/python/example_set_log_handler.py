#! /usr/bin/env python
# coding=utf-8

"""
Custom log format

Steps:
Step 1: Customize the log format
Note: You need to call the setLogHandler function before connecting to the server.
Otherwise, the custom log format will not take effect.
Step 2: Connect to the RPC service
Step 3: Robot login
"""

import pyaubo_sdk

robot_ip = "127.0.0.1"  # Server IP address
robot_port = 30004  # Port number
M_PI = 3.14159265358979323846
robot_rpc_client = pyaubo_sdk.RpcClient()


# Print log content
def print_log(level: int, filename: str, line: int, message: str):
    print("level:", level)
    print("filename:", filename)
    print("line:", line)
    print("message:", message)


# Customize log format
def example_read_log():
    # API call: Override the log system format
    robot_rpc_client.setLogHandler(print_log)


if __name__ == '__main__':
    example_read_log()  # Customize log format
    robot_rpc_client.connect(robot_ip, robot_port)  # API call: Connect to RPC service
    if robot_rpc_client.hasConnected():
        print("Robot rcp_client connected successfully!")
        robot_rpc_client.login("aubo", "123456")  # API call: Robot login
        if robot_rpc_client.hasLogined():
            print("Robot rcp_client logined successfully!")
