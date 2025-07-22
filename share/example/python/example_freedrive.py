#! /usr/bin/env python
# coding=utf-8

"""
Freedrive Teaching

Steps:
Step 1: Connect to the RPC server and log in
Step 2: Enter freedrive teaching mode, exit the mode after 25 seconds
"""
import pyaubo_sdk
import time

robot_ip = "127.0.0.1"  # Server IP address
robot_port = 30004  # Port number
M_PI = 3.14159265358979323846
robot_rpc_client = pyaubo_sdk.RpcClient()


def exampleFreeDrive(cli):
    robot_name = cli.getRobotNames()[0]
    cli.getRuntimeMachine().start()
    # API call: Initiate robot freedrive request
    cli.getRobotInterface(robot_name).getRobotManage().freedrive(True)
    time.sleep(25)
    # API call: End robot freedrive request
    cli.getRobotInterface(robot_name).getRobotManage().freedrive(False)
    pass


if __name__ == '__main__':
    robot_rpc_client.setRequestTimeout(1000)
    robot_rpc_client.connect(robot_ip, robot_port)  # API call: Connect to RPC service
    if robot_rpc_client.hasConnected():
        print("Robot rcp_client connected successfully!")
        robot_rpc_client.login("aubo", "123456")  # API call: Robot login
        if robot_rpc_client.hasLogined():
            print("Robot rcp_client logined successfully!")
            exampleFreeDrive(robot_rpc_client)  # Enter freedrive teaching mode, exit after 25 seconds
