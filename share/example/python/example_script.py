#! /usr/bin/env python
# coding=utf-8

"""
Run script

Steps:
Step 1: Connect to RPC service, robot login
Step 2: Connect to SCRIPT service, robot login
Step 3: Run script
Step 4: When the planner stops running, stop script execution
"""
import time

import pyaubo_sdk

robot_ip = "127.0.0.1"  # Server IP address
M_PI = 3.14159265358979323846
robot_rpc_client = pyaubo_sdk.RpcClient()
robot_script_client = pyaubo_sdk.ScriptClient()


def exampleScript():
    robot_script_client.sendFile("../aubo_script/example_movej.lua")
    pass


if __name__ == '__main__':
    robot_rpc_client.connect(robot_ip, 30004)  # API call: connect to RPC service
    robot_rpc_client.login("aubo", "123456")  # API call: robot login
    robot_script_client.connect(robot_ip,30002)  # API call: connect to SCRIPT
    robot_script_client.login("aubo", "123456")  # API call: robot login

    exampleScript()  # Run script

    while True:
        time.sleep(1)
        runtime_state = robot_rpc_client.getRuntimeMachine().getStatus()
        if runtime_state != pyaubo_sdk.RuntimeState.Running:  # When the planner stops running, stop script execution
            print("Planner state:", runtime_state)
            break
        print("Planner state:", runtime_state)