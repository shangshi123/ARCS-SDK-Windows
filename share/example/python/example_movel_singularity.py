#! /usr/bin/env python
# coding=utf-8

"""
The robot arm moves linearly to a singular position, and the RTDE-subscribed teach pendant popup error information is output to the console.

Steps:
Step 1: Connect and log in to the RPC service
Step 2: Connect and log in to the RTDE service
Step 3: Set RTDE subscription topic
Step 4: Subscribe to the topic and get the teach pendant popup's log level, error code, and error message
Step 5: Move linearly to a singular point
"""
import os
import time
import sys
import pyaubo_sdk

robot_ip = "127.0.0.1"  # Server IP address
rpc_port = 30004  # RPC port number
rtde_port = 30010  # RTDE port number
M_PI = 3.14159265358979323846


# Blocking
def waitArrival(impl):
    cnt = 0
    while impl.getMotionControl().getExecId() == -1:
        cnt += 1
        if cnt > 5:
            print("Motion fail!")
            return -1
        time.sleep(0.05)
        print("getExecId: ", impl.getMotionControl().getExecId())
    id = impl.getMotionControl().getExecId()
    while True:
        id1 = impl.getMotionControl().getExecId()
        if id != id1:
            break
        time.sleep(0.05)


# Move linearly to a singular point
def movel_singularity():
    robot_name = robot_rpc_client.getRobotNames()[0]
    # Joint angles, unit: radians
    waypoint_1_q = [-2.40194, 0.103747, 1.7804, 0.108636, 1.57129, -2.6379]
    # Singular position, format [x, y, z, rx, ry, rz]
    waypoint_2_p = [1000, 1000, 1000, 3.05165, 0.0324355, 1.80417]

    # First move joints to waypoint_1, then move linearly to the singular point waypoint_2
    print("Moving to waypoint_1...")
    robot_rpc_client.getRobotInterface(robot_name).getMotionControl() \
        .moveJoint(waypoint_1_q, 180 * (M_PI / 180), 1000000 * (M_PI / 180), 0, 0)
    waitArrival(robot_rpc_client.getRobotInterface(robot_name))
    print("Moving to waypoint_2...")
    ret = robot_rpc_client.getRobotInterface(robot_name).getMotionControl() \
        .moveLine(waypoint_2_p, 250 * (M_PI / 180), 1000 * (M_PI / 180), 0, 0)
    waitArrival(robot_rpc_client.getRobotInterface(robot_name))


# Print log level, error code, and other information to the console
def printlog(level, source, code, content):
    level_names = ["Critical", "Error", "Warning", "Info", "Debug", "BackTrace"]
    sys.stderr.write(f"[{level_names[level.value]}] {source} - {code} {content}\n")


# Subscription callback function, get log level, error code, error message
def subscribe_callback(parser):
    msgs = parser.popRobotMsgVector()
    for msg in msgs:
        # Get the error message corresponding to the error code
        error_content = pyaubo_sdk.errorCode2Str(msg.code)
        for arg in msg.args:
            pos = error_content.find("{}")
            if pos != -1:
                error_content = error_content[:pos] + arg + error_content[pos + 2:]
            else:
                break
        printlog(msg.level, msg.source, msg.code, error_content)


if __name__ == '__main__':
    # Connect and log in to the RPC service
    robot_rpc_client = pyaubo_sdk.RpcClient()
    robot_rpc_client.connect(robot_ip, rpc_port)
    robot_rpc_client.login("aubo", "123456")

    # Connect and log in to the RTDE service
    robot_rtde_client = pyaubo_sdk.RtdeClient()
    robot_rtde_client.connect(robot_ip, rtde_port)
    robot_rtde_client.login("aubo", "123456")

    # Set RTDE subscription topic
    topic = robot_rtde_client.setTopic(False, ["R1_message"], 200, 0)
    if topic < 0:
        print("Set topic fail!")
        os.exit()

    # Subscribe to the topic and get the teach pendant popup's log level, error code, and error message
    robot_rtde_client.subscribe(topic, subscribe_callback)

    # Move linearly to a singular point
    movel_singularity()

    while True:
        time.sleep(1)
