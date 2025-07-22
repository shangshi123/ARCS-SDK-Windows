#! /usr/bin/env python
# coding=utf-8

"""
Robot Arm Arc Motion

Steps:
Step 1: Connect to RPC service
Step 2: Robot login
Step 3: Arc motion
"""

import time
import math
import pyaubo_sdk

robot_ip = "127.0.0.1"  # Server IP address
robot_port = 30004  # Port number
M_PI = 3.14159265358979323846
robot_rpc_client = pyaubo_sdk.RpcClient()

waypoint1 = []  # Starting waypoint (pose) of the arc trajectory
waypoint2 = []  # Middle waypoint (pose) of the arc trajectory
waypoint3 = []  # Ending waypoint (pose) of the arc trajectory

# Get waypoints for the arc trajectory
# Calculate according to (x-0.12294)^2 + (y-0.54855)^2 + (z-0.26319)^2 = 0.2^2
def get_circle_waypoint():
    global waypoint1
    global waypoint2
    global waypoint3

    z1 = 0.26319
    y1 = 0.7
    x1 = math.sqrt(pow(0.2, 2) - pow(y1 - 0.54855, 2) - pow(z1 - 0.26319, 2))
    waypoint1 = [x1, y1, z1, 3.14, 0.00, 3.14]

    z2 = 0.26319
    y2 = 0.55
    x2 = math.sqrt(pow(0.2, 2) - pow(y2 - 0.54855, 2) - pow(z2 - 0.26319, 2))
    waypoint2 = [x2, y2, z2, 3.14, 0.00, 3.14]

    z3 = 0.26319
    y3 = 0.35
    x3 = math.sqrt(pow(0.2, 2) - pow(y3 - 0.54855, 2) - pow(z3 - 0.26319, 2))
    waypoint3 = [x3, y3, z3, 3.14, 0.00, 3.14]

# Arc motion
def example_movec(robot_name):
    get_circle_waypoint()
    robot_rpc_client.getRobotInterface(robot_name).getMotionControl()\
        .moveLine(waypoint1, 180 * (M_PI / 180), 1000000 * (M_PI / 180), 0, 0)  # API call: Linear motion
    time.sleep(1)

    ret = robot_rpc_client.getRobotInterface(robot_name).getMotionControl()\
        .moveCircle(waypoint2, waypoint3, 180 * (M_PI / 180), 1000000 * (M_PI / 180), 0, 0) # API call: Arc motion

    if ret == 0:
        print("Arc motion succeeded!")
    else:
        print("Arc motion failed! Error code:%d", ret)


if __name__ == '__main__':
    robot_rpc_client.connect(robot_ip, robot_port)  # API call: Connect to RPC service
    if robot_rpc_client.hasConnected():
        print("Robot rcp_client connected successfully!")
        robot_rpc_client.login("aubo", "123456")  # API call: Robot login
        if robot_rpc_client.hasLogined():
            print("Robot rcp_client logined successfully!")
            robot_name = robot_rpc_client.getRobotNames()[0]  # API call: Get robot name
            example_movec(robot_name)  # Arc motion
