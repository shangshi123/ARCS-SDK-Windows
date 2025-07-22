#! /usr/bin/env python
# coding=utf-8

"""
Function: Robot arm joint movement

Steps:
Step 1: Set RPC timeout, connect to RPC service, robot arm login
Step 2: Set motion speed ratio, pass through 3 waypoints in joint movement mode
Step 3: RPC logout, disconnect
"""

import time
import math
import pyaubo_sdk

robot_ip = "127.0.0.1"  # Robot arm IP address
robot_port = 30004  # Port number


# Blocking
def wait_arrival(robot_interface):
    max_retry_count = 5
    cnt = 0

    # API call: Get current motion command ID
    exec_id = robot_interface.getMotionControl().getExecId()

    # Wait for the robot arm to start moving
    while exec_id == -1:
        if cnt > max_retry_count:
            return -1
        time.sleep(0.05)
        cnt += 1
        exec_id = robot_interface.getMotionControl().getExecId()

    # Wait for the robot arm to finish moving
    while robot_interface.getMotionControl().getExecId() != -1:
        time.sleep(0.05)

    return 0


# Pass through 3 waypoints in joint movement mode
def example_movej(rpc_client):
    # Joint angles, unit: radians
    q1 = [0.0 * (math.pi / 180), -15.0 * (math.pi / 180), 100.0 * (math.pi / 180),
          25.0 * (math.pi / 180), 90.0 * (math.pi / 180), 0.0 * (math.pi / 180)
          ]
    q2 = [35.92 * (math.pi / 180), -11.28 * (math.pi / 180), 59.96 * (math.pi / 180),
          -18.76 * (math.pi / 180), 90.0 * (math.pi / 180), 35.92 * (math.pi / 180)
          ]
    q3 = [41.04 * (math.pi / 180), -7.65 * (math.pi / 180), 98.80 * (math.pi / 180),
          16.44 * (math.pi / 180), 90.0 * (math.pi / 180), 11.64 * (math.pi / 180)
          ]

    # API call: Get the robot's name
    robot_name = rpc_client.getRobotNames()[0]

    robot_interface = rpc_client.getRobotInterface(robot_name)

    # API call: Set the robot arm's speed ratio
    robot_interface.getMotionControl().setSpeedFraction(0.75)

    # Joint movement to waypoint q1
    robot_interface.getMotionControl() \
        .moveJoint(q1, 80 * (math.pi / 180), 60 * (math.pi / 180), 0, 0)

    # Blocking
    ret = wait_arrival(robot_interface)
    if ret == 0:
        print("Joint movement to waypoint 1 succeeded")
    else:
        print("Joint movement to waypoint 1 failed")

    # Joint movement to waypoint q2
    robot_interface.getMotionControl() \
        .moveJoint(q2, 80 * (math.pi / 180), 60 * (math.pi / 180), 0, 0)

    # Blocking
    ret = wait_arrival(robot_interface)
    if ret == 0:
        print("Joint movement to waypoint 2 succeeded")
    else:
        print("Joint movement to waypoint 2 failed")

    # Joint movement to waypoint q3
    robot_interface.getMotionControl() \
        .moveJoint(q3, 80 * (math.pi / 180), 60 * (math.pi / 180), 0, 0)

    # Blocking
    ret = wait_arrival(robot_interface)
    if ret == 0:
        print("Joint movement to waypoint 3 succeeded")
    else:
        print("Joint movement to waypoint 3 failed")


if __name__ == '__main__':
    robot_rpc_client = pyaubo_sdk.RpcClient()
    robot_rpc_client.setRequestTimeout(1000)  # API call: Set RPC timeout
    robot_rpc_client.connect(robot_ip, robot_port)  # API call: Connect to RPC service
    if robot_rpc_client.hasConnected():
        print("RPC client connected successfully!")
        robot_rpc_client.login("aubo", "123456")  # API call: Login
        if robot_rpc_client.hasLogined():
            print("RPC client logged in successfully!")
            example_movej(robot_rpc_client)  # Pass through 3 waypoints in joint movement mode
            robot_rpc_client.logout()  # Logout
            robot_rpc_client.disconnect()  # Disconnect

