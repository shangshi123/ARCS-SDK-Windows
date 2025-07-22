#! /usr/bin/env python
# coding=utf-8

"""
Robot Arm Linear Motion

Steps:
Step 1: Set RPC timeout, connect to RPC service, robot arm login
Step 2: Move to starting position with joint motion, then pass through 3 waypoints with linear motion
Step 3: RPC logout, disconnect
"""
import time
import math
import pyaubo_sdk

robot_ip = "127.0.0.1"  # Server IP address
robot_port = 30004  # Port number
robot_rpc_client = pyaubo_sdk.RpcClient()


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


# Move to starting position with joint motion, then pass through 3 waypoints with linear motion in sequence
def example_movel(rpc_client):
    # Waypoint, represented by joint angles, unit: radians
    q = [0.0 * (math.pi / 180), -15.0 * (math.pi / 180), 100.0 * (math.pi / 180), 25.0 * (math.pi / 180),
         90.0 * (math.pi / 180), 0.0 * (math.pi / 180)]
    # Waypoint, represented by pose, format [x,y,z,rx,ry,rz], position unit: meters, orientation unit: radians
    pose1 = [0.551, -0.124, 0.419, -3.134, 0.004, 1.567]
    pose2 = [0.552, 0.235, 0.419, -3.138, 0.007, 1.567]
    pose3 = [0.551, -0.124, 0.261, -3.134, 0.0042, 1.569]

    # API call: Get robot name
    robot_name = rpc_client.getRobotNames()[0]

    robot_interface = rpc_client.getRobotInterface(robot_name)

    # API call: Set robot arm speed fraction
    robot_interface.getMotionControl().setSpeedFraction(0.75)

    # API call: Set tool center point (TCP offset relative to flange center)
    tcp_offset = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    robot_interface.getRobotConfig().setTcpOffset(tcp_offset)

    # Joint motion to initial position
    robot_interface.getMotionControl() \
        .moveJoint(q, 80 * (math.pi / 180), 60 * (math.pi / 180), 0, 0)

    # Blocking
    ret = wait_arrival(robot_interface)
    if ret == 0:
        print("Joint motion to initial position succeeded")
    else:
        print("Joint motion to initial position failed")

    # Pass through 3 waypoints with linear motion in sequence
    # Linear motion
    robot_interface.getMotionControl() \
        .moveLine(pose1, 1.2, 0.25, 0, 0)

    # Blocking
    ret = wait_arrival(robot_interface)
    if ret == 0:
        print("Linear motion to waypoint 1 succeeded")
    else:
        print("Linear motion to waypoint 1 failed")

    # Linear motion
    robot_interface.getMotionControl() \
        .moveLine(pose2, 1.2, 0.25, 0, 0)

    # Blocking
    ret = wait_arrival(robot_interface)
    if ret == 0:
        print("Linear motion to waypoint 2 succeeded")
    else:
        print("Linear motion to waypoint 2 failed")

    # Linear motion
    robot_interface.getMotionControl() \
        .moveLine(pose3, 1.2, 0.25, 0, 0)

    # Blocking
    ret = wait_arrival(robot_interface)
    if ret == 0:
        print("Linear motion to waypoint 3 succeeded")
    else:
        print("Linear motion to waypoint 3 failed")


if __name__ == '__main__':
    robot_rpc_client = pyaubo_sdk.RpcClient()
    robot_rpc_client.setRequestTimeout(1000)  # API call: Set RPC timeout
    robot_rpc_client.connect(robot_ip, robot_port)  # API call: Connect to RPC service
    if robot_rpc_client.hasConnected():
        print("RPC client connected successfully!")
        robot_rpc_client.login("aubo", "123456")  # API call: Login
        example_movel(robot_rpc_client)  # Move to starting position with joint motion, then pass through 3 waypoints with linear motion
        robot_rpc_client.logout()  # Logout
        robot_rpc_client.disconnect()  # Disconnect
    else:
        print("RPC client connection failed!")
