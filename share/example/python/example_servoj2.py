#! /usr/bin/env python
# coding=utf-8

"""
Function: Robot Arm ServoJ Motion

Steps:
Step 1: Set RPC timeout, connect to RPC service, robot arm login
Step 2: ServoJ motion
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


"""
Test: Rotate joint 1 by 120 degrees, planning time is 10 seconds,
At the 5th second, send the second point, rotate joint 3 by 90 degrees
Conclusion: If a new target point is sent before the robot reaches the target point,
the robot will abandon the original target point and move directly to the new target point
"""


def example_servoj(rpc_client):
    # Joint angles, unit: radians
    q = [0.0 * (math.pi / 180), -15.0 * (math.pi / 180), 100.0 * (math.pi / 180),
         25.0 * (math.pi / 180), 90.0 * (math.pi / 180), 0.0 * (math.pi / 180)
         ]

    # API call: Get the robot's name
    robot_name = rpc_client.getRobotNames()[0]

    robot_interface = rpc_client.getRobotInterface(robot_name)

    # API call: Set the robot arm speed ratio
    robot_interface.getMotionControl().setSpeedFraction(0.75)

    # Move joints to waypoint q
    robot_interface.getMotionControl() \
        .moveJoint(q, 80 * (math.pi / 180), 60 * (math.pi / 180), 0, 0)

    # Blocking
    ret = wait_arrival(robot_interface)
    if ret == 0:
        print("Joint moved to initial position successfully")
    else:
        print("Joint failed to move to initial position")

    # API call: Enable Servo mode
    robot_interface.getMotionControl().setServoMode(True)

    # Wait to enter Servo mode
    i = 0
    while not robot_interface.getMotionControl().isServoModeEnabled():
        if i > 5:
            print("Failed to enable Servo mode! Current Servo status is ", robot_interface
                  .getMotionControl()
                  .isServoModeEnabled())
            return -1
        time.sleep(0.005)
        i += 1

    q1 = [-120.0 * (math.pi / 180), -15.0 * (math.pi / 180),
          100.0 * (math.pi / 180), 25.0 * (math.pi / 180),
          90.0 * (math.pi / 180), 0.0 * (math.pi / 180)]
    print("Moving to the first target point")
    # API call: Joint servo motion
    robot_interface.getMotionControl().servoJoint(q1, 0.0, 0.0, 10, 0.0, 0.0)

    time.sleep(5)

    q2 = [0, 0, 90.0 / 180 * math.pi, 0, 0, 0]
    print("Moving to the second target point")
    # API call: Joint servo motion
    robot_interface.getMotionControl().servoJoint(q2, 0.0, 0.0, 10, 0.0, 0.0)

    # Wait for motion to finish
    while not robot_interface.getRobotState().isSteady():
        time.sleep(0.005)

    print("ServoJ motion finished")

    # Disable Servo mode
    robot_interface.getMotionControl().setServoMode(False)

    # Wait to exit Servo mode
    i = 0
    while robot_interface.getMotionControl().isServoModeEnabled():
        if i > 5:
            print("Failed to disable Servo mode! Current Servo mode is ",
                  robot_interface.getMotionControl().isServoModeEnabled())
            return -1
        time.sleep(0.005)
        i += 1

    return 0


if __name__ == '__main__':
    robot_rpc_client = pyaubo_sdk.RpcClient()
    robot_rpc_client.setRequestTimeout(1000)  # API call: Set RPC timeout
    robot_rpc_client.connect(robot_ip, robot_port)  # API call: Connect to RPC service
    if robot_rpc_client.hasConnected():
        print("RPC client connected successfully!")
        robot_rpc_client.login("aubo", "123456")  # API call: Login
        if robot_rpc_client.hasLogined():
            print("RPC client logged in successfully!")
            example_servoj(robot_rpc_client)  # ServoJ example
            robot_rpc_client.logout()  # Logout
            robot_rpc_client.disconnect()  # Disconnect

