#! /usr/bin/env python
# coding=utf-8

"""
Robot Arm Forward and Inverse Kinematics

Steps:
Step 1: Connect to the RPC service
Step 2: Log in to the robot arm
Step 3: Inverse kinematics of the robot arm
Step 4: Forward kinematics of the robot arm
"""

import pyaubo_sdk

robot_ip = "127.0.0.1"  # Server IP address
robot_port = 30004  # Port number
M_PI = 3.14159265358979323846
robot_rpc_client = pyaubo_sdk.RpcClient()
# Inverse Kinematics
def exampleInverseK(robot_name):
    robot_name = robot_rpc_client.getRobotNames()[0]  # API call: Get the robot's name
    print("Inverse Kinematics")
    # Target pose of the robot arm
    waypoint_1_p = [0.549273, -0.12094,   0.464501,
                    3.13802,  0.00122674, 1.57095]
    # Reference joint angles of the robot arm, unit: radian
    waypoint_0_q = [0.00123059, -0.26063, 1.7441,
                    0.437503,   1.56957,  0.00107586]
    # API call: Inverse kinematics
    res = robot_rpc_client.getRobotInterface(robot_name).getRobotAlgorithm().inverseKinematics(waypoint_0_q, waypoint_1_p)
    print("Inverse kinematics function return value:", res[1])
    print("Joint angles obtained from inverse kinematics:", res[0])
    pass

# Forward Kinematics
def exampleForwardK(robot_name):
    robot_name = robot_rpc_client.getRobotNames()[0]  # API call: Get the robot's name
    # Target joint angles of the robot arm
    q = [0.0012302916520035012, -0.17709153384169424, 1.3017793878477586,
         -0.08835407161244224, 1.5695657489560777, 0.0010722296812234074]
    # API call: Forward kinematics
    res = robot_rpc_client.getRobotInterface(robot_name).getRobotAlgorithm().forwardKinematics(q)
    print("Forward kinematics function return value:", res[1])
    print("Pose obtained from forward kinematics:", res[0])
    pass

    if __name__ == '__main__':
        robot_rpc_client.connect(robot_ip, robot_port)  # API call: Connect to RPC service
        if robot_rpc_client.hasConnected():
            print("Robot rpc_client connected successfully!")
            robot_rpc_client.login("aubo", "123456")  # API call: Robot arm login
            if robot_rpc_client.hasLogined():
                print("Robot rpc_client logged in successfully!")
                robot_name = robot_rpc_client.getRobotNames()[0]  # API call: Get the robot's name
                exampleInverseK(robot_name)  # Inverse kinematics
                exampleForwardK(robot_name)  # Forward kinematics

