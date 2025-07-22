#! /usr/bin/env python
# coding=utf-8

"""
Conversion between robot arm Euler angles and quaternions

Steps:
Step 1: Connect to RPC service
Step 2: Robot login
Step 3: Euler angles to quaternion
Step 4: Quaternion to Euler angles
"""
import time

import pyaubo_sdk

robot_ip = "127.0.0.1"  # Server IP address
robot_port = 30004  # Port number
M_PI = 3.14159265358979323846
robot_rpc_client = pyaubo_sdk.RpcClient()


# Euler angles to quaternion
def exampleRpyToQuat():
    print("Euler angles to quaternion")
    # Euler angles
    rpy = [0.611, 0.785, 0.960]
    # API call: Euler angles to quaternion
    quat = robot_rpc_client.getMath().rpyToQuaternion(rpy)
    print("Quaternion:", quat)


# Quaternion to Euler angles
def exampleQuatToRpy():
    print("Quaternion to Euler angles")
    # Quaternion
    quat = [0.834721517970497, 0.07804256900772265, 0.4518931575790371, 0.3048637712043723]
    # API call: Quaternion to Euler angles
    rpy = robot_rpc_client.getMath().quaternionToRpy(quat)
    print("Euler angles:", rpy)
    pass


# Given the pose of the flange center under Base, calculate the pose of TCP under Base
def example_flange_to_tcp():
    # API call: Get the robot's name
    robot_name = robot_rpc_client.getRobotNames()[0]
    robot_interface = robot_rpc_client.getRobotInterface(robot_name)
    # API call: Set TCP offset
    tcp_offset = [0.01, 0.02, 0.03, 0.1, 0.2, 0.0]
    robot_interface.getRobotConfig().setTcpOffset(tcp_offset)
    time.sleep(0.1)
    # API call: Get the current pose of the flange center under Base
    actual_flange_pose_on_base = robot_interface.getRobotState().getToolPose()
    print(f"Current pose of flange center under Base: {actual_flange_pose_on_base}")
    # API call: Calculate the pose of TCP under Base based on the pose of the flange center under Base
    actual_tcp_pose_on_base = robot_rpc_client.getMath().poseTrans(actual_flange_pose_on_base, tcp_offset)
    print(f"Pose of TCP under Base: {actual_tcp_pose_on_base}")


# Given the pose of TCP under Base, calculate the pose of the flange center under Base
def example_tcp_to_flange():
    # API call: Get the robot's name
    robot_name = robot_rpc_client.getRobotNames()[0]
    robot_interface = robot_rpc_client.getRobotInterface(robot_name)
    # API call: Set TCP offset
    tcp_offset = [0.01, 0.02, 0.03, 0.1, 0.2, 0.0]
    robot_interface.getRobotConfig().setTcpOffset(tcp_offset)
    time.sleep(0.1)
    # API call: Get the current pose of TCP under Base
    actual_tcp_pose_on_base = robot_interface.getRobotState().getTcpPose()
    print(f"Current pose of TCP under Base: {actual_tcp_pose_on_base}")
    # API call: Calculate the pose of the flange center under Base based on the pose of TCP under Base
    actual_flange_pose_on_base = robot_rpc_client.getMath().poseTrans(actual_tcp_pose_on_base, robot_rpc_client.getMath().poseInverse(tcp_offset))
    print(f"Pose of flange center under Base: {actual_flange_pose_on_base}")


if __name__ == '__main__':
    robot_rpc_client.connect(robot_ip, robot_port)  # API call: Connect to RPC service
    if robot_rpc_client.hasConnected():
        print("Robot rcp_client connected successfully!")
        robot_rpc_client.login("aubo", "123456")  # API call: Robot login
        if robot_rpc_client.hasLogined():
            print("Robot rcp_client logined successfully!")
            exampleRpyToQuat()  # Euler angles to quaternion
            exampleQuatToRpy()  # Quaternion to Euler angles
            example_flange_to_tcp()  # Given the pose of the flange center under Base, calculate the pose of TCP under Base
            example_tcp_to_flange()  # Given the pose of TCP under Base, calculate the pose of the flange center under Base

