#! /usr/bin/env python
# coding=utf-8

"""
Get robot configuration information

Steps:
Step 1: Connect to the RPC service
Step 2: Robot login
Step 3: Get robot configuration information
"""

import pyaubo_sdk

robot_ip = "127.0.0.1"  # Server IP address
robot_port = 30004  # Port number
M_PI = 3.14159265358979323846
robot_rpc_client = pyaubo_sdk.RpcClient()


# Get robot related configuration
def exampleConfig(robot_name):
    robot_name = robot_rpc_client.getRobotNames()[0]  # API call: Get robot name
    # API call: Get robot name
    name = robot_rpc_client.getRobotInterface(robot_name).getRobotConfig().getName()
    print("Robot name:", name)
    # API call: Get robot degrees of freedom
    dof = robot_rpc_client.getRobotInterface(robot_name).getRobotConfig().getDof()
    print("Robot degrees of freedom:", dof)
    # API call: Get robot servo control cycle (read from hardware abstraction layer)
    cycle_time = robot_rpc_client.getRobotInterface(robot_name).getRobotConfig().getCycletime()
    print("Servo control cycle:", cycle_time)
    # API call: Get default tool acceleration, unit: m/s^2
    default_tool_acc = robot_rpc_client.getRobotInterface(robot_name).getRobotConfig().getDefaultToolAcc()
    print("Default tool acceleration:", default_tool_acc)
    # API call: Get default tool speed, unit: m/s
    default_tool_speed = robot_rpc_client.getRobotInterface(robot_name).getRobotConfig().getDefaultToolSpeed()
    print("Default tool speed:", default_tool_speed)
    # API call: Get default joint acceleration, unit: rad/s
    default_joint_acc = robot_rpc_client.getRobotInterface(robot_name).getRobotConfig().getDefaultJointAcc()
    print("Default joint acceleration", default_joint_acc)
    # API call: Get default joint speed, unit: rad/s
    default_joint_speed = robot_rpc_client.getRobotInterface(robot_name).getRobotConfig().getDefaultJointSpeed()
    print("Default joint speed", default_joint_speed)
    # API call: Get robot type code
    robot_type = robot_rpc_client.getRobotInterface(robot_name).getRobotConfig().getRobotType()
    print("Robot type code", robot_type)
    # API call: Get robot subtype code
    sub_robot_type = robot_rpc_client.getRobotInterface(robot_name).getRobotConfig().getRobotSubType()
    print("Robot subtype code", sub_robot_type)
    # API call: Get control box type code
    control_box_type = robot_rpc_client.getRobotInterface(robot_name).getRobotConfig().getControlBoxType()
    print("Control box type code", control_box_type)
    # API call: Get mounting pose (robot base coordinate system relative to world coordinate system)
    mounting_pose = robot_rpc_client.getRobotInterface(robot_name).getRobotConfig().getMountingPose()
    print("Mounting pose (robot base coordinate system relative to world coordinate system)", mounting_pose)
    # API call: Set collision sensitivity level
    level = 6
    robot_rpc_client.getRobotInterface(robot_name).getRobotConfig().setCollisionLevel(level)
    print("Set collision sensitivity level:", level)
    # API call: Get collision sensitivity level
    level = robot_rpc_client.getRobotInterface(robot_name).getRobotConfig().getCollisionLevel()
    print("Collision sensitivity level", level)
    # API call: Set collision stop type
    collision_stop_type = 1
    robot_rpc_client.getRobotInterface(robot_name).getRobotConfig().setCollisionStopType(collision_stop_type)
    print("Set collision stop type", collision_stop_type)
    # API call: Get collision stop type
    collision_stop_type = robot_rpc_client.getRobotInterface(robot_name).getRobotConfig().getCollisionStopType()
    print("Collision stop type", collision_stop_type)
    # API call: Get robot DH parameters
    kin_param = robot_rpc_client.getRobotInterface(robot_name).getRobotConfig().getKinematicsParam(True)
    print("Robot DH parameters", kin_param)
    # API call: Get DH parameter compensation value at specified temperature
    temperature = 20
    kin_compensate = robot_rpc_client.getRobotInterface(robot_name).getRobotConfig().getKinematicsCompensate(
        temperature)
    print("DH parameter compensation value at specified temperature", kin_compensate)
    # API call: Get available end force sensor names
    tcp_force_sensor_name = robot_rpc_client.getRobotInterface(robot_name).getRobotConfig().getTcpForceSensorNames()
    print("Available end force sensor names", tcp_force_sensor_name)
    # API call: Get end force offset
    tcp_force_offset = robot_rpc_client.getRobotInterface(robot_name).getRobotConfig().getTcpForceOffset()
    print("End force offset", tcp_force_offset)
    # API call: Get available base force sensor names
    base_force_sensor_names = robot_rpc_client.getRobotInterface(robot_name).getRobotConfig().getBaseForceSensorNames()
    print("Available base force sensor names", base_force_sensor_names)
    # API call: Get base force offset
    base_force_offset = robot_rpc_client.getRobotInterface(robot_name).getRobotConfig().getBaseForceOffset()
    print("Base force offset", base_force_offset)
    # API call: Get safety parameter checksum CRC32
    check_sum = robot_rpc_client.getRobotInterface(robot_name).getRobotConfig().getSafetyParametersCheckSum()
    print("Safety parameter checksum CRC32", check_sum)
    # API call: Get joint maximum position (physical limit)
    joint_max_positions = robot_rpc_client.getRobotInterface(robot_name).getRobotConfig().getJointMaxPositions()
    print("Joint maximum position (physical limit)", joint_max_positions)
    # API call: Get joint minimum position (physical limit)
    joint_min_positions = robot_rpc_client.getRobotInterface(robot_name).getRobotConfig().getJointMinPositions()
    print("Joint minimum position (physical limit)", joint_min_positions)
    # API call: Get joint maximum speed (physical limit)
    joint_max_speeds = robot_rpc_client.getRobotInterface(robot_name).getRobotConfig().getJointMaxSpeeds()
    print("Joint maximum speed (physical limit)", joint_max_speeds)
    # API call: Get joint maximum acceleration (physical limit)
    joint_max_acc = robot_rpc_client.getRobotInterface(robot_name).getRobotConfig().getJointMaxAccelerations()
    print("Joint maximum acceleration (physical limit)", joint_max_acc)
    # API call: Get TCP maximum speed (physical limit)
    tcp_max_speeds = robot_rpc_client.getRobotInterface(robot_name).getRobotConfig().getTcpMaxSpeeds()
    print("TCP maximum speed (physical limit)", tcp_max_speeds)
    # API call: Get TCP maximum acceleration (physical limit)
    tcp_max_acc = robot_rpc_client.getRobotInterface(robot_name).getRobotConfig().getTcpMaxAccelerations()
    print("TCP maximum acceleration (physical limit)", tcp_max_acc)
    # API call: Get robot installation posture
    gravity = robot_rpc_client.getRobotInterface(robot_name).getRobotConfig().getGravity()
    print("Robot installation posture", gravity)
    # API call: Get TCP offset
    tcp_offset = robot_rpc_client.getRobotInterface(robot_name).getRobotConfig().getTcpOffset()
    print("TCP offset", tcp_offset)
    # API call: Get end payload
    payload = robot_rpc_client.getRobotInterface(robot_name).getRobotConfig().getPayload()
    print("End payload as follows:")
    print("mass:", payload[0])
    print("cog:", payload[1])
    print("aom:", payload[2])
    print("inertia:", payload[3])
    # API call: Get firmware update process
    firmware_update_process = robot_rpc_client.getRobotInterface(robot_name).getRobotConfig().getFirmwareUpdateProcess()
    print("Firmware update process", firmware_update_process)


if __name__ == '__main__':
    robot_rpc_client.connect(robot_ip, robot_port)  # API call: Connect to RPC service
    if robot_rpc_client.hasConnected():
        print("Robot rcp_client connected successfully!")
        robot_rpc_client.login("aubo", "123456")  # API call: Robot login
        if robot_rpc_client.hasLogined():
            print("Robot rcp_client logined successfully!")
            robot_name = robot_rpc_client.getRobotNames()[0]  # API call: Get robot name
            exampleConfig(robot_name)  # Get robot related configuration

