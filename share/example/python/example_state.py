#! /usr/bin/env python
# coding=utf-8

"""
Get robot arm status information

Steps:
Step 1: Connect to RPC service
Step 2: Robot arm login
Step 3: Get robot arm status information
"""

import pyaubo_sdk

robot_ip = "127.0.0.1"  # Server IP address
robot_port = 30004  # Port number
M_PI = 3.14159265358979323846
robot_rpc_client = pyaubo_sdk.RpcClient()


# Get robot arm status information
def exampleState(robot_name):
    # API call: Get robot mode status
    robot_mode_type = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getRobotModeType()
    print("Robot mode status:", robot_mode_type)
    # API call: Get safety mode
    safety_mode_type = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getSafetyModeType()
    print("Safety mode:", safety_mode_type)
    # API call: Whether the robot has stopped
    is_steady = robot_rpc_client.getRobotInterface(robot_name).getRobotState().isSteady()
    print("Has the robot stopped:", is_steady)
    # API call: Whether the robot is within safety limits
    is_within_safety_limits = robot_rpc_client.getRobotInterface(robot_name).getRobotState().isWithinSafetyLimits()
    print("Is the robot within safety limits:", is_within_safety_limits)
    # API call: Get TCP pose
    tcp_pose = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getTcpPose()
    print("TCP pose:", tcp_pose)
    # API call: Get current target pose
    target_tcp_pose = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getTargetTcpPose()
    print("Current target pose:", target_tcp_pose)
    # API call: Get tool end pose (without TCP offset)
    tool_pose = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getToolPose()
    print("Tool end pose (without TCP offset):", tool_pose)
    # API call: Get TCP speed
    tcp_speed = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getTcpSpeed()
    print("TCP speed:", tcp_speed)
    # API call: Get TCP force / torque
    tcp_force = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getTcpForce()
    print("TCP force / torque:", tcp_force)
    # API call: Get elbow position
    elbow_postion = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getElbowPosistion()
    print("Elbow position:", elbow_postion)
    # API call: Get elbow velocity
    elbow_velocity = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getElbowVelocity()
    print("Elbow velocity:", elbow_velocity)
    # API call: Get base force / torque
    base_force = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getBaseForce()
    print("Base force / torque:", base_force)
    # API call: Get TCP target pose
    tcp_target_pose = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getTcpTargetPose()
    print("TCP target pose:", tcp_target_pose)
    # API call: Get TCP target speed
    tcp_target_speed = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getTcpTargetSpeed()
    print("TCP target speed:", tcp_target_speed)
    # API call: Get TCP target force / torque
    tcp_target_force = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getTcpTargetForce()
    print("TCP target force / torque:", tcp_target_force)
    # API call: Get robot arm joint state
    joint_state = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getJointState()
    print("Robot arm joint state:", joint_state)
    # API call: Get joint servo status
    joint_servo_mode = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getJointServoMode()
    print("Joint servo status:", joint_servo_mode)
    # API call: Get robot arm joint angles
    joint_positions = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getJointPositions()
    print("Robot arm joint angles:", joint_positions)
    # API call: Get robot arm joint speeds
    joint_speeds = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getJointSpeeds()
    print("Robot arm joint speeds:", joint_speeds)
    # API call: Get robot arm joint accelerations
    joint_acc = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getJointAccelerations()
    print("Robot arm joint accelerations:", joint_acc)
    # API call: Get robot arm joint torque
    joint_torque_sensors = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getJointTorqueSensors()
    print("Robot arm joint torque:", joint_torque_sensors)
    # API call: Get base force sensor readings
    base_force_sensors = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getBaseForceSensor()
    print("Base force sensor readings:", base_force_sensors)
    # API call: Get TCP force sensor readings
    tcp_force_sensors = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getTcpForceSensors()
    print("TCP force sensor readings:", tcp_force_sensors)
    # API call: Get robot arm joint currents
    joint_currents = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getJointCurrents()
    print("Robot arm joint currents:", joint_currents)
    # API call: Get robot arm joint voltages
    joint_voltages = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getJointVoltages()
    print("Robot arm joint voltages:", joint_voltages)
    # API call: Get robot arm joint temperatures
    joint_temperatures = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getJointTemperatures()
    print("Robot arm joint temperatures:", joint_temperatures)
    # API call: Get joint UniqueId
    joint_unique_ids = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getJointUniqueIds()
    print("Joint UniqueId:", joint_unique_ids)
    # API call: Get joint firmware version
    joint_firmware_versions = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getJointFirmwareVersions()
    print("Joint firmware version:", joint_firmware_versions)
    # API call: Get joint hardware version
    joint_hardward_versions = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getJointHardwareVersions()
    print("Joint hardware version:", joint_hardward_versions)
    # API call: Get master board UniqueId
    master_board_unique_id = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getMasterBoardUniqueId()
    print("Master board UniqueId:", master_board_unique_id)
    # API call: Get MasterBoard firmware version
    master_board_firmware_version = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getMasterBoardFirmwareVersion()
    print("MasterBoard firmware version:", master_board_firmware_version)
    # API call: Get MasterBoard hardware version
    master_board_hardware_version = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getMasterBoardHardwareVersion()
    print("MasterBoard hardware version:", master_board_hardware_version)
    # API call: Get slave board UniqueId
    slave_board_unique_id = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getSlaveBoardUniqueId()
    print("Slave board UniqueId:", slave_board_unique_id)
    # API call: Get SlaveBoard firmware version
    slave_board_firmware_version = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getSlaveBoardFirmwareVersion()
    print("SlaveBoard firmware version:", slave_board_firmware_version)
    # API call: Get SlaveBoard hardware version
    slave_board_hardware_version = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getSlaveBoardHardwareVersion()
    print("SlaveBoard hardware version:", slave_board_hardware_version)
    # API call: Get tool end global unique ID
    tool_unique_id = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getToolUniqueId()
    print("Tool end global unique ID:", tool_unique_id)
    # API call: Get tool end firmware version
    tool_firmware_version = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getToolFirmwareVersion()
    print("Tool end firmware version:", tool_firmware_version)
    # API call: Get tool end hardware version
    tool_hardware_version = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getToolHardwareVersion()
    print("Tool end hardware version:", tool_hardware_version)
    # API call: Get pedestal global unique ID
    pedestal_unique_id = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getPedestalUniqueId()
    print("Pedestal global unique ID:", pedestal_unique_id)
    # API call: Get pedestal firmware version
    pedestal_firmware_version = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getPedestalFirmwareVersion()
    print("Pedestal firmware version:", pedestal_firmware_version)
    # API call: Get pedestal hardware version
    pedestal_hardware_version = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getPedestalHardwareVersion()
    print("Pedestal hardware version:", pedestal_hardware_version)
    # API call: Get robot arm joint target position angles
    joint_target_positions = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getJointTargetPositions()
    print("Robot arm joint target position angles:", joint_target_positions)
    # API call: Get robot arm joint target speeds
    joint_target_speeds = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getJointTargetSpeeds()
    print("Robot arm joint target speeds:", joint_target_speeds)
    # API call: Get robot arm joint target accelerations
    joint_target_acc = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getJointTargetAccelerations()
    print("Robot arm joint target accelerations:", joint_target_acc)
    # API call: Get robot arm joint target torque
    joint_target_torques = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getJointTargetTorques()
    print("Robot arm joint target torque:", joint_target_torques)
    # API call: Get robot arm joint target currents
    joint_target_currents = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getJointTargetCurrents()
    print("Robot arm joint target currents:", joint_target_currents)
    # API call: Get control box temperature
    control_box_temperature = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getControlBoxTemperature()
    print("Control box temperature:", control_box_temperature)
    # API call: Get main bus voltage
    main_voltage = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getMainVoltage()
    print("Main bus voltage:", main_voltage)
    # API call: Get main bus current
    main_current = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getMainCurrent()
    print("Main bus current:", main_current)
    # API call: Get robot voltage
    robot_voltage = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getRobotVoltage()
    print("Robot voltage:", robot_voltage)
    # API call: Get robot current
    robot_current = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getRobotCurrent()
    print("Robot current:", robot_current)


if __name__ == '__main__':
    robot_rpc_client.connect(robot_ip, robot_port)  # API call: Connect to RPC service
    if robot_rpc_client.hasConnected():
        print("Robot rcp_client connected successfully!")
        robot_rpc_client.login("aubo", "123456")  # API call: Robot arm login
        if robot_rpc_client.hasLogined():
            print("Robot rcp_client logined successfully!")
            robot_name = robot_rpc_client.getRobotNames()[0]  # API call: Get robot name
            exampleState(robot_name)  # Get robot arm status information
