#! /usr/bin/env python
# coding=utf-8

"""
servoj motion

Steps:
Step 1: Connect to the RPC service, robot login, set RPC request timeout
Step 2: Read .offt trajectory file
Step 3: Move joints to the first point in the trajectory
Step 4: Enable servo mode
Step 5: Perform servoj motion
Step 6: Disable servo mode
Step 7: Disconnect RPC connection
"""
import pyaubo_sdk
import time

robot_ip = "127.0.0.1"  # Server IP address
robot_port = 30004  # Port number
M_PI = 3.14159265358979323846
robot_rpc_client = pyaubo_sdk.RpcClient()


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

def servoj():
    robot_name = robot_rpc_client.getRobotNames()[0]  # API call: Get the robot's name
    robot = robot_rpc_client.getRobotInterface(robot_name)

    # Read trajectory file and load trajectory points
    file = open('../c++/trajs/record6.offt')
    traj = []
    for line in file:
        str_list = line.split(",")
        float_list = []
        for strs in str_list:
            float_list.append(float(strs))
        traj.append(float_list)

    traj_sz = len(traj)
    if traj_sz == 0:
        print("No trajectory points")
    else:
        print("Number of loaded trajectory points: ", traj_sz)

    # Move joints to the first point
    # The current position must be consistent with the first point in the trajectory, otherwise it is easy to cause large overshoot
    print("goto p1")
    robot_rpc_client.getRuntimeMachine().start()
    mc = robot.getMotionControl()
    mc.moveJoint(traj[0], M_PI, M_PI, 0., 0.)
    waitArrival(robot)

    # Enable servo mode
    robot.getMotionControl().setServoMode(True)
    i = 0
    while not mc.isServoModeEnabled():
        i = i + 1
        if i > 5:
            print("Servo Mode is ", mc.isServoModeEnabled())
            return -1
        time.sleep(0.005)

    traj.remove(traj[0])
    for q in traj:
        mc.servoJoint(q, 0.1, 0.2, 0.005, 0.1, 200)
        time.sleep(0.005)

    # Disable servo mode
    mc.setServoMode(False)
    print("Servoj end")
    return 0


if __name__ == '__main__':
    robot_rpc_client.setRequestTimeout(1000)  # API call: Set RPC request timeout
    robot_rpc_client.connect(robot_ip, robot_port)  # API call: Connect to RPC service
    if robot_rpc_client.hasConnected():
        print("Robot rcp_client connected successfully!")
        robot_rpc_client.login("aubo", "123456")  # API call: Robot login
        if robot_rpc_client.hasLogined():
            print("Robot rcp_client logined successfully!")
            servoj()
            robot_rpc_client.disconnect()  # API call: Disconnect RPC connection

