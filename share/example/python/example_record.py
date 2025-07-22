#! /usr/bin/env python
# coding=utf-8

"""
PathBuffer motion

Steps:
Step 1: Connect to RPC service, robot login, set RPC request timeout
Step 2: Read .offt trajectory file and load trajectory points
Step 3: Move joints to the first waypoint in the trajectory file
Step 4: Release path buffer
Step 5: Create a new path point buffer "rec"
Step 6: Add waypoints from the trajectory file to the "rec" buffer
Step 7: Time-consuming operations such as calculation and optimization; if the parameters passed in are the same, it will not recalculate
Step 8: Check whether the "rec" buffer is valid,
    If valid, execute the next step; otherwise, keep blocking
Step 9: Execute the cached path
Step 10: Disconnect RPC connection
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


# Wait for MovePathBuffer to finish
def waitMovePathBufferFinished(impl):
    while impl.getMotionControl().getExecId() == -1:
        time.sleep(0.05)
    while True:
        ids = impl.getMotionControl().getExecId()
        if ids == -1:
            break
        time.sleep(0.05)


if __name__ == '__main__':
    robot_rpc_client.setRequestTimeout(1000)  # API call: Set RPC request timeout
    robot_rpc_client.connect(robot_ip, robot_port)  # API call: Connect to RPC service
    if robot_rpc_client.hasConnected():
        print("Robot rcp_client connected successfully!")
        robot_rpc_client.login("aubo", "123456")  # API call: Robot login
        if robot_rpc_client.hasLogined():
            print("Robot rcp_client logined successfully!")

            robot_name = robot_rpc_client.getRobotNames()[0]  # API call: Get robot name
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
            print("goto p1")
            mc = robot.getMotionControl()
            mc.moveJoint(traj[0], M_PI, M_PI, 0., 0.)
            waitArrival(robot)

            print("pathBufferAlloc")
            mc.pathBufferFree("rec")  # API call: Release path buffer
            mc.pathBufferAlloc("rec", 2, traj_sz)  # API call: Create a new path point buffer

            # Add waypoints from traj_q to the "rec" buffer
            # Add every 10 waypoints as a list
            # When the number of remaining waypoints is less than or equal to 10, add them as the last group
            offset = 10
            it = 0
            while True:
                print("pathBufferAppend ", offset)
                mc.pathBufferAppend("rec", traj[it:it + 10:1])  # API call: Add waypoints to path buffer
                it += 10
                if offset + 10 >= traj_sz:
                    print("pathBufferAppend ", traj_sz)
                    mc.pathBufferAppend("rec", traj[it:traj_sz:1])
                    break
                offset += 10

            interval = 0.005
            print("pathBufferEval ", mc.pathBufferEval("rec", [], [],
                                                       interval))  # API call: Time-consuming operations such as calculation and optimization; if the parameters passed in are the same, it will not recalculate
            while not mc.pathBufferValid("rec"):
                print("pathBufferValid: ", mc.pathBufferValid("rec"))  # API call: Whether the buffer with the specified name is valid
                time.sleep(0.005)
            mc.movePathBuffer("rec")  # API call: Execute the cached path
            waitMovePathBufferFinished(robot_rpc_client.getRobotInterface(robot_name))
            print("end movePathBuffer")
            robot_rpc_client.disconnect()  # API call: Disconnect RPC connection

