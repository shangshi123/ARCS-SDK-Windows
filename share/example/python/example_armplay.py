#! /usr/bin/env python
# coding=utf-8

"""
PathBuffer Motion

Steps:
Step 1: Connect to RPC service, robot login, set RPC request timeout
Step 2: Read JSON file
Step 3: Joint motion to the first waypoint in the JSON file
Step 4: Release path buffer
Step 5: Create a new path point buffer "rec"
Step 6: Add waypoints from the JSON file to the "rec" buffer
Step 7: Time-consuming operations such as calculation and optimization; will not recalculate if parameters are unchanged
Step 8: Check if the "rec" buffer is valid,
    If valid, proceed to the next step; otherwise, keep blocking
Step 9: Execute the buffered path
Step 10: Disconnect RPC connection

"""

import pyaubo_sdk
import time
import json
import sys

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
        id = impl.getMotionControl().getExecId()
        if id == -1:
            break
        time.sleep(0.05)


if __name__ == '__main__':
    robot_rpc_client.setRequestTimeout(1000)  # API call: Set RPC timeout
    robot_rpc_client.connect(robot_ip, robot_port)  # API call: Connect to RPC service
    if robot_rpc_client.hasConnected():
        print("Robot rcp_client connected successfully!")
        robot_rpc_client.login("aubo", "123456")  # API call: Robot login
        if robot_rpc_client.hasLogined():
            print("Robot rcp_client logined successfully!")

            robot_rpc_client.setRequestTimeout(40)  # API call: Set RPC request timeout

            # Read JSON file
            f = open('../c++/trajs/aubo-joint-test-0929-10.armplay')
            input = json.load(f)
            traj = input['jointlist']
            interval = input['interval']
            print("Sampling time interval: ", interval)

            traj_sz = len(traj)
            if traj_sz == 0:
                print("No waypoints")
                sys.exit()
            else:
                print("Number of loaded points: ", len(traj))

            robot_name = robot_rpc_client.getRobotNames()[0]  # API call: Get robot name
            mc = robot_rpc_client.getRobotInterface(robot_name).getMotionControl()

            # Joint motion
            q1 = traj[0]
            print("q1: ", q1)
            print("goto p1")
            mc.moveJoint(q1, M_PI, M_PI, 0., 0.)
            waitArrival(robot_rpc_client.getRobotInterface(robot_name))

            print("pathBufferAlloc")
            mc.pathBufferFree("rec")  # API call: Release path buffer
            mc.pathBufferAlloc("rec", 2, traj_sz)  # API call: Create a new path point buffer

            # Add waypoints from traj to "rec" buffer
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

            print("pathBufferEval ", mc.pathBufferEval("rec", [], [], interval))  # API call: Calculation, optimization, etc.; will not recalculate if parameters are unchanged
            while not mc.pathBufferValid("rec"):
                print("pathBufferValid: ", mc.pathBufferValid("rec"))  # API call: Check if buffer with specified name is valid
                time.sleep(0.0005)
            mc.movePathBuffer("rec")  # API call: Execute buffered path
            waitMovePathBufferFinished(robot_rpc_client.getRobotInterface(robot_name))
            print("end movePathBuffer")
            robot_rpc_client.disconnect()  # API call: Disconnect RPC connection

