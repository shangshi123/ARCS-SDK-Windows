#include "example_6.h"

#include <thread>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <sstream>
#include <fstream>
#include "util.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define SERVER_HOST "127.0.0.1"
#define SERVER_PORT 30000

void Example_6::demo()
{
    ServiceInterface robotService;

    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    /** Interface call: login ***/
    ret = robotService.robotServiceLogin(SERVER_HOST, SERVER_PORT, "aubo",
                                         "123456");
    if (ret == aubo_robot_namespace::InterfaceCallSuccCode) {
        std::cout << "login successful." << std::endl;
    } else {
        std::cerr << "login failed." << std::endl;
    }

    aubo_robot_namespace::ToolInEndDesc
        toolDesc; // Tools used to calibrate coordinates
    toolDesc.toolInEndPosition.x = 0;
    toolDesc.toolInEndPosition.y = 0;
    toolDesc.toolInEndPosition.z = 0;

    toolDesc.toolInEndOrientation.w = 1;
    toolDesc.toolInEndOrientation.x = 0;
    toolDesc.toolInEndOrientation.y = 0;
    toolDesc.toolInEndOrientation.z = 0;

    /** Set coordinate system parameters **/
    aubo_robot_namespace::CoordCalibrateByJointAngleAndTool userCoord;
    userCoord.coordType = aubo_robot_namespace::WorldCoordinate;
    userCoord.methods = aubo_robot_namespace::
        Origin_AnyPointOnPositiveXAxis_AnyPointOnFirstQuadrantOfXOYPlane;
    userCoord.toolDesc = toolDesc;

    // [0]={0,-0.12726,-1.32112,0.37693,-1.570796,0}
    userCoord.wayPointArray[0].jointPos[0] = 0;
    userCoord.wayPointArray[0].jointPos[1] = -0.12726;
    userCoord.wayPointArray[0].jointPos[2] = -1.32112;
    userCoord.wayPointArray[0].jointPos[3] = 0.37693;
    userCoord.wayPointArray[0].jointPos[4] = -1.570796;
    userCoord.wayPointArray[0].jointPos[5] = 0;

    // [1]={0,-0.26958,-1.43050,0.40987,-1.570796,0}
    userCoord.wayPointArray[1].jointPos[0] = 0;
    userCoord.wayPointArray[1].jointPos[1] = -0.26958;
    userCoord.wayPointArray[1].jointPos[2] = -1.43050;
    userCoord.wayPointArray[1].jointPos[3] = 0.40987;
    userCoord.wayPointArray[1].jointPos[4] = -1.570796;
    userCoord.wayPointArray[1].jointPos[5] = 0;

    // [2]={-0.13974,-0.30070,-1.451246,0.420254,-1.57079,-0.139750}
    userCoord.wayPointArray[2].jointPos[0] = -0.13974;
    userCoord.wayPointArray[2].jointPos[1] = -0.30070;
    userCoord.wayPointArray[2].jointPos[2] = -1.451246;
    userCoord.wayPointArray[2].jointPos[3] = 0.420254;
    userCoord.wayPointArray[2].jointPos[4] = -1.570796;
    userCoord.wayPointArray[2].jointPos[5] = -0.139750;

    /** Interface call: Initialize motion properties***/
    robotService.robotServiceInitGlobalMoveProfile();

    /** Interface call: Set the maximum acceleration of the articulated motion
     * ***/
    aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
    jointMaxAcc.jointPara[0] = 50.0 / 180.0 * M_PI;
    jointMaxAcc.jointPara[1] = 50.0 / 180.0 * M_PI;
    jointMaxAcc.jointPara[2] = 50.0 / 180.0 * M_PI;
    jointMaxAcc.jointPara[3] = 50.0 / 180.0 * M_PI;
    jointMaxAcc.jointPara[4] = 50.0 / 180.0 * M_PI;
    jointMaxAcc.jointPara[5] =
        50.0 / 180.0 * M_PI; // The interface requires the unit to be radians
    robotService.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);

    /** Interface call: set the maximum speed of articulated motion ***/
    aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
    jointMaxVelc.jointPara[0] = 50.0 / 180.0 * M_PI;
    jointMaxVelc.jointPara[1] = 50.0 / 180.0 * M_PI;
    jointMaxVelc.jointPara[2] = 50.0 / 180.0 * M_PI;
    jointMaxVelc.jointPara[3] = 50.0 / 180.0 * M_PI;
    jointMaxVelc.jointPara[4] = 50.0 / 180.0 * M_PI;
    jointMaxVelc.jointPara[5] =
        50.0 / 180.0 * M_PI; // The interface requires the unit to be radians
    robotService.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);

    /** Interface call: Set the maximum acceleration of the end type motion
     * Linear motion belongs to the end type motion***/
    double lineMoveMaxAcc;
    lineMoveMaxAcc = 2.0; // Units m/s2
    robotService.robotServiceSetGlobalMoveEndMaxLineAcc(lineMoveMaxAcc);

    /** Interface call: Set the maximum speed of the end type motion Linear
     * motion belongs to the end type motion ***/
    double lineMoveMaxVelc;
    lineMoveMaxVelc = 2.0; // Units m/s
    robotService.robotServiceSetGlobalMoveEndMaxLineVelc(lineMoveMaxVelc);

    /** The arm moves to the ready position **/
    double jointAngle[aubo_robot_namespace::ARM_DOF] = { 0 };
    jointAngle[0] = 0;
    jointAngle[1] = -0.12726;
    jointAngle[2] = -1.32112;
    jointAngle[3] = 0.37693;
    jointAngle[4] = -1.570796;
    jointAngle[5] = 0;

    if (robotService.robotServiceJointMove(jointAngle, true) !=
        aubo_robot_namespace::InterfaceCallSuccCode) //关节运动至准备点
    {
        std::cerr << "Movement to preparation point failed.　ret:" << ret
                  << std::endl;
    }

    aubo_robot_namespace::Pos position;
    aubo_robot_namespace::ToolInEndDesc toolDesc_1; // Tool during mvoe
    toolDesc_1.toolInEndPosition.x = 0;
    toolDesc_1.toolInEndPosition.y = 0;
    toolDesc_1.toolInEndPosition.z = 0;

    toolDesc_1.toolInEndOrientation.w = 1;
    toolDesc_1.toolInEndOrientation.x = 0;
    toolDesc_1.toolInEndOrientation.y = 0;
    toolDesc_1.toolInEndOrientation.z = 0;

    for (int i = 0; i < 10; i++) {
        position.x = 0.0;
        position.y = 0.0;
        position.z = 0.0;
        // Keep the current pose moving through the linear motion to move the
        // tool end point (toolDesc_1) to the specified position of the
        // specified coordinate system (userCoord).
        if (robotService.robotMoveLineToTargetPosition(userCoord, position,
                                                       toolDesc_1, true) !=
            aubo_robot_namespace::InterfaceCallSuccCode) {
            std::cerr << "robotMoveLineToTargetPosition.　ret:" << ret
                      << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        position.x = (i % 3 == 0) ? 0.15 : 0.0;
        position.y = (i % 3 == 1) ? 0.15 : 0.0;
        ;
        position.z = (i % 3 == 2) ? -0.15 : 0.0;
        ;
        // Keep the current pose moving through the linear motion to move the
        // tool end point (toolDesc_1) to the specified position of the
        // specified coordinate system (userCoord).
        if (robotService.robotMoveLineToTargetPosition(userCoord, position,
                                                       toolDesc_1, true) !=
            aubo_robot_namespace::InterfaceCallSuccCode) {
            std::cerr << "robotMoveLineToTargetPosition.　ret:" << ret
                      << std::endl;
        }
    }

    /** Robotic arm shutdown**/
    robotService.robotServiceRobotShutdown();

    /** Interface call: logout　**/
    robotService.robotServiceLogout();
}

void Example_6::example_MoveLtoPosition()
{
    ServiceInterface robotService;

    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    /** 接口调用: 登录 ***/
    ret = robotService.robotServiceLogin(SERVER_HOST, SERVER_PORT, "aubo",
                                         "123456");
    if (ret == aubo_robot_namespace::InterfaceCallSuccCode) {
        std::cerr << "登录成功." << std::endl;
    } else {
        std::cerr << "登录成功." << std::endl;
    }

    /** 如果是连接真实机械臂，需要对机械臂进行初始化　**/
    aubo_robot_namespace::ROBOT_SERVICE_STATE result;

    //工具动力学参数
    aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
    memset(&toolDynamicsParam, 0, sizeof(toolDynamicsParam));

    ret = robotService.rootServiceRobotStartup(
        toolDynamicsParam /**工具动力学参数**/, 6 /*碰撞等级*/,
        true /*是否允许读取位姿　默认为true*/, true, /*保留默认为true */
        1000,                                        /*保留默认为1000 */
        result);                                     /*机械臂初始化*/
    if (ret == aubo_robot_namespace::InterfaceCallSuccCode) {
        std::cerr << "机械臂初始化成功." << std::endl;
    } else {
        std::cerr << "机械臂初始化失败." << std::endl;
    }
    /** 接口调用: 初始化运动属性 ***/
    robotService.robotServiceInitGlobalMoveProfile();

    /** 接口调用: 设置关节型运动的最大加速度 ***/
    aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
    jointMaxAcc.jointPara[0] = 50.0 / 180.0 * M_PI;
    jointMaxAcc.jointPara[1] = 50.0 / 180.0 * M_PI;
    jointMaxAcc.jointPara[2] = 50.0 / 180.0 * M_PI;
    jointMaxAcc.jointPara[3] = 50.0 / 180.0 * M_PI;
    jointMaxAcc.jointPara[4] = 50.0 / 180.0 * M_PI;
    jointMaxAcc.jointPara[5] = 50.0 / 180.0 * M_PI; //接口要求单位是弧度
    robotService.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);

    /** 接口调用: 设置关节型运动的最大速度 ***/
    aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
    jointMaxVelc.jointPara[0] = 50.0 / 180.0 * M_PI;
    jointMaxVelc.jointPara[1] = 50.0 / 180.0 * M_PI;
    jointMaxVelc.jointPara[2] = 50.0 / 180.0 * M_PI;
    jointMaxVelc.jointPara[3] = 50.0 / 180.0 * M_PI;
    jointMaxVelc.jointPara[4] = 50.0 / 180.0 * M_PI;
    jointMaxVelc.jointPara[5] = 50.0 / 180.0 * M_PI; //接口要求单位是弧度
    robotService.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);

    /** 机械臂运动到准备位置 **/
    double jointAngle[aubo_robot_namespace::ARM_DOF] = { 0 };
    jointAngle[0] = 0.0 / 180.0 * M_PI;
    jointAngle[1] = 0.0 / 180.0 * M_PI;
    jointAngle[2] = 90.0 / 180.0 * M_PI;
    jointAngle[3] = 0.0 / 180.0 * M_PI;
    jointAngle[4] = 90.0 / 180.0 * M_PI;
    jointAngle[5] = 0.0 / 180.0 * M_PI;
    ret = robotService.robotServiceJointMove(jointAngle, true);
    //关节运动至准备点
    if (ret != aubo_robot_namespace::InterfaceCallSuccCode) {
        std::cerr << "运动0失败.　ret:" << ret << std::endl;
    }

    /** 接口调用: 初始化运动属性 ***/
    robotService.robotServiceInitGlobalMoveProfile();

    /** 接口调用: 设置末端型运动的最大加速度 　　直线运动属于末端型运动***/
    double lineMoveMaxAcc;
    lineMoveMaxAcc = 2; //单位米每秒
    robotService.robotServiceSetGlobalMoveEndMaxLineAcc(lineMoveMaxAcc);

    /** 接口调用: 设置末端型运动的最大速度 直线运动属于末端型运动 ***/
    double lineMoveMaxVelc;
    lineMoveMaxVelc = 2; //单位米每秒
    robotService.robotServiceSetGlobalMoveEndMaxLineVelc(lineMoveMaxVelc);

    aubo_robot_namespace::wayPoint_S currentWaypoint;
    ret = robotService.robotServiceGetCurrentWaypointInfo(currentWaypoint);
    if (ret != aubo_robot_namespace::InterfaceCallSuccCode) {
        std::cerr << "获取当前路点信息失败.　ret:" << ret << std::endl;
    }

    /** 设置坐标系参数 **/
    aubo_robot_namespace::CoordCalibrateByJointAngleAndTool userCoord;
    userCoord.coordType = aubo_robot_namespace::BaseCoordinate;

    /** 设置工具端参数 **/
    aubo_robot_namespace::ToolInEndDesc toolDesc;
    toolDesc.toolInEndPosition.x = 0;
    toolDesc.toolInEndPosition.y = 0;
    toolDesc.toolInEndPosition.z = 0;

    toolDesc.toolInEndOrientation.w = 1;
    toolDesc.toolInEndOrientation.x = 0;
    toolDesc.toolInEndOrientation.y = 0;
    toolDesc.toolInEndOrientation.z = 0;

    for (int i = 0; i < 10; i++) {
        aubo_robot_namespace::Pos position;
        double offsetY = (i % 3 > 1) ? 0.0 : (0.18 * (i % 3));
        double offsetZ = (i % 3 > 1) ? 0.0 : (0.18);
        position.x = currentWaypoint.cartPos.position.x;
        position.y = currentWaypoint.cartPos.position.y + offsetY;
        position.z = currentWaypoint.cartPos.position.z + offsetZ;

        //保持当前位姿通过直线运动的方式运动到目标位置
        ret = robotService.robotMoveLineToTargetPosition(userCoord, position,
                                                         toolDesc, true);
        if (ret != aubo_robot_namespace::InterfaceCallSuccCode) {
            std::cerr << "robotMoveLineToTargetPosition.　ret:" << ret
                      << std::endl;
        }
    }

    /** 机械臂Shutdown **/
    robotService.robotServiceRobotShutdown();

    /** 接口调用: 退出登录　**/
    robotService.robotServiceLogout();
}

void Example_6::example_MoveJtoPosition()
{
    ServiceInterface robotService;

    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    /** 接口调用: 登录 ***/
    ret = robotService.robotServiceLogin(SERVER_HOST, SERVER_PORT, "aubo",
                                         "123456");
    if (ret == aubo_robot_namespace::InterfaceCallSuccCode) {
        std::cerr << "登录成功." << std::endl;
    } else {
        std::cerr << "登录成功." << std::endl;
    }

    /** 如果是连接真实机械臂，需要对机械臂进行初始化　**/
    aubo_robot_namespace::ROBOT_SERVICE_STATE result;

    //工具动力学参数
    aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
    memset(&toolDynamicsParam, 0, sizeof(toolDynamicsParam));

    ret = robotService.rootServiceRobotStartup(
        toolDynamicsParam /**工具动力学参数**/, 6 /*碰撞等级*/,
        true /*是否允许读取位姿　默认为true*/, true, /*保留默认为true */
        1000,                                        /*保留默认为1000 */
        result);                                     /*机械臂初始化*/
    if (ret == aubo_robot_namespace::InterfaceCallSuccCode) {
        std::cerr << "机械臂初始化成功." << std::endl;
    } else {
        std::cerr << "机械臂初始化失败." << std::endl;
    }
    /** 接口调用: 初始化运动属性 ***/
    robotService.robotServiceInitGlobalMoveProfile();

    /** 接口调用: 设置关节型运动的最大加速度 ***/
    aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
    jointMaxAcc.jointPara[0] = 50.0 / 180.0 * M_PI;
    jointMaxAcc.jointPara[1] = 50.0 / 180.0 * M_PI;
    jointMaxAcc.jointPara[2] = 50.0 / 180.0 * M_PI;
    jointMaxAcc.jointPara[3] = 50.0 / 180.0 * M_PI;
    jointMaxAcc.jointPara[4] = 50.0 / 180.0 * M_PI;
    jointMaxAcc.jointPara[5] = 50.0 / 180.0 * M_PI; //接口要求单位是弧度
    robotService.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);

    /** 接口调用: 设置关节型运动的最大速度 ***/
    aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
    jointMaxVelc.jointPara[0] = 50.0 / 180.0 * M_PI;
    jointMaxVelc.jointPara[1] = 50.0 / 180.0 * M_PI;
    jointMaxVelc.jointPara[2] = 50.0 / 180.0 * M_PI;
    jointMaxVelc.jointPara[3] = 50.0 / 180.0 * M_PI;
    jointMaxVelc.jointPara[4] = 50.0 / 180.0 * M_PI;
    jointMaxVelc.jointPara[5] = 50.0 / 180.0 * M_PI; //接口要求单位是弧度
    robotService.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);

    /** 机械臂运动到准备位置 **/
    double jointAngle[aubo_robot_namespace::ARM_DOF] = { 0 };
    jointAngle[0] = 0.0 / 180.0 * M_PI;
    jointAngle[1] = 0.0 / 180.0 * M_PI;
    jointAngle[2] = 90.0 / 180.0 * M_PI;
    jointAngle[3] = 0.0 / 180.0 * M_PI;
    jointAngle[4] = 90.0 / 180.0 * M_PI;
    jointAngle[5] = 0.0 / 180.0 * M_PI;
    ret = robotService.robotServiceJointMove(jointAngle, true);
    //关节运动至准备点
    if (ret != aubo_robot_namespace::InterfaceCallSuccCode) {
        std::cerr << "运动0失败.　ret:" << ret << std::endl;
    }

    aubo_robot_namespace::wayPoint_S currentWaypoint;
    ret = robotService.robotServiceGetCurrentWaypointInfo(currentWaypoint);
    if (ret != aubo_robot_namespace::InterfaceCallSuccCode) {
        std::cerr << "获取当前路点信息失败.　ret:" << ret << std::endl;
    }
    /** 设置坐标系参数 **/
    aubo_robot_namespace::CoordCalibrateByJointAngleAndTool userCoord;
    userCoord.coordType = aubo_robot_namespace::BaseCoordinate;
    /** 设置工具端参数 **/
    aubo_robot_namespace::ToolInEndDesc toolDesc;
    toolDesc.toolInEndPosition.x = 0;
    toolDesc.toolInEndPosition.y = 0;
    toolDesc.toolInEndPosition.z = 0;

    for (int i = 0; i < 10; i++) {
        aubo_robot_namespace::Pos position;
        double offsetY = (i % 3 > 1) ? 0.0 : (0.18 * (i % 3));
        double offsetZ = (i % 3 > 1) ? 0.0 : (0.18);
        position.x = currentWaypoint.cartPos.position.x;
        position.y = currentWaypoint.cartPos.position.y + offsetY;
        position.z = currentWaypoint.cartPos.position.z + offsetZ;

        //保持当前位姿通过直线运动的方式运动到目标位置
        ret = robotService.robotMoveJointToTargetPosition(userCoord, position,
                                                          toolDesc, true);
        if (ret != aubo_robot_namespace::InterfaceCallSuccCode) {
            std::cerr << "robotMoveLineToTargetPosition.　ret:" << ret
                      << std::endl;
        }
    }

    /** 机械臂Shutdown **/
    robotService.robotServiceRobotShutdown();

    /** 接口调用: 退出登录　**/
    robotService.robotServiceLogout();
}

void Example_6::Robot_Move()
{
    ServiceInterface robotService;

    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    /** Interface call: login ***/
    ret = robotService.robotServiceLogin(SERVER_HOST, SERVER_PORT, "aubo",
                                         "123456");
    if (ret == aubo_robot_namespace::InterfaceCallSuccCode) {
        std::cout << "login successful." << std::endl;
    } else {
        std::cerr << "login failed." << std::endl;
    }

    // set tool kinematics para
    aubo_robot_namespace::ToolInEndDesc toolInEndDesc;
    toolInEndDesc.toolInEndPosition.x = 0.03;
    toolInEndDesc.toolInEndPosition.y = 0.05;
    toolInEndDesc.toolInEndPosition.z = 0.08;
    toolInEndDesc.toolInEndOrientation.w = 1.0;
    toolInEndDesc.toolInEndOrientation.x = 0.0;
    toolInEndDesc.toolInEndOrientation.y = 0.0;
    toolInEndDesc.toolInEndOrientation.z = 0.0;

    aubo_robot_namespace::CoordCalibrateByJointAngleAndTool userCoord;
    //    userCoord.coordType = aubo_robot_namespace::EndCoordinate;
    userCoord.coordType = aubo_robot_namespace::BaseCoordinate;
    userCoord.toolDesc = toolInEndDesc;

    if (ret != aubo_robot_namespace::InterfaceCallSuccCode) {
        std::cerr << "Movement to waypoint 2 failed.　ret:" << ret << std::endl;
    }

    double rotateAxisOnUserCoord[] = { 0, 0, 1 };
    aubo_robot_namespace::JointVelcAccParam moveMaxVelc;
    for (int i = 0; i < 6; i++) {
        moveMaxVelc.jointPara[i] = 175.0;
    }

    robotService.robotServiceSetGlobalMoveJointMaxVelc(moveMaxVelc);
    robotService.robotServiceSetGlobalMoveEndMaxLineVelc(1.8);
    robotService.robotServiceSetToolKinematicsParam(toolInEndDesc);

    robotService.robotServiceRotateMove(userCoord, rotateAxisOnUserCoord,
                                        60 * M_PI / 180, true);
    printf("============== %f \n", 90 * M_PI / 180);
}
