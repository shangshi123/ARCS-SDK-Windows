#include "aubo_sdk/rpc.h"
#include <math.h>
#ifdef WIN32
#include <Windows.h>
#endif
using namespace arcs::common_interface;
using namespace arcs::aubo_sdk;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Blocking function: The program continues only after the robot arm reaches the target waypoint
int waitArrival(RobotInterfacePtr impl)
{
    const int max_retry_count = 5;
    int cnt = 0;

    // API call: Get the current motion command ID
    int exec_id = impl->getMotionControl()->getExecId();

    // Wait for the robot arm to start moving
    while (exec_id == -1) {
        if (cnt++ > max_retry_count) {
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        exec_id = impl->getMotionControl()->getExecId();
    }

    // Wait for the robot arm to finish the action
    while (impl->getMotionControl()->getExecId() != -1) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return 0;
}

// Arc motion
void exampleMoveArc(RpcClientPtr cli)
{
    // API call: Get the robot's name
    auto robot_name = cli->getRobotNames().front();

    auto robot_interface = cli->getRobotInterface(robot_name);

    // API call: Set the speed fraction for robot arm movement
    robot_interface->getMotionControl()->setSpeedFraction(0.3);

    // API call: Set the tool center point
    std::vector<double> tcp_offset(6, 0.0);
    robot_interface->getRobotConfig()->setTcpOffset(tcp_offset);

    // Joint angles, unit: radians
    std::vector<double> joint_angle = { 0.0,
                                        -15.0 * (M_PI / 180),
                                        100.0 * (M_PI / 180),
                                        25.0 * (M_PI / 180),
                                        90.0 * (M_PI / 180),
                                        0.0 };
    // API call: Move joints to initial position
    robot_interface->getMotionControl()->moveJoint(
        joint_angle, 80 * (M_PI / 180), 60 * (M_PI / 180), 0, 0);

    // Blocking
    int ret = waitArrival(robot_interface);
    if (ret == 0) {
        std::cout << "Joint moved to initial position successfully" << std::endl;
    } else {
        std::cout << "Joint failed to move to initial position" << std::endl;
    }

    // Pose
    std::vector<double> pose1 = {
        0.440164, -0.119, 0.2976, 2.456512, 0, 1.5708
    };

    std::vector<double> pose2 = { 0.440164, -0.00249391, 0.398658,
                                  2.45651,  0,           1.5708 };

    std::vector<double> pose3 = { 0.440164, 0.166256, 0.297599,
                                  2.45651,  0,        1.5708 };

    // API call: Linear motion to the starting point of the arc—pose1
    robot_interface->getMotionControl()->moveLine(pose1, 1.2, 0.25, 0.025, 0);

    // Blocking
    ret = waitArrival(robot_interface);
    if (ret == 0) {
        std::cout << "Linear motion to arc starting point succeeded" << std::endl;
    } else {
        std::cout << "Linear motion to arc starting point failed" << std::endl;
    }

    // API call: Arc motion
    robot_interface->getMotionControl()->moveCircle(pose2, pose3, 1.2, 0.25, 0,
                                                    0);

    // Blocking
    ret = waitArrival(robot_interface);
    if (ret == 0) {
        std::cout << "Arc motion succeeded" << std::endl;
    } else {
        std::cout << "Arc motion failed" << std::endl;
    }
}

// Arc motion
void exampleMoveArc2(RpcClientPtr cli)
{
    // API call: Get the robot's name
    auto robot_name = cli->getRobotNames().front();

    auto robot_interface = cli->getRobotInterface(robot_name);

    // API call: Set the speed fraction for robot arm movement
    robot_interface->getMotionControl()->setSpeedFraction(0.3);

    // API call: Set the tool center point
    std::vector<double> tcp_offset(6, 0.0);
    robot_interface->getRobotConfig()->setTcpOffset(tcp_offset);

    // Joint angles, unit: radians
    std::vector<double> joint_angle = { 0.0,
                                        -15.0 * (M_PI / 180),
                                        100.0 * (M_PI / 180),
                                        25.0 * (M_PI / 180),
                                        90.0 * (M_PI / 180),
                                        0.0 };
    // API call: Move joints to initial position
    robot_interface->getMotionControl()->moveJoint(
        joint_angle, 80 * (M_PI / 180), 60 * (M_PI / 180), 0, 0);

    // Blocking
    int ret = waitArrival(robot_interface);
    if (ret == 0) {
        std::cout << "Joint moved to initial position successfully" << std::endl;
    } else {
        std::cout << "Joint failed to move to initial position" << std::endl;
    }

    // Pose
    std::vector<double> pose1 = {
        0.440164, -0.119, 0.2976, 2.456512, 0, 1.5708
    };

    std::vector<double> pose2 = { 0.440164, -0.00249391, 0.398658,
                                  2.45651,  0,           1.5708 };

    std::vector<double> pose3 = { 0.440164, 0.166256, 0.297599,
                                  2.45651,  0,        1.5708 };

    // API call: Linear motion to the starting point of the arc—pose1
    robot_interface->getMotionControl()->moveLine(pose1, 1.2, 0.25, 0.025, 0);

    // Blocking
    ret = waitArrival(robot_interface);
    if (ret == 0) {
        std::cout << "Linear motion to arc starting point succeeded" << std::endl;
    } else {
        std::cout << "Linear motion to arc starting point failed" << std::endl;
    }

    CircleParameters param;
    param.pose_via = pose2;
    param.pose_to = pose3;
    param.a = 1.2;
    param.v = 0.25;

    // API call: Arc motion
    robot_interface->getMotionControl()->moveCircle2(param);

    // Blocking
    ret = waitArrival(robot_interface);
    if (ret == 0) {
        std::cout << "Arc motion succeeded" << std::endl;
    } else {
        std::cout << "Arc motion failed" << std::endl;
    }
}

// Circular motion
void exampleMoveC(RpcClientPtr cli)
{
    // API call: Get the robot's name
    auto robot_name = cli->getRobotNames().front();

    auto robot_interface = cli->getRobotInterface(robot_name);

    // API call: Set the speed fraction for robot arm movement
    robot_interface->getMotionControl()->setSpeedFraction(0.3);

    // API call: Set the tool center point
    std::vector<double> tcp_offset(6, 0.0);
    robot_interface->getRobotConfig()->setTcpOffset(tcp_offset);

    // Joint angles, unit: radians
    std::vector<double> joint_angle = { 0.0,
                                        -15.0 * (M_PI / 180),
                                        100.0 * (M_PI / 180),
                                        25.0 * (M_PI / 180),
                                        90.0 * (M_PI / 180),
                                        0.0 };
    // API call: Move joints to initial position
    robot_interface->getMotionControl()->moveJoint(
        joint_angle, 80 * (M_PI / 180), 60 * (M_PI / 180), 0, 0);

    // Blocking
    int ret = waitArrival(robot_interface);
    if (ret == 0) {
        std::cout << "Joint moved to initial position successfully" << std::endl;
    } else {
        std::cout << "Joint failed to move to initial position" << std::endl;
    }

    // Pose
    std::vector<double> pose1 = {
        0.440164, -0.119, 0.2976, 2.456512, 0, 1.5708
    };

    std::vector<double> pose2 = { 0.440164, -0.00249391, 0.398658,
                                  2.45651,  0,           1.5708 };

    std::vector<double> pose3 = { 0.440164, 0.166256, 0.297599,
                                  2.45651,  0,        1.5708 };

    // API call: Linear motion to the starting point of the circle—pose1
    robot_interface->getMotionControl()->moveLine(pose1, 1.2, 0.25, 0.025, 0);

    // Blocking
    ret = waitArrival(robot_interface);
    if (ret == 0) {
        std::cout << "Linear motion to circle starting point succeeded" << std::endl;
    } else {
        std::cout << "Linear motion to circle starting point failed" << std::endl;
    }

    // API call: Tool pose remains unchanged relative to the coordinate system of the arc path point
    robot_interface->getMotionControl()->setCirclePathMode(0);

    // Calculate the midpoint of the other half of the arc
    auto result =
        cli->getMath()->calculateCircleFourthPoint(pose1, pose2, pose3, 1);

    if (std::get<1>(result) == 0) {
        std::cout << "Failed to calculate midpoint of the other half of the arc, cannot complete circular motion" << std::endl;
    } else {
        auto pose4 = std::get<0>(result);
        // Perform circular motion for 3 rounds
        for (int i = 0; i < 3; i++) {
            // API call: Arc motion
            robot_interface->getMotionControl()->moveCircle(pose2, pose3, 1.2,
                                                            0.25, 0, 0);

            // Blocking
            ret = waitArrival(robot_interface);
            if (ret == -1) {
                std::cout << "Circular motion failed" << std::endl;
            }

            // Perform the other half arc motion
            robot_interface->getMotionControl()->moveCircle(pose4, pose1, 1.2,
                                                            0.25, 0, 0);

            // Blocking
            ret = waitArrival(robot_interface);
            if (ret == 0) {
                std::cout << "Circular motion succeeded" << std::endl;
            } else {
                std::cout << "Circular motion failed" << std::endl;
            }
        }
    }
}

// Circular motion
void exampleMoveC2(RpcClientPtr cli)
{
    // API call: Get the robot's name
    auto robot_name = cli->getRobotNames().front();

    auto robot_interface = cli->getRobotInterface(robot_name);

    // API call: Set the speed fraction for robot arm movement
    robot_interface->getMotionControl()->setSpeedFraction(0.3);

    // API call: Set the tool center point
    std::vector<double> tcp_offset(6, 0.0);
    robot_interface->getRobotConfig()->setTcpOffset(tcp_offset);

    // Joint angles, unit: radians
    std::vector<double> joint_angle = { 0.0,
                                        -15.0 * (M_PI / 180),
                                        100.0 * (M_PI / 180),
                                        25.0 * (M_PI / 180),
                                        90.0 * (M_PI / 180),
                                        0.0 };
    // API call: Move joints to initial position
    robot_interface->getMotionControl()->moveJoint(
        joint_angle, 80 * (M_PI / 180), 60 * (M_PI / 180), 0, 0);

    // Blocking
    int ret = waitArrival(robot_interface);
    if (ret == 0) {
        std::cout << "Joint moved to initial position successfully" << std::endl;
    } else {
        std::cout << "Joint failed to move to initial position" << std::endl;
    }

    // Pose
    std::vector<double> pose1 = {
        0.440164, -0.119, 0.2976, 2.456512, 0, 1.5708
    };

    std::vector<double> pose2 = { 0.440164, -0.00249391, 0.398658,
                                  2.45651,  0,           1.5708 };

    std::vector<double> pose3 = { 0.440164, 0.166256, 0.297599,
                                  2.45651,  0,        1.5708 };

    // API call: Linear motion to the starting point of the circle—pose1
    robot_interface->getMotionControl()->moveLine(pose1, 1.2, 0.25, 0.025, 0);

    // Blocking
    ret = waitArrival(robot_interface);
    if (ret == 0) {
        std::cout << "Linear motion to circle starting point succeeded" << std::endl;
    } else {
        std::cout << "Linear motion to circle starting point failed" << std::endl;
    }

    // API call: Tool pose remains unchanged relative to the coordinate system of the arc path point
    robot_interface->getMotionControl()->setCirclePathMode(0);

    // Calculate the midpoint of the other half of the arc
    auto result =
        cli->getMath()->calculateCircleFourthPoint(pose1, pose2, pose3, 1);
    if (std::get<1>(result) == 0) {
        std::cout << "Failed to calculate midpoint of the other half of the arc, cannot complete circular motion" << std::endl;
    } else {
        auto pose4 = std::get<0>(result);
        // Perform circular motion for 3 rounds
        for (int i = 0; i < 3; i++) {
            CircleParameters param1;
            param1.pose_via = pose2;
            param1.pose_to = pose3;
            param1.a = 1.2;
            param1.v = 0.25;
            param1.blend_radius = 0.0;
            // API call: Arc motion
            robot_interface->getMotionControl()->moveCircle2(param1);

            // Blocking
            ret = waitArrival(robot_interface);
            if (ret == -1) {
                std::cout << "Circular motion failed" << std::endl;
            }

            // Perform the other half arc motion
            CircleParameters param2;
            param2.pose_via = pose4;
            param2.pose_to = pose1;
            param2.a = 1.2;
            param2.v = 0.25;
            param2.blend_radius = 0.0;
            robot_interface->getMotionControl()->moveCircle2(param2);

            // Blocking
            waitArrival(cli->getRobotInterface(robot_name));
            if (ret == 0) {
                std::cout << "Circular motion succeeded" << std::endl;
            } else {
                std::cout << "Circular motion failed" << std::endl;
            }
        }
    }
}

void exampleMoveC3(RpcClientPtr cli)
{
    // API call: Get the robot's name
    auto robot_name = cli->getRobotNames().front();

    auto robot_interface = cli->getRobotInterface(robot_name);

    // API call: Set the speed fraction for robot arm movement
    robot_interface->getMotionControl()->setSpeedFraction(1.0);

    // API call: Set the tool center point
    std::vector<double> tcp_offset(6, 0.0);
    robot_interface->getRobotConfig()->setTcpOffset(tcp_offset);

    // API call: Start the planner
    cli->getRuntimeMachine()->start();
    auto cur_plan_context = cli->getRuntimeMachine()->getPlanContext();
    // API call: Get new thread id
    auto task_id = cli->getRuntimeMachine()->newTask();
    cli->getRuntimeMachine()->setPlanContext(
        task_id, std::get<1>(cur_plan_context), std::get<2>(cur_plan_context));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Joint angles, unit: radians
    std::vector<double> joint_angle = { 0.0,
                                        -15.0 * (M_PI / 180),
                                        100.0 * (M_PI / 180),
                                        25.0 * (M_PI / 180),
                                        90.0 * (M_PI / 180),
                                        0.0 };
    // API call: Move joints to initial position
    robot_interface->getMotionControl()->moveJoint(
        joint_angle, 80 * (M_PI / 180), 60 * (M_PI / 180), 0, 0);

    // Blocking
    int ret = waitArrival(robot_interface);
    if (ret == 0) {
        std::cout << "Joint moved to initial position successfully" << std::endl;
    } else {
        std::cout << "Joint failed to move to initial position" << std::endl;
    }

    // Pose
    std::vector<double> pose1 = {
        0.440164, -0.119, 0.2976, 2.456512, 0, 1.5708
    };

    std::vector<double> pose2 = { 0.440164, -0.00249391, 0.398658,
                                  2.45651,  0,           1.5708 };

    std::vector<double> pose3 = { 0.440164, 0.166256, 0.297599,
                                  2.45651,  0,        1.5708 };

    // API call: Linear motion to the starting point of the circle—pose1
    robot_interface->getMotionControl()->moveLine(pose1, 1.2, 0.25, 0.025, 0);

    // Blocking
    ret = waitArrival(robot_interface);
    if (ret == 0) {
        std::cout << "Linear motion to circle starting point succeeded" << std::endl;
    } else {
        std::cout << "Linear motion to circle starting point failed" << std::endl;
    }

    // API call: Tool pose remains unchanged relative to the coordinate system of the arc path point
    robot_interface->getMotionControl()->setCirclePathMode(0);

    // Calculate the midpoint of the other half of the arc
    auto result =
        cli->getMath()->calculateCircleFourthPoint(pose1, pose2, pose3, 1);

    // Continuous during circular motion
    if (std::get<1>(result) == 0) {
        std::cout << "Failed to calculate midpoint of the other half of the arc, cannot complete circular motion" << std::endl;
    } else {
        auto pose4 = std::get<0>(result);
        // Perform circular motion for 3 rounds
        for (int i = 0; i < 3; i++) {
            // API call: Arc motion
            robot_interface->getMotionControl()->moveCircle(pose2, pose3, 1.2,
                                                            0.25, 0.025, 0);

            // Perform the other half arc motion
            robot_interface->getMotionControl()->moveCircle(pose4, pose1, 1.2,
                                                            0.25, 0.025, 0);
        }
    }

    // Blocking
    ret = waitArrival(robot_interface);
    if (ret == 0) {
        std::cout << "Circular motion succeeded" << std::endl;
    } else {
        std::cout << "Circular motion failed" << std::endl;
    }
    // API call: Stop the planner
    cli->getRuntimeMachine()->stop();
    // API call: Delete thread
    cli->getRuntimeMachine()->deleteTask(task_id);
}

/**
 * Function: Robot arm arc motion and circular motion
 * Steps:
 * Step 1: Set RPC timeout, connect to RPC service, robot login
 * Step 2: Arc motion exampleMoveArc—implemented with moveCircle API
 * Step 3: Arc motion exampleMoveArc2—implemented with moveCircle2 API
 * Step 4: Circular motion exampleMoveC—implemented with moveCircle API
 * Step 5: Circular motion exampleMoveC2—implemented with moveCircle2 API
 * Step 6: RPC logout, disconnect
 */

#define LOCAL_IP "127.0.0.1"

int main(int argc, char **argv)
{
#ifdef WIN32
    // Set Windows console output code page to UTF-8
    SetConsoleOutputCP(CP_UTF8);
#endif

    auto rpc_cli = std::make_shared<RpcClient>();
    // API call: Set RPC timeout
    rpc_cli->setRequestTimeout(1000);
    // API call: Connect to RPC service
    rpc_cli->connect(LOCAL_IP, 30004);
    // API call: Login
    rpc_cli->login("aubo", "123456");

    // Arc motion
    exampleMoveArc(rpc_cli);

    // Arc motion
    exampleMoveArc2(rpc_cli);

    // Circular motion
    exampleMoveC(rpc_cli);

    // Circular motion
    exampleMoveC2(rpc_cli);

    // Circular motion
    exampleMoveC3(rpc_cli);

    // API call: Logout
    rpc_cli->logout();
    // API call: Disconnect
    rpc_cli->disconnect();

    return 0;
}
