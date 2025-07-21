#include "aubo_sdk/rpc.h"
#ifdef WIN32
#include <windows.h>
#endif

using namespace arcs::common_interface;
using namespace arcs::aubo_sdk;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Implement blocking functionality: The program continues after the robot arm reaches the target waypoint
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

    // Wait for the robot arm to finish moving
    while (impl->getMotionControl()->getExecId() != -1) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return 0;
}

void exampleMovej(RpcClientPtr cli)
{
    // Joint angles, unit: radians
    std::vector<double> joint_angle1 = {
        0.0 * (M_PI / 180),  -15.0 * (M_PI / 180), 100.0 * (M_PI / 180),
        25.0 * (M_PI / 180), 90.0 * (M_PI / 180),  0.0 * (M_PI / 180)
    };

    std::vector<double> joint_angle2 = {
        35.92 * (M_PI / 180),  -11.28 * (M_PI / 180), 59.96 * (M_PI / 180),
        -18.76 * (M_PI / 180), 90.0 * (M_PI / 180),   35.92 * (M_PI / 180)
    };

    std::vector<double> joint_angle3 = {
        41.04 * (M_PI / 180), -7.65 * (M_PI / 180), 98.80 * (M_PI / 180),
        16.44 * (M_PI / 180), 90.0 * (M_PI / 180),  11.64 * (M_PI / 180)
    };

    // API call: Get the robot's name
    auto robot_name = cli->getRobotNames().front();

    auto robot_interface = cli->getRobotInterface(robot_name);

    // API call: Set the speed fraction of the robot arm
    robot_interface->getMotionControl()->setSpeedFraction(0.3);

    // API call: Joint movement
    robot_interface->getMotionControl()->moveJoint(
        joint_angle1, 80 * (M_PI / 180), 60 * (M_PI / 180), 0, 0);
    // Blocking
    int ret = waitArrival(robot_interface);
    if (ret == 0) {
        std::cout << "Joint moved to waypoint 1 successfully" << std::endl;
    } else {
        std::cout << "Joint failed to move to waypoint 1" << std::endl;
    }

    // API call: Joint movement
    robot_interface->getMotionControl()->moveJoint(
        joint_angle2, 80 * (M_PI / 180), 60 * (M_PI / 180), 0, 0);
    // Blocking
    ret = waitArrival(robot_interface);
    if (ret == 0) {
        std::cout << "Joint moved to waypoint 2 successfully" << std::endl;
    } else {
        std::cout << "Joint failed to move to waypoint 2" << std::endl;
    }

    // API call: Joint movement
    robot_interface->getMotionControl()->moveJoint(
        joint_angle3, 80 * (M_PI / 180), 60 * (M_PI / 180), 0, 0);
    // Blocking
    ret = waitArrival(robot_interface);
    if (ret == 0) {
        std::cout << "Joint moved to waypoint 3 successfully" << std::endl;
    } else {
        std::cout << "Joint failed to move to waypoint 3" << std::endl;
    }
}

/**
 * Function: Robot arm joint movement
 * Steps:
 * Step 1: Set RPC timeout, connect to RPC service, robot login
 * Step 2: Set motion speed fraction, pass through 3 waypoints by joint movement
 * Step 3: RPC logout, disconnect
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

    // Joint movement
    exampleMovej(rpc_cli);

    // API call: Logout
    rpc_cli->logout();
    // API call: Disconnect
    rpc_cli->disconnect();

    return 0;
}
