#include "aubo_sdk/rpc.h"
#ifdef WIN32
#include <windows.h>
#endif

using namespace arcs::common_interface;
using namespace arcs::aubo_sdk;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Blocking function: The program continues only after the robot reaches the target waypoint
int waitArrival(RobotInterfacePtr impl)
{
    const int max_retry_count = 5;
    int cnt = 0;

    // API call: Get the current motion command ID
    int exec_id = impl->getMotionControl()->getExecId();

    // Wait for the robot to start moving
    while (exec_id == -1) {
        if (cnt++ > max_retry_count) {
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        exec_id = impl->getMotionControl()->getExecId();
    }

    // Wait for the robot to finish the action
    while (impl->getMotionControl()->getExecId() != -1) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return 0;
}

// Example 1: TCP moves along the Z direction in the base/user coordinate system
void exampleMovelOffset1(RpcClientPtr cli)
{
    // API call: Get the robot's name
    auto robot_name = cli->getRobotNames().front();
    auto robot_interface = cli->getRobotInterface(robot_name);

    // API call: Set the tool center point (TCP offset relative to the flange center)
    std::vector<double> tcp_offset(6, 0.0);
    robot_interface->getRobotConfig()->setTcpOffset(tcp_offset);

    // API call: Get the current TCP pose in the base coordinate system
    std::vector<double> current_tcp_on_base(6, 0.0);
    current_tcp_on_base = robot_interface->getRobotState()->getTcpPose();

    // API call: Get the target TCP pose offset by 0.1m along the Z+ axis in the base/user coordinate system (relative to the base coordinate system)
    std::vector<double> target_tcp_on_base(6, 0.0);
    std::vector<double> path_offset = { 0, 0, 0.1, 0, 0, 0 };
    target_tcp_on_base =
        cli->getMath()->poseAdd(current_tcp_on_base, path_offset);

    // API call: Set the robot's speed fraction
    robot_interface->getMotionControl()->setSpeedFraction(0.75);

    // API call: Move linearly to the target position
    robot_interface->getMotionControl()->moveLine(target_tcp_on_base, 1.2, 0.25,
                                                  0.025, 0);
    // Blocking
    int ret = waitArrival(robot_interface);
    if (ret == 0) {
        std::cout << "Linear movement to target position succeeded!" << std::endl;
    } else {
        std::cout << "Linear movement to target position failed!" << std::endl;
    }
}

// Example 2: TCP moves along the Z direction in the tool coordinate system
void exampleMovelOffset2(RpcClientPtr cli)
{
    // API call: Get the robot's name
    auto robot_name = cli->getRobotNames().front();
    auto robot_interface = cli->getRobotInterface(robot_name);

    // API call: Set the tool center point (TCP offset relative to the flange center)
    std::vector<double> tcp_offset(6, 0.0);
    robot_interface->getRobotConfig()->setTcpOffset(tcp_offset);

    // API call: Get the current TCP pose in the base coordinate system
    std::vector<double> current_tcp_on_base(6, 0.0);
    current_tcp_on_base = robot_interface->getRobotState()->getTcpPose();

    // API call: Get the target TCP pose offset by 0.1m along the Z+ axis in the tool coordinate system (relative to the base coordinate system)
    std::vector<double> target_tcp_on_base(6, 0.0);
    std::vector<double> path_offset = { 0, 0, 0.1, 0, 0, 0 };
    target_tcp_on_base =
        cli->getMath()->poseTrans(current_tcp_on_base, path_offset);

    // API call: Set the robot's speed fraction
    robot_interface->getMotionControl()->setSpeedFraction(0.75);

    // API call: Move linearly to the target position
    robot_interface->getMotionControl()->moveLine(target_tcp_on_base, 1.2, 0.25,
                                                  0.025, 0);
    // Blocking
    int ret = waitArrival(robot_interface);
    if (ret == 0) {
        std::cout << "Linear movement to target position succeeded!" << std::endl;
    } else {
        std::cout << "Linear movement to target position failed!" << std::endl;
    }
}

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

    // Example 1: TCP moves along the Z direction in the base/user coordinate system
    exampleMovelOffset1(rpc_cli);

    // Example 2: TCP moves along the Z direction in the tool coordinate system
    exampleMovelOffset2(rpc_cli);

    // API call: Logout
    rpc_cli->logout();
    // API call: Disconnect
    rpc_cli->disconnect();

    return 0;
}
