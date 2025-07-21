#include "aubo_sdk/rpc.h"
#ifdef WIN32
#include <windows.h>
#endif

using namespace arcs::common_interface;
using namespace arcs::aubo_sdk;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Implement blocking: The program continues only after the robot arm reaches the target waypoint
int waitArrival(RobotInterfacePtr impl)
{
    const int max_retry_count = 20;
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

void examplePathOffsetWeave(RpcClientPtr cli)
{
    // API call: Start runtime.
    // Note: Weave motion requires runtime to be started, otherwise weaving will not take effect.
    cli->getRuntimeMachine()->start();

    // Joint angles, unit: radians
    std::vector<double> joint_angle = {
        0.0 * (M_PI / 180),  -15.0 * (M_PI / 180), 100.0 * (M_PI / 180),
        25.0 * (M_PI / 180), 90.0 * (M_PI / 180),  0.0 * (M_PI / 180)
    };
    // Pose
    std::vector<double> pose1 = { 0.551, -0.295, 0.261, -3.135, 0.0, 1.569 };
    std::vector<double> pose2 = { 0.551, 0.295, 0.261, -3.135, 0.0, 1.569 };

    // API call: Get the robot's name
    auto robot_name = cli->getRobotNames().front();

    auto robot_interface = cli->getRobotInterface(robot_name);

    // API call: Set the speed fraction of the robot arm
    robot_interface->getMotionControl()->setSpeedFraction(0.75);

    // API call: Set the tool center point (TCP offset relative to the flange center)
    std::vector<double> tcp_offset(6, 0.0);
    robot_interface->getRobotConfig()->setTcpOffset(tcp_offset);

    // API call: Move joints to the starting position
    robot_interface->getMotionControl()->moveJoint(
        joint_angle, 80 * (M_PI / 180), 60 * (M_PI / 180), 0, 0);

    // Blocking
    int ret = waitArrival(robot_interface);
    if (ret == 0) {
        std::cout << "Joint movement to starting position succeeded!" << std::endl;
    } else {
        std::cout << "Joint movement to starting position failed!" << std::endl;
    }

    // API call: Linear movement to position 1
    robot_interface->getMotionControl()->moveLine(pose1, 1.2, 0.25, 0.0, 0);

    // Blocking
    ret = waitArrival(robot_interface);
    if (ret == 0) {
        std::cout << "Linear movement to position 1 succeeded!" << std::endl;
    } else {
        std::cout << "Linear movement to position 1 failed!" << std::endl;
    }

    std::string params = "{\"type\":\"SINE\",\"step\":0.01,\"amplitude\": "
                         "[0.01,0.01],\"hold_distance\": "
                         "[0,0],\"hold_time\":[0,0], "
                         "\"angle\":0, \"direction\":0}";

    // API call: Start weaving
    robot_interface->getMotionControl()->weaveStart(params);

    // API call: Linear movement to position 2
    robot_interface->getMotionControl()->moveLine(pose2, 0.05, 0.01, 0.0, 0.0);

    // API call: Enable path offset
    // Note: pathOffsetEnable must be used together with pathOffsetDisable
    robot_interface->getMotionControl()->pathOffsetEnable();

    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    // API call: Set path offset
    for (int i = 0; i < 10; i++) {
        robot_interface->getMotionControl()->pathOffsetSet(
            { 0, 0, 0.15, 0, 0, 0 }, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        robot_interface->getMotionControl()->pathOffsetSet(
            { 0, 0, -0.15, 0, 0, 0 }, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // Blocking
    ret = waitArrival(robot_interface);
    if (ret == 0) {
        std::cout << "Weave motion to position 2 succeeded!" << std::endl;
    } else {
        std::cout << "Weave motion to position 2 failed!" << std::endl;
    }

    // API call: End weaving
    robot_interface->getMotionControl()->weaveEnd();

    // API call: Disable path offset
    robot_interface->getMotionControl()->pathOffsetDisable();
    // Add delay here to ensure pathOffsetDisable is executed
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Stop runtime after weave motion ends
    cli->getRuntimeMachine()->stop();
}

/**
 * Function: Robot arm weave centerline offset motion
 * Steps:
 * Step 1: Set RPC timeout, connect to RPC service, robot login
 * Step 2: Set motion speed fraction and tool center point
 * Step 3: Start runtime,
 *       first move joints to starting position,
 *       then linear movement to position 1,
 *       start weaving, move to position 2 with weave motion, enable and set path offset,
 *       after reaching position 2, end weaving, disable path offset, stop runtime
 * Step 4: RPC logout, disconnect
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

    // Weave centerline offset motion
    examplePathOffsetWeave(rpc_cli);

    // API call: Logout
    rpc_cli->logout();
    // API call: Disconnect
    rpc_cli->disconnect();

    return 0;
}
