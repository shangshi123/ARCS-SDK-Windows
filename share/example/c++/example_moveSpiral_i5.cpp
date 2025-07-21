#include "aubo_sdk/rpc.h"
#ifdef WIN32
#include <windows.h>
#endif

using namespace arcs::common_interface;
using namespace arcs::aubo_sdk;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Implement blocking functionality: The program continues execution only after the robot arm reaches the target waypoint
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

void exampleMoveSpiral1(RpcClientPtr cli)
{
    // Joint angles, unit: radians
    std::vector<double> start_q = {
        -0.04723656192459002, -0.3147961077805709, 2.122706013891513,
        1.388465989105001,    1.59432305862766,    -0.04096820515125211
    };

    // Pose
    std::vector<double> start_p = { 0.4992287020163843,    -0.1430213015569043,
                                    0.1929798566172833,    2.619351290960044,
                                    3.577791911729708e-05, 1.570799472227775 };

    // API call: Get the robot's name
    auto robot_name = cli->getRobotNames().front();

    auto robot_interface = cli->getRobotInterface(robot_name);

    // API call: Set the speed fraction of the robot arm
    robot_interface->getMotionControl()->setSpeedFraction(0.3);

    // API call: Set the tool center point (TCP offset relative to the flange center)
    std::vector<double> tcp_offset(6, 0.0);
    robot_interface->getRobotConfig()->setTcpOffset(tcp_offset);

    // API call: Move joints to the starting position
    robot_interface->getMotionControl()->moveJoint(start_q, 80 * (M_PI / 180),
                                                   60 * (M_PI / 180), 0, 0);
    // Blocking
    int ret = waitArrival(robot_interface);
    if (ret == 0) {
        std::cout << "Joint movement to starting position succeeded!" << std::endl;
    } else {
        std::cout << "Joint movement to starting position failed!" << std::endl;
    }

    // Spiral motion center position
    std::vector<double> center_position = { 0.4992155149838216,
                                            -0.08141538203368692,
                                            0.1929808028919965 };
    std::vector<double> center_pose(6, 0.);

    // Spiral motion based on the XY plane of the base coordinate system
    // Center position
    for (int i = 0; i < 3; i++) {
        center_pose[i] = center_position[i];
    }
    // Center orientation
    for (int i = 3; i < 6; i++) {
        center_pose[i] = 0.;
    }

    // Set spiral motion parameters
    SpiralParameters param;
    param.spiral = 0.005; // Step size of radius increase per turn in spiral motion
    param.helix = 0.005;    // Step size in the z-axis direction in spiral motion
    param.angle = 6 * M_PI; // Spiral motion angle range
    param.plane = 0;        // Based on XY plane
    param.frame = center_pose;

    robot_interface->getMotionControl()->moveSpiral(param, 0, 0.25, 1.2, 0);
    ret = waitArrival(robot_interface);
    if (ret == 0) {
        std::cout << "Spiral motion succeeded" << std::endl;
    } else {
        std::cout << "Spiral motion failed" << std::endl;
    }
}

void exampleMoveSpiral2(RpcClientPtr cli)
{
    // Joint angles, unit: radians
    std::vector<double> start_q = {
        -0.04723656192459002, -0.3147961077805709, 2.122706013891513,
        1.388465989105001,    1.59432305862766,    -0.04096820515125211
    };

    // Pose
    std::vector<double> start_p = { 0.4992287020163843,    -0.1430213015569043,
                                    0.1929798566172833,    2.619351290960044,
                                    3.577791911729708e-05, 1.570799472227775 };

    // API call: Get the robot's name
    auto robot_name = cli->getRobotNames().front();

    auto robot_interface = cli->getRobotInterface(robot_name);

    // API call: Set the speed fraction of the robot arm
    robot_interface->getMotionControl()->setSpeedFraction(0.3);

    // API call: Set the tool center point (TCP offset relative to the flange center)
    std::vector<double> tcp_offset(6, 0.0);
    robot_interface->getRobotConfig()->setTcpOffset(tcp_offset);

    // API call: Move joints to the starting position
    robot_interface->getMotionControl()->moveJoint(start_q, 80 * (M_PI / 180),
                                                   60 * (M_PI / 180), 0, 0);
    // Blocking
    int ret = waitArrival(robot_interface);
    if (ret == 0) {
        std::cout << "Joint movement to starting position succeeded!" << std::endl;
    } else {
        std::cout << "Joint movement to starting position failed!" << std::endl;
    }

    // Spiral motion center position
    std::vector<double> center_position = { 0.4992155149838216,
                                            -0.08141538203368692,
                                            0.1929808028919965 };
    std::vector<double> center_pose(6, 0.);

    // Spiral motion based on the XY plane of the TCP coordinate system
    // Center position
    for (int i = 0; i < 3; i++) {
        center_pose[i] = center_position[i];
    }
    // Center orientation
    auto tcp_pose = robot_interface->getRobotState()->getTcpPose();
    for (int i = 3; i < 6; i++) {
        center_pose[i] = tcp_pose[i];
    }

    // Set spiral motion parameters
    SpiralParameters param;
    param.spiral = 0.005; // Step size of radius increase per turn in spiral motion
    param.helix = 0.005;     // Step size in the z-axis direction in spiral motion
    param.angle = 10 * M_PI; // Spiral motion angle range
    param.plane = 0;         // Based on XY plane
    param.frame = center_pose;

    robot_interface->getMotionControl()->moveSpiral(param, 0, 0.25, 1.2, 0);
    ret = waitArrival(robot_interface);
    if (ret == 0) {
        std::cout << "Spiral motion succeeded" << std::endl;
    } else {
        std::cout << "Spiral motion failed" << std::endl;
    }
}

/**
 * Function: Robot arm spiral motion
 * Steps:
 * Step 1: Set RPC timeout, connect to RPC service, robot login
 * Step 2: Set motion speed fraction and tool center point
 * Step 3: First move joints to the starting position, then perform spiral motion
 * Step 4: RPC logout and disconnect
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

    // Spiral motion
    exampleMoveSpiral1(rpc_cli);

    // Spiral motion
    // exampleMoveSpiral2(rpc_cli);

    // API call: Logout
    rpc_cli->logout();
    // API call: Disconnect
    rpc_cli->disconnect();

    return 0;
}
