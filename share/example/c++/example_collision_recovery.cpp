#include "aubo_sdk/rpc.h"
#include <aubo/error_stack/error_stack.h>
#ifdef WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif
#include <math.h>
using namespace arcs::common_interface;
using namespace arcs::aubo_sdk;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Calculate the absolute difference between current joint angles and target joint angles
double distance(const std::vector<double> &a, const std::vector<double> &b)
{
    double res = 0.;
    if (a.size() != b.size()) {
        return -1;
    }

    for (int i = 0; i < (int)a.size(); i++) {
        res += (a[i] - b[i]) * (a[i] - b[i]);
    }
    return sqrt(res);
}

// Blocking function: The program continues only when the robot arm reaches the target waypoint
void waitArrival(RobotInterfacePtr impl, std::vector<double> target)
{
    while (1) {
        // API call: Get current joint angles
        std::vector cur = impl->getRobotState()->getJointPositions();
        // Break the loop when the absolute difference between current and target joint angles is less than 0.0001
        double dis = distance(cur, target);
        if (dis < 0.0001) {
            break;
        }
        printf("Current joint angles:%f,%f,%f,%f,%f,%f\n", cur.at(0), cur.at(1),
               cur.at(2), cur.at(3), cur.at(4), cur.at(5));
        printf("Target joint angles:%f,%f,%f,%f,%f,%f\n", target.at(0), target.at(1),
               target.at(2), target.at(3), target.at(4), target.at(5));
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

// Joint movement recovery
void collision_recovery_movej(RpcClientPtr cli)

{
    // API call: Get the robot's name
    auto name = cli->getRobotNames().front();
    auto impl = cli->getRobotInterface(name);

    // API call: Get the position to recover to after collision
    auto joint = impl->getMotionControl()->getPauseJointPositions();

    // Print the position to recover to
    printf("Position to recover to:%f,%f,%f,%f,%f,%f\n", joint.at(0), joint.at(1),
           joint.at(2), joint.at(3), joint.at(4), joint.at(5));

    // API call: Simulate teaching movement after collision
    impl->getMotionControl()->resumeSpeedLine({ 0, 0, -0.05, 0, 0, 0 }, 1.2,
                                              100);
#ifdef WIN32
    Sleep(1000 * 1);
#else
    usleep(1000 * 1000 * 1);
#endif
    // API call: Stop teaching movement
    impl->getMotionControl()->resumeStopLine(10, 10);
#ifdef WIN32
    Sleep(1000 * 1);
#else
    usleep(1000 * 1000 * 1);
#endif

    // API call: Move joints to the pause point
    impl->getMotionControl()->resumeMoveJoint(joint, 1, 1, 0);
    // Blocking
    waitArrival(impl, joint);

    // API call: Resume planner operation
    cli->getRuntimeMachine()->resume();
}

// Linear movement recovery
void collision_recovery_movel(RpcClientPtr cli)

{
    // API call: Get the robot's name
    auto name = cli->getRobotNames().front();
    auto impl = cli->getRobotInterface(name);

    // API call: Set tool center point (TCP offset relative to flange center)
    std::vector<double> tcp_offset(6, 0.0);
    impl->getRobotConfig()->setTcpOffset(tcp_offset);

    // API call: Get the position to recover to after collision
    auto joint = impl->getMotionControl()->getPauseJointPositions();

    // Print the joint angle position to recover to
    printf("Joint angle position to recover to:%f,%f,%f,%f,%f,%f\n", joint.at(0), joint.at(1),
           joint.at(2), joint.at(3), joint.at(4), joint.at(5));

    // API call: Simulate teaching movement after collision
    impl->getMotionControl()->resumeSpeedLine({ 0, 0, -0.05, 0, 0, 0 }, 1.2,
                                              100);
#ifdef WIN32
    Sleep(1000 * 1);
#else
    usleep(1000 * 1000 * 1);
#endif
    // API call: Stop teaching movement
    impl->getMotionControl()->resumeStopLine(10, 10);
#ifdef WIN32
    Sleep(1000 * 1);
#else
    usleep(1000 * 1000 * 1);
#endif

    // API call: Use forward kinematics to get the pose of the pause point
    auto result = impl->getRobotAlgorithm()->forwardKinematics(joint);

    if (0 == std::get<1>(result)) {
        // API call: Linear movement to the pause point
        impl->getMotionControl()->resumeMoveLine(std::get<0>(result), 1.2, 0.25,
                                                 0.0);
    }
    // Blocking
    waitArrival(impl, joint);
    std::cout << "Linear movement to pause point" << std::endl;

    // API call: Resume planner operation
    cli->getRuntimeMachine()->resume();
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

    // Joint movement recovery
    collision_recovery_movej(rpc_cli);

    // Linear movement recovery
    //    collision_recovery_movel(rpc_cli);

    // API call: Logout
    rpc_cli->logout();
    // API call: Disconnect
    rpc_cli->disconnect();

    return 0;
}
