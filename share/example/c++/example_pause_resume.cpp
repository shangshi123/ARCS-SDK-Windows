#include <iostream>
#include <thread>
#include <chrono>
#include "aubo_sdk/rpc.h"

#ifdef WIN32
#include <windows.h>
#endif

using namespace arcs::common_interface;
using namespace arcs::aubo_sdk;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void robotMotionControl(RpcClientPtr cli)
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

    std::vector<double> joint_angle4 = {
        41.04 * (M_PI / 180), -27.03 * (M_PI / 180), 115.35 * (M_PI / 180),
        52.37 * (M_PI / 180), 90.0 * (M_PI / 180),   11.64 * (M_PI / 180)
    };

    // API call: Get the robot's name
    auto robot_name = cli->getRobotNames().front();

    auto robot_interface = cli->getRobotInterface(robot_name);

    // API call: Start the runtime machine
    cli->getRuntimeMachine()->start();

    std::this_thread::sleep_for(std::chrono::seconds(1));

    // API call: Set the robot arm's speed fraction
    robot_interface->getMotionControl()->setSpeedFraction(1);

    // API call: Get the runtime status
    auto runtime_status = cli->getRuntimeMachine()->getRuntimeState();

    while (runtime_status != RuntimeState::Stopped) {
        // API call: Joint movement
        robot_interface->getMotionControl()->moveJoint(
            joint_angle1, 80 * (M_PI / 180), 60 * (M_PI / 180), 0, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        // API call: Joint movement
        robot_interface->getMotionControl()->moveJoint(
            joint_angle2, 80 * (M_PI / 180), 60 * (M_PI / 180), 0, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        // API call: Joint movement
        robot_interface->getMotionControl()->moveJoint(
            joint_angle3, 80 * (M_PI / 180), 60 * (M_PI / 180), 0, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        // API call: Joint movement
        robot_interface->getMotionControl()->moveJoint(
            joint_angle4, 80 * (M_PI / 180), 60 * (M_PI / 180), 0, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        // API call: Get the runtime machine status
        runtime_status = cli->getRuntimeMachine()->getRuntimeState();
    }

    // API call: Stop joint movement
    robot_interface->getMotionControl()->stopJoint(30);
}

// Control pause/resume/stop
void controlOperations(RpcClientPtr rpc_cli, bool &exit_flag)
{
    std::string input;
    while (!exit_flag) {
        std::cout
            << "Please enter command (p/r/s): p means pause motion, r means resume motion, s means stop motion"
            << std::endl;

        std::cin >> input;

        if (input == "p") {
            // API call: Pause runtime
            rpc_cli->getRuntimeMachine()->pause();
            std::cout << "Runtime paused" << std::endl;
        } else if (input == "r") {
            // API call: Resume runtime
            rpc_cli->getRuntimeMachine()->resume();
            std::cout << "Runtime resumed" << std::endl;
        } else if (input == "s") {
            // API call: Stop runtime
            // Note: abort cannot stop robot arm motion
            rpc_cli->getRuntimeMachine()->abort();
            exit_flag = true; // Set exit flag to end thread
            std::cout << "Runtime stopped" << std::endl;
        } else {
            std::cout << "Invalid command, please re-enter" << std::endl;
        }
    }
}

#define LOCAL_IP "127.0.0.1"

int main(int argc, char **argv)
{
#ifdef WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif

    auto rpc_cli = std::make_shared<RpcClient>();
    // API call: Set RPC timeout
    rpc_cli->setRequestTimeout(1000);
    // API call: Connect to RPC service
    rpc_cli->connect(LOCAL_IP, 30004);
    // API call: Login
    rpc_cli->login("aubo", "123456");

    // Exit thread flag
    bool exit_flag = false;

    // Create threads for controlling robot motion and operations
    std::thread motion_thread(robotMotionControl, rpc_cli);
    std::thread control_thread(controlOperations, rpc_cli, std::ref(exit_flag));

    // Wait for threads to finish
    motion_thread.join();
    control_thread.join();

    // API call: Logout
    rpc_cli->logout();
    // API call: Disconnect
    rpc_cli->disconnect();

    return 0;
}
