#include "aubo_sdk/rpc.h"
#ifdef WIN32
#include <windows.h>
#endif

using namespace arcs::common_interface;
using namespace arcs::aubo_sdk;

// Wait for the robot to enter the target mode
void waitForRobotMode(RobotInterfacePtr robot_interface,
                      RobotModeType target_mode)
{
    // API call: Get the current mode of the robot
    auto current_mode = robot_interface->getRobotState()->getRobotModeType();

    while (current_mode != target_mode) {
        std::cout << "Current robot mode:" << current_mode << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        current_mode = robot_interface->getRobotState()->getRobotModeType();
    }
}

void exampleStartup(RpcClientPtr cli)
{
    // API call: Get the robot's name
    auto robot_name = cli->getRobotNames().front();

    auto robot_interface = cli->getRobotInterface(robot_name);

    // API call: Set payload
    double mass = 0.0;
    std::vector<double> cog(3, 0.0);
    std::vector<double> aom(3, 0.0);
    std::vector<double> inertia(6, 0.0);
    robot_interface->getRobotConfig()->setPayload(mass, cog, aom, inertia);

    // API call: Get the current mode of the robot
    auto robot_mode = robot_interface->getRobotState()->getRobotModeType();

    if (robot_mode == RobotModeType::Running) {
        std::cout << "Robot brake released, in running mode" << std::endl;

    } else {
        // API call: Robot initiates power-on request
        robot_interface->getRobotManage()->poweron();

        // Wait for the robot to enter idle mode
        waitForRobotMode(robot_interface, RobotModeType::Idle);

        std::cout << "Robot powered on successfully, current mode:"
                  << robot_interface->getRobotState()->getRobotModeType()
                  << std::endl;

        // API call: Robot initiates brake release request
        cli->getRobotInterface(robot_name)->getRobotManage()->startup();

        // Wait for the robot to enter running mode
        waitForRobotMode(robot_interface, RobotModeType::Running);

        std::cout << "Robot brake released successfully, current mode:"
                  << robot_interface->getRobotState()->getRobotModeType()
                  << std::endl;
    }
}

void examplePoweroff(RpcClientPtr cli)
{
    // API call: Get the robot's name
    auto robot_name = cli->getRobotNames().front();

    auto robot_interface = cli->getRobotInterface(robot_name);

    // API call: Robot power off
    robot_interface->getRobotManage()->poweroff();

    // Wait for the robot to enter power-off mode
    waitForRobotMode(robot_interface, RobotModeType::PowerOff);

    std::cout << "Robot powered off successfully, current mode:"
              << robot_interface->getRobotState()->getRobotModeType()
              << std::endl;
}

/**
 * Function: Robot power on and power off
 * Steps:
 * Step 1: Set RPC timeout, connect to RPC service, login
 * Step 2: Set payload, power on and release brake
 * Step 3: Robot power off
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

    // Robot power on
    exampleStartup(rpc_cli);

    // Robot power off
    //    examplePoweroff(rpc_cli);

    // API call: Logout
    rpc_cli->logout();
    // API call: Disconnect
    rpc_cli->disconnect();

    return 0;
}
