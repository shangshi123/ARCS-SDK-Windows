#include "aubo_sdk/rpc.h"
#ifdef WIN32
#include <windows.h>
#endif

using namespace arcs::common_interface;
using namespace arcs::aubo_sdk;

// Example 1: Dismiss popup via alarm interface
void examplePopupDismiss1(RpcClientPtr cli)
{
    // API call: Get the robot's name
    auto robot_name = cli->getRobotNames().front();

    auto robot_interface = cli->getRobotInterface(robot_name);

    // Dismiss all popups
    robot_interface->getTrace()->alarm(TraceLevel::INFO, 2, { "" });
}

// Example 2: Dismiss popup via IO input
// Note: To make IO effective, the robot arm must enter linkage mode
void examplePopupDismiss2(RpcClientPtr cli)
{
    // API call: Get the robot's name
    auto robot_name = cli->getRobotNames().front();
    auto robot = cli->getRobotInterface(robot_name);

    // API call: Set the action of DI00 to dismiss popup.
    StandardInputAction input_action = StandardInputAction::PopupDismiss;
    robot->getIoControl()->setStandardDigitalInputAction(0, input_action);

    std::cout << "Note: When DI00 is high level (DI00 and 0V are shorted), the popup will be dismissed." << std::endl;
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

    // Example 1: Dismiss popup via alarm interface
    examplePopupDismiss1(rpc_cli);

    // Example 2: Dismiss popup via IO input
    // Note: To make IO effective, the robot arm must enter linkage mode
    // examplePopupDismiss2(rpc_cli);

    // API call: Logout
    rpc_cli->logout();
    // API call: Disconnect
    rpc_cli->disconnect();

    return 0;
}
