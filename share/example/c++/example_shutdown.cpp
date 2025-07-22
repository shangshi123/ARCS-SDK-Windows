#include "aubo_sdk/rpc.h"
#ifdef WIN32
#include <windows.h>
#endif

using namespace arcs::aubo_sdk;

#define ip_local "127.0.0.1"

void exampleShutdown1(RpcClientPtr cli)
{
    // API call: Shutdown
    int ret = cli->shutdown();

    std::this_thread::sleep_for(std::chrono::seconds(2));

    std::cout << "ret = " << ret << std::endl;
}

void exampleShutdown2(RpcClientPtr cli)
{
    // API call: Get the name of the robot arm
    auto robot_name = cli->getRobotNames().front();

    // API call: Shutdown
    cli->getRobotInterface(robot_name)
        ->getRobotConfig()
        ->setHardwareCustomParameters("tp_shutdown = true");
}

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
    rpc_cli->connect(ip_local, 30004);
    // API call: Login
    rpc_cli->login("aubo", "123456");

    // Example 1: Shutdown via shutdown API
    exampleShutdown1(rpc_cli);

    // Example 2: Shutdown via setHardwareCustomParameters API
    //    exampleShutdown2(rpc_cli);

    // API call: Logout
    rpc_cli->logout();
    // API call: Disconnect
    rpc_cli->disconnect();

    return 0;
}
