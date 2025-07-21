#include "aubo_sdk/rpc.h"
#ifdef WIN32
#include <windows.h>
#endif

using namespace arcs::aubo_sdk;

#define LOCAL_IP "127.0.0.1"

int main(int argc, char **argv)
{
#ifdef WIN32
    // Set Windows console output code page to UTF-8
    SetConsoleOutputCP(CP_UTF8);
#endif
    // RPC client object
    auto rpc_cli = std::make_shared<arcs::aubo_sdk::RpcClient>();
    // API call: set RPC timeout
    rpc_cli->setRequestTimeout(1000);
    // API call: connect to RPC service
    rpc_cli->connect(LOCAL_IP, 30004);
    // API call: login
    rpc_cli->login("aubo", "123456");
    // API call: get robot name
    auto robot_name = rpc_cli->getRobotNames().front();
    auto impl = rpc_cli->getRobotInterface(robot_name);

    // API call: enable freedrive mode
    impl->getRobotManage()->freedrive(true);

    // Wait to enter freedrive mode
    int i = 0;
    while (!impl->getRobotManage()->isFreedriveEnabled()) {
        if (i++ > 5) {
            std::cerr << "Failed to enable freedrive mode" << std::endl;
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    std::cout << "Freedrive mode enabled successfully" << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(25));

    // API call: exit freedrive mode
    impl->getRobotManage()->freedrive(false);

    // Wait to exit freedrive mode
    while (impl->getRobotManage()->isFreedriveEnabled()) {
        if (i++ > 5) {
            std::cerr << "Failed to disable freedrive mode" << std::endl;
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cout << "Freedrive mode disabled successfully" << std::endl;

    // API call: logout
    rpc_cli->logout();
    // API call: disconnect
    rpc_cli->disconnect();

    return 0;
}
