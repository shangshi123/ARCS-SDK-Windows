#include "aubo_sdk/rpc.h"
#ifdef WIN32
#include <windows.h>
#endif

using namespace arcs::common_interface;
using namespace arcs::aubo_sdk;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void exampleRunProgram(RpcClientPtr cli)
{
    // Teach pendant project file name
    std::string program_name = "test";
    // API call: Load the .pro project programmed by the teach pendant
    // The project file needs to be placed in the /root/arcs_ws/program directory
    // Only enter the file name, no need to add a suffix
    cli->getRuntimeMachine()->loadProgram(program_name);
    // API call: Run the project
    cli->getRuntimeMachine()->runProgram();

    RuntimeState program_status;
    while (1) {
        // API call: Get the project running status
        program_status = cli->getRuntimeMachine()->getRuntimeState();
        std::cout << "Current project running status: " << program_status << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
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

    // Run teach pendant project
    exampleRunProgram(rpc_cli);

    // API call: Logout
    rpc_cli->logout();
    // API call: Disconnect
    rpc_cli->disconnect();

    return 0;
}
