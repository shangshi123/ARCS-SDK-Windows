#include "aubo_sdk/rpc.h"

using namespace arcs::common_interface;
using namespace arcs::aubo_sdk;

#define LOCAL_IP "127.0.0.1"

void example_read_log(RpcClientPtr cli) {
  // API call: Reset the format of the log system
  cli->setLogHandler([](int level, const char *filename, int line,
                        const std::string &message) {
    std::cout << "level: " << level << std::endl;
    std::cout << "filename: " << filename << std::endl;
    std::cout << "line: " << line << std::endl;
    std::cout << "message: " << message << std::endl;
  });
}
/**
 * Function: Customize the format of the log
 * Steps:
 * Step 1: Customize the format of the log.
 * Note: Call the setLogHandler function before connecting to the server.
 * Otherwise, the customized log format will not take effect.
 * Step 2: Connect to the RPC service and log in to the robot arm
 * Step 3: Log out and disconnect from the RPC service
 */
int main() {
  // RPC client object
  auto rpc_cli = std::make_shared<RpcClient>();
  // Customize the format of the log
  example_read_log(rpc_cli);
  // API call: Set RPC timeout
  rpc_cli->setRequestTimeout(1000);
  // API call: Connect to the RPC service
  rpc_cli->connect(LOCAL_IP, 30004);
  // API call: Log in
  rpc_cli->login("aubo", "123456");

  // API call: Log out
  rpc_cli->logout();
  // API call: Disconnect
  rpc_cli->disconnect();
  return 0;
}
