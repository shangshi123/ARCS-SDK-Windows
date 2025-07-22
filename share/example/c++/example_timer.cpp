#include "aubo_sdk/rpc.h"
#include <math.h>
#include <thread>

using namespace arcs::common_interface;
using namespace arcs::aubo_sdk;

#define LOCAL_IP "127.0.0.1"

void example1(RpcClientPtr rpc_cli) {
  // API call: Timer timer1 starts timing
  rpc_cli->getRuntimeMachine()->timerStart("timer1");
  std::this_thread::sleep_for(std::chrono::milliseconds(7500));
  // API call: Timer timer1 stops timing
  rpc_cli->getRuntimeMachine()->timerStop("timer1");
  // API call: Get the duration timed by timer timer1
  std::cout << rpc_cli->getRuntimeMachine()->getTimer("timer1") << std::endl;

  // API call: Timer timer1 starts timing
  rpc_cli->getRuntimeMachine()->timerStart("timer1");
  std::this_thread::sleep_for(std::chrono::milliseconds(7500));
  // API call: Timer timer1 stops timing
  rpc_cli->getRuntimeMachine()->timerStop("timer1");
  // API call: Get the duration timed by timer timer1
  std::cout << rpc_cli->getRuntimeMachine()->getTimer("timer1") << std::endl;

  // API call: Reset timer timer1
  rpc_cli->getRuntimeMachine()->timerReset("timer1");
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  // API call: Get the duration timed by timer timer1
  std::cout << rpc_cli->getRuntimeMachine()->getTimer("timer1") << std::endl;
  // API call: Delete timer timer1
  rpc_cli->getRuntimeMachine()->timerDelete("timer1");
}

void example2(RpcClientPtr rpc_cli) {
  // API call: Timer timer1 starts timing
  rpc_cli->getRuntimeMachine()->timerStart("timer1");
  std::this_thread::sleep_for(std::chrono::milliseconds(7500));
  // API call: Get the duration timed by timer timer1
  std::cout << rpc_cli->getRuntimeMachine()->getTimer("timer1") << std::endl;
  // API call: Reset timer timer1
  rpc_cli->getRuntimeMachine()->timerReset("timer1");

  // API call: Timer timer1 starts timing
  rpc_cli->getRuntimeMachine()->timerStart("timer1");
  std::this_thread::sleep_for(std::chrono::milliseconds(7500));
  // API call: Get the duration timed by timer timer1
  std::cout << rpc_cli->getRuntimeMachine()->getTimer("timer1") << std::endl;
  // API call: Reset timer timer1
  rpc_cli->getRuntimeMachine()->timerReset("timer1");
  // API call: Delete timer timer1
  rpc_cli->getRuntimeMachine()->timerDelete("timer1");
}

int main(int argc, char **argv) {
  auto rpc_cli = std::make_shared<RpcClient>();
  // API call: Set RPC timeout
  rpc_cli->setRequestTimeout(1000);
  // API call: Connect to RPC service
  rpc_cli->connect(LOCAL_IP, 30004);
  // API call: Login
  rpc_cli->login("aubo", "123456");

  example1(rpc_cli);
  example2(rpc_cli);

  // API call: Logout
  rpc_cli->logout();
  // API call: Disconnect
  rpc_cli->disconnect();

  return 0;
}
