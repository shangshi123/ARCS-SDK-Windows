#include "aubo_sdk/rpc.h"
#ifdef WIN32
#include <Windows.h>
#endif
#define LOCAL_IP "127.0.0.1"

void read_test() {
  // RPC client object
  auto rpc_client = std::make_shared<arcs::aubo_sdk::RpcClient>();
  // API call: set RPC timeout
  rpc_client->setRequestTimeout(1000);
  // API call: connect to RPC service
  rpc_client->connect(LOCAL_IP, 30004);
  // API call: login
  rpc_client->login("aubo", "123456");
  // API call: get robot name
  auto robot_name = rpc_client->getRobotNames().front();
  auto impl = rpc_client->getRobotInterface(robot_name);

  // Create a container with 10 threads
  std::vector<std::thread> ths;
  for (int i = 0; i < 10; i++) {
    // Add a thread to the container, each thread executes the following operations
    ths.emplace_back([&]() {
      while (1) {
        try {
          // API call: get TCP offset
          impl->getRobotConfig()->getTcpOffset();
          // Output a '·' character and flush the output stream
          std::cerr << "·" << std::flush;
        } catch (const arcs::aubo_sdk::AuboException &e) {
          // Print exception information
          std::cerr << std::this_thread::get_id() << ":" << e.what()
                    << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
    });
  }

  // Wait for all threads to finish
  for (auto &t : ths) {
    t.join();
  }
}

void connect_test() {
  // Create a container with 10 threads
  std::vector<std::thread> ths;
  for (int i = 0; i < 10; i++) {
    // Add a thread to the container, each thread executes the following operations
    ths.emplace_back([&]() {
      // RPC client object
      auto rpc_client = std::make_shared<arcs::aubo_sdk::RpcClient>();
      // API call: set RPC timeout
      rpc_client->setRequestTimeout(1000);
      while (1) {
        try {
          // API call: connect to RPC service
          rpc_client->connect(LOCAL_IP, 30004);
          // API call: login
          rpc_client->login("aubo", "123456");
          std::this_thread::sleep_for(std::chrono::milliseconds(7));
          // API call: logout
          rpc_client->logout();
          // API call: disconnect from RPC service
          rpc_client->disconnect();
          std::this_thread::sleep_for(std::chrono::milliseconds(7));
        } catch (const arcs::aubo_sdk::AuboException &e) {
          // Print exception information
          std::cerr << std::this_thread::get_id() << ":" << e.what()
                    << std::endl;
        }
      }
    });
  }

  // Wait for all threads to finish
  for (auto &t : ths) {
    t.join();
  }
}

int main(int argc, char **argv) {
#ifdef WIN32
  // Set Windows console output code page to UTF-8
  SetConsoleOutputCP(CP_UTF8);
#endif
  connect_test();
  //  read_test();

  return 0;
}
