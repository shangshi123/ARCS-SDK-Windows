#include "aubo_sdk/rpc.h"
#include "aubo_sdk/script.h"
#include <fstream>
#include <thread>
#ifdef WIN32
#include <Windows.h>
#endif
using namespace arcs::common_interface;
using namespace arcs::aubo_sdk;
using namespace std;
/**
 * Function: Run local script program
 * Steps:
 * Step 1: Connect to RPC service and log in to the robot arm
 * Step 2: Connect to SCRIPT service and log in to the robot arm
 * Step 3: Enter the script file name or absolute path
 * Case 1: If the lua script is copied to the executable program path (build/bin), enter the script file name
 * Case 2: If the lua script is not in the executable program path (build/bin), enter the absolute path of the script
 * Step 4: Read the file and run the script program. If opening the file fails, exit the program.
 */

#define LOCAL_IP "127.0.0.1"

int main(int argc, char **argv) {
#ifdef WIN32
  // Set Windows console output code page to UTF-8
  SetConsoleOutputCP(CP_UTF8);
#endif
  auto rpc = std::make_shared<RpcClient>();
  // API call: Set RPC timeout
  rpc->setRequestTimeout(1000);
  // API call: Connect to RPC service
  rpc->connect(LOCAL_IP, 30004);
  // API call: Login
  rpc->login("aubo", "123456");

  auto script = std::make_shared<ScriptClient>();
  // API call: Connect to SCRIPT service
  script->connect(LOCAL_IP, 30002);
  // API call: Login
  script->login("aubo", "123456");

  // Enter script file name
  char file_name[20];
  cin >> file_name;

  // Open file
  ifstream file;
  file.open(file_name);
  // If opening the file fails, exit the program
  if (!file) {
    cout << "open fail." << endl;
    exit(1);
  }

  // Read and print the contents of the script file
  std::string str_all;
  while (!file.eof()) {
    std::string str_line;
    getline(file, str_line);
    str_all += str_line;
    str_all += "\n";
  }

  // Find "\r\n\r\n" in the string
  size_t found = str_all.find("\r\n\r\n");

  // Loop to find and replace all matches
  while (found != std::string::npos) {
    str_all.replace(found, 4, "\n"); // 4 is the length of "\r\n\r\n"
    found = str_all.find("\r\n\r\n", found + 1);
  }
  str_all += "\r\n\r\n";
  cout << "Script content:" << endl << str_all << endl;
  file.close();

  // API call: Send local script content to controller
  script->sendString(str_all);

  // Wait for the planner to start
  int i = 0;
  while (1) {
    if (i++ > 5) {
      std::cerr << "Planner failed to start" << std::endl;
      return -1;
    }
    if (rpc->getRuntimeMachine()->getStatus() == RuntimeState::Running) {
      std::cout << "Planner started successfully, start executing script" << std::endl;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  // Wait for the planner to stop
  while (1) {
    if (rpc->getRuntimeMachine()->getStatus() == RuntimeState::Stopped) {
      std::cout << "Planner stopped, script execution completed" << std::endl;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  // API call: Logout
  rpc->logout();
  // API call: Disconnect
  rpc->disconnect();

  // API call: Logout
  script->logout();
  // API call: Disconnect
  script->disconnect();

  return 0;
}
