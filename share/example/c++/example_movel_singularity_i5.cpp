#include "aubo_sdk/rpc.h"
#include "aubo_sdk/rtde.h"
#include <aubo/error_stack/error_stack.h>
#include <math.h>
#ifdef WIN32
#include <windows.h>
#endif

using namespace arcs::common_interface;
using namespace arcs::aubo_sdk;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Implement blocking: The program continues only after the robot arm reaches the target waypoint
int waitArrival(RobotInterfacePtr impl) {
  const int max_retry_count = 5;
  int cnt = 0;

  // API call: Get the current motion command ID
  int exec_id = impl->getMotionControl()->getExecId();

  // Wait for the robot arm to start moving
  while (exec_id == -1) {
    if (cnt++ > max_retry_count) {
      return -1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    exec_id = impl->getMotionControl()->getExecId();
  }

  // Wait for the robot arm to finish the action
  while (impl->getMotionControl()->getExecId() != -1) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  return 0;
}

// Linear movement
void exampleMovel(RpcClientPtr cli) {
  // Joint angles, unit: radians
  std::vector<double> joint_angle = {0.0 * (M_PI / 180),   -15.0 * (M_PI / 180),
                                     100.0 * (M_PI / 180), 25.0 * (M_PI / 180),
                                     90.0 * (M_PI / 180),  0.0 * (M_PI / 180)};

  // Pose at singularity
  std::vector<double> pose_singularity = {1000,    1000,      1000,
                                          3.05165, 0.0324355, 1.80417};

  // API call: Get the robot's name
  auto robot_name = cli->getRobotNames().front();

  auto robot_interface = cli->getRobotInterface(robot_name);

  // API call: Set robot arm speed fraction
  robot_interface->getMotionControl()->setSpeedFraction(0.8);

  // API call: Set tool center point (TCP offset relative to flange center)
  std::vector<double> tcp_offset(6, 0.0);
  robot_interface->getRobotConfig()->setTcpOffset(tcp_offset);

  // API call: Move joints to starting position
  robot_interface->getMotionControl()->moveJoint(joint_angle, 80 * (M_PI / 180),
                                                 60 * (M_PI / 180), 0, 0);
  // Blocking
  int ret = waitArrival(robot_interface);
  if (ret == 0) {
    std::cout << "Joint movement to starting position succeeded!" << std::endl;
  } else {
    std::cout << "Joint movement to starting position failed!" << std::endl;
  }

  // API call: Linear movement to singularity
  robot_interface->getMotionControl()->moveLine(pose_singularity, 1.2, 0.25,
                                                0.025, 0);
  ret = waitArrival(robot_interface);
  if (ret == 0) {
    std::cout << "Linear movement succeeded!" << std::endl;
  } else {
    std::cout << "Linear movement failed!" << std::endl;
  }
}

void printlog(int level, const char *source, int code, std::string content) {
  static const char *level_names[] = {"Critical", "Error", "Warning",
                                      "Info",     "Debug", "BackTrace"};
  fprintf(stderr, "[%s] %s - %d %s\n", level_names[level], source, code,
          content.c_str());
}

/**
 * Function: Robot arm linear movement to singularity
 * Steps:
 * Step 1: Set RPC timeout, connect to RPC service, robot arm login
 * Step 2: Connect to RTDE service, login
 * Step 3: RTDE set topic, subscribe to topic to print logs from error_stack
 * Step 4: Set movement speed fraction and tool center point
 * Step 5: First move joints to starting position, then move linearly to singularity
 * Step 6: RTDE logout, disconnect
 * Step 7: RPC logout, disconnect
 */

#define LOCAL_IP "127.0.0.1"

int main(int argc, char **argv) {
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

  auto rtde_cli = std::make_shared<RtdeClient>();
  // API call: Connect to RTDE service
  rtde_cli->connect(LOCAL_IP, 30010);
  // API call: Login
  rtde_cli->login("aubo", "123456");

  // API call: Set RTDE topic
  int topic = rtde_cli->setTopic(false, {"R1_message"}, 200, 0);
  if (topic < 0) {
    std::cout << "Failed to set topic!" << std::endl;
    return -1;
  }

  // API call: Subscribe to topic
  rtde_cli->subscribe(topic, [](InputParser &parser) {
    arcs::common_interface::RobotMsgVector msgs;
    msgs = parser.popRobotMsgVector();
    for (size_t i = 0; i < msgs.size(); i++) {
      auto &msg = msgs[i];
      std::string error_content = arcs::error_stack::errorCode2Str(msg.code);
      for (auto it : msg.args) {
        auto pos = error_content.find("{}");
        if (pos != std::string::npos) {
          error_content.replace(pos, 2, it);
        } else {
          break;
        }
      }
      // Print log information
      printlog(msg.level, msg.source.c_str(), msg.code, error_content);
    }
  });

  // Linear movement to singularity
  exampleMovel(rpc_cli);

  // API call: Remove topic
  rtde_cli->removeTopic(false, topic);
  // API call: Logout
  rtde_cli->logout();
  // API call: Disconnect
  rtde_cli->disconnect();

  // API call: Logout
  rpc_cli->logout();
  // API call: Disconnect
  rpc_cli->disconnect();

  return 0;
}
