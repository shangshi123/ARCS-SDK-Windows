#include "aubo_sdk/rpc.h"
#ifdef WIN32
#include <windows.h>
#endif

using namespace arcs::common_interface;
using namespace arcs::aubo_sdk;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Blocking function: The program continues only after the robot reaches the target waypoint
int waitArrival(RobotInterfacePtr impl) {
  const int max_retry_count = 5;
  int cnt = 0;

  // API call: Get the current motion command ID
  int exec_id = impl->getMotionControl()->getExecId();

  // Wait for the robot to start moving
  while (exec_id == -1) {
    if (cnt++ > max_retry_count) {
      return -1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    exec_id = impl->getMotionControl()->getExecId();
  }

  // Wait for the robot to finish the action
  while (impl->getMotionControl()->getExecId() != -1) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  return 0;
}

void exampleMovel(RpcClientPtr cli) {
  // Joint angles, unit: radians
  std::vector<double> joint_angle = {0.0 * (M_PI / 180),   -15.0 * (M_PI / 180),
                                     100.0 * (M_PI / 180), 25.0 * (M_PI / 180),
                                     90.0 * (M_PI / 180),  0.0 * (M_PI / 180)};
  // Pose
  std::vector<double> pose1 = {-0.155944, -0.727344, 0.439066,
                               3.05165,   0.0324355, 1.80417};
  std::vector<double> pose2 = {-0.581143, -0.357548, 0.439066,
                               3.05165,   0.0324355, 1.80417};
  std::vector<double> pose3 = {0.503502, -0.420646, 0.439066,
                               3.05165,  0.0324355, 1.80417};

  // API call: Get the robot's name
  auto robot_name = cli->getRobotNames().front();

  auto robot_interface = cli->getRobotInterface(robot_name);

  // API call: Set the robot's speed fraction
  robot_interface->getMotionControl()->setSpeedFraction(0.75);

  // API call: Set the tool center point (TCP offset relative to flange center)
  std::vector<double> tcp_offset(6, 0.0);
  robot_interface->getRobotConfig()->setTcpOffset(tcp_offset);

  // API call: Move joints to the starting position
  robot_interface->getMotionControl()->moveJoint(joint_angle, 80 * (M_PI / 180),
                                                 60 * (M_PI / 180), 0, 0);
  // Blocking
  int ret = waitArrival(robot_interface);
  if (ret == 0) {
    std::cout << "Joint movement to starting position succeeded!" << std::endl;
  } else {
    std::cout << "Joint movement to starting position failed!" << std::endl;
  }

  // API call: Linear movement to position 1
  robot_interface->getMotionControl()->moveLine(pose1, 1.2, 0.25, 0.025, 0);
  // Blocking
  ret = waitArrival(robot_interface);
  if (ret == 0) {
    std::cout << "Linear movement to position 1 succeeded!" << std::endl;
  } else {
    std::cout << "Linear movement to position 1 failed!" << std::endl;
  }

  // API call: Linear movement to position 2
  robot_interface->getMotionControl()->moveLine(pose2, 1.2, 0.25, 0.025, 0);
  // Blocking
  ret = waitArrival(robot_interface);
  if (ret == 0) {
    std::cout << "Linear movement to position 2 succeeded!" << std::endl;
  } else {
    std::cout << "Linear movement to position 2 failed!" << std::endl;
  }

  // API call: Linear movement to position 3
  robot_interface->getMotionControl()->moveLine(pose3, 1.2, 0.25, 0.025, 0);
  // Blocking
  ret = waitArrival(robot_interface);
  if (ret == 0) {
    std::cout << "Linear movement to position 3 succeeded!" << std::endl;
  } else {
    std::cout << "Linear movement to position 3 failed!" << std::endl;
  }
}

/**
 * Function: Robot linear movement
 * Steps:
 * Step 1: Set RPC timeout, connect to RPC service, robot login
 * Step 2: Set motion speed fraction and tool center point
 * Step 3: First move joints to the starting position, then pass through 3 waypoints with linear movement
 * Step 4: RPC logout and disconnect
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

  // Joint movement
  exampleMovel(rpc_cli);

  // API call: Logout
  rpc_cli->logout();
  // API call: Disconnect
  rpc_cli->disconnect();

  return 0;
}
