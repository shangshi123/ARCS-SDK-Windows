#include "aubo_sdk/rpc.h"
#ifdef WIN32
#include <windows.h>
#endif

using namespace arcs::common_interface;
using namespace arcs::aubo_sdk;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Implement blocking functionality: The program continues only after the robot arm reaches the target waypoint
int waitArrival(RobotInterfacePtr impl, int max_retry_count) {
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
    std::cout << "exec_id: " << exec_id << std::endl;
  }

  // Wait for the robot arm to finish the action
  while (exec_id != -1) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    exec_id = impl->getMotionControl()->getExecId();
    std::cout << "exec_id: " << exec_id << std::endl;
  }

  return 0;
}

// Wait for spline motion to finish
void waitMoveSplineFinished(RpcClientPtr impl) {
  // API call: Get the robot's name
  auto robot_name = impl->getRobotNames().front();

  auto robot_interface = impl->getRobotInterface(robot_name);

  // Wait for spline motion to start
  while (robot_interface->getMotionControl()->getExecId() == -1) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  std::cout << "Spline motion started" << std::endl;

  while (1) {
    auto id = robot_interface->getMotionControl()->getExecId();
    if (id == -1) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

// Spline motion
void exampleMoveSpline(RpcClientPtr rpc_cli)

{
  // Waypoints, represented by joint angles, unit: radians
  std::vector<double> joint_angle1 = {0.0,
                                      -15.0 * (M_PI / 180),
                                      100.0 * (M_PI / 180),
                                      25.0 * (M_PI / 180),
                                      90.0 * (M_PI / 180),
                                      0.0};

  std::vector<double> joint_angle2 = {
      19.25 * (M_PI / 180), -6.08 * (M_PI / 180), 78.87 * (M_PI / 180),
      -5.05 * (M_PI / 180), 90.0 * (M_PI / 180),  19.25 * (M_PI / 180)};

  std::vector<double> joint_angle3 = {
      28.20 * (M_PI / 180),  -12.87 * (M_PI / 180), 52.22 * (M_PI / 180),
      -24.91 * (M_PI / 180), 90.0 * (M_PI / 180),   28.2 * (M_PI / 180)};

  std::vector<double> joint_angle4 = {
      38.20 * (M_PI / 180),  -15.58 * (M_PI / 180), 58.16 * (M_PI / 180),
      -16.25 * (M_PI / 180), 90.0 * (M_PI / 180),   38.20 * (M_PI / 180)};

  std::vector<double> joint_angle5 = {
      38.20 * (M_PI / 180), -15.73 * (M_PI / 180), 91.05 * (M_PI / 180),
      16.79 * (M_PI / 180), 90.0 * (M_PI / 180),   38.20 * (M_PI / 180)};

  // API call: Get the robot's name
  auto robot_name = rpc_cli->getRobotNames().front();

  auto robot_interface = rpc_cli->getRobotInterface(robot_name);

  // API call: Set the robot arm's speed ratio
  robot_interface->getMotionControl()->setSpeedFraction(0.8);

  std::cout << "Add the first waypoint" << std::endl;
  // API call: Add the first waypoint for spline motion
  rpc_cli->getRobotInterface(robot_name)
      ->getMotionControl()
      ->moveSpline(joint_angle1, 80 * (M_PI / 180), 60 * (M_PI / 180), 0);

  std::cout << "Add the second waypoint" << std::endl;
  // API call: Add the second waypoint for spline motion
  rpc_cli->getRobotInterface(robot_name)
      ->getMotionControl()
      ->moveSpline(joint_angle2, 80 * (M_PI / 180), 60 * (M_PI / 180), 0);

  std::cout << "Add the third waypoint" << std::endl;
  // API call: Add the third waypoint for spline motion
  rpc_cli->getRobotInterface(robot_name)
      ->getMotionControl()
      ->moveSpline(joint_angle3, 80 * (M_PI / 180), 60 * (M_PI / 180), 0);

  std::cout << "Add the fourth waypoint" << std::endl;
  // API call: Add the fourth waypoint for spline motion
  rpc_cli->getRobotInterface(robot_name)
      ->getMotionControl()
      ->moveSpline(joint_angle4, 80 * (M_PI / 180), 180 * (M_PI / 180), 0);

  std::cout << "Add the fifth waypoint" << std::endl;
  // API call: Add the fifth waypoint for spline motion
  rpc_cli->getRobotInterface(robot_name)
      ->getMotionControl()
      ->moveSpline(joint_angle5, 80 * (M_PI / 180), 60 * (M_PI / 180), 0);

  // API call: When the joint angle parameter is empty, finish adding waypoints.
  // After the algorithm calculation is completed, the robot arm starts executing the spline motion.
  // Therefore, when more waypoints are added, due to the longer algorithm calculation time,
  // the robot arm may not move immediately.
  rpc_cli->getRobotInterface(robot_name)
      ->getMotionControl()
      ->moveSpline({}, 80 * (M_PI / 180), 60 * (M_PI / 180), 0.005);

  // Wait for spline motion to finish
  waitMoveSplineFinished(rpc_cli);

  std::cout << "Spline motion finished" << std::endl;
}

/**
 * Function: Robot arm performs spline motion in joint space
 * Steps:
 * Step 1: Set RPC timeout, connect to RPC service, robot arm login
 * Step 2: Set motion speed ratio, pass through 5 discrete waypoints using spline motion
 * Step 3: RPC logout, disconnect
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

  // Spline motion
  exampleMoveSpline(rpc_cli);

  // API call: Logout
  rpc_cli->logout();
  // API call: Disconnect
  rpc_cli->disconnect();

  return 0;
}

