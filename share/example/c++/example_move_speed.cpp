#include "aubo_sdk/rpc.h"
#include "aubo_sdk/rtde.h"
#include <aubo/error_stack/error_stack.h>
#include <map>
#ifdef WIN32
#include <windows.h>
#endif

using namespace arcs::common_interface;
using namespace arcs::aubo_sdk;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Joint teaching
void exampleSpeedJoint(RpcClientPtr impl) {
  // API call: Get the robot's name
  auto robot_name = impl->getRobotNames().front();
  auto robot_interface = impl->getRobotInterface(robot_name);

  // API call: Set the robot arm speed ratio
  robot_interface->getMotionControl()->setSpeedFraction(0.3);

  // Input string
  std::string input_angle;
  // Define input string and key-value mapping
  std::map<std::string, int> keymap;
  keymap["j1+"] = 1;
  keymap["j1-"] = 2;
  keymap["j2+"] = 3;
  keymap["j2-"] = 4;
  keymap["j3+"] = 5;
  keymap["j3-"] = 6;
  keymap["j4+"] = 7;
  keymap["j4-"] = 8;
  keymap["j5+"] = 9;
  keymap["j5-"] = 10;
  keymap["j6+"] = 11;
  keymap["j6-"] = 12;
  keymap["s"] = 13;
  keymap["exit"] = 14;

  // Initialize loop control variable
  bool continue_loop = true;

  // speedJoint speed
  double speed = 0.2;

  int cnt = 0;
  while (continue_loop) {
    std::cout << "Please enter the joint angle for the robot arm to move: " << std::endl;
    // Show valid input prompt
    if (cnt++ == 0) {
      std::cout << "Valid input values: "
                   "j1+, j1-, j2+, j2-, j3+, j3-, j4+, j4-, j5+, j5-, j6+"
                   ", j6-, s, exit"
                << std::endl;
      std::cout << "j1+ means joint 1 positive direction, j1- means joint 1 negative direction, j2+"
                   " means joint 2 positive direction, j2-"
                   " means joint 2 negative direction, j3+ means joint 3 positive direction, j3-"
                   " means joint 3 negative direction, j4+"
                   " means joint 4 positive direction, j4- means joint 4 negative direction, j5+"
                   " means joint 5 positive direction, j5-"
                   " means joint 5 negative direction, j6+ means joint 6 positive direction, j6-"
                   " means joint 6 negative direction, s means stop motion, exit means exit loop"
                << std::endl;
    }

    std::cin >> input_angle;
    char value = keymap[input_angle];
    switch (value) {
    case 1: {
      std::vector<double> qd = {speed, 0.0, 0.0, 0.0, 0.0, 0.0};

      // API call: Joint 1 moves in positive direction
      robot_interface->getMotionControl()->speedJoint(qd, 1.5, 100);
      break;
    }
    case 2: {
      std::vector<double> qd = {speed * (-1), 0.0, 0.0, 0.0, 0.0, 0.0};

      // API call: Joint 1 moves in negative direction
      robot_interface->getMotionControl()->speedJoint(qd, 1.5, 100);
      break;
    }
    case 3: {
      std::vector<double> qd = {0.0, speed, 0.0, 0.0, 0.0, 0.0};

      // API call: Joint 2 moves in positive direction
      robot_interface->getMotionControl()->speedJoint(qd, 1.5, 100);
      break;
    }
    case 4: {
      std::vector<double> qd = {0.0, speed * (-1), 0.0, 0.0, 0.0, 0.0};

      // API call: Joint 2 moves in negative direction
      robot_interface->getMotionControl()->speedJoint(qd, 1.5, 100);
      break;
    }
    case 5: {
      std::vector<double> qd = {0.0, 0.0, speed, 0.0, 0.0, 0.0};

      // API call: Joint 3 moves in positive direction
      robot_interface->getMotionControl()->speedJoint(qd, 1.5, 100);
      break;
    }

    case 6: {
      std::vector<double> qd = {0.0, 0.0, speed * (-1), 0.0, 0.0, 0.0};

      // API call: Joint 3 moves in negative direction
      robot_interface->getMotionControl()->speedJoint(qd, 1.5, 100);
      break;
    }
    case 7: {
      std::vector<double> qd = {0.0, 0.0, 0.0, speed, 0.0, 0.0};

      // API call: Joint 4 moves in positive direction
      robot_interface->getMotionControl()->speedJoint(qd, 1.5, 100);
      break;
    }
    case 8: {
      std::vector<double> qd = {0.0, 0.0, 0.0, speed * (-1), 0.0, 0.0};
      // API call: Joint 4 moves in negative direction
      robot_interface->getMotionControl()->speedJoint(qd, 1.5, 100);
      break;
    }
    case 9: {
      std::vector<double> qd = {0.0, 0.0, 0.0, 0.0, speed, 0.0};

      // API call: Joint 5 moves in positive direction
      robot_interface->getMotionControl()->speedJoint(qd, 1.5, 100);
      break;
    }
    case 10: {
      std::vector<double> qd = {0.0, 0.0, 0.0, 0.0, speed * (-1), 0.0};

      // API call: Joint 5 moves in negative direction
      robot_interface->getMotionControl()->speedJoint(qd, 1.5, 100);
      break;
    }
    case 11: {
      std::vector<double> qd = {0.0, 0.0, 0.0, 0.0, 0.0, speed};

      // API call: Joint 6 moves in positive direction
      robot_interface->getMotionControl()->speedJoint(qd, 1.5, 100);
      break;
    }
    case 12: {
      std::vector<double> qd = {0.0, 0.0, 0.0, 0.0, 0.0, speed * (-1)};

      // API call: Joint 6 moves in negative direction
      robot_interface->getMotionControl()->speedJoint(qd, 1.5, 100);
      break;
    }
    case 13: {
      // API call: Stop joint speed following motion
      robot_interface->getMotionControl()->stopJoint(31);

      break;
    }
    case 14: {
      // Exit loop
      continue_loop = false;
      break;
    }
    default: {
      std::cerr << "Invalid input value" << std::endl;
      std::cerr << "Valid input values: "
                   "j1+, j1-, j2+, j2-, j3+, j3-, j4+, j4-, j5+, j5-, j6+"
                   ", j6-, s, exit"
                << std::endl;
    }
    }
  }
}

// Pose teaching in base coordinate system
void exampleSpeedLine(RpcClientPtr impl) {
  // API call: Get the robot's name
  auto robot_name = impl->getRobotNames().front();
  auto robot_interface = impl->getRobotInterface(robot_name);

  // API call: Set the robot arm speed ratio
  robot_interface->getMotionControl()->setSpeedFraction(0.3);

  // API call: Set tool center point (TCP offset relative to flange center)
  std::vector<double> tcp_offset(6, 0.0);
  robot_interface->getRobotConfig()->setTcpOffset(tcp_offset);

  // Input string
  std::string input_axis;
  // Define input string and key-value mapping
  std::map<std::string, int> keymap;
  keymap["x+"] = 1;
  keymap["x-"] = 2;
  keymap["y+"] = 3;
  keymap["y-"] = 4;
  keymap["z+"] = 5;
  keymap["z-"] = 6;
  keymap["rx+"] = 7;
  keymap["rx-"] = 8;
  keymap["ry+"] = 9;
  keymap["ry-"] = 10;
  keymap["rz+"] = 11;
  keymap["rz-"] = 12;
  keymap["s"] = 13;
  keymap["exit"] = 14;

  // Initialize loop control variable
  bool continue_loop = true;

  // speedLine speed
  double speed = 0.25;

  int cnt = 0;
  while (continue_loop) {
    std::cout << "Please enter the axis for the robot arm to move: " << std::endl;
    // Show valid input prompt
    if (cnt++ == 0) {
      std::cout << "Valid input values: "
                   "x+, x-, y+, y-, z+, z-, rx+, rx-, ry+, ry-, rz+"
                   ", rz-, s, exit"
                << std::endl;
      std::cout << "x+ means linear motion in positive x axis, x- means linear motion in negative x axis, y+"
                   " means linear motion in positive y axis, y-"
                   " means linear motion in negative y axis, z+ means linear motion in positive z axis, z-"
                   " means linear motion in negative z axis, rx+"
                   " means rotational motion in positive x axis, rx- means rotational motion in negative x axis, ry+"
                   " means rotational motion in positive y axis, ry-"
                   " means rotational motion in negative y axis, rz+ means rotational motion in positive z axis, rz-"
                   " means rotational motion in negative z axis, s means stop motion, exit means exit loop"
                << std::endl;
    }

    std::cin >> input_axis;
    char value = keymap[input_axis];
    switch (value) {
    case 1: {
      std::vector<double> qd = {speed, 0.0, 0.0, 0.0, 0.0, 0.0};

      // API call: TCP moves linearly in positive x axis of base coordinate system
      robot_interface->getMotionControl()->speedLine(qd, 1.2, 100);
      break;
    }
    case 2: {
      std::vector<double> qd = {speed * (-1), 0.0, 0.0, 0.0, 0.0, 0.0};

      // API call: TCP moves linearly in negative x axis of base coordinate system
      robot_interface->getMotionControl()->speedLine(qd, 1.2, 100);
      break;
    }
    case 3: {
      std::vector<double> qd = {0.0, speed, 0.0, 0.0, 0.0, 0.0};

      // API call: TCP moves linearly in positive y axis of base coordinate system
      robot_interface->getMotionControl()->speedLine(qd, 1.2, 100);
      break;
    }
    case 4: {
      std::vector<double> qd = {0.0, speed * (-1), 0.0, 0.0, 0.0, 0.0};

      // API call: TCP moves linearly in negative y axis of base coordinate system
      robot_interface->getMotionControl()->speedLine(qd, 1.2, 100);
      break;
    }
    case 5: {
      std::vector<double> qd = {0.0, 0.0, speed, 0.0, 0.0, 0.0};

      // API call: TCP moves linearly in positive z axis of base coordinate system
      robot_interface->getMotionControl()->speedLine(qd, 1.2, 100);
      break;
    }

    case 6: {
      std::vector<double> qd = {0.0, 0.0, speed * (-1), 0.0, 0.0, 0.0};

      // API call: TCP moves linearly in negative z axis of base coordinate system
      robot_interface->getMotionControl()->speedLine(qd, 1.2, 100);
      break;
    }
    case 7: {
      std::vector<double> qd = {0.0, 0.0, 0.0, speed, 0.0, 0.0};

      // API call: TCP rotates in positive x axis of base coordinate system
      robot_interface->getMotionControl()->speedLine(qd, 1.2, 100);
      break;
    }
    case 8: {
      std::vector<double> qd = {0.0, 0.0, 0.0, speed * (-1), 0.0, 0.0};

      // API call: TCP rotates in negative x axis of base coordinate system
      robot_interface->getMotionControl()->speedLine(qd, 1.2, 100);
      break;
    }
    case 9: {
      std::vector<double> qd = {0.0, 0.0, 0.0, 0.0, speed, 0.0};

      // API call: TCP rotates in positive y axis of base coordinate system
      robot_interface->getMotionControl()->speedLine(qd, 1.2, 100);
      break;
    }
    case 10: {
      std::vector<double> qd = {0.0, 0.0, 0.0, 0.0, speed * (-1), 0.0};

      // API call: TCP rotates in negative y axis of base coordinate system
      robot_interface->getMotionControl()->speedLine(qd, 1.2, 100);
      break;
    }
    case 11: {
      std::vector<double> qd = {0.0, 0.0, 0.0, 0.0, 0.0, speed};

      // API call: TCP rotates in positive z axis of base coordinate system
      robot_interface->getMotionControl()->speedLine(qd, 1.2, 100);
      break;
    }
    case 12: {
      std::vector<double> qd = {0.0, 0.0, 0.0, 0.0, 0.0, speed * (-1)};

      // API call: TCP rotates in negative z axis of base coordinate system
      robot_interface->getMotionControl()->speedLine(qd, 1.2, 100);
      break;
    }
    case 13: {
      // API call: Stop linear speed following motion
      robot_interface->getMotionControl()->stopLine(10, 10);

      break;
    }
    case 14: {
      // Exit loop
      continue_loop = false;
      break;
    }
    default: {
      std::cerr << "Invalid input value" << std::endl;
      std::cerr << "Valid input values: "
                   "x+, x-, y+, y-, z+, z-, rx+, rx-, ry+, ry-, rz+"
                   ", rz-, s, exit"
                << std::endl;
    }
    }
  }
}

// Print log information
void printlog(int level, const char *source, int code, std::string content) {
  static const char *level_names[] = {"Critical", "Error", "Warning",
                                      "Info",     "Debug", "BackTrace"};
  fprintf(stderr, "[%s] %s - %d %s\n", level_names[level], source, code,
          content.c_str());
}

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

  // Joint teaching
  exampleSpeedJoint(rpc_cli);

  // Pose teaching in base coordinate system
  //  exampleSpeedLine(rpc_cli);

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
