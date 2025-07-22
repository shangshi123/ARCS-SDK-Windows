#include <cstring>
#include <fstream>
#include <math.h>
#include <thread>

#include "aubo_sdk/rpc.h"
#include "aubo_sdk/rtde.h"
#include <aubo/error_stack/error_stack.h>
#ifdef WIN32
#include <windows.h>
#endif
using namespace arcs::aubo_sdk;
using namespace arcs::common_interface;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
// Implements blocking: the program continues only after the robot reaches the target waypoint
int waitArrival(RobotInterfacePtr impl) {
  // API call: Get the current motion command ID
  int exec_id = impl->getMotionControl()->getExecId();

  int cnt = 0;
  // Maximum retry count for getting exec_id while waiting for the robot to start moving
  int max_retry_count = 5;

  // Wait for the robot to start moving
  while (exec_id == -1) {
    if (cnt++ > max_retry_count) {
      return -1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    exec_id = impl->getMotionControl()->getExecId();
  }

  // Wait for the robot motion to complete
  while (exec_id != -1) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    exec_id = impl->getMotionControl()->getExecId();
  }

  return 0;
}

// Wait for spline motion to finish
void waitMoveSplineFinished(RpcClientPtr impl) {
  // API call: Get the robot's name
  auto robot_name = impl->getRobotNames().front();

  auto robot_interface = impl->getRobotInterface(robot_name);

  std::cout << "Waiting for spline motion to start" << std::endl;
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
  std::cout << "Spline motion finished" << std::endl;
}

class TrajectoryIo {
public:
  // Constructor, takes the filename to open as a parameter
  TrajectoryIo(const char *filename) {
    input_file_.open(filename, std::ios::in);
  }

  // Check if the file was opened successfully
  bool open() {
    if (!input_file_.is_open()) {
      std::cerr << "Unable to open trajectory file. Please check if the input file path is correct."
                << std::endl;
      return false;
    }
    return true;
  }
  ~TrajectoryIo() { input_file_.close(); }

  // Parse trajectory data from the file,
  // and convert it into a two-dimensional std::vector.
  // It reads the file line by line, parses each line into a set of double values,
  // and stores these values in a nested two-dimensional vector.
  std::vector<std::vector<double>> parse() {
    std::vector<std::vector<double>> res;
    std::string tmp;
    int linenum = 1;
    while (std::getline(input_file_, tmp, '\n')) {
      try {
        auto q = split(tmp, ",");
        res.push_back(q);
      } catch (const char *p) {
        std::cerr << "Line: " << linenum << " \"" << p << "\""
                  << " is not a number of double" << std::endl;
        break;
      }
      linenum++;
    }
    return res;
  }

  // Split the string and convert to double type
  std::vector<double> split(const std::string &str, const char *delim) {
    std::vector<double> res;
    if ("" == str) {
      return res;
    }
    // First convert the string to char* type
    char *strs = new char[str.length() + 1]; // Don't forget
    std::strcpy(strs, str.c_str());

    char *p = std::strtok(strs, delim);
    char *endp = nullptr;
    while (p) {
      double v = std::strtod(p, &endp);
      if (endp[0] != 0 && endp[0] != '\r') {
        delete[] strs;
        strs = nullptr;
        throw p;
      }
      res.push_back(v); // Store in result array
      p = std::strtok(nullptr, delim);
    }

    if (strs) {
      delete[] strs;
      strs = nullptr;
    }

    return res;
  }

private:
  std::ifstream input_file_; // Input file stream
};

// Spline motion
void exampleSpline(RpcClientPtr cli) {
  // Read trajectory file
  auto filename = "../trajs/coffee_spline.txt";
  TrajectoryIo input(filename);

  // Try to open the trajectory file, if unable to open, return directly
  if (!input.open()) {
    return;
  }

  // Parse trajectory data
  auto traj = input.parse();

  // Check if there are waypoints in the trajectory file,
  // If the number is 0, output an error message and return
  auto traj_sz = traj.size();
  if (traj_sz == 0) {
    std::cerr << "The number of waypoints in the trajectory file is 0." << std::endl;
    return;
  }

  // API call: Get the robot's name
  auto robot_name = cli->getRobotNames().front();

  auto robot_interface = cli->getRobotInterface(robot_name);

  // API call: Set the robot's speed fraction
  robot_interface->getMotionControl()->setSpeedFraction(0.3);

  // API call: Move joints to the first waypoint in the trajectory file
  std::vector<double> joint_angle = traj[0];
  cli->getRobotInterface(robot_name)
      ->getMotionControl()
      ->moveJoint(joint_angle, 80 * (M_PI / 180), 60 * (M_PI / 180), 0., 0.);

  // Blocking
  int ret = waitArrival(robot_interface);
  if (ret == 0) {
    std::cout << "Successfully moved joints to the first waypoint in the trajectory file" << std::endl;
  } else {
    std::cout << "Failed to move joints to the first waypoint in the trajectory file" << std::endl;
  }

  std::cout << "Adding waypoints from the trajectory file" << std::endl;
  // API call: Add waypoints from the trajectory file
  for (int i = 1; i < (int)traj.size(); i++) {
    cli->getRobotInterface(robot_name)
        ->getMotionControl()
        ->moveSpline(traj[i], 1, 1, 0);
  }

  // API call: When the joint angle parameter is empty, stop adding waypoints.
  // After waiting for the algorithm to finish calculating, the robot starts executing the spline motion.
  // Therefore, the more waypoints added, the longer the algorithm calculation time,
  // and the robot may not start moving immediately.
  cli->getRobotInterface(robot_name)
      ->getMotionControl()
      ->moveSpline({}, 1, 1, 0.005);
}

void printlog(int level, const char *source, int code, std::string content) {
  static const char *level_names[] = {"Critical", "Error", "Warning",
                                      "Info",     "Debug", "BackTrace"};
  fprintf(stderr, "[%s] %s - %d %s\n", level_names[level], source, code,
          content.c_str());
}

/**
 * Function: Spline motion in joint space using discrete trajectory from .txt file
 * Steps:
 * Step 1: Set RPC timeout, connect to RPC service, robot login
 * Step 2: Connect to RTDE service, login
 * Step 3: RTDE set topic, subscribe to topic to print log information from error_stack
 * Step 4: Read and parse .txt trajectory file
 * Step 5: Add waypoints from the file and perform spline motion
 * Step 6: RTDE logout and disconnect
 * Step 7: RPC logout and disconnect
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

  // Spline motion
  exampleSpline(rpc_cli);

  // Wait for spline motion to finish
  waitMoveSplineFinished(rpc_cli);

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
