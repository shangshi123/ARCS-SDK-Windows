#include <cstring>
#include <fstream>
#include <math.h>
#ifdef WIN32
#include <windows.h>
#endif

#include "aubo_sdk/rpc.h"
#include <csignal>

using namespace arcs::aubo_sdk;
using namespace arcs::common_interface;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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
  // and convert it to a 2D std::vector.
  // Reads the file line by line, parses each line into a group of double values,
  // and stores these values in a nested 2D vector.
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

  // Split string and convert to double type
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

// Implement blocking: the program continues only when the robot arm reaches the target waypoint
int waitArrival(RobotInterfacePtr impl) {
  // Get the current motion command ID
  int exec_id = impl->getMotionControl()->getExecId();

  int cnt = 0;
  // Maximum retry count for waiting for the robot arm to start moving
  int max_retry_count = 50;

  // Wait for the robot arm to start moving
  while (exec_id == -1) {
    if (cnt++ > max_retry_count) {
      return -1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    exec_id = impl->getMotionControl()->getExecId();
  }

  // Wait for the robot arm to finish moving
  while (exec_id != -1) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    exec_id = impl->getMotionControl()->getExecId();
  }

  return 0;
}

// Check if buffer is valid
int isBufferValid(RobotInterfacePtr impl) {
  // Maximum retry count for calling pathBufferValid
  int max_retry_count = 5;
  // Number of times pathBufferValid is called
  int cnt = 0;
  bool isValid = impl->getMotionControl()->pathBufferValid("rec");

  while (!isValid) {
    if (cnt++ > max_retry_count) {
      return -1;
    }
    isValid = impl->getMotionControl()->pathBufferValid("rec");
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return 0;
}

#define LOCAL_IP "127.0.0.1"

auto rpc_cli = std::make_shared<RpcClient>();
int task_id;

// Define signal handler function
void signalHandler(int signal) {
  if (signal == SIGINT) {
    std::cout << "Received SIGINT signal." << std::endl;
    if (rpc_cli) {
      rpc_cli->getRuntimeMachine()->stop();
      rpc_cli->getRuntimeMachine()->deleteTask(task_id);
      rpc_cli->logout();
      rpc_cli->disconnect();
    }
    // Exit program
    exit(0);
  }
}

int main(int argc, char **argv) {
#ifdef WIN32
  // Set Windows console output code page to UTF-8
  SetConsoleOutputCP(CP_UTF8);
#endif
  // Register signal handler function
  signal(SIGINT, signalHandler);

  // Read trajectory file
  //
  // The waypoints in CoffeCappuccino-heart-R_filtered.csv are TCP relative to the user coordinate system
  //    auto filename = "../trajs/CoffeCappuccino-heart-R_filtered.csv";
  auto filename = "../trajs/CoffeCappuccino-heart-L_filtered.csv";
  TrajectoryIo input(filename);

  // Try to open trajectory file, return directly if unable to open
  if (!input.open()) {
    return 0;
  }

  // Parse trajectory data
  auto traj = input.parse();

  // Check if there are waypoints in the trajectory file,
  // If the number is 0, output an error message and return
  auto traj_sz = traj.size();
  if (traj_sz == 0) {
    std::cerr << "The number of waypoints in the trajectory file is 0." << std::endl;
    return 0;
  }

  // API call: Set RPC timeout
  rpc_cli->setRequestTimeout(10000);
  // API call: Connect to RPC service
  rpc_cli->connect(LOCAL_IP, 30004);
  // API call: Login
  rpc_cli->login("aubo", "123456");

  // API call: Get the robot's name
  auto robot_name = rpc_cli->getRobotNames().front();

  auto robot_interface = rpc_cli->getRobotInterface(robot_name);

  // API call: Set the speed ratio of the robot arm
  robot_interface->getMotionControl()->setSpeedFraction(1);

  // TCP offset pose relative to the center of the flange
  std::vector<double> tcp_offset(6);
  tcp_offset[0] = -0.093;
  tcp_offset[1] = 0.0007249;
  tcp_offset[2] = 0.152;
  tcp_offset[3] = 90.66 / 180.0 * M_PI;
  tcp_offset[4] = -0.7635 / 180.0 * M_PI;
  tcp_offset[5] = -91.49 / 180.0 * M_PI;
  //    std::vector<double> tcp_offset(6);
  //    tcp_offset[0] = 0.0;
  //    tcp_offset[1] = 0.0;
  //    tcp_offset[2] = 0.0;
  //    tcp_offset[3] = 0.0;
  //    tcp_offset[4] = 0.0;
  //    tcp_offset[5] = 0.0;

  // API call: Set the TCP offset relative to the center of the flange
  robot_interface->getRobotConfig()->setTcpOffset(tcp_offset);

  // The poses of three calibration points
  std::vector<double> coord_q0(6), coord_q1(6), coord_q2(6);
  coord_q0[0] = -1.627214;
  coord_q0[1] = -0.360182;
  coord_q0[2] = 1.783065;
  coord_q0[3] = -1.063912;
  coord_q0[4] = -1.642473;
  coord_q0[5] = -1.510366;

  coord_q1[0] = -0.806772;
  coord_q1[1] = -0.344142;
  coord_q1[2] = 1.806783;
  coord_q1[3] = -1.079834;
  coord_q1[4] = -0.824188;
  coord_q1[5] = -1.444983;

  coord_q2[0] = -0.702519;
  coord_q2[1] = -0.105247;
  coord_q2[2] = 2.113576;
  coord_q2[3] = -1.022056;
  coord_q2[4] = -0.720357;
  coord_q2[5] = -1.430912;

  // API call: Use forward kinematics to get the joint angles of the calibration points
  auto [coord_p0, ret0] =
      robot_interface->getRobotAlgorithm()->forwardKinematics(coord_q0);
  auto [coord_p1, ret1] =
      robot_interface->getRobotAlgorithm()->forwardKinematics(coord_q1);
  auto [coord_p2, ret2] =
      robot_interface->getRobotAlgorithm()->forwardKinematics(coord_q2);

  // API call: User coordinate system calibration
  auto [coord, ret3] = rpc_cli->getMath()->calibrateCoordinate(
      {coord_p0, coord_p1, coord_p2}, 0);

  // Create a variable named traj_q,
  // Its type is the same as the expression traj
  decltype(traj) traj_q;

  // API call: Start the planner
  rpc_cli->getRuntimeMachine()->start();
  auto cur_plan_context = rpc_cli->getRuntimeMachine()->getPlanContext();
  // API call: Get a new thread id
  task_id = rpc_cli->getRuntimeMachine()->newTask();
  rpc_cli->getRuntimeMachine()->setPlanContext(
      task_id, std::get<1>(cur_plan_context), std::get<2>(cur_plan_context));

  // API call: Move joints to the origin
  auto home_q = {0.0,
                 -15.0 / 180 * M_PI,
                 100.0 / 180 * M_PI,
                 25.0 / 180 * M_PI,
                 90.0 / 180 * M_PI,
                 0.0};
  robot_interface->getMotionControl()->moveJoint(home_q, 30 * (M_PI / 180),
                                                 30 * (M_PI / 180), 0., 0.);
  // Blocking
  int ret = waitArrival(robot_interface);
  if (ret == 0) {
    std::cout << "Joint movement to origin succeeded" << std::endl;
  } else {
    std::cout << "Joint movement to origin failed" << std::endl;
  }

  // API call: Get the current joint angles,
  // As the reference joint angles for the inverse kinematics later in the code
  auto current_q = robot_interface->getRobotState()->getJointPositions();

  // Convert the TCP waypoint poses in the user coordinate system from the trajectory file to the TCP waypoint poses in the base coordinate system,
  // Then use inverse kinematics to get the joint angles,
  // And save the joint angles of the waypoints in the variable traj_q
  for (const auto &p : traj) {
    // API call: Get the TCP pose in the base coordinate system
    auto rp = rpc_cli->getMath()->poseTrans(coord, p);
    // API call: Use inverse kinematics to get joint angles
    auto [q, ret] =
        robot_interface->getRobotAlgorithm()->inverseKinematics(current_q, rp);
    current_q = q;
    traj_q.push_back(q);
  }

  // API call: Move joints to the first waypoint in the trajectory file
  robot_interface->getMotionControl()->moveJoint(traj_q[0], 30 * (M_PI / 180),
                                                 30 * (M_PI / 180), 0., 0.);
  // Blocking
  ret = waitArrival(robot_interface);
  if (ret == 0) {
    std::cout << "Joint movement to the first waypoint in the trajectory file succeeded" << std::endl;
  } else {
    std::cout << "Joint movement to the first waypoint in the trajectory file failed" << std::endl;
  }

  // API call: Clear buffer "rec"
  robot_interface->getMotionControl()->pathBufferFree("rec");

  // API call: Create a new buffer "rec", and specify the trajectory motion type and number of trajectory points
  robot_interface->getMotionControl()->pathBufferAlloc("rec", 3, traj_sz);

  // Add waypoints from the trajectory file to the path buffer in groups,
  // 10 points per group,
  // If the number of remaining waypoints to be added is less than or equal to 10, add them as the last group
  size_t offset = 10;
  auto it = traj_q.begin();
  while (true) {
    std::cout << "Add trajectory waypoint " << offset << std::endl;
    // API call: Add trajectory waypoints to the buffer path
    robot_interface->getMotionControl()->pathBufferAppend(
        "rec", std::vector<std::vector<double>>{it, it + 10});
    it += 10;
    if (offset + 10 >= traj_sz) {
      std::cout << "Add trajectory waypoint " << traj_sz << std::endl;
      // API call: Add trajectory waypoints to the buffer path
      robot_interface->getMotionControl()->pathBufferAppend(
          "rec", std::vector<std::vector<double>>{it, traj_q.end()});
      break;
    }

    offset += 10;
  }

  // API call: Perform time-consuming operations such as calculation and optimization to optimize the trajectory
  robot_interface->getMotionControl()->pathBufferEval("rec", {3, 3, 3, 3, 3, 3},
                                                      {2, 2, 2, 2, 2, 2}, 0);

  // Check whether the path buffer is valid
  if (isBufferValid(robot_interface) == -1) {
    std::cerr << "Path buffer is invalid, unable to perform trajectory motion" << std::endl;
  } else {
    // API call: Execute trajectory motion
    robot_interface->getMotionControl()->movePathBuffer("rec");

    // Wait for trajectory motion to complete
    ret = waitArrival(robot_interface);
    if (ret == -1) {
      std::cerr << "Trajectory motion failed" << std::endl;
    } else {
      std::cout << "Trajectory motion finished" << std::endl;
    }
  }

  // API call: Stop the planner
  rpc_cli->getRuntimeMachine()->stop();
  // API call: Delete thread
  rpc_cli->getRuntimeMachine()->deleteTask(task_id);

  // API call: RPC logout
  rpc_cli->logout();
  // API call: RPC disconnect
  rpc_cli->disconnect();

  return 0;
}
