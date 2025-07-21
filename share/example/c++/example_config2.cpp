#include "aubo_sdk/rpc.h"
#include <unordered_map>
#ifdef WIN32
#include <windows.h>
#endif

using namespace arcs::aubo_sdk;

// Template function: Print variables of type std::vector<T>
template <typename T> void printVec(std::vector<T> param, std::string name) {
  std::cout << name << ": ";

  for (int i = 0; i < param.size(); i++) {
    std::cout << param.at(i);
    if (i != param.size() - 1) {
      std::cout << ", ";
    }
  }
  std::cout << std::endl;
}

// Template function: Print variables of type std::vector<std::vector<T>>
template <typename T> void print2Vec(std::vector<T> param, std::string name) {
  std::cout << name << ": " << std::endl;
  for (size_t i = 0; i < param.size(); i++) {
    std::cout << "  Group " << i + 1 << ": ";
    for (size_t j = 0; j < param.at(i).size(); j++) {
      std::cout << param.at(i).at(j);

      if (j != param.at(i).size() - 1) {
        std::cout << ", ";
      } else {
        std::cout << std::endl;
      }
    }

    if ((param.size() - 1) == i) {
      std::cout << std::endl;
    }
  }
}

void exampleConfig(RpcClientPtr cli) {
  // API call: Get robot name
  auto robot_name = cli->getRobotNames().front();

  // API call: Get robot name
  std::string name;
  name = cli->getRobotInterface(robot_name)->getRobotConfig()->getName();
  std::cout << "Get robot name: " << name << std::endl;

  // API call: Get robot degrees of freedom
  int dof;
  dof = cli->getRobotInterface(robot_name)->getRobotConfig()->getDof();
  std::cout << "Get robot degrees of freedom: " << dof << std::endl;

  // API call: Get robot servo control cycle (read from hardware abstraction layer)
  double cycle_time;
  cycle_time =
      cli->getRobotInterface(robot_name)->getRobotConfig()->getCycletime();
  std::cout << "Get servo control cycle: " << cycle_time << std::endl;

  // API call: Get robot type code
  std::string robot_type;
  robot_type =
      cli->getRobotInterface(robot_name)->getRobotConfig()->getRobotType();
  std::cout << "Get robot type code: " << robot_type << std::endl;

  // API call: Get robot subtype code
  std::string sub_type;
  sub_type =
      cli->getRobotInterface(robot_name)->getRobotConfig()->getRobotSubType();
  std::cout << "Get robot subtype code: " << sub_type << std::endl;

  // API call: Get control box type code
  std::string control_box_type;
  control_box_type =
      cli->getRobotInterface(robot_name)->getRobotConfig()->getControlBoxType();
  std::cout << "Get control box type code: " << control_box_type << std::endl;

  // API call: Set collision sensitivity level
  int level = 10;
  cli->getRobotInterface(robot_name)
      ->getRobotConfig()
      ->setCollisionLevel(level);
  std::cout << "Set collision sensitivity level: " << level << std::endl;

  std::this_thread::sleep_for(std::chrono::seconds(1));

  // API call: Get collision sensitivity level
  int collision_level;
  collision_level =
      cli->getRobotInterface(robot_name)->getRobotConfig()->getCollisionLevel();
  std::cout << "Get collision sensitivity level: " << collision_level << std::endl;

  // API call: Set collision stop type
  int type = 1;
  cli->getRobotInterface(robot_name)
      ->getRobotConfig()
      ->setCollisionStopType(type);
  std::cout << "Set collision stop type: " << type << std::endl;

  // API call: Get collision stop type
  int collision_stop_type;
  collision_stop_type = cli->getRobotInterface(robot_name)
                            ->getRobotConfig()
                            ->getCollisionStopType();
  std::cout << "Get collision stop type: " << collision_stop_type << std::endl;

  // API call: Get theoretical values of robot DH parameters
  std::unordered_map<std::string, std::vector<double>> kin_param1;
  kin_param1 = cli->getRobotInterface(robot_name)
                   ->getRobotConfig()
                   ->getKinematicsParam(true);
  std::cout << "Get theoretical values of robot DH parameters:" << std::endl;
  for (auto it = kin_param1.begin(); it != kin_param1.end(); it++) {
    std::cout << it->first << ": ";
    int size = it->second.size();
    for (int i = 0; i < size; i++) {
      std::cout << it->second.at(i) << " ";
    }
    std::cout << std::endl;
  }

  // API call: Get actual values of robot DH parameters
  std::unordered_map<std::string, std::vector<double>> kin_param2;
  kin_param2 = cli->getRobotInterface(robot_name)
                   ->getRobotConfig()
                   ->getKinematicsParam(false);
  std::cout << "Get actual values of robot DH parameters:" << std::endl;
  for (auto it = kin_param2.begin(); it != kin_param2.end(); it++) {
    std::cout << it->first << ": ";
    int size = it->second.size();
    for (int i = 0; i < size; i++) {
      std::cout << it->second.at(i) << " ";
    }
    std::cout << std::endl;
  }

  // API call: Get DH parameter compensation values at specified temperature
  std::unordered_map<std::string, std::vector<double>> kin_compensate;
  double temperature = 20;
  kin_compensate = cli->getRobotInterface(robot_name)
                       ->getRobotConfig()
                       ->getKinematicsCompensate(temperature);
  std::cout << "Get robot DH parameter compensation values:" << std::endl;
  for (auto it = kin_compensate.begin(); it != kin_compensate.end(); it++) {
    std::cout << it->first << ": ";
    int size = it->second.size();
    for (int i = 0; i < size; i++) {
      std::cout << it->second.at(i) << " ";
    }
    std::cout << std::endl;
  }

  // API call: Get available end force sensor names
  std::vector<std::string> tcp_force_sensor_names;
  tcp_force_sensor_names = cli->getRobotInterface(robot_name)
                               ->getRobotConfig()
                               ->getTcpForceSensorNames();
  printVec<std::string>(tcp_force_sensor_names, "Get end force sensor names");

  // API call: Get safety parameter checksum CRC32
  uint32_t check_sum;
  check_sum = cli->getRobotInterface(robot_name)
                  ->getRobotConfig()
                  ->getSafetyParametersCheckSum();
  std::cout << "Get safety parameter checksum CRC32: " << check_sum << std::endl;

  // API call: Get joint maximum position (physical limit)
  std::vector<double> joint_max_positions;
  joint_max_positions = cli->getRobotInterface(robot_name)
                            ->getRobotConfig()
                            ->getJointMaxPositions();
  printVec<double>(joint_max_positions, "Get joint maximum position (physical limit)");

  // API call: Get joint minimum position (physical limit)
  std::vector<double> joint_min_positions;
  joint_min_positions = cli->getRobotInterface(robot_name)
                            ->getRobotConfig()
                            ->getJointMinPositions();
  printVec<double>(joint_min_positions, "Get joint minimum position (physical limit)");

  // API call: Get joint maximum speed (physical limit)
  std::vector<double> joint_max_speeds;
  joint_max_speeds =
      cli->getRobotInterface(robot_name)->getRobotConfig()->getJointMaxSpeeds();
  printVec<double>(joint_max_speeds, "Get joint maximum speed (physical limit)");

  // API call: Get joint maximum acceleration (physical limit)
  std::vector<double> joint_max_acc;
  joint_max_acc = cli->getRobotInterface(robot_name)
                      ->getRobotConfig()
                      ->getJointMaxAccelerations();
  printVec<double>(joint_max_acc, "Get joint maximum acceleration (physical limit)");

  // API call: Get robot gravity acceleration
  std::vector<double> gravity;
  gravity = cli->getRobotInterface(robot_name)->getRobotConfig()->getGravity();
  printVec<double>(gravity, "Get robot gravity acceleration");

  // API call: Set TCP offset
  std::vector<double> tcp_offset1 = {-0.1, 0.2, 0.3, 3.14, 0.0, 1.57};
  cli->getRobotInterface(robot_name)
      ->getRobotConfig()
      ->setTcpOffset(tcp_offset1);
  std::this_thread::sleep_for(std::chrono::seconds(1));
  printVec<double>(tcp_offset1, "Set TCP offset to");

  // API call: Get TCP offset
  std::vector<double> tcp_offset2;
  tcp_offset2 =
      cli->getRobotInterface(robot_name)->getRobotConfig()->getTcpOffset();
  printVec<double>(tcp_offset2, "Get TCP offset");

  // API call: Set payload
  double mass1 = 2.5;
  std::vector<double> cog1(3, 0.0);
  std::vector<double> aom1(3, 0.0);
  std::vector<double> inertia1(6, 0.0);
  cli->getRobotInterface(robot_name)
      ->getRobotConfig()
      ->setPayload(mass1, cog1, aom1, inertia1);
  std::this_thread::sleep_for(std::chrono::seconds(1));
  std::cout << "Set end payload to:" << std::endl;
  std::cout << "mass: " << mass1 << std::endl;
  printVec<double>(cog1, "cog");
  printVec<double>(aom1, "aom");
  printVec<double>(inertia1, "intertia");

  // API call: Get end payload
  auto payload =
      cli->getRobotInterface(robot_name)->getRobotConfig()->getPayload();
  std::cout << "Get end payload:" << std::endl;
  std::cout << "mass: " << std::get<0>(payload) << std::endl;
  std::vector<double> cog2 = std::get<1>(payload);
  printVec<double>(cog2, "cog");
  std::vector<double> aom2 = std::get<2>(payload);
  printVec<double>(aom2, "aom");
  std::vector<double> inertia2 = std::get<3>(payload);
  printVec<double>(inertia2, "intertia");

  // API call: Get joint friction parameters
  auto joint_friction_param =
      cli->getRobotInterface(robot_name)
          ->getRobotConfig()
          ->getHardwareCustomParameters("joint_friction_params");
  std::cout << "Joint friction parameters: " << joint_friction_param << std::endl;

}

#define ip_local "127.0.0.1"

/**
 * Function: Get robot configuration information
 * Steps:
 * Step 1: Set RPC timeout, connect to RPC service, login
 * Step 2: Get robot related configuration information
 * Step 3: Logout, disconnect RPC connection
 */
int main(int argc, char **argv) {
#ifdef WIN32
  // Set Windows console output code page to UTF-8
  SetConsoleOutputCP(CP_UTF8);
#endif

  auto rpc_cli = std::make_shared<RpcClient>();
  // API call: Set RPC timeout
  rpc_cli->setRequestTimeout(1000);
  // API call: Connect to RPC service
  rpc_cli->connect(ip_local, 30004);
  // API call: Login
  rpc_cli->login("aubo", "123456");

  // Get robot related configuration information
  exampleConfig(rpc_cli);

  // API call: Logout
  rpc_cli->logout();
  // API call: Disconnect
  rpc_cli->disconnect();

  return 0;
}
