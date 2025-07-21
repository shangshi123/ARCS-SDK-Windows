#include "aubo_sdk/rpc.h"
#ifdef WIN32
#include <Windows.h>
#endif

using namespace arcs::common_interface;
using namespace arcs::aubo_sdk;

// Template function: print variable of type std::vector<T>
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

// Template function: print variable of type std::vector<std::vector<T>>
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

void exampleForwardK(RpcClientPtr impl) {
  // API call: get robot name
  auto robot_name = impl->getRobotNames().front();

  // API call: set tcp offset
  std::vector<double> offset = {0.1, 0.2, 0.3, 0.0, 0.0, 0.0};
  impl->getRobotInterface(robot_name)->getRobotConfig()->setTcpOffset(offset);

  // API call: get current joint angles
  std::vector<double> q =
      impl->getRobotInterface(robot_name)->getRobotState()->getJointPositions();

  // API call: forward kinematics to get current TCP pose
  auto result = impl->getRobotInterface(robot_name)
                    ->getRobotAlgorithm()
                    ->forwardKinematics(q);
  printVec(std::get<0>(result), "Forward kinematics: current TCP pose");
}

void exampleForwardToolK(RpcClientPtr impl) {
  // API call: get robot name
  auto robot_name = impl->getRobotNames().front();

  // API call: set tcp offset
  std::vector<double> offset = {0.1, 0.2, 0.3, 0.0, 0.0, 0.0};
  impl->getRobotInterface(robot_name)->getRobotConfig()->setTcpOffset(offset);

  // API call: get current joint angles
  std::vector<double> q =
      impl->getRobotInterface(robot_name)->getRobotState()->getJointPositions();

  // API call: forward kinematics to get flange end center pose
  auto result = impl->getRobotInterface(robot_name)
                    ->getRobotAlgorithm()
                    ->forwardToolKinematics(q);
  printVec(std::get<0>(result), "Forward kinematics: flange end center pose");
}

void exampleInverseK(RpcClientPtr impl) {
  // API call: get robot name
  auto robot_name = impl->getRobotNames().front();

  // API call: set tcp offset
  std::vector<double> offset = {0.1, 0.2, 0.3, 0.0, 0.0, 0.0};
  impl->getRobotInterface(robot_name)->getRobotConfig()->setTcpOffset(offset);

  // API call: get current joint angles
  std::vector<double> qnear =
      impl->getRobotInterface(robot_name)->getRobotState()->getJointPositions();

  // API call: get current tcp value as target pose
  std::vector<double> p =
      impl->getRobotInterface(robot_name)->getRobotState()->getTcpPose();
  printVec(p, "Current TCP pose");

  // API call: get optimal inverse kinematics solution based on TCP pose
  auto result1 = impl->getRobotInterface(robot_name)
                     ->getRobotAlgorithm()
                     ->inverseKinematics(qnear, p);
  printVec(std::get<0>(result1), "Inverse kinematics: optimal joint angles from TCP pose");

  // API call: get all inverse kinematics solutions based on TCP pose
  auto result2 = impl->getRobotInterface(robot_name)
                     ->getRobotAlgorithm()
                     ->inverseKinematicsAll(p);
  print2Vec(std::get<0>(result2), "Inverse kinematics: all joint angles from TCP pose");
}

void exampleInverseToolK(RpcClientPtr impl) {
  // API call: get robot name
  auto robot_name = impl->getRobotNames().front();

  // API call: set tcp offset
  std::vector<double> offset = {0.1, 0.2, 0.3, 0.0, 0.0, 0.0};
  impl->getRobotInterface(robot_name)->getRobotConfig()->setTcpOffset(offset);

  // API call: get current joint angles
  std::vector<double> qnear =
      impl->getRobotInterface(robot_name)->getRobotState()->getJointPositions();

  // API call: get current flange end center value as target pose
  std::vector<double> p =
      impl->getRobotInterface(robot_name)->getRobotState()->getToolPose();
  printVec(p, "Current flange end center pose");

  // API call: get optimal inverse kinematics solution based on flange end center pose
  auto result1 = impl->getRobotInterface(robot_name)
                     ->getRobotAlgorithm()
                     ->inverseToolKinematics(qnear, p);
  printVec(std::get<0>(result1),
           "Inverse kinematics: optimal joint angles from flange end center pose");

  // API call: get all inverse kinematics solutions based on flange end center pose
  auto result2 = impl->getRobotInterface(robot_name)
                     ->getRobotAlgorithm()
                     ->inverseToolKinematicsAll(p);
  print2Vec(std::get<0>(result2),
            "Inverse kinematics: all joint angles from flange end center pose");
}

#define LOCAL_IP "127.0.0.1"

int main(int argc, char **argv) {
#ifdef WIN32
  // Set Windows console output code page to UTF-8
  SetConsoleOutputCP(CP_UTF8);
#endif
  auto rpc_cli = std::make_shared<RpcClient>();
  // API call: set RPC timeout
  rpc_cli->setRequestTimeout(1000);
  // API call: connect to RPC service
  rpc_cli->connect(LOCAL_IP, 30004);
  // API call: login
  rpc_cli->login("aubo", "123456");

  // Forward kinematics: get TCP pose
  exampleForwardK(rpc_cli);

  // Forward kinematics: get flange end center pose
  exampleForwardToolK(rpc_cli);

  // Inverse kinematics: get joint angles from TCP pose
  exampleInverseK(rpc_cli);

  // Inverse kinematics: get joint angles from flange end center pose
  exampleInverseToolK(rpc_cli);

  // API call: logout
  rpc_cli->logout();
  // API call: disconnect
  rpc_cli->disconnect();

  return 0;
}
