#include "aubo_sdk/rpc.h"
#include "math.h"
#ifdef WIN32
#include <Windows.h>
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace arcs::common_interface;
using namespace arcs::aubo_sdk;

// clang-format off

// Template function: print the value and name of a single variable
template <typename T> void printSingle(T param, std::string name) {
  std::cout << name << ": " << param << std::endl;
}

// Template function: print variables of type std::vector<T>
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

// Template function: print variables of type std::vector<std::vector<T>>
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

void examplePoseAdd(RpcClientPtr impl) {
  std::vector<double> p1 = {0.2, 0.5, 0.1, 1.57, 0, 0};
  std::vector<double> p2 = {0.2, 0.5, 0.6, 1.57, 0, 0};

  // API call: pose addition
  auto result = impl->getMath()->poseAdd(p1, p2);

  printVec<double>(result, "poseAdd");
}

void examplePoseSub(RpcClientPtr impl) {
  std::vector<double> p1 = {0.2, 0.5, 0.1, 1.57, 0, 0};
  std::vector<double> p2 = {0.2, 0.5, 0.6, 1.57, 0, 0};

  // API call: pose subtraction
  auto result = impl->getMath()->poseSub(p1, p2);

  printVec<double>(result, "poseSub");
}

void exampleInterpolatePose(RpcClientPtr impl) {
  std::vector<double> p1 = {0.2, 0.2, 0.4, 0, 0, 0};
  std::vector<double> p2 = {0.2, 0.2, 0.6, 0, 0, 0};
  double alpha = 0.5;

  // API call: calculate linear interpolation
  auto result = impl->getMath()->interpolatePose(p1, p2, alpha);

  printVec<double>(result, "interpolatePose");
}

void examplePoseTrans(RpcClientPtr impl) {
  // Pose of B relative to A
  std::vector<double> F_B_A = {0.2, 0.5, 0.1, 1.57, 0, 0};
  // Pose of C relative to B
  std::vector<double> F_C_B = {0.2, 0.5, 0.6, 1.57, 0, 0};

  // API call: pose transformation to get pose of C relative to A
  auto F_C_A = impl->getMath()->poseTrans(F_B_A, F_C_B);

  printVec<double>(F_C_A, "poseTrans");
}

void examplePoseTransInv(RpcClientPtr impl) {
  // Pose of C relative to A
  std::vector<double> F_C_A = {0.4, -0.0996016, 0.600478, 3.14, 0, 0};
  // Pose of C relative to B
  std::vector<double> F_C_B = {0.2, 0.5, 0.6, 1.57, 0, 0};
  // API call: inverse pose transformation to get pose of B relative to A
  auto F_B_A = impl->getMath()->poseTransInv(F_C_A, F_C_B);

  printVec<double>(F_B_A, "poseTransInv");
}

void examplePoseInverse(RpcClientPtr impl) {
  std::vector<double> p = {0.2, 0.5, 0.1, 1.57, 0, 3.14};
  // API call: get inverse of pose
  auto result = impl->getMath()->poseInverse(p);

  printVec<double>(result, "poseInverse");
}

void examplePoseDistance(RpcClientPtr impl) {
  std::vector<double> p1 = {0.1, 0.3, 0.1, 0.3142, 0.0, 1.571};
  std::vector<double> p2 = {0.2, 0.5, 0.6, 0, -0.172, 0.0};

  // API call: get position distance between two poses
  auto result = impl->getMath()->poseDistance(p1, p2);

  printSingle(result, "poseDistance");
}

void examplePoseEqual(RpcClientPtr impl) {
  std::vector<double> p1 = {0.1, 0.3, 0.1, 0.3142, 0.0, 1.571};
  std::vector<double> p2 = {0.1, 0.3, 0.1, 0.3142, 0.0, 1.5711};
  // API call: check if two poses are equal
  auto result = impl->getMath()->poseEqual(p1, p2);
  printSingle(result, "poseEqual");
}

// RPY to quaternion
void exampleRpyToQuat(RpcClientPtr impl) {
  std::vector<double> rpy = {0.611, 0.785, 0.960};
  // API call: RPY to quaternion
  auto quat = impl->getMath()->rpyToQuaternion(rpy);

  printVec(quat, "RPY -> Quaternion");
}

// Quaternion to RPY
void exampleQuatToRpy(RpcClientPtr impl) {
  std::vector<double> quat = {0.834722, 0.0780426, 0.451893, 0.304864};
  // API call: quaternion to RPY
  auto rpy = impl->getMath()->quaternionToRpy(quat);

  printVec(rpy, "Quaternion -> RPY");
}

// Coordinate system calibration
void exampleCalibrateCoordinate(RpcClientPtr impl) {
  // API call: get robot name
  auto robot_name = impl->getRobotNames().front();

  std::vector<double> tcp_offset = {0.17734, 0.00233, 0.14682, 0.0, 0.0, 0.0};

  // API call: set TCP offset
  impl->getRobotInterface(robot_name)
      ->getRobotConfig()
      ->setTcpOffset(tcp_offset);

  // TCP pose in base coordinate system
  std::vector<double> coord_p0(6), coord_p1(6), coord_p2(6);
  coord_p0[0] = 0.55462;
  coord_p0[1] = 0.06219;
  coord_p0[2] = 0.37175;
  coord_p0[3] = -3.142;
  coord_p0[4] = 0.0;
  coord_p0[5] = 1.580;

  coord_p1[0] = 0.63746;
  coord_p1[1] = 0.11805;
  coord_p1[2] = 0.37175;
  coord_p1[3] = -3.142;
  coord_p1[4] = 0.0;
  coord_p1[5] = 1.580;

  coord_p2[0] = 0.40441;
  coord_p2[1] = 0.28489;
  coord_p2[2] = 0.37174;
  coord_p2[3] = -3.142;
  coord_p2[4] = 0.0;
  coord_p2[5] = 1.580;

  // API call: user coordinate system calibration
  // API call: get pose of user coordinate system relative to base coordinate system
  auto [user_on_base, ret] =
      impl->getMath()->calibrateCoordinate({coord_p0, coord_p1, coord_p2}, 0);

  // API call: get current TCP pose in base coordinate system
  auto tcp_on_base =
      impl->getRobotInterface(robot_name)->getRobotState()->getTcpPose();

  // API call: get inverse pose of user coordinate system relative to base coordinate system
  auto tcp_on_base_inv = impl->getMath()->poseInverse(user_on_base);

  // API call: get current TCP pose in user coordinate system
  auto tcp_on_user = impl->getMath()->poseTrans(tcp_on_base_inv, tcp_on_base);

  printVec(tcp_on_base, "TCP pose in base coordinate system");
  printVec(tcp_offset, "TCP offset");
  printVec(user_on_base, "User coordinate system pose relative to base coordinate system");
  printVec(tcp_on_user, "TCP pose in user coordinate system");
}

// Blocking function: program continues when the robot reaches the target waypoint
int waitArrival(RobotInterfacePtr impl) {
  const int max_retry_count = 5;
  int cnt = 0;

  // API call: get current motion command ID
  int exec_id = impl->getMotionControl()->getExecId();

  // Wait for robot to start moving
  while (exec_id == -1) {
    if (cnt++ > max_retry_count) {
      return -1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    exec_id = impl->getMotionControl()->getExecId();
  }

  // Wait for robot action to complete
  while (impl->getMotionControl()->getExecId() != -1) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  return 0;
}

// Calculate the midpoint of the other half of the circle arc
void exampleCalculateCircleFourthPoint(RpcClientPtr impl) {
  // API call: get robot name
  auto robot_name = impl->getRobotNames().front();

  std::vector<double> tcp_offset = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // API call: set TCP offset
  impl->getRobotInterface(robot_name)
      ->getRobotConfig()
      ->setTcpOffset(tcp_offset);

  // API call: set robot speed fraction
  impl->getRobotInterface(robot_name)
      ->getMotionControl()
      ->setSpeedFraction(0.3);

  std::vector<double> q = {0.00,
                           -10.43 / 180 * M_PI,
                           87.39 / 180 * M_PI,
                           7.82 / 180 * M_PI,
                           90.0 / 180 * M_PI,
                           0.0 / 180 * M_PI};

  // API call: joint motion to start position
  impl->getRobotInterface(robot_name)
      ->getMotionControl()
      ->moveJoint(q, 1.2, 1.0, 0, 0);
  // Blocking
  int ret = waitArrival(impl->getRobotInterface(robot_name));
  if (ret == 0) {
    std::cout << "Joint motion to initial position succeeded" << std::endl;
  } else {
    std::cout << "Joint motion to initial position failed" << std::endl;
  }

  std::vector<double> p1 = {0.5488696249770836,     -0.1214996547187204,
                            0.2631931199112321,     -3.14159198038469,
                            -3.673205103150083e-06, 1.570796326792424};

  std::vector<double> p2 = {0.5488696249770835,   -0.1214996547187207,
                            0.3599720701808493,   -3.14159198038469,
                            -3.6732051029273e-06, 1.570796326792423};

  std::vector<double> p3 = {0.5488696249770836,     -0.0389996547187214,
                            0.3599720701808496,     -3.141591980384691,
                            -3.673205102557476e-06, 1.570796326792422};

  // API call: linear motion to the first point of the circle
  impl->getRobotInterface(robot_name)
      ->getMotionControl()
      ->moveLine(p1, 1.2, 0.25, 0, 0);
  // Blocking
  ret = waitArrival(impl->getRobotInterface(robot_name));
  if (ret == 0) {
    std::cout << "Linear motion to the starting point of the circle succeeded" << std::endl;
  } else {
    std::cout << "Linear motion to the starting point of the circle failed" << std::endl;
  }

  // API call: calculate the midpoint of the other half of the circle arc
  auto [p4, retval] =
      impl->getMath()->calculateCircleFourthPoint(p1, p2, p3, 1);

  if (retval == 0) {
    std::cerr << "Failed to calculate the midpoint of the other half of the circle arc, cannot complete circular motion" << std::endl;
  } else {
    // API call: set to fixed mode
    impl->getRobotInterface(robot_name)
        ->getMotionControl()
        ->setCirclePathMode(0);

    // API call: perform circular arc motion
    impl->getRobotInterface(robot_name)
        ->getMotionControl()
        ->moveCircle(p2, p3, 1.2, 0.25, 0, 0);
    // Blocking
    ret = waitArrival(impl->getRobotInterface(robot_name));
    if (ret == -1) {
      std::cout << "Circular motion failed" << std::endl;
    }

    // API call: perform the other half of the circular arc motion
    impl->getRobotInterface(robot_name)
        ->getMotionControl()
        ->moveCircle(p4, p1, 1.2, 0.25, 0, 0);
    // Blocking
    ret = waitArrival(impl->getRobotInterface(robot_name));
    if (ret == 0) {
      std::cout << "Circular motion succeeded" << std::endl;
    } else {
      std::cout << "Circular motion failed" << std::endl;
    }
  }
}

#define LOCAL_IP "127.0.0.1"

int main(int argc, char **argv) {
#ifdef WIN32
  // Set Windows console output code page to UTF-8
  SetConsoleOutputCP(CP_UTF8);
#endif
  auto rpc_cli = std::make_shared<RpcClient>();
  // API call: set RPC timeout, unit: ms
  rpc_cli->setRequestTimeout(1000);
  // API call: connect to RPC service
  rpc_cli->connect(LOCAL_IP, 30004);
  // API call: login
  rpc_cli->login("aubo", "123456");

  // Pose addition
  examplePoseAdd(rpc_cli);

  // Pose subtraction
  examplePoseSub(rpc_cli);

  // Pose linear interpolation
  exampleInterpolatePose(rpc_cli);

  // Pose transformation
  examplePoseTrans(rpc_cli);

  // Inverse pose transformation
  examplePoseTransInv(rpc_cli);

  // Inverse of pose
  examplePoseInverse(rpc_cli);

  // Pose distance (excluding orientation)
  examplePoseDistance(rpc_cli);

  // Check if poses are equal
  examplePoseEqual(rpc_cli);

  // RPY to quaternion
  exampleRpyToQuat(rpc_cli);

  // Quaternion to RPY
  exampleQuatToRpy(rpc_cli);

  // Coordinate system calibration
  exampleCalibrateCoordinate(rpc_cli);

  // Calculate the midpoint of the other half of the circle arc
  //  exampleCalculateCircleFourthPoint(rpc_cli);

  // API call: logout
  rpc_cli->logout();
  // API call: disconnect
  rpc_cli->disconnect();

  return 0;
}
