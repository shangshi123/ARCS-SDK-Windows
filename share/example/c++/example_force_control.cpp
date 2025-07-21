#include <math.h>
#include "aubo_sdk/rpc.h"
#include <fstream>
#ifdef _WIN32
#include <Windows.h>
#endif

using namespace arcs::common_interface;
using namespace arcs::aubo_sdk;
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define EMBEDDED
std::ofstream file("force.csv", std::ios::app);

// Implement blocking functionality: The program continues only after the robot reaches the target waypoint
int waitArrival(RobotInterfacePtr impl)
{
    int cnt = 0;
    while (impl->getMotionControl()->getExecId() == -1) {
        if (cnt++ > 5) {
            std::cout << "Motion fail!" << std::endl;
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    auto id = impl->getMotionControl()->getExecId();
    while (1) {
        auto id1 = impl->getMotionControl()->getExecId();
        if (id != id1) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return 0;
}

template <typename T>
inline std::ostream &operator<<(std::ostream &os, const std::vector<T> &list)
{
    for (size_t i = 0; i < list.size(); i++) {
        os << list.at(i);
        if (i != (list.size() - 1)) {
            os << ",";
        }
    }
    return os;
}

// Calculate Euclidean distance between two points, default second point is origin
inline double calculateDistance(
    const std::vector<double> &p1,
    const std::vector<double> &p2 = std::vector<double>(6, 0))
{
    if ((6 == p1.size()) && (6 == p2.size())) {
        double sum = 0.;
        for (int i = 0; i < 3; i++) {
            sum += pow(p1[i] - p2[i], 2);
        }
        return std::sqrt(sum);
    }
    return 0;
}

// Read force sensor data and save to force.csv file
void tcpSensorTest(RpcClientPtr cli)
{
    // API call: Get robot name
    auto robot_name = cli->getRobotNames().front();
#ifdef EMBEDDED
    // Set force sensor type to embedded sensor
    std::vector<double> sensor_pose = { 0, 0, 0, 0, 0, 0 };
    cli->getRobotInterface(robot_name)
        ->getRobotConfig()
        ->selectTcpForceSensor("embedded");
#else
    // Set force sensor type to external KW sensor
    std::vector<double> sensor_pose = { 0, 0, 0.047, 0, 0, 0 };
    cli->getRobotInterface(robot_name)
        ->getRobotConfig()
        ->selectTcpForceSensor("kw_ftsensor");
#endif
    while (1) {
        auto sensor_data = cli->getRobotInterface(robot_name)
                               ->getRobotState()
                               ->getTcpForceSensors();

        std::cout << "--------------------------------------" << std::endl;
        for (int i = 0; i < sensor_data.size(); i++) {
            std::cout << "Get TCP force sensor reading: " << i + 1 << ": "
                      << sensor_data[i] << std::endl;
        }
        std::cout << "--------------------------------------" << std::endl;
        file << sensor_data << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

// Force sensor calibration
ForceSensorCalibResult tcpSensorcalibration(
    RpcClientPtr cli, std::vector<std::vector<double>> joints)
{
    auto robot_name = cli->getRobotNames().front();

    printf("goto p0\n");
    // Move joints to the first reference point for load identification
    cli->getRobotInterface(robot_name)
        ->getMotionControl()
        ->moveJoint(joints[0], 10 * (M_PI / 180), 5 * (M_PI / 180), 0, 0);
    // Blocking
    waitArrival(cli->getRobotInterface(robot_name));
    // Wait for robot to stabilize
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Record joint position and TCP force sensor data at the first reference point
    auto q1 = cli->getRobotInterface(robot_name)
                  ->getRobotState()
                  ->getJointPositions();
    auto tcp_force1 = cli->getRobotInterface(robot_name)
                          ->getRobotState()
                          ->getTcpForceSensors();

    printf("goto p1\n");
    // Move joints to the second reference point for load identification
    cli->getRobotInterface(robot_name)
        ->getMotionControl()
        ->moveJoint(joints[1], 20 * (M_PI / 180), 10 * (M_PI / 180), 0, 0);
    // Blocking
    waitArrival(cli->getRobotInterface(robot_name));
    // Wait for robot to stabilize
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Record joint position and TCP force sensor data at the second reference point
    auto q2 = cli->getRobotInterface(robot_name)
                  ->getRobotState()
                  ->getJointPositions();
    auto tcp_force2 = cli->getRobotInterface(robot_name)
                          ->getRobotState()
                          ->getTcpForceSensors();

    printf("goto p2\n");
    // Move joints to the third reference point for load identification
    cli->getRobotInterface(robot_name)
        ->getMotionControl()
        ->moveJoint(joints[2], 20 * (M_PI / 180), 10 * (M_PI / 180), 0, 0);
    // Blocking
    waitArrival(cli->getRobotInterface(robot_name));
    // Wait for robot to stabilize
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Record joint position and TCP force sensor data at the third reference point
    auto q3 = cli->getRobotInterface(robot_name)
                  ->getRobotState()
                  ->getJointPositions();
    auto tcp_force3 = cli->getRobotInterface(robot_name)
                          ->getRobotState()
                          ->getTcpForceSensors();

    // Forward kinematics to get poses of three reference points
    auto pose1 = cli->getRobotInterface(robot_name)
                     ->getRobotAlgorithm()
                     ->forwardKinematics(q1);

    auto pose2 = cli->getRobotInterface(robot_name)
                     ->getRobotAlgorithm()
                     ->forwardKinematics(q2);

    auto pose3 = cli->getRobotInterface(robot_name)
                     ->getRobotAlgorithm()
                     ->forwardKinematics(q3);

    std::vector<std::vector<double>> calib_forces{ tcp_force1, tcp_force2,
                                                   tcp_force3 };
    std::vector<std::vector<double>> calib_poses{ std::get<0>(pose1),
                                                  std::get<0>(pose2),
                                                  std::get<0>(pose3) };
    // Force sensor calibration algorithm (three-point calibration method)
    // Pass sensor data and poses of three reference points to this API for load identification
    auto result = cli->getRobotInterface(robot_name)
                      ->getRobotAlgorithm()
                      ->calibrateTcpForceSensor(calib_forces, calib_poses);
    return result;
}

void exampleForceControl(RpcClientPtr cli)
{
    auto robot_name = cli->getRobotNames().front();
#ifdef EMBEDDED
    // Set force sensor type to embedded sensor
    std::vector<double> sensor_pose = { 0, 0, 0, 0, 0, 0 };
    cli->getRobotInterface(robot_name)
        ->getRobotConfig()
        ->selectTcpForceSensor("embedded");
#else
    // Set force sensor type to external KW sensor
    std::vector<double> sensor_pose = { 0, 0, 0.047, 0, 0, 0 };
    cli->getRobotInterface(robot_name)
        ->getRobotConfig()
        ->selectTcpForceSensor("kw_ftsensor");
#endif

    // Set sensor installation pose
    cli->getRobotInterface(robot_name)
        ->getRobotConfig()
        ->setTcpForceSensorPose(sensor_pose);

    // Set TCP offset
    std::vector<double> tcp_pose = sensor_pose;
    cli->getRobotInterface(robot_name)
        ->getRobotConfig()
        ->setTcpOffset(tcp_pose);

    // Reference points for load identification
    std::vector<double> joint1 = { -0.261799, 0.261799, 1.309,
                                   1.0472,    1.39626,  0.0 };
    std::vector<double> joint2 = { -0.628319, 0.471239, 1.65806,
                                   -0.471239, 0.0,      0.0 };
    std::vector<double> joint3 = { -0.628319, 0.366519, 1.74533,
                                   -0.10472,  1.5708,   0.0 };
    // Force sensor calibration
    auto calib_result = tcpSensorcalibration(cli, { joint1, joint2, joint3 });
    std::cout << "force_offset: " << std::get<0>(calib_result) << std::endl;
    std::cout << "com: " << std::get<1>(calib_result) << std::endl;
    std::cout << "mass: " << std::get<2>(calib_result) << std::endl;

    if (calculateDistance(std::get<0>(calib_result)) < 0.0001) {
        std::cout << "Calibration error, please check sensor data!" << std::endl;
        exit(-1);
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::cout << "go to start_joint" << std::endl;
    std::vector<double> start_joint = { 0 / 180 * M_PI,     16.41 / 180 * M_PI,
                                        76.36 / 180 * M_PI, 7.87 / 180 * M_PI,
                                        90.21 / 180 * M_PI, 0 / 180 * M_PI };
    // Move joints to starting position
    cli->getRobotInterface(robot_name)
        ->getMotionControl()
        ->moveJoint(start_joint, 10 * (M_PI / 180), 5 * (M_PI / 180), 0, 0);
    waitArrival(cli->getRobotInterface(robot_name));
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // (Use the result of load identification to) set payload
    cli->getRobotInterface(robot_name)
        ->getRobotConfig()
        ->setPayload(std::get<2>(calib_result), std::get<1>(calib_result),
                     { 0. }, { 0. });
    // Set force sensor offset
    cli->getRobotInterface(robot_name)
        ->getRobotConfig()
        ->setTcpForceOffset(std::get<0>(calib_result));

    // Admittance mass
    std::vector<double> admittance_m = { 10.0, 10.0, 10.0, 2.0, 2.0, 2.0 };
    // Admittance damping
    // Used to adjust the speed during force control motion; higher damping means lower speed under the same force
    std::vector<double> admittance_d = {
        200.0, 200.0, 200.0, 20.0, 20.0, 20.0
    };
    // Admittance stiffness
    // Used to adjust robot elasticity; higher stiffness means faster rebound
    std::vector<double> admittance_k = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    // Set force control dynamic model
    cli->getRobotInterface(robot_name)
        ->getForceControl()
        ->setDynamicModel(admittance_m, admittance_d, admittance_k);

    // Compliance axis (direction) selection
    std::vector<bool> compliance = { true, true, true, true, true, true };
    // Target force/torque
    std::vector<double> target_wrench(6, 0.);
    // Speed limits
    std::vector<double> speed_limits(6, 2.0);
    // Set force control reference (target) value.
    cli->getRobotInterface(robot_name)
        ->getForceControl()
        ->setTargetForce(std::vector<double>(6, 0.), compliance, target_wrench,
                         speed_limits, TaskFrameType::NONE);

    std::cout << "Press the key 's'/'q' to enable/disable force control mode."
              << std::endl;
    while (1) {
        std::cout << "Please input your choose: " << std::endl;
        char key;
        std::cin >> key;
        switch (key) {
        case 's':
            if (cli->getRobotInterface(robot_name)
                    ->getForceControl()
                    ->isFcEnabled()) {
                std::cout << "The robot has already been force control mode. "
                             "Can't enable force control mode"
                          << std::endl;
                break;
            } else {
                // Enable force control
                cli->getRobotInterface(robot_name)
                    ->getForceControl()
                    ->fcEnable();
                std::cout << "Enter force control mode" << std::endl;
            }
            break;
        case 'q':
            if (!cli->getRobotInterface(robot_name)
                     ->getForceControl()
                     ->isFcEnabled()) {
                std::cout << "The robot has already quit force control mode. "
                             "Can't disable force control mode"
                          << std::endl;
                break;
            } else {
                // Disable force control
                cli->getRobotInterface(robot_name)
                    ->getForceControl()
                    ->fcDisable();
                std::cout << "Quit force control mode" << std::endl;
            }
            break;
        default:
            std::cout << "Please input 's' or 'q'." << std::endl;
            break;
        }
    }
}

#define LOCAL_IP "127.0.0.1"
int main(int argc, char **argv)
{
#ifdef _WIN32
    // Set Windows console output code page to UTF-8
    SetConsoleOutputCP(CP_UTF8);
#endif
    auto rpc_cli = std::make_shared<RpcClient>();
    rpc_cli->setRequestTimeout(1000);
    // API call: Connect to RPC service
    rpc_cli->connect(LOCAL_IP, 30004);
    // API call: Login
    rpc_cli->login("aubo", "123456");

    // Enable and disable force control
    exampleForceControl(rpc_cli);

    // Read force sensor data and save to force.csv file
    //    tcpSensorTest(rpc_cli);
}