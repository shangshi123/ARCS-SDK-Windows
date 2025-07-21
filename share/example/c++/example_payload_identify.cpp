#include <mutex>
#include <fstream>
#include "aubo_sdk/rpc.h"
#include "aubo_sdk/rtde.h"
#ifdef WIN32
#include <windows.h>
#endif

using namespace arcs::common_interface;
using namespace arcs::aubo_sdk;
std::mutex rtde_mtx_;

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

// Implement blocking functionality: When the robot arm moves to the target waypoint, the program continues execution
int waitArrivel(RobotInterfacePtr impl)
{
    int cnt = 0;
    while (impl->getMotionControl()->getExecId() == -1) {
        if (cnt++ > 5) {
            std::cout << "Motion fail!" << std::endl;
            exit(-1);
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

void waitMovePathBufferFinished(RobotInterfacePtr impl)
{
    while (impl->getMotionControl()->getExecId() == -1) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    while (1) {
        auto id = impl->getMotionControl()->getExecId();
        if (id == -1) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void generateTraj(RpcClientPtr cli, TrajConfig &traj_conf)
{
    // API call: Get the robot's name
    auto robot_name = cli->getRobotNames().front();

    cli->getRobotInterface(robot_name)
        ->getRobotAlgorithm()
        ->generatePayloadIdentifyTraj("identify_traj", traj_conf);
    while (1) {
        auto ret = cli->getRobotInterface(robot_name)
                       ->getRobotAlgorithm()
                       ->payloadIdentifyTrajGenFinished();
        if (ret == 0) {
            std::cout << "Payload identification trajectory generation completed!" << std::endl;
            break;
        } else if (ret == 1) {
            std::cout << "Payload identification trajectory is being generated..." << std::endl;
        } else {
            std::cout << "Trajectory generation failed, ret=" << ret << std::endl;
            exit(-1);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    if (cli->getRobotInterface(robot_name)
            ->getMotionControl()
            ->pathBufferEval("identify_traj", {}, {}, 0.005) < 0) {
        std::cout << "pathBufferEval error!" << std::endl;
        exit(-1);
    }

    while (!cli->getRobotInterface(robot_name)
                ->getMotionControl()
                ->pathBufferValid("identify_traj")) {
        std::cout << "Trajectory optimization in progress..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    std::cout << "Payload identification trajectory generation completed!" << std::endl;
}

void runTraj(RpcClientPtr cli, RtdeClientPtr rtde_cli,
             const std::string &data_file_name, TrajConfig &traj_conf)
{
    std::ofstream file(data_file_name, std::ios::out | std::ios::trunc);
    auto robot_name = cli->getRobotNames().front();
    cli->getRobotInterface(robot_name)
        ->getMotionControl()
        ->moveJoint(traj_conf.init_joint, 5.0, 3.0, 0, 0);
    waitArrivel(cli->getRobotInterface(robot_name));
    cli->getRobotInterface(robot_name)
        ->getMotionControl()
        ->movePathBuffer("identify_traj");
    cli->getRobotInterface(robot_name)
        ->getRobotManage()
        ->startRecord(data_file_name);
    waitMovePathBufferFinished(cli->getRobotInterface(robot_name));
    std::cout << "Trajectory execution completed" << std::endl;
    cli->getRobotInterface(robot_name)->getRobotManage()->stopRecord();
}

void examplePayloadIdentify(RpcClientPtr cli, RtdeClientPtr rtde_cli)
{
    // Generate excitation trajectory
    TrajConfig traj_conf;
    traj_conf.move_axis = PayloadIdentifyMoveAxis::Joint_3_6;
    traj_conf.max_velocity = { 3.0, 3.0 }; // Dimension matches the number of moving joints
    traj_conf.max_acceleration = { 5.0, 5.0 }; // Dimension matches the number of moving joints
    traj_conf.lower_joint_bound = { -1.5, -2 };
    traj_conf.upper_joint_bound = { 1.5, 2 };
    traj_conf.init_joint = { -0.0, -0.2618, 1.047, -0.1745, 1.57, 0.0 };
    generateTraj(cli, traj_conf);
    std::cout << "Please remove the payload, press Enter to start running the trajectory:" << std::endl;
    std::cin.get();
    runTraj(cli, rtde_cli, "data_no_payload.csv", traj_conf);
    std::cout << "Please install the payload, press Enter to start running the trajectory:" << std::endl;
    std::cin.get();
    runTraj(cli, rtde_cli, "data_with_payload.csv", traj_conf);

    auto robot_name = cli->getRobotNames().front();
    auto ret =
        cli->getRobotInterface(robot_name)
            ->getRobotAlgorithm()
            ->payloadIdentify("data_no_payload.csv", "data_with_payload.csv");
    while (1) {
        auto ret = cli->getRobotInterface(robot_name)
                       ->getRobotAlgorithm()
                       ->payloadCalculateFinished();
        if (ret == 0) {
            std::cout << "Payload calculation completed!" << std::endl;
            break;
        } else if (ret == 1) {
            std::cout << "Payload is being calculated..." << std::endl;
        } else {
            std::cout << "Payload calculation failed, ret=" << ret << std::endl;
            exit(-1);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    auto result = cli->getRobotInterface(robot_name)
                      ->getRobotAlgorithm()
                      ->getPayloadIdentifyResult();
    std::cout << "Calculation result:" << std::endl;
    std::cout << "mass: " << std::get<0>(result) << std::endl;
    std::cout << "com: " << std::get<1>(result) << std::endl;
    std::cout << "inertia: " << std::get<3>(result) << std::endl;
}

/**
 * Function: Payload identification
 * Steps:
 * Step 1: Connect to the RPC service and log in
 * Step 2: Power on the robot arm
 * Step 3: Set the limits for the excitation trajectory and generate the excitation trajectory
 * Step 4: Run the offline trajectory without payload, collect joint position, joint velocity, and joint current via rtde
 * Step 5: Run the offline trajectory with payload, collect joint position, joint velocity, and joint current via rtde
 * Step 6: Calculate the identification result
 */
#define LOCAL_IP "127.0.0.1"

int main(int argc, char **argv)
{
#ifdef WIN32
    // Set Windows console output code page to UTF-8
    SetConsoleOutputCP(CP_UTF8);
#endif

    auto rpc_cli = std::make_shared<RpcClient>();
    rpc_cli->setRequestTimeout(1000);
    // API call: Connect to the RPC service
    rpc_cli->connect(LOCAL_IP, 30004);
    // API call: Log in
    rpc_cli->login("aubo", "123456");

    auto rtde_cli = std::make_shared<RtdeClient>();
    // API call: Connect to the RTDE service
    rtde_cli->connect(LOCAL_IP, 30010);
    // API call: Log in
    rtde_cli->login("aubo", "123456");

    examplePayloadIdentify(rpc_cli, rtde_cli);
}
