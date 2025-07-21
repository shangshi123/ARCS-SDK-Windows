#include <mutex>
#include <fstream>
#include <math.h>
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
// Implement blocking functionality: The program continues only when the robot arm reaches the target waypoint
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
void runTraj(RpcClientPtr cli, std::vector<double> q1, std::vector<double> q2,
             std::vector<double> q3, std::string file_name)
{
    auto robot_name = cli->getRobotNames().front();
    cli->getRobotInterface(robot_name)
        ->getRobotManage()
        ->startRecord(file_name);
    cli->getRobotInterface(robot_name)
        ->getMotionControl()
        ->moveJoint(q1, 10 * (M_PI / 180), 5 * (M_PI / 180), 0, 0);
    waitArrivel(cli->getRobotInterface(robot_name));
    cli->getRobotInterface(robot_name)
        ->getMotionControl()
        ->moveJoint(q2, 10 * (M_PI / 180), 5 * (M_PI / 180), 0, 0);
    waitArrivel(cli->getRobotInterface(robot_name));
    cli->getRobotInterface(robot_name)
        ->getMotionControl()
        ->moveJoint(q3, 10 * (M_PI / 180), 5 * (M_PI / 180), 0, 0);
    waitArrivel(cli->getRobotInterface(robot_name));
    cli->getRobotInterface(robot_name)
        ->getMotionControl()
        ->moveJoint(q2, 10 * (M_PI / 180), 5 * (M_PI / 180), 0, 0);
    waitArrivel(cli->getRobotInterface(robot_name));
    cli->getRobotInterface(robot_name)
        ->getMotionControl()
        ->moveJoint(q1, 10 * (M_PI / 180), 5 * (M_PI / 180), 0, 0);
    waitArrivel(cli->getRobotInterface(robot_name));
    cli->getRobotInterface(robot_name)->getRobotManage()->stopRecord();
}

int confirmPayloadIdentifyTraj(RpcClientPtr cli, std::vector<double> point1,
                               std::vector<double> point2,
                               std::vector<double> point3)
{
    auto robot_name = cli->getRobotNames().front();
    std::string file_name = "payload_test_data.csv";
    std::string file_path = "/root/arcs_ws/log/.trace/" + file_name;
    std::ofstream file(file_path, std::ios::out | std::ios::trunc);
    if (!file.is_open()) {
        std::cerr << "file open fail" << std::endl;
        exit(-1);
    }

    auto dof = cli->getRobotInterface(robot_name)->getRobotConfig()->getDof();
#define VEC(name, realdof, judgedof)                                        \
    (realdof == judgedof                                                    \
         ? #name "1," #name "2," #name "3," #name "4," #name "5," #name "6" \
         : #name "1," #name "2," #name "3," #name "4," #name "5," #name     \
                 "6," #name "7")
    // To match the diagnostic file format, add the Time header
    file << "Time"
         << ",";
    file << VEC(q, dof, 6) << ",";
    file << VEC(qd, dof, 6) << ",";
    file << VEC(qdd, dof, 6) << ",";
    file << VEC(current, dof, 6) << ",";
    file << VEC(temperature, dof, 6) << ",";
    file << std::endl;
#undef VEC
    std::vector<double> qd(dof, 0.);
    std::vector<double> qdd(dof, 0.);
    std::vector<double> current(dof, 0.);
    std::vector<double> temperature(dof, 0.);

    auto q1_q2 = cli->getRobotInterface(robot_name)
                     ->getRobotAlgorithm()
                     ->pathMovej(point1, 0., point2, 0., 0.001);
    std::cout << "q1_q2.size: " << q1_q2.size() << std::endl;
    for (size_t i = 0; i < q1_q2.size(); i++) {
        // Write time data
        file << 0 << ",";
        file << q1_q2[i] << ",";
        file << qd << ",";
        file << qdd << ",";
        file << current << ",";
        file << temperature << ",";
        file << std::endl;
    }
    auto q2_q3 = cli->getRobotInterface(robot_name)
                     ->getRobotAlgorithm()
                     ->pathMovej(point2, 0., point3, 0., 0.001);
    std::cout << "q2_q3.size: " << q2_q3.size() << std::endl;
    for (size_t i = 0; i < q2_q3.size(); i++) {
        // Write time data
        file << 0 << ",";
        file << q2_q3[i] << ",";
        file << qd << ",";
        file << qdd << ",";
        file << current << ",";
        file << temperature << ",";
        file << std::endl;
    }
    // Concatenate forward and reverse trajectory data
    for (std::vector<std::vector<double>>::iterator it = q2_q3.end() - 1;
         it >= q2_q3.begin(); it--) {
        // Write time data
        file << 0 << ",";
        file << *it << ",";
        file << qd << ",";
        file << qdd << ",";
        file << current << ",";
        file << temperature << ",";
        file << std::endl;
    }
    for (std::vector<std::vector<double>>::iterator it = q1_q2.end() - 1;
         it >= q1_q2.begin(); it--) {
        // Write time data
        file << 0 << ",";
        file << *it << ",";
        file << qd << ",";
        file << qdd << ",";
        file << current << ",";
        file << temperature << ",";
        file << std::endl;
    }

    file.close();

    auto ret = cli->getRobotInterface(robot_name)
                   ->getRobotAlgorithm()
                   ->payloadIdentify1(file_name);

    // The pre-screening stage takes about 1 second in total, you can remove the status fetching below
    while (1) {
        auto ret = cli->getRobotInterface(robot_name)
                       ->getRobotAlgorithm()
                       ->payloadCalculateFinished();
        if (ret == 0) {
            std::cout << "Pre-screening succeeded, trajectory points are available!" << std::endl;
            return ret;
        } else if (ret == 1) {
            std::cout << "Pre-screening in progress..." << std::endl;
        } else {
            std::cout << "Pre-screening failed, trajectory points are not available, ret=" << ret << std::endl;
            return ret;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    return ret;
}

void examplePayloadIdentify1(RpcClientPtr cli)
{
    std::vector<double> point1 = { -0.261799, 0.261799, 1.309,
                                   1.0472,    1.39626,  0.0 };
    std::vector<double> point2 = { -0.628319, 0.471239, 1.65806,
                                   -0.471239, 0.0,      0.0 };
    std::vector<double> point3 = { -0.628319, 0.366519, 1.74533,
                                   -0.10472,  1.5708,   0.0 };

    // Pre-screening
    if (confirmPayloadIdentifyTraj(cli, point1, point2, point3) != 0) {
        exit(1);
    } else {
        std::string file = "test_data.csv";
        runTraj(cli, point1, point2, point3, file);
        auto robot_name = cli->getRobotNames().front();
        auto ret = cli->getRobotInterface(robot_name)
                       ->getRobotAlgorithm()
                       ->payloadIdentify1(file);
        while (1) {
            auto ret = cli->getRobotInterface(robot_name)
                           ->getRobotAlgorithm()
                           ->payloadCalculateFinished();
            if (ret == 0) {
                std::cout << "Payload calculation completed!" << std::endl;
                break;
            } else if (ret == 1) {
                std::cout << "Payload calculation in progress..." << std::endl;
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
}

/**
 * Function: Payload identification
 * Steps:
 * Step 1: Connect to RPC service and log in
 * Step 2: Power on the robot arm
 * Step 3: Set three target points
 * Step 4: Move joints to target points, collect joint position, joint velocity, joint acceleration, joint current
 * Step 5: Calculate identification result
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
    // API call: Connect to RPC service
    rpc_cli->connect(LOCAL_IP, 30004);
    // API call: Log in
    rpc_cli->login("aubo", "123456");

    auto rtde_cli = std::make_shared<RtdeClient>();
    // API call: Connect to RTDE service
    rtde_cli->connect(LOCAL_IP, 30010);
    // API call: Log in
    rtde_cli->login("aubo", "123456");

    examplePayloadIdentify1(rpc_cli);
}
