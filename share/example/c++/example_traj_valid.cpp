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

// Check whether the given trajectory is valid, perform interpolation and inverse kinematics verification
bool exampleTrajectoryValid(RpcClientPtr impl, const std::vector<double> &p1,
                            const std::vector<double> &p2, int num_points)
{
    // If num_points is less than 2, interpolation cannot be performed
    if (num_points < 2) {
        throw std::invalid_argument("num_points must be at least 2");
    }

    // API call: Get the robot's name
    auto robot_name = impl->getRobotNames().front();

    // API call: Set tcp offset
    std::vector<double> offset = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    impl->getRobotInterface(robot_name)->getRobotConfig()->setTcpOffset(offset);

    // Calculate the alpha value for each interpolation point and call interpolatePose
    for (int i = 0; i < num_points; ++i) {
        double alpha =
            static_cast<double>(i) / (num_points - 1); // alpha varies evenly from 0 to 1

        // API call: Calculate linear interpolation
        auto pose = impl->getMath()->interpolatePose(p1, p2, alpha);

        // API call: Based on the calculated interpolated pose, check whether a valid inverse solution can be found
        auto result = impl->getRobotInterface(robot_name)
                          ->getRobotAlgorithm()
                          ->inverseKinematicsAll(pose);

        if (std::get<1>(result) != 0) {
            std::cout << "Inverse kinematics failed, inverseKinematicsAll return value:"
                      << std::get<1>(result) << std::endl;
            std::cout << "Trajectory planning failed" << std::endl;
            return false;
        }
    }

    // If all interpolation points are valid, trajectory planning is successful
    std::cout << "Trajectory planning succeeded" << std::endl;
    return true;
}

#define LOCAL_IP "127.0.0.1"

int main(int argc, char **argv)
{
#ifdef WIN32
    // Set Windows console output code page to UTF-8
    SetConsoleOutputCP(CP_UTF8);
#endif
    auto rpc_cli = std::make_shared<RpcClient>();
    // API call: Set RPC timeout, unit: ms
    rpc_cli->setRequestTimeout(1000);
    // API call: Connect to RPC service
    rpc_cli->connect(LOCAL_IP, 30004);
    // API call: Login
    rpc_cli->login("aubo", "123456");

    // Starting pose and target pose
    std::vector<double> pose1 = { 0.551, -0.295, 0.261, -3.135, 0.0, 1.569 };
    std::vector<double> pose2 = { 0.551, 0.295, 0.261, -3.135, 0.0, 0 };

    // Number of interpolation points
    int num_points = 30;

    // Check whether the given trajectory is valid, perform interpolation and inverse kinematics verification
    exampleTrajectoryValid(rpc_cli, pose1, pose2, num_points);

    // API call: Logout
    rpc_cli->logout();
    // API call: Disconnect
    rpc_cli->disconnect();

    return 0;
}
