#include <math.h>
#include <thread>
#include "aubo_sdk/rpc.h"

using namespace arcs::common_interface;
using namespace arcs::aubo_sdk;

/**
 * Function: Set kinematic model compensation parameters and get DH parameters
 * Steps:
 * Step 1: Connect to RPC service and log in to the robot
 * Step 2: Set kinematic model compensation parameters
 * Step 3: Get robot DH parameters
 */
#define LOCAL_IP "127.0.0.1"

int main(int argc, char **argv)
{
    auto rpc_cli = std::make_shared<RpcClient>();
    // API call: connect to RPC service
    rpc_cli->connect(LOCAL_IP, 30004);
    // API call: log in
    rpc_cli->login("aubo", "123456");

    // API call: get robot name
    auto name = rpc_cli->getRobotNames().front();
    auto impl = rpc_cli->getRobotInterface(name);

    // Kinematic model compensation data
    std::string value =
        "[dh_comp]\r\n"
        "alpha = [0.0, 0.000886627, -0.00305258, 0.00168948, 0.000534071, "
        "0.00557807]\r\n"
        "a = [0.0, -8.72e-05, 0.000741, -0.0001693, -0.0001688, 1.82e-05]\r\n"
        "d = [-0.0235, 0.0001258, -0.0001258, 0.0001258, 0.000586, "
        "-0.0005141]\r\n"
        "theta = [0.0, 0.000439823, -0.0181933, -0.0217206, -0.0178146, "
        "0.0]\r\n"
        "beta  = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]\r\n";

    // API call: set kinematic model compensation parameters
    impl->getRobotConfig()->setPersistentParameters(value);

    std::this_thread::sleep_for(std::chrono::seconds(10));

    // API call: get robot DH parameters
    auto dh = impl->getRobotConfig()->getKinematicsParam(true);
    for (auto iter : dh) {
        std::cout << iter.first << ":";
        for (int i = 0; i < (int)iter.second.size(); i++) {
            std::cout << iter.second.at(i) << ",";
        }
        std::cout << std::endl;
    }
}
