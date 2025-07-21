#include "aubo_sdk/rpc.h"
#ifdef WIN32
#include <windows.h>
#endif

#include "unistd.h"
#include <cstring>
#include <fstream>

using namespace arcs::common_interface;
using namespace arcs::aubo_sdk;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

template <typename T>
inline std::ostream &operator<<(std::ostream &os, const std::vector<T> &vd)
{
    if (vd.size() == 0) {
        os << "<none>";
        return os;
    }
    for (size_t i = 0; i < vd.size(); i++) {
        os << vd[i];
        if (i < vd.size() - 1) {
            os << ", ";
        }
    }
    return os;
}

// Implement blocking functionality: The program continues only when the robot arm reaches the target waypoint
int waitArrival(const std::vector<AxisInterfacePtr> &axis,
                const std::vector<std::string> names,
                const std::vector<double> way)
{
    int cnt_steady = 0;
    int cnt = 0;

    while (true) {
        for (size_t i = 0; i < names.size(); i++) {
            std::cout << "real pos: " << axis[i]->getExtAxisPosition()
                      << " cmd pos: " << way[i] << std::endl;
            if (std::abs(axis[i]->getExtAxisPosition() - way[i]) < 10e-5) {
                cnt_steady++;
            }
        }
        if (cnt_steady == (int)names.size()) {
            return 0;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        if (cnt++ > 1000) {
            cnt = 0;
            return -1;
        }
    }

    return 0;
}

/**
 * Function: Robot arm joint movement
 * Steps:
 * Step 1: Set RPC timeout, connect to RPC service, robot arm login
 * Step 2: Set motion speed ratio, pass through 3 waypoints in joint motion mode
 * Step 3: RPC logout, disconnect
 */

void exampleMovej(RpcClientPtr cli)
{
    std::vector<double> servo_r_j(6, 0.0);
    std::vector<double> servo_ex_j(3, 0.0);
    std::vector<double> way1(3, 0.0);

    std::vector<AxisInterfacePtr> axis;
    auto names = cli->getAxisNames();

    axis.resize(names.size());
    for (size_t i = 0; i < names.size(); i++) {
        axis[i] = cli->getAxisInterface(names[i]);
    }

    printf("name: %s\n", names[0].c_str());
    axis[1]->followAnotherAxis(names[0], 0.0, 0.0);
    std::cout << "axis: " << axis.size() << std::endl;
    std::cout << "names: " << names.size() << std::endl;

    for (;;) {
        for (size_t i = 0; i < names.size(); i++) {
            way1[i] = -3.0;
            axis[i]->moveExtJoint(way1[i], 3.0, 1.0, 0.5);
        }

        int ret = waitArrival(axis, names, way1);
        if (ret == 0) {
            std::cout << "Joint movement to waypoint 1 succeeded" << std::endl;
        } else {
            std::cout << "Joint movement to waypoint 1 failed" << std::endl;
        }

        for (size_t i = 0; i < names.size(); i++) {
            way1[i] = 3.0;
            axis[i]->moveExtJoint(way1[i], 3.0, 1.0, 0.5);
        }

        ret = waitArrival(axis, names, way1);
        if (ret == 0) {
            std::cout << "Joint movement to waypoint 2 succeeded" << std::endl;
        } else {
            std::cout << "Joint movement to waypoint 2 failed" << std::endl;
        }
    }
}

#define LOCAL_IP "172.19.19.112"

int main(int argc, char **argv)
{
#ifdef WIN32
    // Set Windows console output code page to UTF-8
    SetConsoleOutputCP(CP_UTF8);
#endif

    auto rpc_cli = std::make_shared<RpcClient>();
    // API call: Set RPC timeout
    rpc_cli->setRequestTimeout(1000);
    // API call: Connect to RPC service

    rpc_cli->connect("172.19.19.112", 30004);
    // API call: Login
    rpc_cli->login("aubo", "123456");

    // Joint movement
    exampleMovej(rpc_cli);

    // API call: Logout
    rpc_cli->logout();
    // API call: Disconnect
    rpc_cli->disconnect();

    return 0;
}
