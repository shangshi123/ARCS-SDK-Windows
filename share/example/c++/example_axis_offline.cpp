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

class TrajectoryIo
{
public:
    // Constructor, accepts the filename to open as a parameter
    TrajectoryIo(const char *filename)
    {
        input_file_.open(filename, std::ios::in);
    }

    // Check if the file was successfully opened
    bool open()
    {
        if (!input_file_.is_open()) {
            std::cerr << "Unable to open trajectory file. Please check if the input file path is correct."
                      << std::endl;
            return false;
        }
        return true;
    }
    ~TrajectoryIo() { input_file_.close(); }

    // Parse trajectory data from the file,
    // and convert it into a two-dimensional std::vector.
    // It reads the file line by line, parses each line into a set of double values,
    // and stores these values in a nested two-dimensional vector.
    std::vector<std::vector<double>> parse()
    {
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
    std::vector<double> split(const std::string &str, const char *delim)
    {
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

// Implement blocking: The program continues only when the robot arm reaches the target waypoint
int waitArrival(RobotInterfacePtr impl)
{
    // API call: Get current motion command ID
    int exec_id = impl->getMotionControl()->getExecId();

    int cnt = 0;
    // Maximum retry count for getting exec_id while waiting for the robot arm to start moving
    int max_retry_count = 50;

    // Wait for the robot arm to start moving
    while (exec_id == -1) {
        if (cnt++ > max_retry_count) {
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        exec_id = impl->getMotionControl()->getExecId();
    }

    // Wait for the robot arm action to complete
    while (exec_id != -1) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        exec_id = impl->getMotionControl()->getExecId();
    }

    return 0;
}

// Implement blocking: The program continues only when the robot arm reaches the target waypoint
int waitAxesArrival(const std::vector<AxisInterfacePtr> &axis,
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

    auto robot_name = rpc_cli->getRobotNames().front();

    auto robot_interface = rpc_cli->getRobotInterface(robot_name);
    // API call: Set robot arm speed fraction
    robot_interface->getMotionControl()->setSpeedFraction(1);

    auto filename = "../../example/c++/trajs/waypoints_dof-9_0-90-1.offt";
    TrajectoryIo input(filename);

    // Try to open trajectory file, return directly if unable to open
    if (!input.open()) {
        return 0;
    }

    // Parse trajectory data
    auto traj = input.parse();

    // Check if there are waypoints in the trajectory file,
    // If the number is 0, output error message and return
    auto traj_sz = traj.size();
    if (traj_sz == 0) {
        std::cerr << "Number of waypoints in trajectory file is 0." << std::endl;
        return 0;
    }

    std::vector<double> servo_r_j(6, 0.0);
    std::vector<double> servo_ex_j(3, 0.0);

    std::vector<AxisInterfacePtr> axis;
    auto names = rpc_cli->getAxisNames();

    axis.resize(names.size());
    for (size_t i = 0; i < names.size(); i++) {
        axis[i] = rpc_cli->getAxisInterface(names[i]);
    }

    printf("name: %s\n", names[0].c_str());
    axis[1]->followAnotherAxis(names[0], 0.0, 0.0);

    for (int i = 0; i < 6; i++) {
        servo_r_j[i] = traj[0][i];
    }

    for (int i = 0; i < 3; i++) {
        servo_ex_j[i] = traj[0][i + 6];
    }

    for (size_t i = 0; i < names.size(); i++) {
        // servo_ex_j[i] = -3.0;
        axis[i]->moveExtJoint(servo_ex_j[i], 1.0, 1.0, 0.5);
    }

    int ret = waitAxesArrival(axis, names, servo_ex_j);
    if (ret == 0) {
        std::cout << "External axis joint moved to waypoint 1 successfully" << std::endl;
    } else {
        std::cout << "External axis joint failed to move to waypoint 1" << std::endl;
    }

    // API call: Move joints to the first waypoint in the trajectory file
    robot_interface->getMotionControl()->moveJoint(servo_r_j, 30 * (M_PI / 180),
                                                   30 * (M_PI / 180), 0., 0.);

    // Blocking
    ret = waitArrival(robot_interface);
    if (ret == 0) {
        std::cout << "Joint moved to the first waypoint in the trajectory file successfully" << std::endl;
    } else {
        std::cout << "Joint failed to move to the first waypoint in the trajectory file" << std::endl;
    }

    // API call: Enable servo mode
    robot_interface->getMotionControl()->setServoModeSelect(3);
    // std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    // Wait to enter servo mode
    int i = 0;
    while (!robot_interface->getMotionControl()->isServoModeEnabled()) {
        if (i++ > 5) {
            std::cout << "Failed to enable Servo mode! Current servo status is "
                      << rpc_cli->getRobotInterface(robot_name)
                             ->getMotionControl()
                             ->isServoModeEnabled()
                      << std::endl;
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    for (size_t i = 1; i < traj.size(); i++) {
        for (size_t j = 0; j < 6; j++) {
            servo_r_j[j] = traj[i][j];
        }
        for (size_t j = 0; j < 3; j++) {
            servo_ex_j[j] = traj[i][j + 6];
        }
        std::cout << "servo_r_j: " << servo_r_j << std::endl;
        // API call: Joint servo motion
        int ret = rpc_cli->getRobotInterface(robot_name)
                      ->getMotionControl()
                      ->servoJointWithAxes(servo_r_j, servo_ex_j, 10.0, 3.1, 10,
                                           0.1, 200);
        if (ret == 2) {
            i--;
        }

        usleep(1000 * 2);
    }
    // API call: Logout
    rpc_cli->logout();
    // API call: Disconnect
    rpc_cli->disconnect();

    return 0;
}
