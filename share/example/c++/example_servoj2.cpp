#include <cstring>
#include <fstream>
#include <math.h>
#ifdef WIN32
#include <windows.h>
#endif

#include "aubo_sdk/rpc.h"

using namespace arcs::aubo_sdk;
using namespace arcs::common_interface;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Implement blocking: The program continues only after the robot arm reaches the target waypoint
int waitArrival(RobotInterfacePtr impl)
{
    const int max_retry_count = 5;
    int cnt = 0;

    // API call: Get current motion command ID
    int exec_id = impl->getMotionControl()->getExecId();

    // Wait for the robot arm to start moving
    while (exec_id == -1) {
        if (cnt++ > max_retry_count) {
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        exec_id = impl->getMotionControl()->getExecId();
    }

    // Wait for the robot arm to finish moving
    while (impl->getMotionControl()->getExecId() != -1) {
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

class TrajectoryIo
{
public:
    // Constructor, takes the filename to open as a parameter
    TrajectoryIo(const char *filename)
    {
        input_file_.open(filename, std::ios::in);
    }

    // Check if the file was opened successfully
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
    // and convert it to a 2D std::vector.
    // Reads the file line by line, parses each line into a set of double values,
    // and stores these values in a nested 2D vector.
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
        // Convert the string to char* type first
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

/**
 * Test 1: Rotate joint 1 by 120 degrees, planning time is 10 seconds,
 * at the 5th second, send the second point to rotate joint 3 by 90 degrees
 * Conclusion: If a new target point is sent before the robot reaches the original target,
 * the robot will abandon the original target and move directly to the new target
 */
int exampleServoj1(RpcClientPtr cli)
{
    // API call: Get robot name
    auto robot_name = cli->getRobotNames().front();
    auto robot_interface = cli->getRobotInterface(robot_name);

    // API call: Set robot arm speed fraction
    robot_interface->getMotionControl()->setSpeedFraction(0.3);

    // Joint angles, unit: radians
    std::vector<double> joint_angle = {
        0.0 * (M_PI / 180),  -15.0 * (M_PI / 180), 100.0 * (M_PI / 180),
        25.0 * (M_PI / 180), 90.0 * (M_PI / 180),  0.0 * (M_PI / 180)
    };

    // API call: Joint movement
    robot_interface->getMotionControl()->moveJoint(
        joint_angle, 80 * (M_PI / 180), 60 * (M_PI / 180), 0, 0);
    // Blocking
    int ret = waitArrival(robot_interface);
    if (ret == 0) {
        std::cout << "Joint moved to initial position successfully" << std::endl;
    } else {
        std::cout << "Joint failed to move to initial position" << std::endl;
    }

    // API call: Enable servo mode
    robot_interface->getMotionControl()->setServoMode(true);

    // Wait to enter servo mode
    int i = 0;
    while (!robot_interface->getMotionControl()->isServoModeEnabled()) {
        if (i++ > 5) {
            std::cout << "Failed to enable Servo mode! Current servo status is "
                      << cli->getRobotInterface(robot_name)
                             ->getMotionControl()
                             ->isServoModeEnabled()
                      << std::endl;
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    std::vector<double> q1 = { -120.0 * (M_PI / 180), -15.0 * (M_PI / 180),
                               100.0 * (M_PI / 180),  25.0 * (M_PI / 180),
                               90.0 * (M_PI / 180),   0.0 * (M_PI / 180) };
    std::cout << "Moving to the first target point" << std::endl;
    // API call: Joint servo movement
    robot_interface->getMotionControl()->servoJoint(q1, 0.1, 0.2, 10, 0.1, 200);

    std::this_thread::sleep_for(std::chrono::seconds(5));

    std::vector<double> q2 = { 0, 0, 90.0 / 180 * M_PI, 0, 0, 0 };
    std::cout << "Moving to the second target point" << std::endl;
    // API call: Joint servo movement
    robot_interface->getMotionControl()->servoJoint(q2, 0.1, 0.2, 10, 0.1, 200);

    // Wait for movement to finish
    while (!cli->getRobotInterface(robot_name)->getRobotState()->isSteady()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    std::cout << "Servoj movement finished" << std::endl;

    // Disable servo mode
    robot_interface->getMotionControl()->setServoMode(false);

    // Wait to exit servo mode
    i = 0;
    while (robot_interface->getMotionControl()->isServoModeEnabled()) {
        if (i++ > 5) {
            std::cout
                << "Failed to disable Servo mode! Current Servo mode is "
                << robot_interface->getMotionControl()->isServoModeEnabled()
                << std::endl;
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    return 0;
}

/**
 * Test 2: Use servoj to track a trajectory, target point interval is 5ms
 */
int exampleServoj2(RpcClientPtr cli)
{
    // Read trajectory file
    auto filename = "../trajs/record6.offt";
    TrajectoryIo input(filename);

    // Try to open trajectory file, return directly if failed
    if (!input.open()) {
        return 0;
    }

    // Parse trajectory data
    auto traj = input.parse();

    // Check if there are waypoints in the trajectory file,
    // if the number is 0, output error message and return
    auto traj_sz = traj.size();
    if (traj_sz == 0) {
        std::cerr << "Number of waypoints in trajectory file is 0." << std::endl;
        return 0;
    }

    // API call: Get robot name
    auto robot_name = cli->getRobotNames().front();
    auto robot_interface = cli->getRobotInterface(robot_name);

    // API call: Set robot arm speed fraction
    robot_interface->getMotionControl()->setSpeedFraction(1);

    // API call: Move joint to the first point in the trajectory to avoid large overshoot
    robot_interface->getMotionControl()->moveJoint(traj[0], 80 * (M_PI / 180),
                                                   60 * (M_PI / 180), 0, 0);

    // Blocking
    int ret = waitArrival(robot_interface);
    if (ret == 0) {
        std::cout << "Joint moved to the first waypoint in trajectory file successfully" << std::endl;
    } else {
        std::cout << "Joint failed to move to the first waypoint in trajectory file" << std::endl;
    }

    // API call: Enable servo mode
    cli->getRobotInterface(robot_name)->getMotionControl()->setServoMode(true);

    // Wait to enter servo mode
    int i = 0;
    while (!cli->getRobotInterface(robot_name)
                ->getMotionControl()
                ->isServoModeEnabled()) {
        if (i++ > 5) {
            std::cout << "Failed to enable Servo mode! Current Servo mode is "
                      << cli->getRobotInterface(robot_name)
                             ->getMotionControl()
                             ->isServoModeEnabled()
                      << std::endl;
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    for (size_t i = 1; i < traj.size(); i++) {
        // API call: Joint servo movement
        cli->getRobotInterface(robot_name)
            ->getMotionControl()
            ->servoJoint(traj[i], 0.1, 0.2, 10, 0.1, 200);

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // Wait for movement to finish
    while (!cli->getRobotInterface(robot_name)->getRobotState()->isSteady()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    std::cout << "Servoj movement finished" << std::endl;

    // API call: Disable servo mode
    cli->getRobotInterface(robot_name)->getMotionControl()->setServoMode(false);

    // Wait to exit servo mode
    while (cli->getRobotInterface(robot_name)
               ->getMotionControl()
               ->isServoModeEnabled()) {
        if (i++ > 5) {
            std::cout << "Failed to disable Servo mode! Current Servo mode is "
                      << cli->getRobotInterface(robot_name)
                             ->getMotionControl()
                             ->isServoModeEnabled()
                      << std::endl;
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    return 0;
}

/**
 * Test 3: Use servoj mode 2 to track a trajectory, target point interval is 5ms
 */
int exampleServoj3(RpcClientPtr cli)
{
    // Read trajectory file
    auto filename = "../trajs/record6.offt";
    TrajectoryIo input(filename);

    // Try to open trajectory file, return directly if failed
    if (!input.open()) {
        return 0;
    }

    // Parse trajectory data
    auto traj = input.parse();

    // Check if there are waypoints in the trajectory file,
    // if the number is 0, output error message and return
    auto traj_sz = traj.size();
    if (traj_sz == 0) {
        std::cerr << "Number of waypoints in trajectory file is 0." << std::endl;
        return 0;
    }

    // API call: Get robot name
    auto robot_name = cli->getRobotNames().front();
    auto robot_interface = cli->getRobotInterface(robot_name);

    // API call: Set robot arm speed fraction
    robot_interface->getMotionControl()->setSpeedFraction(1);

    // API call: Move joint to the first point in the trajectory to avoid large overshoot
    robot_interface->getMotionControl()->moveJoint(traj[0], 80 * (M_PI / 180),
                                                   60 * (M_PI / 180), 0, 0);

    // Blocking
    int ret = waitArrival(robot_interface);
    if (ret == 0) {
        std::cout << "Joint moved to the first waypoint in trajectory file successfully" << std::endl;
    } else {
        std::cout << "Joint failed to move to the first waypoint in trajectory file" << std::endl;
    }

    // API call: Enable servo mode 2
    cli->getRobotInterface(robot_name)
        ->getMotionControl()
        ->setServoModeSelect(2);

    // Wait to enter servo mode 2
    int i = 0;
    while (cli->getRobotInterface(robot_name)
               ->getMotionControl()
               ->getServoModeSelect() != 2) {
        if (i++ > 50) {
            std::cout << "Failed to enable Servo mode 2! Current Servo mode is "
                      << cli->getRobotInterface(robot_name)
                             ->getMotionControl()
                             ->getServoModeSelect()
                      << std::endl;
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    for (size_t i = 1; i < traj.size(); i++) {
        // API call: Joint servo movement
        cli->getRobotInterface(robot_name)
            ->getMotionControl()
            ->servoJoint(traj[i], 0.1, 0.2, 0.1, 0.1, 200);

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // Wait for movement to finish
    while (!cli->getRobotInterface(robot_name)->getRobotState()->isSteady()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    std::cout << "Servoj movement finished" << std::endl;

    // API call: Disable servo mode 2
    cli->getRobotInterface(robot_name)
        ->getMotionControl()
        ->setServoModeSelect(0);

    // Wait to exit servo mode 2
    while (cli->getRobotInterface(robot_name)
               ->getMotionControl()
               ->getServoModeSelect() != 0) {
        if (i++ > 50) {
            std::cout << "Failed to disable Servo mode 2! Current Servo mode is "
                      << cli->getRobotInterface(robot_name)
                             ->getMotionControl()
                             ->getServoModeSelect()
                      << std::endl;
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    return 0;
}

#define LOCAL_IP "127.0.0.1"
/**
 * Steps to use servoj function:
 * 1. Use a real-time system to test servoj function
 * 2. Isolate a CPU to ensure no other tasks on it
 * 3. Set the process as a real-time process with maximum priority
 * 4. Bind the real-time process to the CPU isolated in step 2
 */
int main(void)
{
#ifdef WIN32
    // Set Windows console output code page to UTF-8
    SetConsoleOutputCP(CP_UTF8);
#endif
#ifndef _WIN32
    // Maximum and minimum real-time priority
    int sched_max = sched_get_priority_max(SCHED_FIFO);

    // Set real-time scheduling policy and priority
    struct sched_param sParam;
    sParam.sched_priority = sched_max;
    sched_setscheduler(0, SCHED_FIFO, &sParam);
    auto i_schedFlag = sched_getscheduler(0);
    printf("Set scheduling policy = [%d]\n", i_schedFlag);

    // Bind CPU
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(1, &cpuset);

    // bind process to processor 0
    if (sched_setaffinity(0, sizeof(cpuset), &cpuset) < 0) {
        perror("Sched_setaffinity fail!");
    }
#endif

    auto rpc_cli = std::make_shared<RpcClient>();
    // API call: Set RPC timeout
    rpc_cli->setRequestTimeout(1000);
    // API call: Connect to RPC service
    rpc_cli->connect(LOCAL_IP, 30004);
    // API call: Login
    rpc_cli->login("aubo", "123456");

    exampleServoj1(rpc_cli);
    // exampleServoj2(rpc_cli);
    // exampleServoj3(rpc_cli);

    // API call: RPC logout
    rpc_cli->logout();
    // API call: RPC disconnect
    rpc_cli->disconnect();

    return 0;
}
