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

// Implement blocking functionality: The program continues execution only after the robot arm reaches the target waypoint
int waitArrival(RobotInterfacePtr impl)
{
    const int max_retry_count = 5;
    int cnt = 0;

    // API call: Get the current motion command ID
    int exec_id = impl->getMotionControl()->getExecId();

    // Wait for the robot arm to start moving
    while (exec_id == -1) {
        if (cnt++ > max_retry_count) {
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        exec_id = impl->getMotionControl()->getExecId();
    }

    // Wait for the robot arm to finish the action
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
    // Constructor, accepts the filename to open as a parameter
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

/**
 * Test: Use TrackJoint to follow a trajectory, target point interval is 5ms
 */
int exampleTrackJoint(RpcClientPtr cli)
{
    // Read trajectory file
    auto filename = "../trajs/record6.offt";
    TrajectoryIo input(filename);

    // Try to open trajectory file, if unable to open, return directly
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

    // API call: Get the robot's name
    auto robot_name = cli->getRobotNames().front();
    auto robot_interface = cli->getRobotInterface(robot_name);

    // API call: Set the speed fraction of the robot arm
    robot_interface->getMotionControl()->setSpeedFraction(0.8);

    // API call: Move joints to the first point in the trajectory, otherwise it may cause large overshoot
    robot_interface->getMotionControl()->moveJoint(traj[0], 80 * (M_PI / 180),
                                                   60 * (M_PI / 180), 0, 0);

    // Blocking
    int ret = waitArrival(robot_interface);
    if (ret == 0) {
        std::cout << "Joint moved to the first waypoint in the trajectory file successfully" << std::endl;
    } else {
        std::cout << "Joint failed to move to the first waypoint in the trajectory file" << std::endl;
    }

    for (size_t i = 1; i < traj.size(); i++) {
        int traj_queue_size =
            robot_interface->getMotionControl()->getTrajectoryQueueSize();
        std::cout << "traj_queue_size: " << traj_queue_size << std::endl;
        while (traj_queue_size > 8) {
            traj_queue_size =
                robot_interface->getMotionControl()->getTrajectoryQueueSize();

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        robot_interface->getMotionControl()->trackJoint(traj[i], 0.01, 0.5, 1);
    }

    // Wait for motion to finish
    while (!robot_interface->getRobotState()->isSteady()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    robot_interface->getMotionControl()->stopJoint(1);

    std::cout << "trackJoint motion finished" << std::endl;

    return 0;
}

#define LOCAL_IP "127.0.0.1"

/**
 * Steps to use trackJoint function:
 * 1. Use a real-time system to test trackJoint function
 * 2. Isolate a CPU to ensure no other tasks on that CPU
 * 3. Set this process as a real-time process with maximum priority
 * 4. Bind the CPU, bind this real-time process to the CPU isolated in step 2
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

    exampleTrackJoint(rpc_cli);

    // API call: RPC logout
    rpc_cli->logout();
    // API call: RPC disconnect
    rpc_cli->disconnect();

    return 0;
}
