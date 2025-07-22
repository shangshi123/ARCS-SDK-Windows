#include <cstring>
#include <fstream>
#include <math.h>
#ifdef WIN32
#include <windows.h>
#endif
#include <csignal>

#include "aubo_sdk/rpc.h"

using namespace arcs::aubo_sdk;
using namespace arcs::common_interface;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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
    // Reads the file line by line, parses each line into a set of double values,
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

// Implement blocking functionality: when the robot arm moves to the target waypoint, the program continues
int waitArrival(RobotInterfacePtr impl)
{
    // API call: get current motion command ID
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

// Check if buffer is valid
int isBufferValid(RobotInterfacePtr impl)
{
    // Maximum retry count for calling pathBufferValid
    int max_retry_count = 5;
    // Number of times pathBufferValid is called
    int cnt = 0;
    bool isValid = impl->getMotionControl()->pathBufferValid("rec");

    while (!isValid) {
        if (cnt++ > max_retry_count) {
            return -1;
        }
        isValid = impl->getMotionControl()->pathBufferValid("rec");
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return 0;
}

#define LOCAL_IP "127.0.0.1"

auto rpc_cli = std::make_shared<RpcClient>();
int task_id;

// Define signal handler function
void signalHandler(int signal)
{
    if (signal == SIGINT) {
        std::cout << "Received SIGINT signal." << std::endl;
        if (rpc_cli) {
            rpc_cli->getRuntimeMachine()->stop();
            rpc_cli->getRuntimeMachine()->deleteTask(task_id);
            rpc_cli->logout();
            rpc_cli->disconnect();
        }
        // Exit program
        exit(0);
    }
}

int main(int argc, char **argv)
{
#ifdef WIN32
    // Set Windows console output code page to UTF-8
    SetConsoleOutputCP(CP_UTF8);
#endif
    // Register signal handler function
    signal(SIGINT, signalHandler);

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

    // API call: set RPC timeout
    rpc_cli->setRequestTimeout(1000);
    // API call: connect to RPC service
    rpc_cli->connect(LOCAL_IP, 30004);
    // API call: login
    rpc_cli->login("aubo", "123456");

    // API call: get robot name
    auto robot_name = rpc_cli->getRobotNames().front();

    auto robot_interface = rpc_cli->getRobotInterface(robot_name);

    // API call: set robot arm speed fraction
    robot_interface->getMotionControl()->setSpeedFraction(0.3);

    // API call: start planner
    rpc_cli->getRuntimeMachine()->start();

    auto cur_plan_context = rpc_cli->getRuntimeMachine()->getPlanContext();
    // API call: get new thread id
    task_id = rpc_cli->getRuntimeMachine()->newTask();
    rpc_cli->getRuntimeMachine()->setPlanContext(
        task_id, std::get<1>(cur_plan_context), std::get<2>(cur_plan_context));

    // API call: joint motion to the first waypoint in the trajectory file
    robot_interface->getMotionControl()->moveJoint(traj[0], 30 * (M_PI / 180),
                                                   30 * (M_PI / 180), 0., 0.);

    // Blocking
    int ret = waitArrival(robot_interface);
    if (ret == 0) {
        std::cout << "Joint motion to the first waypoint in the trajectory file succeeded" << std::endl;
    } else {
        std::cout << "Joint motion to the first waypoint in the trajectory file failed" << std::endl;
    }

    // API call: clear buffer "rec"
    robot_interface->getMotionControl()->pathBufferFree("rec");

    // API call: create a new buffer "rec", and specify trajectory motion type and number of trajectory points
    robot_interface->getMotionControl()->pathBufferAlloc("rec", 2, traj_sz);

    // Add waypoints from the trajectory file to the path buffer in groups,
    // 10 points per group,
    // If the number of remaining waypoints is less than or equal to 10, add them as the last group
    size_t offset = 10;
    auto it = traj.begin();
    while (true) {
        std::cout << "Adding trajectory waypoints " << offset << std::endl;
        // API call: add trajectory waypoints to buffer path
        robot_interface->getMotionControl()->pathBufferAppend(
            "rec", std::vector<std::vector<double>>{ it, it + 10 });
        it += 10;
        if (offset + 10 >= traj_sz) {
            std::cout << "Adding trajectory waypoints " << traj_sz << std::endl;
            // API call: add trajectory waypoints to buffer path
            robot_interface->getMotionControl()->pathBufferAppend(
                "rec", std::vector<std::vector<double>>{ it, traj.end() });
            break;
        }

        offset += 10;
    }

    // Sampling interval in trajectory file
    double interval = 0.005;

    // API call: perform time-consuming operations such as calculation and optimization to optimize the trajectory
    robot_interface->getMotionControl()->pathBufferEval(
        "rec", { 1, 1, 1, 1, 1, 1 }, { 1, 1, 1, 1, 1, 1 }, interval);

    // Check if path buffer is valid
    if (isBufferValid(robot_interface) == -1) {
        std::cerr << "Path buffer is invalid, cannot perform trajectory motion" << std::endl;
    } else {
        // API call: execute trajectory motion
        robot_interface->getMotionControl()->movePathBuffer("rec");

        // Wait for trajectory motion to complete
        ret = waitArrival(robot_interface);
        if (ret == -1) {
            std::cerr << "Trajectory motion failed" << std::endl;
        } else {
            std::cout << "Trajectory motion ended" << std::endl;
        }
    }

    // API call: stop planner
    rpc_cli->getRuntimeMachine()->stop();
    // API call: delete thread
    rpc_cli->getRuntimeMachine()->deleteTask(task_id);

    // API call: RPC logout
    rpc_cli->logout();
    // API call: RPC disconnect
    rpc_cli->disconnect();

    return 0;
}

