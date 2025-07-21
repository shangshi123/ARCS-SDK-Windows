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
// Implement blocking functionality: The program proceeds only after the robot arm reaches the target waypoint
int waitArrival(RobotInterfacePtr impl)
{
    // API call: Get the current motion command ID
    int exec_id = impl->getMotionControl()->getExecId();

    int cnt = 0;
    // Maximum retry count for getting the maximum exec_id while waiting for the robot arm to start moving
    int max_retry_count = 50;

    // Wait for the robot arm to start moving
    while (exec_id == -1) {
        if (cnt++ > max_retry_count) {
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        exec_id = impl->getMotionControl()->getExecId();
    }

    // Wait for the robot arm to finish the action
    while (exec_id != -1) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        exec_id = impl->getMotionControl()->getExecId();
    }

    return 0;
}

// Check if the buffer is valid
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

int runTraj(RpcClientPtr rpc, const std::string &traj_name,
            const std::string &record_name)
{
    // Read trajectory file
    auto filename = traj_name;
    TrajectoryIo input(filename.c_str());

    // Try to open trajectory file, return directly if unable to open
    if (!input.open()) {
        return 0;
    }

    // Parse trajectory data
    auto traj = input.parse();

    auto traj_sz = traj.size();
    if (traj_sz == 0) {
        std::cerr << "Number of waypoints in trajectory file is 0" << std::endl;
        return 0;
    } else {
        std::cout << " Number of loaded waypoints: " << traj.size() << std::endl;
    }

    // API call: Get the robot's name
    auto robot_name = rpc->getRobotNames().front();

    auto robot_interface = rpc->getRobotInterface(robot_name);

    // API call: Set the speed fraction of the robot arm
    robot_interface->getMotionControl()->setSpeedFraction(0.3);

    // API call: Move joints to the first waypoint in the trajectory file
    robot_interface->getMotionControl()->moveJoint(traj[0], 30 * (M_PI / 180),
                                                   30 * (M_PI / 180), 0., 0.);

    // Blocking
    int ret = waitArrival(robot_interface);
    if (ret == 0) {
        std::cout << "Successfully moved joints to the first waypoint in the trajectory file" << std::endl;
    } else {
        std::cout << "Failed to move joints to the first waypoint in the trajectory file" << std::endl;
    }

    // API call: Start the planner
    rpc->getRuntimeMachine()->start();
    auto cur_plan_context = rpc->getRuntimeMachine()->getPlanContext();
    // API call: Get a new thread id
    auto task_id = rpc->getRuntimeMachine()->newTask();
    rpc->getRuntimeMachine()->setPlanContext(
        task_id, std::get<1>(cur_plan_context), std::get<2>(cur_plan_context));

    // API call: Clear buffer "rec"
    robot_interface->getMotionControl()->pathBufferFree("rec");

    // API call: Create a new buffer "rec", and specify trajectory motion type and number of trajectory points
    robot_interface->getMotionControl()->pathBufferAlloc("rec", 2, traj_sz);

    // Add waypoints from the trajectory file to the path buffer in groups,
    // 10 points per group,
    // If the number of remaining waypoints is less than or equal to 10, add them as the last group
    size_t offset = 10;
    auto it = traj.begin();
    while (true) {
        // API call: Add trajectory waypoints to the buffer path
        robot_interface->getMotionControl()->pathBufferAppend(
            "rec", std::vector<std::vector<double>>{ it, it + 10 });
        it += 10;
        if (offset + 10 >= traj_sz) {
            // API call: Add trajectory waypoints to the buffer path
            robot_interface->getMotionControl()->pathBufferAppend(
                "rec", std::vector<std::vector<double>>{ it, traj.end() });
            break;
        }

        offset += 10;
    }

    // Sampling interval in the trajectory file
    double interval = 0.005;
    //    double interval = 0.02;

    // API call: Perform time-consuming operations such as calculation and optimization to optimize the trajectory
    robot_interface->getMotionControl()->pathBufferEval("rec", {}, {},
                                                        interval);

    // Check if the path buffer is valid
    if (isBufferValid(robot_interface) == -1) {
        std::cerr << "Path buffer is invalid, unable to perform trajectory motion" << std::endl;
    } else {
        robot_interface->getRobotManage()->startRecord(record_name);
        // API call: Execute trajectory motion
        robot_interface->getMotionControl()->movePathBuffer("rec");

        // Wait for trajectory motion to complete
        ret = waitArrival(rpc->getRobotInterface(robot_name));
        if (ret == -1) {
            std::cerr << "Trajectory motion failed" << std::endl;
        } else {
            std::cout << "Trajectory motion finished" << std::endl;
        }
    }
    robot_interface->getRobotManage()->stopRecord();
    // API call: Stop the planner
    rpc->getRuntimeMachine()->stop();
    // API call: Delete thread
    rpc->getRuntimeMachine()->deleteTask(task_id);
}

#define LOCAL_IP "192.168.1.15"

int main(int argc, char **argv)
{
#ifdef WIN32
    // Set Windows console output code page to UTF-8
    SetConsoleOutputCP(CP_UTF8);
#endif

    auto rpc = std::make_shared<RpcClient>();
    // API call: Set RPC timeout
    rpc->setRequestTimeout(1000);
    // API call: Connect to RPC service
    rpc->connect(LOCAL_IP, 30004);
    // API call: Login
    rpc->login("aubo", "123456");
    auto robot_name = rpc->getRobotNames().front();

    auto robot_interface = rpc->getRobotInterface(robot_name);

    // Set filter parameters
    std::stringstream ss;
    ss << "[filter_freq]" << std::endl;
    ss << "    vel_filter_freq  = [100, 100, 100, 100, 100, 100]" << std::endl;
    ss << "    acc_filter_freq  = [100, 100, 100, 100, 100, 100]" << std::endl;
    ss << "    curr_filter_freq = [100, 100, 100, 100, 100, 100]" << std::endl;
    robot_interface->getRobotConfig()->setHardwareCustomParameters(ss.str());

    // Set payload parameters
    robot_interface->getRobotConfig()->setPayload(
        4.1768, { 0, 0.1411, 0.0541 }, {},
        { 0.1114, 0, 0, 0.0312, 0.0363, 0.1125 });

    // Robot serial number
    std::string robot_serial = "AB1115312D000083";

    // Trajectory name
    std::string filename1 = "1_LowSpeed";
    std::cout << "Start trajectory: " << filename1 << std::endl;

    // Start running trajectory
    runTraj(rpc, "../trajs/collision_traj/" + filename1 + ".csv",
            robot_serial + "_" + filename1 + ".csv");

    std::string filename2 = "2_MediumSpeed";
    std::cout << "Start trajectory: " << filename2 << std::endl;
    runTraj(rpc, "../trajs/collision_traj/" + filename2 + ".csv",
            robot_serial + "_" + filename2 + ".csv");

    std::string filename3 = "3_HighSpeed";
    std::cout << "Start trajectory: " << filename3 << std::endl;
    runTraj(rpc, "../trajs/collision_traj/" + filename3 + ".csv",
            robot_serial + "_" + filename3 + ".csv");

    std::string filename4 = "4_LowAcceleration";
    std::cout << "Start trajectory: " << filename4 << std::endl;
    runTraj(rpc, "../trajs/collision_traj/" + filename4 + ".csv",
            robot_serial + "_" + filename4 + ".csv");

    std::string filename5 = "5_MediumAcceleration";
    std::cout << "Start trajectory: " << filename5 << std::endl;
    runTraj(rpc, "../trajs/collision_traj/" + filename5 + ".csv",
            robot_serial + "_" + filename5 + ".csv");

    std::string filename6 = "6_HighAcceleration";
    std::cout << "Start trajectory: " << filename6 << std::endl;
    runTraj(rpc, "../trajs/collision_traj/" + filename6 + ".csv",
            robot_serial + "_" + filename6 + ".csv");

    std::string filename7 = "7_ConstantLowSpeed";
    std::cout << "Start trajectory: " << filename7 << std::endl;
    runTraj(rpc, "../trajs/collision_traj/" + filename7 + ".csv",
            robot_serial + "_" + filename7 + ".csv");

    std::string filename8 = "8_ConstantMediumSpeed";
    std::cout << "Start trajectory: " << filename8 << std::endl;
    runTraj(rpc, "../trajs/collision_traj/" + filename8 + ".csv",
            robot_serial + "_" + filename8 + ".csv");

    std::string filename9 = "9_ConstantHighSpeed";
    std::cout << "Start trajectory: " << filename9 << std::endl;
    runTraj(rpc, "../trajs/collision_traj/" + filename9 + ".csv",
            robot_serial + "_" + filename9 + ".csv");

    std::string filename11 = "11_trajectory_vel2_acc5";
    std::cout << "Start trajectory: " << filename11 << std::endl;
    runTraj(rpc, "../trajs/collision_traj/" + filename11 + ".txt",
            robot_serial + "_" + filename11 + ".csv");

    std::string filename12 = "12_trajectory_vel2_acc20";
    std::cout << "Start trajectory: " << filename12 << std::endl;
    runTraj(rpc, "../trajs/collision_traj/" + filename12 + ".txt",
            robot_serial + "_" + filename12 + ".csv");

    std::string filename13 = "13_trajectory_vel2_acc30";
    std::cout << "Start trajectory: " << filename13 << std::endl;
    runTraj(rpc, "../trajs/collision_traj/" + filename13 + ".txt",
            robot_serial + "_" + filename13 + ".csv");

    std::string filename14 = "14_trajectory_vel2.2_acc10";
    std::cout << "Start trajectory: " << filename14 << std::endl;
    runTraj(rpc, "../trajs/collision_traj/" + filename14 + ".txt",
            robot_serial + "_" + filename14 + ".csv");
}
