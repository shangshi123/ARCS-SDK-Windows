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
#define EMBEDDED

// Implement blocking functionality: The program continues execution only after the robot arm reaches the target waypoint
int waitArrival(RobotInterfacePtr impl)
{
    // API call: Get the current motion command ID
    int exec_id = impl->getMotionControl()->getExecId();

    int cnt = 0;
    // Maximum retry count for getting the largest exec_id while waiting for the robot arm to start moving
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

// Check if the buffer is valid
int isBufferValid(RobotInterfacePtr impl)
{
    // Maximum retry count for calling pathBufferValid
    int max_retry_count = 50;
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

void tcpSensorTest(RpcClientPtr cli)
{
    // API call: Get the robot's name
    auto robot_name = cli->getRobotNames().front();
#ifdef EMBEDDED
    // Built-in sensor
    std::vector<double> sensor_pose = { 0, 0, 0, 0, 0, 0 };
    cli->getRobotInterface(robot_name)
        ->getRobotConfig()
        ->selectTcpForceSensor("embedded");
#else
    // External KW sensor
    std::vector<double> sensor_pose = { 0, 0, 0.047, 0, 0, 0 };
    cli->getRobotInterface(robot_name)
        ->getRobotConfig()
        ->selectTcpForceSensor("kw_ftsensor");
#endif
    while (1) {
        std::cout << "------------------------------------------" << std::endl;
        auto sensor_data = cli->getRobotInterface(robot_name)
                               ->getRobotState()
                               ->getTcpForceSensors();
        std::cout << "force sensor: " << sensor_data << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        auto curr_pose =
            cli->getRobotInterface(robot_name)->getRobotState()->getTcpPose();
        auto payload =
            cli->getRobotInterface(robot_name)->getRobotConfig()->getPayload();
        std::cout << "mass: " << std::get<0>(payload)
                  << " cog: " << std::get<1>(payload) << std::endl;
        auto result =
            cli->getRobotInterface(robot_name)
                ->getRobotAlgorithm()
                ->calibrateTcpForceSensor({ sensor_data }, { curr_pose });

        std::cout << "force offset: " << std::get<0>(result) << std::endl;
        std::cout << "------------------------------------------" << std::endl;
    }
}

void initFcParams(RpcClientPtr cli)
{
    auto robot_name = cli->getRobotNames().front();
#ifdef EMBEDDED
    // Built-in sensor
    std::vector<double> sensor_pose = { 0, 0, 0, 0, 0, 0 };
    cli->getRobotInterface(robot_name)
        ->getRobotConfig()
        ->selectTcpForceSensor("embedded");
#else
    // External KW sensor
    std::vector<double> sensor_pose = { 0, 0, 0.047, 0, 0, 0 };
    cli->getRobotInterface(robot_name)
        ->getRobotConfig()
        ->selectTcpForceSensor("kw_ftsensor");
#endif

    // Set sensor installation pose
    cli->getRobotInterface(robot_name)
        ->getRobotConfig()
        ->setTcpForceSensorPose(sensor_pose);
    // Set TCP offset
    std::vector<double> tcp_pose = { 0, 0, 0.0, 0, 0, 0 };
    cli->getRobotInterface(robot_name)
        ->getRobotConfig()
        ->setTcpOffset(tcp_pose);

    double mass = 0.0;
    std::vector<double> com = { 0.0, 0.0, 0.0 };

    // Force sensor offset, needs to be set according to actual situation
    std::vector<double> force_offset = { 0.0, 0.0, -3.8, 0.0, 0.0, 0.0 };
    // Set payload
    cli->getRobotInterface(robot_name)
        ->getRobotConfig()
        ->setPayload(mass, com, { 0. }, { 0. });
    // Set force sensor offset
    cli->getRobotInterface(robot_name)
        ->getRobotConfig()
        ->setTcpForceOffset(force_offset);

    // Force control m d k parameters, need to be tuned according to environment
    std::vector<double> admittance_m = { 25.0, 25.0, 25.0, 2.0, 2.0, 2.0 };
    std::vector<double> admittance_d = {
        300.0, 300.0, 300.0, 12.0, 12.0, 12.0
    };
    std::vector<double> admittance_k = { 0.0, 0.0, 300.0, 0.0, 0.0, 0.0 };

    cli->getRobotInterface(robot_name)
        ->getForceControl()
        ->setDynamicModel(admittance_m, admittance_d, admittance_k);

    std::vector<bool> compliance = { false, false, true, false, false, false };
    std::vector<double> target_wrench = { 0.0, 0.0, -30.0, 0.0, 0.0, 0.0 };
    std::vector<double> speed_limits(6, 2.0);

    auto feature =
        cli->getRobotInterface(robot_name)->getRobotState()->getTcpPose();
    cli->getRobotInterface(robot_name)
        ->getForceControl()
        ->setTargetForce(feature, compliance, target_wrench, speed_limits,
                         TaskFrameType::TOOL_FORCE);
}

void loadTraj(RpcClientPtr cli, std::vector<std::vector<double>> &traj)
{
    auto robot_name = cli->getRobotNames().front();
    auto robot_interface = cli->getRobotInterface(robot_name);
    // API call: Clear buffer "rec"
    robot_interface->getMotionControl()->pathBufferFree("rec");
    auto traj_sz = traj.size();
    // API call: Create a new buffer "rec", specify trajectory motion type and number of trajectory points
    robot_interface->getMotionControl()->pathBufferAlloc("rec", 2, traj_sz);

    // Add waypoints from the trajectory file to the path buffer in groups,
    // 10 points per group,
    // if the number of remaining points is less than or equal to 10, add them as the last group
    size_t offset = 10;
    auto it = traj.begin();
    while (true) {
        std::cout << "Adding trajectory waypoints " << offset << std::endl;
        // API call: Add trajectory waypoints to the buffer path
        robot_interface->getMotionControl()->pathBufferAppend(
            "rec", std::vector<std::vector<double>>{ it, it + 10 });
        it += 10;
        if (offset + 10 >= traj_sz) {
            std::cout << "Adding trajectory waypoints " << traj_sz << std::endl;
            // API call: Add trajectory waypoints to the buffer path
            robot_interface->getMotionControl()->pathBufferAppend(
                "rec", std::vector<std::vector<double>>{ it, traj.end() });
            break;
        }

        offset += 10;
    }

    // Sampling interval in the trajectory file
    double interval = 0.05;
    //    double interval = 0.02;

    // API call: Perform time-consuming operations such as calculation and optimization to optimize the trajectory
    robot_interface->getMotionControl()->pathBufferEval("rec", {}, {},
                                                        interval);
}

void example1(RpcClientPtr cli)
{
    auto robot_name = cli->getRobotNames().front();
#ifdef EMBEDDED
    // Built-in sensor
    std::vector<double> sensor_pose = { 0, 0, 0, 0, 0, 0 };
    cli->getRobotInterface(robot_name)
        ->getRobotConfig()
        ->selectTcpForceSensor("embedded");
#else
    // External KW sensor
    std::vector<double> sensor_pose = { 0, 0, 0.047, 0, 0, 0 };
    cli->getRobotInterface(robot_name)
        ->getRobotConfig()
        ->selectTcpForceSensor("kw_ftsensor");
#endif

    // Set sensor installation pose
    cli->getRobotInterface(robot_name)
        ->getRobotConfig()
        ->setTcpForceSensorPose(sensor_pose);
    // Set TCP offset
    std::vector<double> tcp_pose = { 0, 0, 0.0, 0, 0, 0 };
    cli->getRobotInterface(robot_name)
        ->getRobotConfig()
        ->setTcpOffset(tcp_pose);

    double mass = 0.0;
    std::vector<double> com = { 0.0, 0.0, 0.0 };
    // Force sensor offset, needs to be set according to actual situation
    std::vector<double> force_offset = { 0, 0, -3.8, 0, 0, 0 };
    // Set payload
    cli->getRobotInterface(robot_name)
        ->getRobotConfig()
        ->setPayload(mass, com, { 0. }, { 0. });

    // Set force sensor offset
    cli->getRobotInterface(robot_name)
        ->getRobotConfig()
        ->setTcpForceOffset(force_offset);

    std::vector<double> admittance_m = { 25.0, 25.0, 25.0, 2.0, 2.0, 2.0 };
    std::vector<double> admittance_d = {
        300.0, 300.0, 300.0, 12.0, 12.0, 12.0
    };
    std::vector<double> admittance_k = { 0.0, 0.0, 300.0, 0.0, 0.0, 0.0 };

    cli->getRobotInterface("rob1")->getForceControl()->setDynamicModel(
        admittance_m, admittance_d, admittance_k);

    // Force control enabled directions
    std::vector<bool> compliance = { false, false, true, false, false, false };
    // Target force
    std::vector<double> target_wrench{ 0.0, 0.0, -3.0, 0.0, 0.0, 0.0 };
    std::vector<double> speed_limits(6, 2.0);

    // Set force control reference frame to FRAME_FORCE, search based on current tool frame
    auto feature =
        cli->getRobotInterface(robot_name)->getRobotState()->getTcpPose();
    TaskFrameType frame_type = TaskFrameType::FRAME_FORCE;
    cli->getRobotInterface(robot_name)
        ->getForceControl()
        ->setTargetForce(feature, compliance, target_wrench, speed_limits,
                         frame_type);

    // Set monitoring search range
    // Search range is box, described by 6 parameters: unit meter
    // double xmin;
    // double xmax;
    // double ymin;
    // double ymax;
    // double zmin;
    // double zmax;
    auto box_frame =
        cli->getRobotInterface(robot_name)->getRobotState()->getTcpPose();
    std::vector<double> box = { -1000.0, 1000.0, -1000.0, 1000.0, 0, 0.1 };
    cli->getRobotInterface(robot_name)
        ->getForceControl()
        ->setSupvPosBox(box_frame, box);

    // Enable admittance control
    cli->getRobotInterface(robot_name)->getForceControl()->fcEnable();
}

int example2(RpcClientPtr rpc)
{
    // API call: Get the robot's name
    auto robot_name = rpc->getRobotNames().front();

    auto robot_interface = rpc->getRobotInterface(robot_name);

    // Read trajectory file
    auto filename = "../trajs/physiotherapy.txt";
    TrajectoryIo input(filename);

    // Try to open trajectory file, return directly if unable to open
    if (!input.open()) {
        return -1;
    }

    // Parse trajectory data
    auto traj = input.parse();
    // Check if there are waypoints in the trajectory file,
    // if the number is 0, output error message and return
    auto traj_sz = traj.size();
    if (traj_sz == 0) {
        std::cerr << "Number of waypoints in trajectory file is 0." << std::endl;
        return -1;
    }
    // API call:
    // Set robot arm speed ratio
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

    // Initialize force control parameters
    initFcParams(rpc);
    loadTraj(rpc, traj);

    if (isBufferValid(robot_interface) == -1) {
        std::cerr << "Path buffer is invalid, cannot perform trajectory motion" << std::endl;
    } else {
        // Run trajectory
        robot_interface->getMotionControl()->movePathBuffer("rec");
        // Move first, then enable force control
        robot_interface->getForceControl()->fcEnable();
    }

    // Wait for trajectory motion to complete
    ret = waitArrival(robot_interface);
    if (ret == -1) {
        std::cerr << "Trajectory motion failed" << std::endl;
    } else {
        std::cout << "Trajectory motion finished" << std::endl;
    }

    robot_interface->getMotionControl()->stopJoint(1);
    robot_interface->getForceControl()->fcDisable();
}
#define LOCAL_IP "192.168.10.236"

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

    /* Search routine */
    // example1(rpc);
    /* Trajectory + force control */
    example2(rpc);
    /* Test force sensor data */
    // tcpSensorTest(rpc);
    return 0;
}
