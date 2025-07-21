#include <cstring>
#include <fstream>
#include <math.h>
#ifdef WIN32
#include <windows.h>
#endif

#include "aubo_sdk/rpc.h"
#include "aubo_sdk/rtde.h"
#include "aubo_sdk/script.h"

using namespace arcs::aubo_sdk;
using namespace arcs::common_interface;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Implement blocking functionality: When the robot arm moves to the target waypoint, the program continues to execute
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

    // Wait for the robot arm action to complete
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
        // First convert the string to be split from string type to char* type
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

const char *SERVOJ_SCRIPT = R"(local aubo = require('aubo')
    local sched = sched or aubo.sched
    local math = aubo.math or math
    local sync = sched.sync
return function(api)
    local names = api:getRobotNames()
    robot = api:getRobotInterface(names[1])
    local function servoj(...)
        return robot:getMotionControl():servoJoint(...)
    end
    local function get_target()
        local target_q = robot:getRobotState():getJointPositions()
        for i=1,6 do
            target_q[i] = api:getRegisterControl():getDoubleInput(i-1)
        end
        return target_q
    end

    local running = true
    robot:getMotionControl():setServoMode(true)
    api:getRegisterControl():setInt32Input(0, 1)
    while running do
        local flag = api:getRegisterControl():getInt32Input(0)
        if flag == 6 then
            servoj(get_target(), 0.0, 0.0, 0.005, 0.0, 0.0)
        elseif flag == -1 then
            running = false
        end
        sync()
    end
    while robot:getRobotState():isSteady() == false do
    end
    -- When the robot arm is moving, setServoMode will fail
    robot:getMotionControl():setServoMode(false)
end


)";

#define LOCAL_IP "127.0.0.1"

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
    rpc_cli->connect(LOCAL_IP, 30004);
    // API call: Login
    rpc_cli->login("aubo", "123456");

    auto rtde_cli = std::make_shared<RtdeClient>();
    // API call: Connect to RTDE service
    rtde_cli->connect(LOCAL_IP, 30010);
    // API call: Login
    rtde_cli->login("aubo", "123456");

    auto script_cli = std::make_shared<ScriptClient>();
    // API call: Connect to SCRIPT service
    script_cli->connect(LOCAL_IP, 30002);
    // API call: Login
    script_cli->login("aubo", "123456");

    // API call: Set topic
    int chanel_out = rtde_cli->setTopic(
        true,
        { "input_int_registers_0", "input_double_registers_0",
          "input_double_registers_1", "input_double_registers_2",
          "input_double_registers_3", "input_double_registers_4",
          "input_double_registers_5" },
        200, 0);

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
        std::cerr << "The number of waypoints in the trajectory file is 0." << std::endl;
        return 0;
    }

    // API call: Get robot's name
    auto robot_name = rpc_cli->getRobotNames().front();

    auto robot_interface = rpc_cli->getRobotInterface(robot_name);

    // API call: Set robot arm speed ratio
    robot_interface->getMotionControl()->setSpeedFraction(0.3);

    // API call: Move joint to the first waypoint in the trajectory file
    rpc_cli->getRobotInterface(robot_name)
        ->getMotionControl()
        ->moveJoint(traj[0], 30 * (M_PI / 180), 30 * (M_PI / 180), 0., 0.);

    // Blocking
    int ret = waitArrival(robot_interface);
    if (ret == 0) {
        std::cout << "Joint moved to the first waypoint in the trajectory file successfully" << std::endl;
    } else {
        std::cout << "Joint failed to move to the first waypoint in the trajectory file" << std::endl;
    }

    // API call: Execute local script
    script_cli->sendString(SERVOJ_SCRIPT);

    size_t i = 1;
    while (1) {
        // API call: Publish topic
        rtde_cli->publish(chanel_out, [&](OutputBuilder &ob) {
            ob.push(6);
            ob.push(traj[i][0]);
            ob.push(traj[i][1]);
            ob.push(traj[i][2]);
            ob.push(traj[i][3]);
            ob.push(traj[i][4]);
            ob.push(traj[i][5]);
            std::cout << i << ": " << traj[i] << std::endl;
            i++;
        });

        if (i == traj_sz) {
            // API call: Publish topic
            rtde_cli->publish(chanel_out, [&](OutputBuilder &ob) {
                std::cout << i << ": end" << std::endl;
                ob.push(-1);
                ob.push(0);
                ob.push(0);
                ob.push(0);
                ob.push(0);
                ob.push(0);
                ob.push(0);
                i++;
            });
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // Blocking to ensure execution of setServoMode(false) in the script to exit servo mode
    while (robot_interface->getMotionControl()->isServoModeEnabled() == true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    std::cout << "servoj motion ended" << std::endl;

    // API call: Remove topic
    rtde_cli->removeTopic(false, chanel_out);

    // API call: RPC logout
    rpc_cli->logout();
    // API call: RPC disconnect
    rpc_cli->disconnect();
    // API call: RTDE logout
    rtde_cli->logout();
    // API call: RTDE disconnect
    rtde_cli->disconnect();
    // API call: SCRIPT logout
    script_cli->logout();
    // API call: SCRIPT disconnect
    script_cli->disconnect();

    return 0;
}
