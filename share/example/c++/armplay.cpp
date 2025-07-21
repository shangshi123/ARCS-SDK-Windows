#include <cstring>
#include <fstream>
#include <math.h>
#ifdef WIN32
#include <windows.h>
#endif

#include <nlohmann/json.hpp>
#include "aubo_sdk/rpc.h"

using namespace arcs::aubo_sdk;
using namespace arcs::common_interface;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace nlohmann;

// Implement blocking functionality: the program continues only after the robot arm reaches the target waypoint
int waitArrival(RobotInterfacePtr impl)
{
    // API call: Get the current motion command ID
    int exec_id = impl->getMotionControl()->getExecId();

    int cnt = 0;
    // Maximum retry count for getting exec_id while waiting for the robot arm to start moving
    int max_retry_count = 50;

    // Wait for the robot to start moving
    while (exec_id == -1) {
        if (cnt++ > max_retry_count) {
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        exec_id = impl->getMotionControl()->getExecId();
    }

    // Wait for the robot to reach the target
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
    // Times called pathBufferValid
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

int main(int argc, char **argv)
{
#ifdef WIN32
    // Set Windows console output code page to UTF-8
    SetConsoleOutputCP(CP_UTF8);
#endif

    // Read trajectory file
    std::ifstream f("../trajs/aubo-joint-test-0929-10.armplay");

    if (!f.is_open()) {
        std::cerr << "Can't open traj file. Please check if path is correct."
                  << std::endl;
        return 0;
    }

    // parse the trajectory file
    auto input = json::parse(f);

    std::vector<std::vector<double>> traj = input["jointlist"];
    auto traj_sz = traj.size();
    if (traj_sz == 0) {
        std::cerr << "Number of waypoints in the trajectory file is 0" << std::endl;
        return 0;
    } else {
        std::cout << " Number of loaded waypoints: " << traj.size() << std::endl;
    }

    auto rpc = std::make_shared<RpcClient>();
    // API call: Set RPC timeout
    rpc->setRequestTimeout(1000);
    // API call: Connect to RPC service
    rpc->connect(LOCAL_IP, 30004);
    // API call: Login
    rpc->login("aubo", "123456");

    // Function call: get robot name
    auto robot_name = rpc->getRobotNames().front();

    auto robot_interface = rpc->getRobotInterface(robot_name);

    // Function call: set robot speed fraction
    robot_interface->getMotionControl()->setSpeedFraction(0.3);

    // Functon call: move joint to first point in trajectory file
    robot_interface->getMotionControl()->moveJoint(traj[0], 30 * (M_PI / 180),
                                                   30 * (M_PI / 180), 0., 0.);

    // Buffer to wait for the robot to reach the first waypoint
    int ret = waitArrival(robot_interface);
    if (ret == 0) {
        std::cout << "Successfully moved joint to the first waypoint in the trajectory file" << std::endl;
    } else {
        std::cout << "Failed to move joint to the first waypoint in the trajectory file" << std::endl;
    }

    // API call: Start the planner
    rpc->getRuntimeMachine()->start();
    auto cur_plan_context = rpc->getRuntimeMachine()->getPlanContext();
    // API call: Get new thread id
    auto task_id = rpc->getRuntimeMachine()->newTask();
    rpc->getRuntimeMachine()->setPlanContext(
        task_id, std::get<1>(cur_plan_context), std::get<2>(cur_plan_context));

    // API call: Clear buffer "rec"
    robot_interface->getMotionControl()->pathBufferFree("rec");

    // API call: Create a new buffer "rec", and specify trajectory motion type and number of trajectory points
    robot_interface->getMotionControl()->pathBufferAlloc("rec", 2, traj_sz);
    // Add waypoints from the trajectory file to the path buffer,
    // Group every 10 points as one group,
    // If the number of remaining waypoints to be added is less than or equal to 10, add them as the last group
    size_t offset = 10;
    auto it = traj.begin();
    while (true) {
        std::cout << "Add trajectory waypoint " << offset << std::endl;
        // API call: Add trajectory waypoints to the cached path
        robot_interface->getMotionControl()->pathBufferAppend(
            "rec", std::vector<std::vector<double>>{ it, it + 10 });
        it += 10;
        if (offset + 10 >= traj_sz) {
            std::cout << "Add trajectory waypoint " << traj_sz << std::endl;
            // API call: Add trajectory waypoints to the cached path
            robot_interface->getMotionControl()->pathBufferAppend(
                "rec", std::vector<std::vector<double>>{ it, traj.end() });
            break;
        }

        offset += 10;
    }

    // Sampling interval in the trajectory file
    double interval = input["interval"];
    //    double interval = 0.02;

    // API call: Time-consuming operations such as calculation and optimization to optimize the trajectory
    robot_interface->getMotionControl()->pathBufferEval(
        "rec", { 1, 1, 1, 1, 1, 1 }, { 1, 1, 1, 1, 1, 1 }, interval);

    // Check if the path buffer is valid
    if (isBufferValid(robot_interface) == -1) {
        std::cerr << "Path buffer is invalid, unable to perform trajectory motion" << std::endl;
    } else {
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

    // API call: Stop the planner
    rpc->getRuntimeMachine()->stop();
    // API call: Delete thread
    rpc->getRuntimeMachine()->deleteTask(task_id);

    // API call: RPC logout
    rpc->logout();
    // API call: RPC disconnect
    rpc->disconnect();

    return 0;
}
