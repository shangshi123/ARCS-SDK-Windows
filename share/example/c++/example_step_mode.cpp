#include "aubo_sdk/rpc.h"
#include "aubo_sdk/rtde.h"
#include <aubo/error_stack/error_stack.h>
#include <iostream>
#include <map>
#include <vector>
#include <unordered_set>
#ifdef WIN32
#include <windows.h>
#endif

using namespace arcs::common_interface;
using namespace arcs::aubo_sdk;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
// Template function: Print variables of type std::vector<T>
template <typename T>
void printVec(std::vector<T> param, std::string name)
{
    std::cout << name << ": ";

    for (int i = 0; i < param.size(); i++) {
        std::cout << param.at(i);
        if (i != param.size() - 1) {
            std::cout << ", ";
        }
    }
    std::cout << std::endl;
}

// Define enumeration for coordinate system types
enum CoordinateSystem
{
    BASE_COORDINATE_SYSTEM = 1,
    TOOL_COORDINATE_SYSTEM = 2,
    USER_COORDINATE_SYSTEM = 3
};

// Define enumeration for action types
enum ActionType
{
    POSITION_MOVE,    // Position movement
    ORIENTATION_MOVE, // Orientation rotation
    EXIT_PROGRAM      // Exit program
};

// Define struct for user input
struct UserInput
{
    ActionType actionType;  // Action type
    char axis;              // Coordinate axis
    bool positiveDirection; // Axis direction
};

// Implement blocking function: The program continues when the robot arm reaches the target waypoint
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

std::vector<double> selectedCoordinate(CoordinateSystem coordinate,
                                       RpcClientPtr impl,
                                       std::vector<double> shift,
                                       std::vector<double> pose)
{
    // API call: Get the robot's name
    auto robot_name = impl->getRobotNames().front();

    std::vector<double> target_pose(6, 0.);

    switch (coordinate) {
    case BASE_COORDINATE_SYSTEM:
        // Teaching movement in base coordinate system, rotation based on fixed coordinate system
        target_pose = impl->getMath()->poseTrans(shift, pose);
        break;
    case TOOL_COORDINATE_SYSTEM:
        // End tool coordinate system, rotation based on end tool coordinate system
        target_pose = impl->getMath()->poseTrans(pose, shift);
        break;
    case USER_COORDINATE_SYSTEM:
        // TCP pose in base coordinate system
        std::vector<double> coord_p0(6), coord_p1(6), coord_p2(6);
        coord_p0[0] = 0.49946;
        coord_p0[1] = 0.32869;
        coord_p0[2] = 0.35522;
        coord_p0[3] = 2.414;
        coord_p0[4] = 0.0;
        coord_p0[5] = 2.357;

        coord_p1[0] = 0.58381;
        coord_p1[1] = 0.41325;
        coord_p1[2] = 0.22122;
        coord_p1[3] = 2.414;
        coord_p1[4] = 0.0;
        coord_p1[5] = 2.357;

        coord_p2[0] = 0.58826;
        coord_p2[1] = 0.41772;
        coord_p2[2] = 0.46730;
        coord_p2[3] = 2.414;
        coord_p2[4] = 0.0;
        coord_p2[5] = 2.357;

        // API call: Get the pose of the user coordinate system relative to the base coordinate system
        auto [user_on_base, ret] = impl->getMath()->calibrateCoordinate(
            { coord_p0, coord_p1, coord_p2 }, 0);

        user_on_base[0] = 0;
        user_on_base[1] = 0;
        user_on_base[2] = 0;

        auto user_on_base_inv = impl->getMath()->poseInverse(user_on_base);

        // API call: Get the current pose of TCP in the user coordinate system
        auto tcp_on_user = impl->getMath()->poseTrans(user_on_base_inv, pose);

        // Do orientation transformation based on fixed coordinate system
        auto utt = impl->getMath()->poseTrans(shift, tcp_on_user);
        target_pose = impl->getMath()->poseTrans(user_on_base, utt);
        break;
    }
    return target_pose;
}

void exampleStepMode(RpcClientPtr impl)
{
    int input_coord;
    std::cout << "Please select coordinate system:" << std::endl;
    std::cout << "1. Base coordinate system" << std::endl;
    std::cout << "2. Tool coordinate system" << std::endl;
    std::cout << "3. User coordinate system" << std::endl;
    std::cout << "Please enter option number:";
    std::cin >> input_coord;

    if (input_coord != 1 && input_coord != 2 && input_coord != 3) {
        std::cerr << "Invalid input value" << std::endl;
        std::cerr << "Valid input values are: "
                     "1, 2, 3"
                  << std::endl;
        return;
    }

    CoordinateSystem selected_coord;

    switch (input_coord) {
    case 1:
        selected_coord = BASE_COORDINATE_SYSTEM;
        break;

    case 2:
        selected_coord = TOOL_COORDINATE_SYSTEM;
        break;

    case 3:
        selected_coord = USER_COORDINATE_SYSTEM;
        break;
    }

    // Input string
    std::string input_axis;

    // Define input string and key mapping
    std::map<std::string, UserInput> keymap = {
        { "x+", { POSITION_MOVE, 'x', true } },
        { "x-", { POSITION_MOVE, 'x', false } },
        { "y+", { POSITION_MOVE, 'y', true } },
        { "y-", { POSITION_MOVE, 'y', false } },
        { "z+", { POSITION_MOVE, 'z', true } },
        { "z-", { POSITION_MOVE, 'z', false } },
        { "rx+", { ORIENTATION_MOVE, 'x', true } },
        { "rx-", { ORIENTATION_MOVE, 'x', false } },
        { "ry+", { ORIENTATION_MOVE, 'y', true } },
        { "ry-", { ORIENTATION_MOVE, 'y', false } },
        { "rz+", { ORIENTATION_MOVE, 'z', true } },
        { "rz-", { ORIENTATION_MOVE, 'z', false } },
        { "exit", { EXIT_PROGRAM, 'x', true } }
    };

    // Initialize loop control variable
    bool continue_loop = true;

    int cnt = 0;
    while (continue_loop) {
        std::cout << "Please enter the axis for robot arm movement: " << std::endl;
        // Display valid input prompt
        if (cnt++ == 0) {
            std::cout << "Valid input values are: "
                         "x+, x-, y+, y-, z+, z-, rx+, rx-, ry+, ry-, rz+"
                         ", rz-, s, exit"
                      << std::endl;
            std::cout << "x+ means positive direction position step on x axis, x-"
                         "means negative direction position step on x axis, y+"
                         "means positive direction position step on y axis, y-"
                         "means negative direction position step on y axis, z+ means positive direction position step on z axis, z-"
                         "means negative direction position step on z axis, rx+"
                         "means positive direction orientation step on x axis, rx-"
                         "means negative direction orientation step on x axis, ry+"
                         "means positive direction orientation step on y axis, ry-"
                         "means negative direction orientation step on y axis, rz+"
                         "means positive direction orientation step on z axis, rz-"
                         "means negative direction orientation step on z axis, exit means exit loop"
                      << std::endl;
        }
        std::cin >> input_axis;

        // Define set of valid input values
        std::unordered_set<std::string> validInputs = {
            "x+",  "x-",  "y+",  "y-",  "z+",  "z-", "rx+",
            "rx-", "ry+", "ry-", "rz+", "rz-", "s",  "exit"
        };

        if (validInputs.find(input_axis) == validInputs.end()) {
            std::cerr << "Invalid input value" << std::endl;
            std::cerr << "Valid input values are: "
                         "x+, x-, y+, y-, z+, z-, rx+, rx-, ry+, ry-, rz+"
                         ", rz-, s, exit"
                      << std::endl;
            continue;
        }

        // API call: Get the robot's name
        auto robot_name = impl->getRobotNames().front();

        auto robot_interface = impl->getRobotInterface(robot_name);

        // API call: Set tool center point (TCP offset relative to flange center)
        std::vector<double> tcp_offset(6, 0.0);
        robot_interface->getRobotConfig()->setTcpOffset(tcp_offset);

        // API call: Set robot arm speed fraction
        robot_interface->getMotionControl()->setSpeedFraction(0.75);

        UserInput userInput = keymap[input_axis];

        std::vector<double> shift(6, 0.);

        // Position step size, unit: m
        double pos_dist = 0.005;
        pos_dist = userInput.positiveDirection ? pos_dist : -pos_dist;

        // Orientation step size, unit: rad
        double ori_dist = 0.1;
        ori_dist = userInput.positiveDirection ? ori_dist : -ori_dist;

        // API call: Get current TCP pose in Base coordinate system
        std::vector<double> current_tcp_on_base(6, 0.0);
        current_tcp_on_base = robot_interface->getRobotState()->getTcpPose();

        // Target pose
        std::vector<double> target_pose_on_base(6, 0.);

        if (userInput.actionType == EXIT_PROGRAM) {
            // Exit loop
            continue_loop = false;

        } else if (userInput.actionType == POSITION_MOVE) {
            // Position movement
            switch (userInput.axis) {
            case 'x':
                shift[0] = pos_dist;
                break;
            case 'y':
                shift[1] = pos_dist;
                break;
            case 'z':
                shift[2] = pos_dist;
                break;
            }

            target_pose_on_base = selectedCoordinate(
                selected_coord, impl, shift, current_tcp_on_base);

            robot_interface->getMotionControl()->moveLine(target_pose_on_base,
                                                          1.2, 0.25, 0, 0);

            // Blocking
            int ret = waitArrival(robot_interface);
            if (ret == 0) {
                std::cout << "Linear movement to target position succeeded!" << std::endl;
            } else {
                std::cout << "Linear movement to target position failed!" << std::endl;
            }
        } else if (userInput.actionType == ORIENTATION_MOVE) {
            // Orientation rotation
            switch (userInput.axis) {
            case 'x':
                shift[3] = ori_dist;
                break;
            case 'y':
                shift[4] = ori_dist;
                break;
            case 'z':
                shift[5] = ori_dist;
                break;
            }

            target_pose_on_base = selectedCoordinate(
                selected_coord, impl, shift, current_tcp_on_base);

            target_pose_on_base[0] = current_tcp_on_base[0];
            target_pose_on_base[1] = current_tcp_on_base[1];
            target_pose_on_base[2] = current_tcp_on_base[2];

            robot_interface->getMotionControl()->moveLine(target_pose_on_base,
                                                          1.2, 0.25, 0, 0);
            // Blocking
            int ret = waitArrival(robot_interface);
            if (ret == 0) {
                std::cout << "Linear movement to target position succeeded!" << std::endl;
            } else {
                std::cout << "Linear movement to target position failed!" << std::endl;
            }
        }
    }
}
