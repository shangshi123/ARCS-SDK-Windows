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

// Template function: Print variable of type std::vector<T>
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

// Define enum for coordinate system types
enum CoordinateSystem
{
    BASE_COORDINATE_SYSTEM = 1,
    TOOL_COORDINATE_SYSTEM = 2,
    USER_COORDINATE_SYSTEM = 3
};

// Define enum for action types
enum ActionType
{
    POSITION_MOVE,    // Position movement
    ORIENTATION_MOVE, // Orientation rotation
    STOP_MOVE,        // Stop movement
    EXIT_PROGRAM      // Exit program
};

// Define struct for user input
struct UserInput
{
    ActionType actionType;  // Action type
    char axis;              // Coordinate axis
    bool positiveDirection; // Axis direction
};

std::vector<double> selectedCoordinate(CoordinateSystem coordinate,
                                       RpcClientPtr impl)
{
    // API call: Get robot name
    auto robot_name = impl->getRobotNames().front();

    std::vector<double> frame(6, 0.);

    switch (coordinate) {
    case BASE_COORDINATE_SYSTEM:
        frame = std::vector<double>(6, 0.);
        break;
    case TOOL_COORDINATE_SYSTEM:
        frame =
            impl->getRobotInterface(robot_name)->getRobotState()->getTcpPose();
        frame[0] = 0;
        frame[1] = 0;
        frame[2] = 0;
        break;
    case USER_COORDINATE_SYSTEM:
        // TCP pose under base coordinate system
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

        // API call: Get user coordinate pose relative to base coordinate
        auto [user_on_base, ret] = impl->getMath()->calibrateCoordinate(
            { coord_p0, coord_p1, coord_p2 }, 0);

        frame = user_on_base;
        frame[0] = 0;
        frame[1] = 0;
        frame[2] = 0;
        break;
    }
    return frame;
}

// Position and orientation teaching in different coordinate systems
void exampleSpeedLine(RpcClientPtr impl)
{
    int input_coord;
    std::cout << "Please select coordinate system:" << std::endl;
    std::cout << "1. Base coordinate system" << std::endl;
    std::cout << "2. Tool coordinate system" << std::endl;
    std::cout << "3. User coordinate system" << std::endl;
    std::cout << "Enter option number: ";
    std::cin >> input_coord;

    if (input_coord != 1 && input_coord != 2 && input_coord != 3) {
        std::cerr << "Invalid input value" << std::endl;
        std::cerr << "Valid input values: "
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
        { "s", { STOP_MOVE, 'x', true } },
        { "exit", { EXIT_PROGRAM, 'x', true } }
    };

    // Initialize loop control variable
    bool continue_loop = true;

    int cnt = 0;
    while (continue_loop) {
        std::cout << "Please enter the axis for robot movement: " << std::endl;
        // Show valid input prompt
        if (cnt++ == 0) {
            std::cout << "Valid input values: "
                         "x+, x-, y+, y-, z+, z-, rx+, rx-, ry+, ry-, rz+"
                         ", rz-, s, exit"
                      << std::endl;
            std::cout
                << "x+ means linear movement in positive x direction, x-"
                   " means linear movement in negative x direction, y+"
                   " means linear movement in positive y direction, y-"
                   " means linear movement in negative y direction, z+ means linear movement in positive z direction, z-"
                   " means linear movement in negative z direction, rx+"
                   " means rotational movement in positive x direction, rx-"
                   " means rotational movement in negative x direction, ry+"
                   " means rotational movement in positive y direction, ry-"
                   " means rotational movement in negative y direction, rz+"
                   " means rotational movement in positive z direction, rz-"
                   " means rotational movement in negative z direction, s means stop movement, exit means exit loop"
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
            std::cerr << "Valid input values: "
                         "x+, x-, y+, y-, z+, z-, rx+, rx-, ry+, ry-, rz+"
                         ", rz-, s, exit"
                      << std::endl;
            continue;
        }

        // API call: Get robot name
        auto robot_name = impl->getRobotNames().front();

        auto robot_interface = impl->getRobotInterface(robot_name);

        // API call: Set robot speed fraction
        robot_interface->getMotionControl()->setSpeedFraction(0.75);

        UserInput userInput = keymap[input_axis];

        std::vector<double> speed(6, 0.);

        // speedLine speed
        double tcp_speed = 0.25;
        double a = 1.2;
        tcp_speed = userInput.positiveDirection ? tcp_speed : -tcp_speed;

        std::vector<double> frame(6, 0.);

        if (userInput.actionType == EXIT_PROGRAM) {
            // Exit loop
            continue_loop = false;
        } else if (userInput.actionType == STOP_MOVE) {
            // API call: Stop position/orientation teaching
            robot_interface->getMotionControl()->stopLine(10, 10);

        } else if (userInput.actionType == POSITION_MOVE) {
            frame = selectedCoordinate(selected_coord, impl);

            // Position movement
            switch (userInput.axis) {
            case 'x':
                speed[0] = tcp_speed;
                break;
            case 'y':
                speed[1] = tcp_speed;
                break;
            case 'z':
                speed[2] = tcp_speed;
                break;
            }

            speed = impl->getMath()->poseTrans(frame, speed);
            speed[3] = 0;
            speed[4] = 0;
            speed[5] = 0;

            robot_interface->getMotionControl()->speedLine(speed, a, 100);
        } else if (userInput.actionType == ORIENTATION_MOVE) {
            frame = selectedCoordinate(selected_coord, impl);
            // Orientation rotation
            switch (userInput.axis) {
            case 'x':
                speed[0] = tcp_speed;
                break;
            case 'y':
                speed[1] = tcp_speed;
                break;
            case 'z':
                speed[2] = tcp_speed;
                break;
            }

            speed = impl->getMath()->poseTrans(frame, speed);

            speed[3] = speed[0];
            speed[4] = speed[1];
            speed[5] = speed[2];
            speed[0] = 0;
            speed[1] = 0;
            speed[2] = 0;

            robot_interface->getMotionControl()->speedLine(speed, a, 100);
        }
    }
}

// Print log information
void printlog(int level, const char *source, int code, std::string content)
{
    static const char *level_names[] = { "Critical", "Error", "Warning",
                                         "Info",     "Debug", "BackTrace" };
    fprintf(stderr, "[%s] %s - %d %s\n", level_names[level], source, code,
            content.c_str());
}

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

    // API call: Set RTDE topic
    int topic = rtde_cli->setTopic(false, { "R1_message" }, 200, 0);
    if (topic < 0) {
        std::cout << "Failed to set topic!" << std::endl;
        return -1;
    }

    // API call: Subscribe to topic
    rtde_cli->subscribe(topic, [](InputParser &parser) {
        arcs::common_interface::RobotMsgVector msgs;
        msgs = parser.popRobotMsgVector();
        for (size_t i = 0; i < msgs.size(); i++) {
            auto &msg = msgs[i];
            std::string error_content =
                arcs::error_stack::errorCode2Str(msg.code);
            for (auto it : msg.args) {
                auto pos = error_content.find("{}");
                if (pos != std::string::npos) {
                    error_content.replace(pos, 2, it);
                } else {
                    break;
                }
            }
            // Print log information
            printlog(msg.level, msg.source.c_str(), msg.code, error_content);
        }
    });

    // Position and orientation teaching in different coordinate systems
    exampleSpeedLine(rpc_cli);

    // API call: Remove topic
    rtde_cli->removeTopic(false, topic);
    // API call: Logout
    rtde_cli->logout();
    // API call: Disconnect
    rtde_cli->disconnect();

    // API call: Logout
    rpc_cli->logout();
    // API call: Disconnect
    rpc_cli->disconnect();

    return 0;
}
