#include "aubo_sdk/rpc.h"
#ifdef WIN32
#include <windows.h>
#endif

using namespace arcs::aubo_sdk;

// Template: Print variables of type std::vector<T>
template <typename T> void printVec(std::vector<T> param, std::string name) {
  std::cout << name << ": ";
  for (int i = 0; i < param.size(); i++) {
    std::cout << param.at(i);
    if (i != param.size() - 1) {
      std::cout << ", ";
    }
  }
  std::cout << std::endl;
}

// Get robot arm state
void exampleState(RpcClientPtr cli) {
    // API call: Get the robot's name
    auto robot_name = cli->getRobotNames().front();

    // API call: Get the robot's mode state
    RobotModeType robot_mode_type;
    robot_mode_type =
            cli->getRobotInterface(robot_name)->getRobotState()->getRobotModeType();
    std::cout << "Robot mode state:" << robot_mode_type << std::endl;

    // API call: Get safety mode
    SafetyModeType safety_mode_type;
    safety_mode_type =
            cli->getRobotInterface(robot_name)->getRobotState()->getSafetyModeType();
    std::cout << "Safety mode:" << safety_mode_type << std::endl;

    // API call: Is the robot in a steady state
    bool is_steady;
    is_steady = cli->getRobotInterface(robot_name)->getRobotState()->isSteady();
    std::cout << "Is the robot in a steady state:" << (is_steady ? "Yes" : "No")
                        << std::endl;

    // API call: Is the robot within safety limits
    bool is_within_safety_limits;
    is_within_safety_limits = cli->getRobotInterface(robot_name)
                                                                ->getRobotState()
                                                                ->isWithinSafetyLimits();
    std::cout << "Is the robot within safety limits:"
                        << (is_within_safety_limits ? "Yes" : "No") << std::endl;

    // API call: Get the actual TCP pose in the base coordinate system
    std::vector<double> actual_tcp_pose;
    actual_tcp_pose =
            cli->getRobotInterface(robot_name)->getRobotState()->getTcpPose();
    printVec<double>(actual_tcp_pose, "Actual TCP pose in base coordinate system");

    // API call: Get the actual pose of the flange center in the base coordinate system
    std::vector<double> actual_flange_pose;
    actual_flange_pose =
            cli->getRobotInterface(robot_name)->getRobotState()->getToolPose();
    printVec<double>(actual_flange_pose, "Actual flange center pose in base coordinate system");

    // API call: Get robot arm joint state
    std::vector<JointStateType> joint_state;
    joint_state =
            cli->getRobotInterface(robot_name)->getRobotState()->getJointState();
    printVec<JointStateType>(joint_state, "Robot arm joint state");

    // API call: Get joint servo state
    std::vector<JointServoModeType> joint_servo_mode;
    joint_servo_mode =
            cli->getRobotInterface(robot_name)->getRobotState()->getJointServoMode();
    printVec<JointServoModeType>(joint_servo_mode, "Joint servo state");

    // API call: Get actual joint position angles of the robot arm
    std::vector<double> joint_positions;
    joint_positions =
            cli->getRobotInterface(robot_name)->getRobotState()->getJointPositions();
    printVec<double>(joint_positions, "Actual joint position angles of the robot arm");

    // API call: Get joint currents of the robot arm
    std::vector<double> joint_currents;
    joint_currents =
            cli->getRobotInterface(robot_name)->getRobotState()->getJointCurrents();
    printVec<double>(joint_currents, "Joint currents of the robot arm");

    // API call: Get joint voltages of the robot arm
    std::vector<double> joint_voltages;
    joint_voltages =
            cli->getRobotInterface(robot_name)->getRobotState()->getJointVoltages();
    printVec<double>(joint_voltages, "Joint voltages of the robot arm");

    // API call: Get joint temperatures of the robot arm
    std::vector<double> joint_temperatures;
    joint_temperatures = cli->getRobotInterface(robot_name)
                                                     ->getRobotState()
                                                     ->getJointTemperatures();
    printVec<double>(joint_temperatures, "Joint temperatures of the robot arm");

    // API call: Get joint UUIDs
    std::vector<std::string> joint_unique_ids;
    joint_unique_ids =
            cli->getRobotInterface(robot_name)->getRobotState()->getJointUniqueIds();
    printVec<std::string>(joint_unique_ids, "Joint UUIDs");

    // API call: Get joint firmware versions
    std::vector<int> joint_firmware_versions;
    joint_firmware_versions = cli->getRobotInterface(robot_name)
                                                                ->getRobotState()
                                                                ->getJointFirmwareVersions();
    printVec<int>(joint_firmware_versions, "Joint firmware versions");

    // API call: Get joint hardware versions
    std::vector<int> joint_hardward_versions;
    joint_hardward_versions = cli->getRobotInterface(robot_name)
                                                                ->getRobotState()
                                                                ->getJointHardwareVersions();
    printVec<int>(joint_hardward_versions, "Joint hardware versions");

    // API call: Get master board firmware UUID
    std::string master_board_unique_id;
    master_board_unique_id = cli->getRobotInterface(robot_name)
                                                             ->getRobotState()
                                                             ->getMasterBoardUniqueId();
    std::cout << "Master board firmware UUID: " << master_board_unique_id << std::endl;

    // API call: Get master board firmware version
    int master_board_firmware_version;
    master_board_firmware_version = cli->getRobotInterface(robot_name)
                                                                            ->getRobotState()
                                                                            ->getMasterBoardFirmwareVersion();
    std::cout << "Master board firmware version: " << master_board_firmware_version
                        << std::endl;

    // API call: Get master board hardware version
    int master_board_hardware_version;
    master_board_hardware_version = cli->getRobotInterface(robot_name)
                                                                            ->getRobotState()
                                                                            ->getMasterBoardHardwareVersion();
    std::cout << "Master board hardware version: " << master_board_hardware_version
                        << std::endl;

    // API call: Get slave board firmware UUID
    std::string slave_board_unique_id;
    slave_board_unique_id = cli->getRobotInterface(robot_name)
                                                            ->getRobotState()
                                                            ->getSlaveBoardUniqueId();
    std::cout << "Slave board firmware UUID: " << slave_board_unique_id << std::endl;

    // API call: Get slave board firmware version
    int slave_board_firmware_version;
    slave_board_firmware_version = cli->getRobotInterface(robot_name)
                                                                         ->getRobotState()
                                                                         ->getSlaveBoardFirmwareVersion();
    std::cout << "Slave board firmware version: " << slave_board_firmware_version
                        << std::endl;

    // API call: Get slave board hardware version
    int slave_board_hardware_version;
    slave_board_hardware_version = cli->getRobotInterface(robot_name)
                                                                         ->getRobotState()
                                                                         ->getSlaveBoardHardwareVersion();
    std::cout << "Slave board hardware version: " << slave_board_hardware_version
                        << std::endl;

    // API call: Get tool UUID
    std::string tool_unique_id;
    tool_unique_id =
            cli->getRobotInterface(robot_name)->getRobotState()->getToolUniqueId();
    std::cout << "Tool UUID: " << tool_unique_id << std::endl;

    // API call: Get tool firmware version
    int tool_firmware_version;
    tool_firmware_version = cli->getRobotInterface(robot_name)
                                                            ->getRobotState()
                                                            ->getToolFirmwareVersion();
    std::cout << "Tool firmware version: " << tool_firmware_version << std::endl;

    // API call: Get tool hardware version
    int tool_hardware_version;
    tool_hardware_version = cli->getRobotInterface(robot_name)
                                                            ->getRobotState()
                                                            ->getToolHardwareVersion();
    std::cout << "Tool hardware version: " << tool_hardware_version << std::endl;

    // API call: Get pedestal UUID
    std::string pedestal_unique_id;
    pedestal_unique_id = cli->getRobotInterface(robot_name)
                                                     ->getRobotState()
                                                     ->getPedestalUniqueId();
    std::cout << "Pedestal UUID: " << pedestal_unique_id << std::endl;

    // API call: Get pedestal firmware version
    int pedestal_firmware_version;
    pedestal_firmware_version = cli->getRobotInterface(robot_name)
                                                                    ->getRobotState()
                                                                    ->getPedestalFirmwareVersion();
    std::cout << "Pedestal firmware version: " << pedestal_firmware_version << std::endl;

    // API call: Get pedestal hardware version
    int pedestal_hardware_version;
    pedestal_hardware_version = cli->getRobotInterface(robot_name)
                                                                    ->getRobotState()
                                                                    ->getPedestalHardwareVersion();
    std::cout << "Pedestal hardware version: " << pedestal_hardware_version << std::endl;

    // API call: Get robot arm joint target position angles
    std::vector<double> joint_target_positions;
    joint_target_positions = cli->getRobotInterface(robot_name)
                                                             ->getRobotState()
                                                             ->getJointTargetPositions();
    printVec<double>(joint_target_positions, "Joint target position angles");

    // API call: Get control box temperature
    double control_box_temperature;
    control_box_temperature = cli->getRobotInterface(robot_name)
                                                                ->getRobotState()
                                                                ->getControlBoxTemperature();
    std::cout << "Control box temperature: " << control_box_temperature << std::endl;

    // API call: Get main bus voltage
    double main_voltage;
    main_voltage =
            cli->getRobotInterface(robot_name)->getRobotState()->getMainVoltage();
    std::cout << "Main bus voltage: " << main_voltage << std::endl;

    // API call: Get main bus current
    double main_current;
    main_current =
            cli->getRobotInterface(robot_name)->getRobotState()->getMainCurrent();
    std::cout << "Main bus current: " << main_current << std::endl;

    // API call: Get robot voltage
    double robot_voltage;
    robot_voltage =
            cli->getRobotInterface(robot_name)->getRobotState()->getRobotVoltage();
    std::cout << "Robot voltage: " << robot_voltage << std::endl;

    // API call: Get robot current
    double robot_current;
    robot_current =
            cli->getRobotInterface(robot_name)->getRobotState()->getRobotCurrent();
    std::cout << "Robot current: " << robot_current << std::endl;
}

#define ip_local "127.0.0.1"

/**
 * Function: Get robot arm state
 * Steps:
 * Step 1: Set RPC timeout, connect to RPC service, login
 * Step 2: Get robot arm state
 * Step 3: Logout, disconnect RPC connection
 */
int main(int argc, char **argv) {
#ifdef WIN32
    // Set Windows console output code page to UTF-8
    SetConsoleOutputCP(CP_UTF8);
#endif

    auto rpc_cli = std::make_shared<RpcClient>();
    // API call: Set RPC timeout
    rpc_cli->setRequestTimeout(1000);
    // API call: Connect to RPC service
    rpc_cli->connect(ip_local, 30004);
    // API call: Login
    rpc_cli->login("aubo", "123456");

    // Get robot arm state
    exampleState(rpc_cli);

    // API call: Logout
    rpc_cli->logout();
    // API call: Disconnect
    rpc_cli->disconnect();

    return 0;
}
