#include <mutex>
#include "aubo_sdk/rpc.h"
#include "aubo_sdk/rtde.h"
#include "thread"
#ifdef WIN32
#include <windows.h>
#endif

using namespace arcs::common_interface;
using namespace arcs::aubo_sdk;
std::mutex rtde_mtx_;

uint64_t in_bit_lower_ = 0;
uint64_t in_bit_upper_ = 0;
int in_int_0_ = -1;
float in_float_0_ = -0.1;
double in_double_0_ = -0.1;
int16_t int16_0_ = 0;
std::vector<int16_t> vec_int16_(64, 0);
uint64_t out_bit_lower_ = 0;
uint64_t out_bit_upper_ = 0;
int out_int_0_ = -1;
float out_float_0_ = -0.1;
double out_double_0_ = -0.1;

RobotModeType robot_mode_ = RobotModeType::NoController; // Robot status
SafetyModeType safety_mode_ = SafetyModeType::Normal;    // Safety status
RuntimeState runtime_state_ = RuntimeState::Stopped; // Planner running status
double actual_main_voltage_ = 0;                     // Main voltage
double actual_robot_voltage_ = 0;                    // Robot voltage

std::vector<double> actual_q_{ std::vector<double>(
    6, 0.) }; // Current joint angle (unit: rad)
std::vector<double> actual_qd_{ std::vector<double>(
    6, 0.) }; // Current joint speed (unit: rad/s)
std::vector<double> actual_TCP_pose_{ std::vector<double>(
    6, 0.) }; // Current TCP position (unit: m) and attitude (unit: rad)
std::vector<double> actual_TCP_speed_{ std::vector<double>(
    6, 0.) }; // Current TCP position speed (unit: m/s) and attitude speed (unit: rad/s)

std::vector<JointStateType> joint_mode_{ std::vector<JointStateType>(
    6, JointStateType::Idle) }; // Joint
std::vector<double> joint_temperatures_{ std::vector<double>(6,
                                                             0.) }; // Joint temperature
std::vector<double> actual_joint_voltage_{ std::vector<double>(
    6, 0.) }; // Joint voltage
std::vector<double> actual_current_{ std::vector<double>(6, 0.) }; // Joint current

uint64_t standard_digital_input_bits_ = 0; // All digital input values (decimal)
uint64_t standard_digital_output_bits_ = 0; // All digital output values (decimal)
uint64_t tool_digital_input_bits_ = 0; // All tool digital values (decimal)
uint64_t tool_digital_output_bits_ = 0; // All tool digital values (decimal)
uint64_t configurable_digital_input_bits_ = 0; // All safety input values (decimal)
uint64_t configurable_digital_output_bits_ = 0; // All safety output values (decimal)
uint64_t link_digital_input_bits_ = 0; // All link input values (decimal)
uint64_t link_digital_output_bits_ = 0; // All link output values (decimal)
int standard_digital_input_num_ = 0;    // Digital input quantity
int standard_digital_output_num_ = 0;   // Digital output quantity
int tool_digital_input_num_ = 0;        // Tool digital input quantity
int tool_digital_output_num_ = 0;       // Tool digital output quantity
int configurable_digital_input_num_ = 0;            // Safety input quantity
int configurable_digital_output_num_ = 0;           // Safety output quantity
int link_digital_input_num_ = 0;                    // Link input quantity
int link_digital_output_num_ = 0;                   // Link output quantity
std::vector<double> standard_analog_input_values_;  // All analog input values
std::vector<double> standard_analog_output_values_; // All analog output values
std::vector<double> tool_analog_input_values_; // All tool analog input values
std::vector<double> tool_analog_output_values_; // All tool analog output values

template <typename T>
void printVec(std::vector<T> param, std::string name)
{
    std::cout << "@:" << name << std::endl;
    for (int i = 0; i < param.size(); i++) {
        std::cout << param.at(i) << ",";
    }
    std::cout << std::endl;
}

template <typename T>
void print2Vec(std::vector<T> param, std::string name)
{
    for (size_t i = 0; i < param.size(); i++) {
        if (0 == i) {
            std::cout << "@:" << name << " offset:" << param.at(i).offset
                      << std::endl;
        }

        for (size_t j = 0; j < param.at(i).value.size(); j++) {
            std::cout << param.at(i).value.at(j);
        }

        if ((param.size() - 1) == i) {
            std::cout << std::endl;
        }
    }
}

template <typename T>
void printVecVec(std::vector<std::vector<T>> param, std::string name)
{
    for (size_t i = 0; i < param.size(); i++) {
        if (0 == i) {
            std::cout << "@:" << name << " size:" << param.size() << std::endl;
        }

        for (size_t j = 0; j < param.at(i).size(); j++) {
            std::cout << param.at(i).at(j) << ",";
        }

        std::cout << std::endl;
    }
}

template <typename T>
void printSingle(T param, std::string name)
{
    std::cout << "@:" << name << " " << param << std::endl;
}

bool bit_value(uint64_t v, int index)
{
    return (v & ((uint64_t)1 << index)) ? true : false;
}

// Print binary bits
void printBit(uint64_t v, std::string name, int bit_length)
{
    std::cout << "@:" << name << std::endl;

    for (int i = 0; i < bit_length; i++) {
        std::cout << bit_value(v, i) << ",";
    }
    std::cout << std::endl;
}

// RTDE publish
void examplePublish(RtdeClientPtr cli)
{
    // API call: set topic
    int topic1 = cli->setTopic(
        true,
        { "input_bit_registers0_to_31", "input_bit_registers32_to_63",
          "input_bit_registers64_to_127", "input_int_registers_0",
          "input_float_registers_0", "input_double_registers_0",
          "input_int16_registers_0", "input_int16_registers0_to_63" },
        1, 1);

    // API call: publish topic
    cli->publish(topic1, [&](arcs::aubo_sdk::OutputBuilder &ro) {
        ro.push(0x00ff);
        ro.push(0x00);
        ro.push(0x00);
        ro.push(44);
        ro.push(3.1);
        ro.push(4.1);
        ro.push(5);
        ro.push(std::vector<int16_t>(64, 1));
    });

    // API call: set topic
    int topic2 = cli->setTopic(true, { "R1_standard_digital_output" }, 1, 2);

    // API call: publish topic
    cli->publish(topic2, [&](arcs::aubo_sdk::OutputBuilder &ro) {
        // Set DO01 to high level
        ro.push(std::tuple<int, bool>(1, true));
    });

    // API call: set topic
    int topic3 = cli->setTopic(true, { "R1_standard_analog_output" }, 1, 3);
    // API call: publish topic
    cli->publish(topic3, [&](arcs::aubo_sdk::OutputBuilder &ro) {
        // Set AO0 and AO1 both to 3.3
        ro.push(std::vector<double>(2, 3.3));
    });

    // API call: set topic
    int topic4 = cli->setTopic(true, { "R1_speed_slider_fraction" }, 1, 4);
    // API call: publish topic
    cli->publish(topic4, [&](arcs::aubo_sdk::OutputBuilder &ro) {
        // Set running speed ratio to 75%
        ro.push(0.75);
    });

    // API call: set topic
    int topic5 = cli->setTopic(
        true,
        { "R1_standard_digital_output_mask", "R1_standard_digital_output" }, 1,
        5);
    // API call: publish topic
    cli->publish(topic5, [&](arcs::aubo_sdk::OutputBuilder &ro) {
        // Set digital output (DO) mask to 255, corresponding to binary 11111111,
        // 1 means DO channel is open, 0 means DO channel is closed
        ro.push(255);
        // Set digital output (DO) output value to 255, corresponding to binary 11111111,
        // 1 means high level, 0 means low level
        ro.push(255);
    });
}

// RPC get IO quantity
void exampleIoNum(RpcClientPtr cli)
{
    // API call: get robot name
    auto robot_name = cli->getRobotNames().front();
    auto robot = cli->getRobotInterface(robot_name);

    standard_digital_input_num_ =
        robot->getIoControl()->getStandardDigitalInputNum();
    standard_digital_output_num_ =
        robot->getIoControl()->getStandardDigitalOutputNum();

    tool_digital_input_num_ = robot->getIoControl()->getToolDigitalInputNum();
    tool_digital_output_num_ = robot->getIoControl()->getToolDigitalOutputNum();

    configurable_digital_input_num_ =
        robot->getIoControl()->getConfigurableDigitalInputNum();
    configurable_digital_output_num_ =
        robot->getIoControl()->getConfigurableDigitalOutputNum();

    link_digital_input_num_ = robot->getIoControl()->getStaticLinkInputNum();
    link_digital_output_num_ = robot->getIoControl()->getStaticLinkOutputNum();
}

// RTDE subscribe
void exampleSubscribe(RtdeClientPtr cli)
{
    // API call: set topic
    int topic5 = cli->setTopic(
        false,
        { "input_bit_registers_r0_to_63", "input_bit_registers_r64_to_127",
          "input_int_registers_r0", "input_float_registers_r0",
          "input_double_registers_r0", "input_int16_registers_r0",
          "input_int16_registers_0_to_63" },
        1, 5);
    // API call: subscribe
    cli->subscribe(topic5, [](InputParser &parser) {
        std::unique_lock<std::mutex> lck(rtde_mtx_);
        in_bit_lower_ = parser.popInt64();
        in_bit_upper_ = parser.popInt64();
        in_int_0_ = parser.popInt32();
        in_float_0_ = parser.popDouble();
        in_double_0_ = parser.popDouble();
        int16_0_ = parser.popInt16();
        vec_int16_ = parser.popVectorInt16();
    });

    // API call: set topic
    int topic6 = cli->setTopic(
        false,
        { "output_bit_registers_0_to_63", "output_bit_registers_64_to_127",
          "output_int_registers_0", "output_float_registers_0",
          "output_double_registers_0" },
        1, 6);
    // API call: subscribe
    cli->subscribe(topic6, [](InputParser &parser) {
        std::unique_lock<std::mutex> lck(rtde_mtx_);
        out_bit_lower_ = parser.popInt64();
        out_bit_upper_ = parser.popInt64();
        out_int_0_ = parser.popInt32();
        out_float_0_ = parser.popDouble();
        out_double_0_ = parser.popDouble();
    });

    // API call: set topic
    int topic7 =
        cli->setTopic(false,
                      { "R1_robot_mode", "R1_safety_mode", "runtime_state",
                        "R1_actual_main_voltage", "R1_actual_robot_voltage" },
                      50, 7);
    // API call: subscribe
    cli->subscribe(topic7, [](InputParser &parser) {
        std::unique_lock<std::mutex> lck(rtde_mtx_);
        robot_mode_ = parser.popRobotModeType();
        safety_mode_ = parser.popSafetyModeType();
        runtime_state_ = parser.popRuntimeState();
        actual_main_voltage_ = parser.popDouble();
        actual_robot_voltage_ = parser.popDouble();
    });

    // API call: set topic
    int topic8 = cli->setTopic(false,
                               { "R1_actual_q", "R1_actual_qd",
                                 "R1_actual_TCP_pose", "R1_actual_TCP_speed" },
                               50, 8);
    // API call: subscribe
    cli->subscribe(topic8, [](InputParser &parser) {
        std::unique_lock<std::mutex> lck(rtde_mtx_);
        actual_q_ = parser.popVectorDouble();
        actual_qd_ = parser.popVectorDouble();
        actual_TCP_pose_ = parser.popVectorDouble();
        actual_TCP_speed_ = parser.popVectorDouble();
    });

    // API call: set topic
    int topic9 = cli->setTopic(false,
                               {

                                   "R1_joint_mode",
                                   "R1_joint_temperatures",
                                   "R1_actual_joint_voltage",
                                   "R1_actual_current",
                               },
                               1, 9);
    // API call: subscribe
    cli->subscribe(topic9, [](InputParser &parser) {
        std::unique_lock<std::mutex> lck(rtde_mtx_);
        joint_mode_ = parser.popVectorJointStateType();
        joint_temperatures_ = parser.popVectorDouble();
        actual_joint_voltage_ = parser.popVectorDouble();
        actual_current_ = parser.popVectorDouble();
    });

    // API call: set topic
    int topic10 = cli->setTopic(
        false,
        { "R1_standard_digital_input_bits", "R1_tool_digital_input_bits",
          "R1_configurable_digital_input_bits", "R1_link_digital_input_bits",
          "R1_standard_digital_output_bits", "R1_tool_digital_output_bits",
          "R1_configurable_digital_output_bits", "R1_link_digital_output_bits",
          "R1_standard_analog_input_values", "R1_tool_analog_input_values",
          "R1_standard_analog_output_values", "R1_tool_analog_output_values" },
        1, 10);
    // API call: subscribe
    cli->subscribe(topic10, [](InputParser &parser) {
        std::unique_lock<std::mutex> lck(rtde_mtx_);
        standard_digital_input_bits_ = parser.popInt64();
        tool_digital_input_bits_ = parser.popInt64();
        configurable_digital_input_bits_ = parser.popInt64();
        link_digital_input_bits_ = parser.popInt64();
        standard_digital_output_bits_ = parser.popInt64();
        tool_digital_output_bits_ = parser.popInt64();
        configurable_digital_output_bits_ = parser.popInt64();
        link_digital_output_bits_ = parser.popInt64();
        standard_analog_input_values_ = parser.popVectorDouble();
        tool_analog_input_values_ = parser.popVectorDouble();
        standard_analog_output_values_ = parser.popVectorDouble();
        tool_analog_output_values_ = parser.popVectorDouble();
    });
}

// Print update information
void update()
{
    std::unique_lock<std::mutex> lck(rtde_mtx_);
    printBit(in_bit_lower_, "input_bit_registers_0_to_63", 64);
    printBit(in_bit_upper_, "input_bit_registers_64_to_127", 64);
    printBit(out_bit_lower_, "output_bit_registers_0_to_63", 64);
    printBit(out_bit_upper_, "output_bit_registers_64_to_127", 64);

    std::cout << std::endl << "input_int_registers_0" << std::endl;
    std::cout << in_int_0_;

    std::cout << std::endl << "input_float_registers_0" << std::endl;
    std::cout << in_float_0_;

    std::cout << std::endl << "input_double_registers_0" << std::endl;
    std::cout << in_double_0_ << std::endl;

    std::cout << std::endl << "output_int_registers_0" << std::endl;
    std::cout << out_int_0_;

    std::cout << std::endl << "output_float_registers_0" << std::endl;
    std::cout << out_float_0_;

    std::cout << std::endl << "output_double_registers_0" << std::endl;
    std::cout << out_double_0_;

    printSingle<RobotModeType>(robot_mode_, "Robot status");
    printSingle<SafetyModeType>(safety_mode_, "Safety status");
    printSingle<RuntimeState>(runtime_state_, "Planner running status");
    printSingle<double>(actual_main_voltage_, "Main voltage");
    printSingle(actual_robot_voltage_, "Robot voltage");

    printVec<double>(actual_q_, "Current joint angle (unit: rad)");
    printVec<double>(actual_qd_, "Current joint speed (unit: rad/s)");
    printVec<double>(actual_TCP_pose_, "Current TCP position (unit: m) and attitude (unit: rad)");
    printVec<double>(actual_TCP_speed_,
                     "Current TCP position speed (unit: m/s) and attitude speed (unit: rad/s)");

    for (size_t i = 0; i < joint_mode_.size(); i++) {
        if (0 == i) {
            std::cout << "@:Joint status" << std::endl;
        }

        std::cout << joint_mode_.at(i) << ",";

        if ((joint_mode_.size() - 1) == i) {
            std::cout << std::endl;
        }
    }
    printVec<double>(joint_temperatures_, "Joint temperature");
    printVec<double>(actual_joint_voltage_, "Joint voltage");
    printVec<double>(actual_current_, "Joint current");

    printBit(standard_digital_input_bits_, "Digital input value",
             standard_digital_input_num_);
    printBit(standard_digital_output_bits_, "Digital output value",
             standard_digital_output_num_);
    printBit(tool_digital_input_bits_, "Tool digital value (including input and output)",
             tool_digital_input_num_);
    printBit(tool_digital_output_bits_, "Tool digital value (including input and output)",
             tool_digital_output_num_);
    printBit(configurable_digital_input_bits_, "Safety input value",
             configurable_digital_input_num_);
    printBit(configurable_digital_output_bits_, "Safety output value",
             configurable_digital_output_num_);
    printBit(link_digital_input_bits_, "Link input value", link_digital_input_num_);
    printBit(link_digital_output_bits_, "Link output value", link_digital_output_num_);
    printVec<double>(standard_analog_input_values_, "Analog input value");
    printVec<double>(tool_analog_input_values_, "Tool analog input value");
    printVec<double>(standard_analog_output_values_, "Analog output value");
    printVec<double>(tool_analog_output_values_, "Tool analog output value");
}

#define LOCAL_IP "127.0.0.1"

int main(int argc, char **argv)
{
#ifdef WIN32
    // Set Windows console output code page to UTF-8
    SetConsoleOutputCP(CP_UTF8);
#endif
    auto rtde_cli = std::make_shared<RtdeClient>();
    // API call: connect to RTDE service
    rtde_cli->connect(LOCAL_IP, 30010);
    // API call: login
    rtde_cli->login("aubo", "123456");

    auto rpc_cli = std::make_shared<RpcClient>();
    // API call: set RPC timeout
    rpc_cli->setRequestTimeout(1000);
    // API call: connect to RPC service
    rpc_cli->connect(LOCAL_IP, 30004);
    // API call: login
    rpc_cli->login("aubo", "123456");

    // RTDE publish
    examplePublish(rtde_cli);

    // RPC get IO quantity
    exampleIoNum(rpc_cli);

    // RTDE subscribe
    exampleSubscribe(rtde_cli);

    while (1) {
        // Print update information
        update();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

