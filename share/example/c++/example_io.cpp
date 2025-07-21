#include "aubo_sdk/rpc.h"
#include <bitset>
#ifdef WIN32
#include <Windows.h>
#endif

using namespace arcs::aubo_sdk;

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

// Digital input - no trigger action
void exampleStandardDigitalInput1(RpcClientPtr cli)
{
    // API call: Get the robot's name
    auto robot_name = cli->getRobotNames().front();
    auto robot = cli->getRobotInterface(robot_name);

    // API call: Get the number of digital inputs
    int di_num = robot->getIoControl()->getStandardDigitalInputNum();
    std::cout << "Number of digital inputs: " << di_num << std::endl;

    // API call: Set all digital input actions to no trigger
    robot->getIoControl()->setDigitalInputActionDefault();

    // Print all digital input values
    for (int i = 0; i < di_num; i++) {
        // API call: Get digital input value
        auto input_value = robot->getIoControl()->getStandardDigitalInput(i);
        std::cout << "Pin " << i << " digital input value: " << input_value << std::endl;
    }
}

// Digital input - trigger action is handguiding
void exampleStandardDigitalInput2(RpcClientPtr cli)
{
    // API call: Get the robot's name
    auto robot_name = cli->getRobotNames().front();
    auto robot = cli->getRobotInterface(robot_name);

    // API call: Set DI00 action to handguiding
    StandardInputAction input_action = StandardInputAction::Handguide;
    robot->getIoControl()->setStandardDigitalInputAction(0, input_action);

    std::cout << "Note: When DI00 is high level (DI00 and 0V are shorted), the robot enters handguiding."
              << std::endl;
    std::cout << "When DI00 is low level (DI00 and 0V are disconnected), the robot exits handguiding."
              << std::endl;
}

// Digital output - no trigger action, user can set output value
void exampleStandardDigitalOutput1(RpcClientPtr cli)
{
    // API call: Get the robot's name
    auto robot_name = cli->getRobotNames().front();
    auto robot = cli->getRobotInterface(robot_name);

    // API call: Get the number of digital outputs
    int do_num = robot->getIoControl()->getStandardDigitalOutputNum();
    std::cout << "Number of digital outputs: " << do_num << std::endl;

    // API call: Set all digital output actions to no trigger.
    // When the digital output action is no trigger, the user can set the digital output value.
    robot->getIoControl()->setDigitalOutputRunstateDefault();

    // API call: Set DO00 to high level
    robot->getIoControl()->setStandardDigitalOutput(0, true);

    std::this_thread::sleep_for(std::chrono::seconds(1));

    // API call: Get the value of DO00
    bool value = robot->getIoControl()->getStandardDigitalOutput(0);
    std::cout << "DO00 value: " << (value ? "High level" : "Low level") << std::endl;
}

// Digital output - set trigger action to handguiding, controller sets output value automatically
void exampleStandardDigitalOutput2(RpcClientPtr cli)
{
    // API call: Get the robot's name
    auto robot_name = cli->getRobotNames().front();
    auto robot = cli->getRobotInterface(robot_name);

    // API call: Get the number of digital outputs
    int do_num = robot->getIoControl()->getStandardDigitalOutputNum();
    std::cout << "Number of digital outputs: " << do_num << std::endl;

    // API call: Set DO00 action to handguiding.
    // When the robot enters handguiding, DO00 value is high level.
    // When the robot exits handguiding, DO00 value is low level.
    // At this time, the user cannot set the value of DO00.
    StandardOutputRunState runstate = StandardOutputRunState::Handguiding;
    robot->getIoControl()->setStandardDigitalOutputRunstate(0, runstate);

    // API call: Enter handguiding
    robot->getRobotManage()->freedrive(true);

    // Wait to enter handguiding mode
    int i = 0;
    while (robot->getRobotManage()->isFreedriveEnabled()) {
        if (i++ > 5) {
            std::cerr << "Failed to enable handguiding mode" << std::endl;
            return;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cout << "Entered handguiding successfully" << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(1));

    // API call: Get the value of DO00
    bool value = robot->getIoControl()->getStandardDigitalOutput(0);
    std::cout << "DO00 value: " << (value ? "High level" : "Low level") << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(3));

    // API call: Exit handguiding
    robot->getRobotManage()->freedrive(false);

    // Wait to exit handguiding mode
    i = 0;
    while (robot->getRobotManage()->isFreedriveEnabled()) {
        if (i++ > 5) {
            std::cerr << "Failed to disable handguiding mode" << std::endl;
            return;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cout << "Exited handguiding successfully" << std::endl;

    // API call: Get the value of DO00
    value = robot->getIoControl()->getStandardDigitalOutput(0);
    std::cout << "DO00 value: " << (value ? "High level" : "Low level") << std::endl;
}

// Analog input
void exampleStandardAnalogInput(RpcClientPtr cli)
{
    // API call: Get the robot's name
    auto robot_name = cli->getRobotNames().front();
    auto robot = cli->getRobotInterface(robot_name);

    // API call: Get the number of analog inputs
    int ai_num = robot->getIoControl()->getStandardAnalogInputNum();
    std::cout << "Number of analog inputs: " << ai_num << std::endl;

    // Print all analog input values
    for (int i = 0; i < ai_num; i++) {
        // API call: Get analog input value
        auto input_value = robot->getIoControl()->getStandardAnalogInput(i);
        std::cout << "Pin " << i << " analog input value: " << input_value << std::endl;
    }
}

// Analog output - no trigger action, user sets output value
void exampleStandardAnalogOutput1(RpcClientPtr cli)
{
    // API call: Get the robot's name
    auto robot_name = cli->getRobotNames().front();
    auto robot = cli->getRobotInterface(robot_name);

    // API call: Get the number of standard analog outputs
    int ao_num = robot->getIoControl()->getStandardAnalogOutputNum();
    std::cout << "Number of analog outputs: " << ao_num << std::endl;

    // API call: Set all analog output actions to no trigger.
    // When the analog output action is no trigger, the user can set the analog output value.
    StandardOutputRunState output_runstate = StandardOutputRunState::None;
    for (int i = 0; i < ao_num; i++) {
        robot->getIoControl()->setStandardAnalogOutputRunstate(i,
                                                               output_runstate);
    }

    // API call: Set AO0 output voltage to 5V
    double value = 5.0;
    robot->getIoControl()->setStandardAnalogOutput(0, value);
    std::cout << "Set AO0 output voltage: " << value << " V" << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(1));

    // API call: Get AO0 output voltage
    value = robot->getIoControl()->getStandardAnalogOutput(0);
    std::cout << "Get AO0 output voltage: " << value << " V" << std::endl;
}

// Analog output - set trigger action to maximum when handguiding, controller sets output value automatically
void exampleStandardAnalogOutput2(RpcClientPtr cli)
{
    // API call: Get the robot's name
    auto robot_name = cli->getRobotNames().front();
    auto robot = cli->getRobotInterface(robot_name);

    // API call: Get the number of standard analog outputs
    int ao_num = robot->getIoControl()->getStandardAnalogOutputNum();
    std::cout << "Number of analog outputs: " << ao_num << std::endl;

    // API call: Set AO0 action to maximum when handguiding
    // When the robot enters handguiding, AO0 value is maximum.
    // When the robot exits handguiding, AO0 value is 0.
    // At this time, the user cannot set the value of AO0.
    StandardOutputRunState output_runstate =
        StandardOutputRunState::Handguiding;
    robot->getIoControl()->setStandardAnalogOutputRunstate(0, output_runstate);

    // API call: Enter handguiding
    robot->getRobotManage()->freedrive(true);

    // Wait to enter handguiding mode
    int i = 0;
    while (robot->getRobotManage()->isFreedriveEnabled()) {
        if (i++ > 5) {
            std::cerr << "Failed to enable handguiding mode" << std::endl;
            return;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cout << "Entered handguiding successfully" << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(1));

    // API call: Get AO0 output voltage
    double value = robot->getIoControl()->getStandardAnalogOutput(0);
    std::cout << "Get AO0 output voltage: " << value << " V" << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(3));

    // API call: Exit handguiding
    robot->getRobotManage()->freedrive(false);

    // Wait to exit handguiding mode
    i = 0;
    while (robot->getRobotManage()->isFreedriveEnabled()) {
        if (i++ > 5) {
            std::cerr << "Failed to disable handguiding mode" << std::endl;
            return;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cout << "Exited handguiding successfully" << std::endl;

    // API call: Get AO0 output voltage
    value = robot->getIoControl()->getStandardAnalogOutput(0);
    std::cout << "Get AO0 output voltage: " << value << " V" << std::endl;
}

// Tool digital input - no trigger action
void exampleToolDigitalInput1(RpcClientPtr cli)
{
    // API call: Get the robot's name
    auto robot_name = cli->getRobotNames().front();
    auto robot = cli->getRobotInterface(robot_name);

    // API call: Get the number of tool digital IOs, including input and output
    int tool_io_num = robot->getIoControl()->getToolDigitalInputNum();
    std::cout << "Get the number of tool digital IOs (including input and output): " << tool_io_num
              << std::endl;

    // API call: Set TOOL_IO[0] as input
    bool isInput = true;
    robot->getIoControl()->setToolIoInput(0, isInput);
    std::this_thread::sleep_for(std::chrono::seconds(0));
    std::cout << "Set TOOL_IO[0] type to: " << (isInput ? "Input" : "Output")
              << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(1));

    // API call: Get TOOL_IO[0] type
    isInput = robot->getIoControl()->isToolIoInput(0);
    std::cout << "Get TOOL_IO[0] type: " << (isInput ? "Input" : "Output")
              << std::endl;

    // API call: Set TOOL_IO[0] action to no trigger
    StandardInputAction input_action = StandardInputAction::Default;
    robot->getIoControl()->setToolDigitalInputAction(0, input_action);

    // API call: Get TOOL_IO[0] value
    bool value = robot->getIoControl()->getToolDigitalInput(0);
    std::cout << "Get TOOL_IO[0] digital input value: " << (value ? "High level" : "Low level")
              << std::endl;
}

// Tool digital input - trigger action is handguiding
void exampleToolDigitalInput2(RpcClientPtr cli)
{
    // API call: Get the robot's name
    auto robot_name = cli->getRobotNames().front();
    auto robot = cli->getRobotInterface(robot_name);

    // API call: Get the number of tool digital IOs, including input and output
    int tool_io_num = robot->getIoControl()->getToolDigitalInputNum();
    std::cout << "Get the number of tool digital IOs (including input and output): " << tool_io_num
              << std::endl;

    // API call: Set TOOL_IO[0] as input
    bool isInput = true;
    robot->getIoControl()->setToolIoInput(0, isInput);
    std::this_thread::sleep_for(std::chrono::seconds(0));
    std::cout << "Set TOOL_IO[0] type to: " << (isInput ? "Input" : "Output")
              << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(1));

    // API call: Get TOOL_IO[0] type
    isInput = robot->getIoControl()->isToolIoInput(0);
    std::cout << "Get TOOL_IO[0] type: " << (isInput ? "Input" : "Output")
              << std::endl;

    // API call: Set TOOL_IO[0] action to handguiding
    StandardInputAction input_action = StandardInputAction::Handguide;
    robot->getIoControl()->setToolDigitalInputAction(0, input_action);

    std::cout << "Note: When TOOL_IO[0] is high level (TOOL_IO[0] and GND are shorted),"
                 "the robot enters handguiding."
              << std::endl;
    std::cout
        << "When TOOL_IO[0] is low level (TOOL_IO[0] and GND are disconnected), the robot exits handguiding."
        << std::endl;
}

// Tool digital output - no trigger action, user sets output value
void exampleToolDigitalOutput1(RpcClientPtr cli)
{
    // API call: Get the robot's name
    auto robot_name = cli->getRobotNames().front();
    auto robot = cli->getRobotInterface(robot_name);

    // API call: Get the number of tool digital IOs, including input and output
    int tool_io_num = robot->getIoControl()->getToolDigitalInputNum();
    std::cout << "Get the number of tool digital IOs (including input and output): " << tool_io_num
              << std::endl;

    // API call: Set TOOL_IO[0] as output
    bool isInput = false;
    robot->getIoControl()->setToolIoInput(0, isInput);
    std::this_thread::sleep_for(std::chrono::seconds(0));
    std::cout << "Set TOOL_IO[0] type to: " << (isInput ? "Input" : "Output")
              << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(1));

    // API call: Get TOOL_IO[0] type
    isInput = robot->getIoControl()->isToolIoInput(0);
    std::cout << "Get TOOL_IO[0] type: " << (isInput ? "Input" : "Output")
              << std::endl;

    // API call: Set TOOL_IO[0] action to no trigger
    // When the tool digital output action is no trigger, the user can set the tool digital output value.
    StandardOutputRunState output_runstate = StandardOutputRunState::None;
    robot->getIoControl()->setToolDigitalOutputRunstate(0, output_runstate);

    // API call: Set TOOL_IO[0] to high level
    bool value = true;
    robot->getIoControl()->setToolDigitalOutput(0, value);
    std::cout << "Set TOOL_IO[0] value: " << (value ? "High level" : "Low level")
              << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(1));

    // API call: Get TOOL_IO[0] value
    value = robot->getIoControl()->getToolDigitalOutput(0);
    std::cout << "Get TOOL_IO[0] value: " << (value ? "High level" : "Low level")
              << std::endl;
}

// Tool digital output - set trigger action to handguiding, controller sets output value automatically
void exampleToolDigitalOutput2(RpcClientPtr cli)
{
    // API call: Get the robot's name
    auto robot_name = cli->getRobotNames().front();
    auto robot = cli->getRobotInterface(robot_name);

    // API call: Get the number of tool digital IOs, including input and output
    int tool_io_num = robot->getIoControl()->getToolDigitalInputNum();
    std::cout << "Get the number of tool digital IOs (including input and output): " << tool_io_num
              << std::endl;

    // API call: Set TOOL_IO[0] as output
    bool isInput = false;
    robot->getIoControl()->setToolIoInput(0, isInput);
    std::this_thread::sleep_for(std::chrono::seconds(0));
    std::cout << "Set TOOL_IO[0] type to: " << (isInput ? "Input" : "Output")
              << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(1));

    // API call: Get TOOL_IO[0] type
    isInput = robot->getIoControl()->isToolIoInput(0);
    std::cout << "Get TOOL_IO[0] type: " << (isInput ? "Input" : "Output")
              << std::endl;

    // API call: Set TOOL_IO[0] action to handguiding.
    // When the robot enters handguiding, TOOL_IO[0] value is high level.
    // When the robot exits handguiding, TOOL_IO[0] value is low level.
    // At this time, the user cannot set the value of TOOL_IO[0].
    StandardOutputRunState output_runstate =
        StandardOutputRunState::Handguiding;
    robot->getIoControl()->setToolDigitalOutputRunstate(0, output_runstate);

    // API call: Enter handguiding
    robot->getRobotManage()->freedrive(true);

    // Wait to enter handguiding mode
    int i = 0;
    while (robot->getRobotManage()->isFreedriveEnabled()) {
        if (i++ > 5) {
            std::cerr << "Failed to enable handguiding mode" << std::endl;
            return;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cout << "Entered handguiding successfully" << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(1));

    // API call: Get TOOL_IO[0] value
    bool value = robot->getIoControl()->getToolDigitalOutput(0);
    std::cout << "Get TOOL_IO[0] value: " << (value ? "High level" : "Low level")
              << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(3));

    // API call: Exit handguiding
    robot->getRobotManage()->freedrive(false);

    // Wait to exit handguiding mode
    i = 0;
    while (robot->getRobotManage()->isFreedriveEnabled()) {
        if (i++ > 5) {
            std::cerr << "Failed to disable handguiding mode" << std::endl;
            return;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cout << "Exited handguiding successfully" << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(1));

    // API call: Get TOOL_IO[0] value
    value = robot->getIoControl()->getToolDigitalOutput(0);
    std::cout << "Get TOOL_IO[0] value: " << (value ? "High level" : "Low level")
              << std::endl;
}

// Tool analog input
void exampleToolAnalogInput(RpcClientPtr cli)
{
    // API call: Get the robot's name
    auto robot_name = cli->getRobotNames().front();
    auto robot = cli->getRobotInterface(robot_name);

    // API call: Get the number of tool analog inputs
    int tool_ai_num = robot->getIoControl()->getToolAnalogInputNum();
    std::cout << "Number of tool analog inputs: " << tool_ai_num << std::endl;

    // API call: Get all tool analog input values
    for (int i = 0; i < tool_ai_num; i++) {
        double value = robot->getIoControl()->getToolAnalogInput(i);
        std::cout << "TOOL_AI[" << i << "] value: " << value << std::endl;
    }
}

// Safety IO
void exampleSafetyIO(RpcClientPtr cli)
{
    // API call: Get the robot's name
    auto robot_name = cli->getRobotNames().front();
    auto robot = cli->getRobotInterface(robot_name);

    // API call: Get the number of safety IO inputs and outputs
    int safety_input_num =
        robot->getIoControl()->getConfigurableDigitalInputNum();
    int safety_output_num =
        robot->getIoControl()->getConfigurableDigitalOutputNum();

    std::cout << "Number of safety IO inputs: " << safety_input_num << std::endl;
    std::cout << "Number of safety IO outputs: " << safety_output_num << std::endl;

    // Get safety IO input values
    for (int i = 0; i < safety_input_num; i++) {
        // API call: Get safety IO input value
        auto value = robot->getIoControl()->getConfigurableDigitalInput(i);
        std::cout << "Pin " << i << " safety IO input value: " << value << std::endl;
    }

    // Get safety IO output values
    for (int i = 0; i < safety_output_num; i++) {
        // API call: Get safety IO output value
        auto value = robot->getIoControl()->getConfigurableDigitalOutput(i);
        std::cout << "Pin " << i << " safety IO output value: " << value << std::endl;
    }
}

// Static link IO
void exampleStaticLinkIO(RpcClientPtr cli)
{
    // API call: Get the robot's name
    auto robot_name = cli->getRobotNames().front();
    auto robot = cli->getRobotInterface(robot_name);

    // API call: Get the number of static link IO inputs and outputs
    int link_input_num = robot->getIoControl()->getStaticLinkInputNum();
    int link_output_num = robot->getIoControl()->getStaticLinkOutputNum();
    std::cout << "Number of static link IO inputs: " << link_input_num << std::endl;
    std::cout << "Number of static link IO outputs: " << link_output_num << std::endl;

    // Get static link IO input values
    int input_value_decimal = robot->getIoControl()->getStaticLinkInputs();
    printBit(input_value_decimal, "All static link IO input values", link_input_num);

    // Get static link IO output values
    int output_value_decimal = robot->getIoControl()->getStaticLinkOutputs();
    printBit(output_value_decimal, "All static link IO output values", link_output_num);
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

    // Digital input - no trigger action
    exampleStandardDigitalInput1(rpc_cli);

    //    // Digital input - trigger action is handguiding
    //    exampleStandardDigitalInput2(rpc_cli);

    // Digital output - no trigger action, user sets output value
    exampleStandardDigitalOutput1(rpc_cli);

    //    // Digital output - set trigger action to handguiding, controller sets output value automatically
    //    exampleStandardDigitalOutput2(rpc_cli);

    // Analog input
    exampleStandardAnalogInput(rpc_cli);

    // Analog output - no trigger action, user sets output value
    exampleStandardAnalogOutput1(rpc_cli);

    //    // Analog output - set trigger action to maximum when handguiding, controller sets output value automatically
    //    exampleStandardAnalogOutput2(rpc_cli);

    // Tool digital input - no trigger action
    exampleToolDigitalInput1(rpc_cli);

    //    // Tool digital input - trigger action is handguiding
    //    exampleToolDigitalInput2(rpc_cli);

    // Tool digital output - no trigger action, user sets output value
    exampleToolDigitalOutput1(rpc_cli);

    //    // Tool digital output - set trigger action to handguiding, controller sets output value automatically
    //    exampleToolDigitalOutput2(rpc_cli);

    // Tool analog input
    exampleToolAnalogInput(rpc_cli);

    // Safety IO
    exampleSafetyIO(rpc_cli);

    // Static link IO
    exampleStaticLinkIO(rpc_cli);

    // API call: Logout
    rpc_cli->logout();
    // API call: Disconnect
    rpc_cli->disconnect();

    return 0;
}
