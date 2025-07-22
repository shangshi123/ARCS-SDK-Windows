#include "aubo_sdk/rpc.h"
#include "thread"

#define LOCAL_IP           "127.0.0.1"
#define MODBUS_IP          "172.16.3.111,502"
#define MODBUS_SERIAL_PORT "/dev/ttyUSB0,9600,N,8,1"

using namespace arcs::common_interface;
using namespace arcs::aubo_sdk;

void printfTitle(std::string str)
{
    std::cout << std::endl << std::endl << str << std::endl;
}

void exampleRegisterInput(RpcClientPtr impl)
{
    printfTitle("->Register_Bool_Input_0_63");
    for (int i = 0; i < 63; i++) {
        if (0 == i % 2) {
            impl->getRegisterControl()->setBoolInput(i, true);
        } else {
            impl->getRegisterControl()->setBoolInput(i, false);
        }
        std::cout << (int)impl->getRegisterControl()->getBoolInput(i) << ",";
    }

    printfTitle("->Register_Bool_Input_64_127");
    for (int i = 64; i < 128; i++) {
        if (0 == i % 2) {
            impl->getRegisterControl()->setBoolInput(i, true);
        } else {
            impl->getRegisterControl()->setBoolInput(i, false);
        }
        std::cout << (int)impl->getRegisterControl()->getBoolInput(i) << ",";
    }

    printfTitle("->Register_Int_Input");
    for (int i = 0; i < 64; i++) {
        if (0 == i % 2) {
            impl->getRegisterControl()->setInt32Input(i, i);
        } else {
            impl->getRegisterControl()->setInt32Input(i, i);
        }
        std::cout << impl->getRegisterControl()->getInt32Input(i) << ",";
    }

    printfTitle("->Register_Float_Input");
    for (int i = 0; i < 64; i++) {
        if (0 == i % 2) {
            impl->getRegisterControl()->setFloatInput(i, 0.1);
        } else {
            impl->getRegisterControl()->setFloatInput(i, 0.2);
        }
        std::cout << impl->getRegisterControl()->getFloatInput(i) << ",";
    }

    printfTitle("->Register_Double_Input");
    for (int i = 0; i < 64; i++) {
        if (0 == i % 2) {
            impl->getRegisterControl()->setDoubleInput(i, 0.3);
        } else {
            impl->getRegisterControl()->setDoubleInput(i, 0.4);
        }
        std::cout << impl->getRegisterControl()->getDoubleInput(i) << ",";
    }
}

void exampleRegisterOutput(RpcClientPtr impl)
{
    printfTitle("->Register_Bool_Output_0_63");
    for (int i = 0; i < 63; i++) {
        if (0 == i % 2) {
            impl->getRegisterControl()->setBoolOutput(i, true);
        } else {
            impl->getRegisterControl()->setBoolOutput(i, false);
        }
        std::cout << (int)impl->getRegisterControl()->getBoolOutput(i) << ",";
    }

    printfTitle("->Register_Bool_Output_64_127");
    for (int i = 64; i < 127; i++) {
        if (0 == i % 2) {
            impl->getRegisterControl()->setBoolOutput(i, true);
        } else {
            impl->getRegisterControl()->setBoolOutput(i, false);
        }
        std::cout << (int)impl->getRegisterControl()->getBoolOutput(i) << ",";
    }

    printfTitle("->Register_Int_Output");
    for (int i = 0; i < 64; i++) {
        if (0 == i % 2) {
            impl->getRegisterControl()->setInt32Output(i, i);
        } else {
            impl->getRegisterControl()->setInt32Output(i, i);
        }
        std::cout << impl->getRegisterControl()->getInt32Output(i) << ",";
    }

    printfTitle("->Register_Float_Output");
    for (int i = 0; i < 64; i++) {
        if (0 == i % 2) {
            impl->getRegisterControl()->setFloatOutput(i, 0.1);
        } else {
            impl->getRegisterControl()->setFloatOutput(i, 0.2);
        }
        std::cout << impl->getRegisterControl()->getFloatOutput(i) << ",";
    }

    printfTitle("->Register_Double_Output");
    for (int i = 0; i < 64; i++) {
        if (0 == i % 2) {
            impl->getRegisterControl()->setDoubleOutput(i, 0.3);
        } else {
            impl->getRegisterControl()->setDoubleOutput(i, 0.4);
        }
        std::cout << impl->getRegisterControl()->getDoubleOutput(i) << ",";
    }
}

void exampleRegisterCustom(RpcClientPtr impl)
{
    impl->getRegisterControl()->setBool("custom1", true);
    bool v_b = false;
    v_b = impl->getRegisterControl()->getBool("custom1", v_b);
    std::cout << std::endl
              << std::endl
              << "->Register_Bool_Custom:" << v_b << std::endl;

    impl->getRegisterControl()->setInt32("custom2", 34);
    int v_i = 0;
    v_i = impl->getRegisterControl()->getInt32("custom2", v_i);
    std::cout << std::endl
              << std::endl
              << "->Register_Int32_Custom:" << v_i << std::endl;

    impl->getRegisterControl()->setFloat("custom3", 0.1);
    float v_f = 0;
    v_f = impl->getRegisterControl()->getFloat("custom3", v_f);
    std::cout << std::endl
              << std::endl
              << "->Register_Float_Custom:" << v_f << std::endl;

    impl->getRegisterControl()->setDouble("custom4", 0.2);
    double v_d = 0;
    v_d = impl->getRegisterControl()->getDouble("custom4", v_d);
    std::cout << std::endl
              << std::endl
              << "->Register_Double_Custom:" << v_d << std::endl;

    impl->getRegisterControl()->setString("custom5", "test");
    std::string v_s = "";
    v_s = impl->getRegisterControl()->getString("custom5", v_s);
    std::cout << std::endl
              << std::endl
              << "->Register_String_Custom:" << v_s << std::endl;

    std::cout << std::endl
              << std::endl
              << "->Register_Clear_Custom:"
              << impl->getRegisterControl()->clearNamedVariable("custom5")
              << std::endl;
}

void exampleModbusTcp(RpcClientPtr impl)
{
    /**
        0 = read digital input
        1 = read digital output
        2 = read register input
        3 = read register output
        4 = write digital output
        5 = write register output
    **/

    // Send custom data
    impl->getRegisterControl()->modbusSendCustomCommand(
        MODBUS_IP, 1, 0x06, { 0x00, 0x02, 0x00, 0x0F });

    // Write single coil
    impl->getRegisterControl()->modbusAddSignal(MODBUS_IP, 1, 0x00, 0x04,
                                                "WRITE_COLI_OUTPUT_01", true);
    auto res = impl->getRegisterControl()->modbusSetOutputSignal(
        "WRITE_COLI_OUTPUT_01", 0x01);

    // Write single holding register
    impl->getRegisterControl()->modbusAddSignal(
        MODBUS_IP, 1, 0x00, 0x05, "WRITE_HOLDING_REGISTER_OUTPUT_01", true);
    res = impl->getRegisterControl()->modbusSetOutputSignal(
        "WRITE_HOLDING_REGISTER_OUTPUT_01", 0x4455);

    // Read coil register (ip, station number, register start address, function code, name)
    impl->getRegisterControl()->modbusAddSignal(MODBUS_IP, 1, 0x00, 0x01,
                                                "READ_COIL_00", true);
    impl->getRegisterControl()->modbusSetSignalUpdateFrequency("READ_COIL_00",
                                                               1);

    // Read discrete input register
    impl->getRegisterControl()->modbusAddSignal(MODBUS_IP, 1, 0x00, 0x00,
                                                "READ_INPUT_00", true);
    impl->getRegisterControl()->modbusSetSignalUpdateFrequency("READ_INPUT_00",
                                                               1);

    // Read holding register
    impl->getRegisterControl()->modbusAddSignal(
        MODBUS_IP, 1, 0x00, 0x03, "READ_HOLDING_REGISTER_00", true);
    impl->getRegisterControl()->modbusSetSignalUpdateFrequency(
        "READ_HOLDING_REGISTER_00", 1);

    // Read input register
    impl->getRegisterControl()->modbusAddSignal(MODBUS_IP, 1, 0x00, 0x02,
                                                "READ_INPUT_REGISTER_00", true);
    impl->getRegisterControl()->modbusSetSignalUpdateFrequency(
        "READ_INPUT_REGISTER_00", 1);

    // Wait for the first data acquisition of the corresponding signal
    std::this_thread::sleep_for(std::chrono::seconds(3));

    for (int i = 0; i < 100; i++) {
        std::cout << "-------------------------" << std::endl;
        auto ret =
            impl->getRegisterControl()->modbusGetSignalStatus("READ_COIL_00");
        std::cout << "READ_COIL_00:" << ret << std::endl;
        auto ret2 =
            impl->getRegisterControl()->modbusGetSignalStatus("READ_INPUT_00");
        std::cout << "READ_INPUT_00:" << ret2 << std::endl;
        auto ret3 = impl->getRegisterControl()->modbusGetSignalStatus(
            "READ_HOLDING_REGISTER_00");
        std::cout << "READ_HOLDING_REGISTER_00:" << ret3 << std::endl;
        auto ret4 = impl->getRegisterControl()->modbusGetSignalStatus(
            "READ_INPUT_REGISTER_00");
        std::cout << "READ_INPUT_REGISTER_00:" << ret4 << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    for (int i = 0; i < 10; i++) {
        impl->getRegisterControl()->modbusSetOutputSignal(
            "WRITE_COLI_OUTPUT_01", 0x01);
        impl->getRegisterControl()->modbusSetOutputSignal(
            "WRITE_COLI_OUTPUT_01", 0x00);
    }

    // Send custom data
    impl->getRegisterControl()->modbusSendCustomCommand(
        MODBUS_IP, 1, 0x06, { 0x00, 0x03, 0x01, 0xFF });

    impl->getRegisterControl()->modbusDeleteSignal("WRITE_COLI_OUTPUT_01");
    impl->getRegisterControl()->modbusDeleteSignal(
        "WRITE_HOLDING_REGISTER_OUTPUT_01");
    impl->getRegisterControl()->modbusDeleteSignal("READ_COIL_00");
    impl->getRegisterControl()->modbusDeleteSignal("READ_INPUT_00");
    impl->getRegisterControl()->modbusDeleteSignal("READ_HOLDING_REGISTER_00");
    impl->getRegisterControl()->modbusDeleteSignal("READ_INPUT_REGISTER_00");
}
