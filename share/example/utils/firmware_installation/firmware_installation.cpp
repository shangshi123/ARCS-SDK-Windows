#include "aubo_sdk/rpc.h"
#include <string.h>
#ifndef _WIN32
#include <unistd.h>
#include <sys/timeb.h>
#else
#include <windows.h>
#include <time.h>
#endif

#define RPC_PORT 30004

using namespace arcs::common_interface;
using namespace arcs::aubo_sdk;

char *getTime(void)
{
    static char sz_time[100];

#ifdef WIN32
    SYSTEMTIME st;
    GetSystemTime(&st); // Get current system time
    sprintf(sz_time, "%02d-%02d %02d:%02d:%02d.%03d", st.wMonth, st.wDay,
            st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
#else
    struct tm *ptm;
    struct timeb st_timeb;
    struct timeval time1;

    ftime(&st_timeb);
    ptm = localtime(&st_timeb.time);
    sprintf(sz_time, "%02d-%02d %02d:%02d:%02d.%03d.%03ld", ptm->tm_mon + 1,
            ptm->tm_mday, ptm->tm_hour, ptm->tm_min, ptm->tm_sec,
            st_timeb.millitm, time1.tv_usec);
#endif
    sz_time[18] = 0;
    return sz_time;
}

auto log_handler = [](int level, const char *filename, int line,
                      const std::string &message) {
    static const char *level_names[] = { "FATAL", "ERROR", "WARNING",
                                         "INFO",  "DEBUG", "BACK_TRACE" };

    // We use fprintf() instead of cerr because we want this to work at
    // static initialization time.
    fprintf(stderr, "%s[%s] %s(%d): %s\n", getTime(), level_names[level],
            filename, line, message.c_str());
    fflush(stderr);
};

void firmwareInstall(RpcClientPtr cli, const std::string &fw_file)
{
    // API call: Get the robot's name
    auto robot_name = cli->getRobotNames().front();

    auto robot_interface = cli->getRobotInterface(robot_name);
    auto robot_config = robot_interface->getRobotConfig();
    auto robot_state = robot_interface->getRobotState();
    auto robot_manage = robot_interface->getRobotManage();

    if (robot_state->isPowerOn()) {
        robot_manage->poweroff();
    }

    std::cout << "update type : " << fw_file << std::endl;

    // Upgrade firmware
    int ret = robot_config->firmwareUpdate(fw_file);
    if (ret != 0) {
        std::cout << "update firmware error！ret : " << ret << std::endl;
        return;
    }

    std::cout << "start update firmware " << std::endl;
    // Add delay to ensure entering maintenance mode first, then get upgrade progress via getFirmwareUpdateProcess
    std::this_thread::sleep_for(std::chrono::seconds(1));

    while (1) {
        // Get firmware installation progress
        std::tuple<std::string, double> update_process;
        try {
            update_process = robot_config->getFirmwareUpdateProcess();
        } catch (...) {
            std::cout << "getFirmwareUpdateProcess error. " << std::endl;
        }
        if (std::get<0>(update_process) == "failed") {
            std::cout << "firmware update failed! " << std::endl;
            break;
        }
        std::cout << "update state : " << std::get<0>(update_process) << " : "
                  << std::get<1>(update_process) << std::endl;

        if (std::get<1>(update_process) == 1) {
            std::cout << "firmware update success. " << std::endl;
            break;
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void printUsage(char *pname)
{
    // clang-format off
    printf("Usage: \n");
    printf("    sudo %s [Options] \n"
           "Options:\n"
           "         -h|--help\n"
           "             this help\n"
           "         -f|--file\n"
           "             firmware path. \n"
           "             example : -f /tmp/firmware_update-1.0.42-rc.10+9030ebb.firm \n"
           "             ** this parameter must be entered, and this file must exist on the target robot! \n"
           "         -n|--node\n"
           "             enter the node you want to upgrade to\n"
           "             example : -n master_mcu,slave_mcu,joint1,joint2,joint3,joint4,joint5,joint6,base,tool\n"
           "             default : all \n"
           "         -i|--ip\n"
           "             enter your target IP address. \n"
           "             example : -i 192.168.1.46 \n"
           "             default : 127.0.0.1 \n"
           "         -F|--force\n"
           "             force all nodes to upgrade to the target version. \n"
           "             example : -F \n"
           , pname);
    // clang-format on
}

// Define struct option structure, used to store command line option parameters
// struct option is used for getopt_long function, it defines the structure information for each long option (such as --file).
// The following are the meanings of each field in struct option:
struct option
{
    const char
        *name; // Option name, such as "help" or "file", used for long options (such as --help)
    int has_arg; // Whether the option requires a parameter. 0 means no parameter, 1 means one parameter, 2 means multiple parameters
    int *flag; // If not nullptr, indicates the storage address of the option's status. If this value is nullptr, getopt_long returns a value to val
    int val; // If flag is nullptr, this value is the option identifier returned by getopt_long, such as 'h', indicating the specific value of the option
};

// Simulate getopt_long function
int getopt_long(int argc, char *const argv[], const char *optstring,
                const struct option *longopts, int *longindex, char *&arg)
{
    static int currentArg = 1; // Current argument index

    constexpr int required_argument = 1;
    constexpr int no_argument = 0;
    if (currentArg >= argc) {
        return -1; // No more arguments
    }

    const char *currentOption = argv[currentArg];

    if (currentOption[0] == '-') {
        // Check long option
        if (currentOption[1] == '-') {
            const char *longOption = &currentOption[2]; // Remove prefix "--"
            for (int i = 0; longopts[i].name != nullptr; ++i) {
                if (strcmp(longOption, longopts[i].name) == 0) {
                    if (longindex) {
                        *longindex = i;
                    }

                    if (longopts[i].has_arg == required_argument) {
                        if (currentArg + 1 < argc) {
                            arg = argv[++currentArg]; // Get parameter value
                        } else {
                            std::cerr << "Option --" << longOption
                                      << " requires an argument!" << std::endl;
                            return '?';
                        }
                    }

                    ++currentArg;
                    return longopts[i].val;
                }
            }
            std::cerr << "Unknown option: --" << longOption << std::endl;
            ++currentArg;
            return '?';
        }

        // Check short option
        char option = currentOption[1];
        const char *optFound = strchr(optstring, option);

        if (optFound) {
            ++currentArg;
            if (optFound[1] == ':') {
                // If the option requires a parameter
                if (currentArg < argc) {
                    arg = argv[currentArg++]; // Get parameter value
                } else {
                    std::cerr << "Option -" << option
                              << " requires an argument!" << std::endl;
                    return '?';
                }
            }
            return option;
        } else {
            std::cerr << "Unknown option: -" << option << std::endl;
            ++currentArg;
            return '?';
        }
    }

    return -1; // Not an option
}

int main(int argc, char **argv)
{
#ifdef WIN32
    // Set Windows console output code page to UTF-8
    SetConsoleOutputCP(CP_UTF8);
#endif
    int opt = -1;
    char *optarg = nullptr; // Used to store the parameter value of the option
    std::string fw_file = "";
    std::string target_ip = "127.0.0.1";
    std::string target_node = "all";
    bool force_flag = false;

    static struct option longopts[] = {
        { "help", 0, nullptr, 'h' },  { "file", 0, nullptr, 'f' },
        { "node", 0, nullptr, 'n' },  { "ip", 0, nullptr, 'i' },
        { "force", 0, nullptr, 'F' }, { 0, 0, 0, 0 }
    };

    while ((opt = getopt_long(argc, argv, "hf:n:i:F", longopts, nullptr,
                              optarg)) != -1) {
        switch (opt) {
        case 'h':
            printUsage(argv[0]);
            exit(0);
            break;
        case 'f':
            fw_file = optarg;
            break;
        case 'n':
            target_node = optarg;
            break;
        case 'i':
            target_ip = optarg;
            break;
        case 'F':
            force_flag = true;
            break;
        case '?':
            printUsage(argv[0]);
            exit(1);
            break;
        default:
            break;
        }
    }

    if (fw_file.size() == 0) {
        std::cout << "firmware update ERROR! " << std::endl;
        std::cout << "please enter the - f parameter. example: -f "
                     "/tmp/firmware_update_XXX.firm"
                  << std::endl;
        exit(-1);
    }

    if (target_node.size() == 0) {
        target_node = "all";
    }

    if (force_flag) {
        size_t pos = 0;
        while ((pos = target_node.find(',', pos)) != std::string::npos) {
            target_node.replace(pos, 1, "!,");

            pos += 2;
        }
        target_node += "!";
    }

    auto rpc_cli = std::make_shared<RpcClient>();
    rpc_cli->setRequestTimeout(1000);
    rpc_cli->setLogHandler(log_handler);
    int rpc_connect_ret = 0;
    do {
        rpc_connect_ret = rpc_cli->connect(target_ip, RPC_PORT);
        if (rpc_connect_ret != AUBO_OK) {
            std::cout << "rpc connect error, reconnect now.  ret = "
                      << rpc_connect_ret << std::endl;
        }
#ifdef WIN32
        Sleep(1000); // Sleep 1 second on Windows
#else
        sleep(1); // Sleep 1 second on Unix
#endif

    } while (rpc_connect_ret != AUBO_OK);

    int rpc_login_ret = rpc_cli->login("aubo", "123456");
    if (rpc_login_ret < 0) {
        std::cout << "rpc login error, ret = " << rpc_login_ret << std::endl;
    }
    // Firmware installation
    firmwareInstall(rpc_cli, fw_file + "#" + target_node);

    // API call: logout
    rpc_cli->logout();
    // API call: disconnect
    rpc_cli->disconnect();

    return 0;
}
