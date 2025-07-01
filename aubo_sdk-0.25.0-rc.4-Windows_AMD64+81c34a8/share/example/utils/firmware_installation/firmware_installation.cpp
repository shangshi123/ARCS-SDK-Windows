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
    GetSystemTime(&st); // 获取当前系统时间
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
    // 接口调用: 获取机器人的名字
    auto robot_name = cli->getRobotNames().front();

    auto robot_interface = cli->getRobotInterface(robot_name);
    auto robot_config = robot_interface->getRobotConfig();
    auto robot_state = robot_interface->getRobotState();
    auto robot_manage = robot_interface->getRobotManage();

    if (robot_state->isPowerOn()) {
        robot_manage->poweroff();
    }

    std::cout << "update type : " << fw_file << std::endl;

    // 升级固件
    int ret = robot_config->firmwareUpdate(fw_file);
    if (ret != 0) {
        std::cout << "update firmware error！ret : " << ret << std::endl;
        return;
    }

    std::cout << "start update firmware " << std::endl;
    // 增加延时，确保先进入维护模式，然后再通过getFirmwareUpdateProcess获取升级进度
    std::this_thread::sleep_for(std::chrono::seconds(1));

    while (1) {
        // 获取固件安装进度
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

// 定义 struct option 结构体，用于存储命令行选项的参数
// struct option 用于 getopt_long 函数，它定义了每个长选项（如
// --file）的结构信息。 以下是 struct option 各字段的含义：
struct option
{
    const char
        *name; // 选项的名称，例如 "help" 或 "file"，用于长选项（如 --help）
    int has_arg; // 该选项是否需要参数。0 表示没有参数，1 表示有一个参数，2
                 // 表示有多个参数
    int *flag; // 如果非 nullptr，表示该选项的状态存储地址。如果该值为
               // nullptr，则 getopt_long 会返回一个值给 val
    int val; // 如果 flag 为 nullptr，该值是 getopt_long 返回的选项标识符，例如
             // 'h'，表示该选项的具体值
};

// 模拟 getopt_long 函数
int getopt_long(int argc, char *const argv[], const char *optstring,
                const struct option *longopts, int *longindex, char *&arg)
{
    static int currentArg = 1; // 当前处理的参数索引

    constexpr int required_argument = 1;
    constexpr int no_argument = 0;
    if (currentArg >= argc) {
        return -1; // 没有更多参数
    }

    const char *currentOption = argv[currentArg];

    if (currentOption[0] == '-') {
        // 检查长选项
        if (currentOption[1] == '-') {
            const char *longOption = &currentOption[2]; // 去掉前缀"--"
            for (int i = 0; longopts[i].name != nullptr; ++i) {
                if (strcmp(longOption, longopts[i].name) == 0) {
                    if (longindex) {
                        *longindex = i;
                    }

                    if (longopts[i].has_arg == required_argument) {
                        if (currentArg + 1 < argc) {
                            arg = argv[++currentArg]; // 获取参数值
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

        // 检查短选项
        char option = currentOption[1];
        const char *optFound = strchr(optstring, option);

        if (optFound) {
            ++currentArg;
            if (optFound[1] == ':') {
                // 如果选项需要一个参数
                if (currentArg < argc) {
                    arg = argv[currentArg++]; // 获取参数值
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

    return -1; // 不是选项
}

int main(int argc, char **argv)
{
#ifdef WIN32
    // 将Windows控制台输出代码页设置为 UTF-8
    SetConsoleOutputCP(CP_UTF8);
#endif
    int opt = -1;
    char *optarg = nullptr; // 用于存储选项的参数值
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
        Sleep(1000); // Windows 下休眠 1 秒
#else
        sleep(1); // Unix 下休眠 1 秒
#endif

    } while (rpc_connect_ret != AUBO_OK);

    int rpc_login_ret = rpc_cli->login("aubo", "123456");
    if (rpc_login_ret < 0) {
        std::cout << "rpc login error, ret = " << rpc_login_ret << std::endl;
    }
    // 固件安装
    firmwareInstall(rpc_cli, fw_file + "#" + target_node);

    // 接口调用: 退出登录
    rpc_cli->logout();
    // 接口调用: 断开连接
    rpc_cli->disconnect();

    return 0;
}
