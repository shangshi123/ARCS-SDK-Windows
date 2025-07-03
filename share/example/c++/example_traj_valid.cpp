#include "aubo_sdk/rpc.h"
#include "math.h"
#ifdef WIN32
#include <Windows.h>
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace arcs::common_interface;
using namespace arcs::aubo_sdk;

// 检查给定的轨迹是否有效，进行插值和逆解验证
bool exampleTrajectoryValid(RpcClientPtr impl, const std::vector<double> &p1,
                            const std::vector<double> &p2, int num_points)
{
    // 如果num_points小于2，则无法进行插值
    if (num_points < 2) {
        throw std::invalid_argument("num_points must be at least 2");
    }

    // 接口调用: 获取机器人的名字
    auto robot_name = impl->getRobotNames().front();

    // 接口调用: 设置 tcp 偏移
    std::vector<double> offset = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    impl->getRobotInterface(robot_name)->getRobotConfig()->setTcpOffset(offset);

    // 计算每个插值点的alpha值，并调用interpolatePose
    for (int i = 0; i < num_points; ++i) {
        double alpha =
            static_cast<double>(i) / (num_points - 1); // alpha从0到1均匀变化

        // 接口调用: 计算线性差值
        auto pose = impl->getMath()->interpolatePose(p1, p2, alpha);

        // 接口调用: 根据计算出的差值位姿，检查是否能找到有效的逆解
        auto result = impl->getRobotInterface(robot_name)
                          ->getRobotAlgorithm()
                          ->inverseKinematicsAll(pose);

        if (std::get<1>(result) != 0) {
            std::cout << "逆解失败, inverseKinematicsAll返回值:"
                      << std::get<1>(result) << std::endl;
            std::cout << "轨迹规划失败" << std::endl;
            return false;
        }
    }

    // 如果所有插值点均有效，表示轨迹规划成功
    std::cout << "轨迹规划成功" << std::endl;
    return true;
}

#define LOCAL_IP "127.0.0.1"

int main(int argc, char **argv)
{
#ifdef WIN32
    // 将Windows控制台输出代码页设置为 UTF-8
    SetConsoleOutputCP(CP_UTF8);
#endif
    auto rpc_cli = std::make_shared<RpcClient>();
    // 接口调用: 设置 RPC 超时, 单位: ms
    rpc_cli->setRequestTimeout(1000);
    // 接口调用: 连接到 RPC 服务
    rpc_cli->connect(LOCAL_IP, 30004);
    // 接口调用: 登录
    rpc_cli->login("aubo", "123456");

    // 起始位姿和目标位姿
    std::vector<double> pose1 = { 0.551, -0.295, 0.261, -3.135, 0.0, 1.569 };
    std::vector<double> pose2 = { 0.551, 0.295, 0.261, -3.135, 0.0, 0 };

    // 插值点数量
    int num_points = 30;

    // 检查给定的轨迹是否有效，进行插值和逆解验证
    exampleTrajectoryValid(rpc_cli, pose1, pose2, num_points);

    // 接口调用: 退出登录
    rpc_cli->logout();
    // 接口调用: 断开连接
    rpc_cli->disconnect();

    return 0;
}
