/** @file  robot_config.h
 *  @brief 获取机器人配置接口，如获取DH参数、碰撞等级、安装位姿等等
 */
#ifndef AUBO_SDK_ROBOT_CONFIG_H
#define AUBO_SDK_ROBOT_CONFIG_H

#include <vector>
#include <unordered_map>

#include <aubo/global_config.h>
#include <aubo/type_def.h>

namespace arcs {
namespace common_interface {

class ARCS_ABI_EXPORT RobotConfig
{
public:
    RobotConfig();
    virtual ~RobotConfig();

    /**
     * 获取机器人的名字
     *
     * @return 返回机器人的名字
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getName(self: pyaubo_sdk.RobotConfig) -> str
     *
     * @par Lua函数原型
     * getName() -> string
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.getName","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":"rob1"}
     *
     */
    std::string getName();

    /**
     * 获取机器人的自由度(从硬件抽象层读取)
     *
     * @return 返回机器人的自由度
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getDof(self: pyaubo_sdk.RobotConfig) -> int
     *
     * @par Lua函数原型
     * getDof() -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.getDof","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":6}
     *
     */
    int getDof();

    /**
     * 获取机器人的伺服控制周期(从硬件抽象层读取)
     *
     * @return 机器人的伺服控制周期
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getCycletime(self: pyaubo_sdk.RobotConfig) -> float
     *
     * @par Lua函数原型
     * getCycletime() -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.getCycletime","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0.005}
     *
     */
    double getCycletime();

    /**
     * 预设缓速模式下的速度缩减比例
     *
     * @param level 缓速等级 1, 2
     * @param fraction
     *
     * @return 成功返回0；失败返回错误码
     * AUBO_BUSY
     * AUBO_BAD_STATE
     * AUBO_INVL_ARGUMENT
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.setSlowDownFraction","params":[1,0.8],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int setSlowDownFraction(int level, double fraction);

    /**
     * 获取预设的缓速模式下的速度缩减比例
     *
     * @param level 缓速等级 1, 2
     * @return 返回预设的缓速模式下的速度缩减比例
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.getSlowDownFraction","params":[1],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0.5}
     *
     */
    double getSlowDownFraction(int level);

    /**
     * 获取默认的工具端加速度，单位m/s^2
     *
     * @return 默认的工具端加速度
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getDefaultToolAcc(self: pyaubo_sdk.RobotConfig) -> float
     *
     * @par Lua函数原型
     * getDefaultToolAcc() -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.getDefaultToolAcc","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0.0}
     */
    double getDefaultToolAcc();

    /**
     * 获取默认的工具端速度，单位m/s
     *
     * @return 默认的工具端速度
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getDefaultToolSpeed(self: pyaubo_sdk.RobotConfig) -> float
     *
     * @par Lua函数原型
     * getDefaultToolSpeed() -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.getDefaultToolSpeed","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0.0}
     *
     */
    double getDefaultToolSpeed();

    /**
     * 获取默认的关节加速度，单位rad/s^2
     *
     * @return 默认的关节加速度
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getDefaultJointAcc(self: pyaubo_sdk.RobotConfig) -> float
     *
     * @par Lua函数原型
     * getDefaultJointAcc() -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.getDefaultJointAcc","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0.0}
     *
     */
    double getDefaultJointAcc();

    /**
     * 获取默认的关节速度，单位rad/s
     *
     * @return 默认的关节速度
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getDefaultJointSpeed(self: pyaubo_sdk.RobotConfig) -> float
     *
     * @par Lua函数原型
     * getDefaultJointSpeed() -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.getDefaultJointSpeed","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0.0}
     *
     */
    double getDefaultJointSpeed();

    /**
     * 获取机器人类型代码
     *
     * @return 机器人类型代码
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getRobotType(self: pyaubo_sdk.RobotConfig) -> str
     *
     * @par Lua函数原型
     * getRobotType() -> string
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.getRobotType","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":"aubo_i5H"}
     *
     */
    std::string getRobotType();

    /**
     * 获取机器人子类型代码
     *
     * @return 机器人子类型代码
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getRobotSubType(self: pyaubo_sdk.RobotConfig) -> str
     *
     * @par Lua函数原型
     * getRobotSubType() -> string
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.getRobotSubType","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":"B0"}
     *
     */
    std::string getRobotSubType();

    /**
     * 获取控制柜类型代码
     *
     * @return 控制柜类型代码
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getControlBoxType(self: pyaubo_sdk.RobotConfig) -> str
     *
     * @par Lua函数原型
     * getControlBoxType() -> string
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.getControlBoxType","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":"cb_ISStation"}
     *
     */
    std::string getControlBoxType();

    /**
     * 设置安装位姿(机器人的基坐标系相对于世界坐标系)  world->base
     *
     * 一般在多机器人系统中使用，默认为 [0,0,0,0,0,0]
     *
     * @param pose 安装位姿
     * @return 成功返回0; 失败返回错误码
     * AUBO_BUSY
     * AUBO_BAD_STATE
     * -AUBO_INVL_ARGUMENT
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * setMountingPose(self: pyaubo_sdk.RobotConfig, arg0: List[float]) -> int
     *
     * @par Lua函数原型
     * setMountingPose(pose: table) -> nil
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.setMountingPose","params":[[0.0,0.0,0.0,0.0,0.0,0.0]],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int setMountingPose(const std::vector<double> &pose);

    /**
     * 获取安装位姿(机器人的基坐标系相对于世界坐标系)
     *
     * @return 安装位姿
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getMountingPose(self: pyaubo_sdk.RobotConfig) -> List[float]
     *
     * @par Lua函数原型
     * getMountingPose() -> table
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.getMountingPose","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":[0.0,0.0,0.0,0.0,0.0,0.0]}
     *
     */
    std::vector<double> getMountingPose();

    // 将机器人绑定到一个坐标系，如果这个坐标系是运动的，那机器人也会跟着运动
    // 应用于地轨或者龙门
    // 这个函数调用的时候 frame 和 ROBOTBASE 的相对关系就固定了
    int attachRobotBaseTo(const std::string &frame);
    std::string getRobotBaseParent();

    // 设置工件数据，编程点位都是基于工件坐标系
    int setWorkObjectData(const WObjectData &wobj);

    /**
     * 设置碰撞灵敏度等级
     * 数值越大越灵敏
     *
     * @param level 碰撞灵敏度等级
     * 0: 关闭碰撞检测功能
     * 1~9: 碰撞灵敏等级
     * @return 成功返回0; 失败返回错误码
     * AUBO_BUSY
     * AUBO_BAD_STATE
     * -AUBO_INVL_ARGUMENT
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * setCollisionLevel(self: pyaubo_sdk.RobotConfig, arg0: int) -> int
     *
     * @par Lua函数原型
     * setCollisionLevel(level: number) -> nil
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.setCollisionLevel","params":[6],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int setCollisionLevel(int level);

    /**
     * 获取碰撞灵敏度等级
     *
     * @return 碰撞灵敏度等级
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getCollisionLevel(self: pyaubo_sdk.RobotConfig) -> int
     *
     * @par Lua函数原型
     * getCollisionLevel() -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.getCollisionLevel","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":6}
     *
     */
    int getCollisionLevel();

    /**
     * 设置碰撞停止类型
     *
     * @param type 类型 \n
     * 0: 碰撞后浮动，即碰撞之后进入拖动示教模式 \n
     * 1: 碰撞后静止 \n
     * 2: 碰撞后抱闸
     *
     * @return 成功返回0; 失败返回错误码
     * AUBO_BUSY
     * AUBO_BAD_STATE
     * -AUBO_INVL_ARGUMENT
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * setCollisionStopType(self: pyaubo_sdk.RobotConfig, arg0: int) -> int
     *
     * @par Lua函数原型
     * setCollisionStopType(type: number) -> nil
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.setCollisionStopType","params":[1],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int setCollisionStopType(int type);

    /**
     * 获取碰撞停止类型
     *
     * @return 返回碰撞停止类型 \n
     * 0: 碰撞后浮动，即碰撞之后进入拖动示教模式 \n
     * 1: 碰撞后静止 \n
     * 2: 碰撞后抱闸
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getCollisionStopType(self: pyaubo_sdk.RobotConfig) -> int
     *
     * @par Lua函数原型
     * getCollisionStopType() -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.getCollisionStopType","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":1}
     *
     */
    int getCollisionStopType();

    /**
     * 设置机器人的原点位置
     *
     * @param positions 关节角度
     * @return 成功返回0；失败返回错误码
     * AUBO_BUSY
     * AUBO_BAD_STATE
     * -AUBO_INVL_ARGUMENT
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.setHomePosition","params":[[0.0,-0.2617993877991494,1.74532925199433,0.4363323129985824,1.570796326794897,0.0]],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int setHomePosition(const std::vector<double> &positions);

    /**
     * 获取机器人的原点位置
     *
     * @return
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.getHomePosition","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":[0.0,-0.2617993877991494,1.74532925199433,0.4363323129985824,1.570796326794897,0.0]}
     *
     */
    std::vector<double> getHomePosition();

    /**
     * 设置拖动阻尼
     *
     * @param damp 阻尼
     *
     * @return 成功返回0；失败返回错误码
     * AUBO_BUSY
     * AUBO_BAD_STATE
     * -AUBO_INVL_ARGUMENT
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * setFreedriveDamp(self: pyaubo_sdk.RobotConfig, arg0: List[float]) -> int
     *
     * @par Lua函数原型
     * setFreedriveDamp(damp: table) -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.setFreedriveDamp","params":[[0.5,0.5,0.5,0.5,0.5,0.5]],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int setFreedriveDamp(const std::vector<double> &damp);

    /**
     * 获取拖动阻尼
     *
     * @return 拖动阻尼
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getFreedriveDamp(self: pyaubo_sdk.RobotConfig) -> List[float]
     *
     * @par Lua函数原型
     * getFreedriveDamp() -> table
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.getFreedriveDamp","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":[0.5,0.5,0.5,0.5,0.5,0.5]}
     *
     */
    std::vector<double> getFreedriveDamp();

    /**
     * 获取机器人DH参数
     * alpha a d theta beta
     *
     * @param real 读取真实参数(理论值+补偿值)或者理论参数
     * @return 返回机器人DH参数
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getKinematicsParam(self: pyaubo_sdk.RobotConfig, arg0: bool) -> Dict[str,
     * List[float]]
     *
     * @par Lua函数原型
     * getKinematicsParam(real: boolean) -> table
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.getKinematicsParam","params":[true],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":{"a":[0.0,-0.0001255999959539622,0.4086348000024445,0.3757601999930339,-0.00018950000230688602,3.7000001611886546e-05],
     * "alpha":[0.0,-1.5701173967707458,3.1440308735524347,3.14650750358636,-1.5703093767832055,1.5751177669179182],"beta":[0.0,0.0,0.0,0.0,0.0,0.0],
     * "d":[0.122,0.12154769999941345,-4.769999941345304e-05,4.769999941345304e-05,0.10287890000385232,0.116],
     * "theta":[3.141592653589793,-1.5707963267948966,0.0,-1.5707963267948966,0.0,0.0]}}
     *
     */
    std::unordered_map<std::string, std::vector<double>> getKinematicsParam(
        bool real);

    /**
     * 获取指定温度下的DH参数补偿值:alpha a d theta beta
     *
     * @param ref_temperature 参考温度 ℃，默认20℃
     * @return 返回DH参数补偿值
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getKinematicsCompensate(self: pyaubo_sdk.RobotConfig, arg0: float) ->
     * Dict[str, List[float]]
     *
     * @par Lua函数原型
     * getKinematicsCompensate(ref_temperature: number) -> table
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.getKinematicsCompensate","params":[20],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":{"a":[0.0,-0.0001255999959539622,0.0006348000024445355,-0.0002398000069661066,-0.00018950000230688602,3.7000001611886546e-05],
     * "alpha":[0.0,0.000678930024150759,0.002438219962641597,0.0049148499965667725,0.00048695001169107854,0.004321440123021603],
     * "beta":[0.0,0.0,0.0,0.0,0.0,0.0],"d":[0.0,4.769999941345304e-05,-4.769999941345304e-05,4.769999941345304e-05,0.0003789000038523227,0.0],
     * "theta":[0.0,0.0,0.0,0.0,0.0,0.0]}}
     *
     */
    std::unordered_map<std::string, std::vector<double>>
    getKinematicsCompensate(double ref_temperature);

    /**
     * 设置标准 DH 补偿到机器人
     *
     * @param param
     *
     * @return 成功返回0；失败返回错误码
     * AUBO_BUSY
     * AUBO_BAD_STATE
     * -AUBO_INVL_ARGUMENT
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     */
    int setKinematicsCompensate(
        const std::unordered_map<std::string, std::vector<double>> &param);

    /**
     * 设置需要保存到接口板底座的参数
     *
     * @param param 补偿数据
     *
     * @return 成功返回0；失败返回错误码
     * AUBO_BUSY
     * AUBO_BAD_STATE
     * -AUBO_INVL_ARGUMENT
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * setPersistentParameters(self: pyaubo_sdk.RobotConfig, arg0: str) -> int
     *
     * @par Lua函数原型
     * setPersistentParameters(param: string) -> nil
     *
     */
    int setPersistentParameters(const std::string &param);

    /**
     * 设置硬件抽象层自定义参数
     * 目的是为了做不同硬件之间的兼容
     *
     * @param param 自定义参数
     *
     * @return 成功返回0；失败返回错误码
     * AUBO_BUSY
     * AUBO_BAD_STATE
     * -AUBO_INVL_ARGUMENT
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     */
    int setHardwareCustomParameters(const std::string &param);

    /**
     * 获取硬件抽象层自定义参数
     *
     * @param
     * @return 返回硬件抽象层自定义的参数
     *
     * @throws arcs::common_interface::AuboException
     *
     */
    std::string getHardwareCustomParameters(const std::string &param);

    /**
     * 设置机器人关节零位
     *
     * @return 成功返回0；失败返回错误码
     * AUBO_BUSY
     * AUBO_BAD_STATE
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * setRobotZero(self: pyaubo_sdk.RobotConfig) -> int
     *
     * @par Lua函数原型
     * setRobotZero() -> nil
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.setRobotZero","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int setRobotZero();

    /**
     * 获取可用的末端力矩传感器的名字
     *
     * @return 返回可用的末端力矩传感器的名字
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getTcpForceSensorNames(self: pyaubo_sdk.RobotConfig) -> List[str]
     *
     * @par Lua函数原型
     * getTcpForceSensorNames() -> table
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.getTcpForceSensorNames","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":[]}
     *
     */
    std::vector<std::string> getTcpForceSensorNames();

    /**
     * 设置末端力矩传感器
     * 如果存在内置的末端力矩传感器，默认将使用内置的力矩传感器
     *
     * @param name 末端力矩传感器的名字
     *
     * @return 成功返回0；失败返回错误码
     * AUBO_BUSY
     * AUBO_BAD_STATE
     * -AUBO_INVL_ARGUMENT
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * selectTcpForceSensor(self: pyaubo_sdk.RobotConfig, arg0: str) -> int
     *
     * @par Lua函数原型
     * selectTcpForceSensor(name: string) -> nil
     *
     */
    int selectTcpForceSensor(const std::string &name);

    /**
     * 设置传感器安装位姿
     *
     * @param sensor_pose 传感器安装位姿
     *
     * @return 成功返回0；失败返回错误码
     * AUBO_BUSY
     * AUBO_BAD_STATE
     * -AUBO_INVL_ARGUMENT
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     */
    int setTcpForceSensorPose(const std::vector<double> &sensor_pose);

    /**
     * 获取传感器安装位姿
     *
     * @return 传感器安装位姿
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.getTcpForceSensorPose","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":[0.0,0.0,0.0,0.0,0.0,0.0]}
     *
     */
    std::vector<double> getTcpForceSensorPose();

    /**
     * 是否安装了末端力矩传感器
     *
     * @return 安装返回true; 没有安装返回false
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * hasTcpForceSensor(self: pyaubo_sdk.RobotConfig) -> bool
     *
     * @par Lua函数原型
     * hasTcpForceSensor() -> boolean
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.hasTcpForceSensor","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":true}
     *
     */
    bool hasTcpForceSensor();

    /**
     * 设置末端力矩偏移
     *
     * @param force_offset 末端力矩偏移
     * @return 成功返回0; 失败返回错误码
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * setTcpForceOffset(self: pyaubo_sdk.RobotConfig, arg0: List[float]) -> int
     *
     * @par Lua函数原型
     * setTcpForceOffset(force_offset: table) -> nil
     *
     */
    int setTcpForceOffset(const std::vector<double> &force_offset);

    /**
     * 获取末端力矩偏移
     *
     * @return 返回末端力矩偏移
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getTcpForceOffset(self: pyaubo_sdk.RobotConfig) -> List[float]
     *
     * @par Lua函数原型
     * getTcpForceOffset() -> table
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.getTcpForceOffset","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":[0.0,0.0,0.0,0.0,0.0,0.0]}
     *
     */
    std::vector<double> getTcpForceOffset();

    /**
     * 获取可用的底座力矩传感器的名字
     *
     * @return 返回可用的底座力矩传感器的名字
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getBaseForceSensorNames(self: pyaubo_sdk.RobotConfig) -> List[str]
     *
     * @par Lua函数原型
     * getBaseForceSensorNames() -> table
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.getBaseForceSensorNames","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":[]}
     *
     */
    std::vector<std::string> getBaseForceSensorNames();

    /**
     * 设置底座力矩传感器
     * 如果存在内置的底座力矩传感器，默认将使用内置的力矩传感器
     *
     * @param name 底座力矩传感器的名字
     *
     * @return 成功返回0；失败返回错误码
     * AUBO_BUSY
     * AUBO_BAD_STATE
     * -AUBO_INVL_ARGUMENT
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * selectBaseForceSensor(self: pyaubo_sdk.RobotConfig, arg0: str) -> int
     *
     * @par Lua函数原型
     * selectBaseForceSensor(name: string) -> nil
     *
     */
    int selectBaseForceSensor(const std::string &name);

    /**
     * 是否安装了底座力矩传感器
     *
     * @return 安装返回true;没有安装返回false
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * hasBaseForceSensor(self: pyaubo_sdk.RobotConfig) -> bool
     *
     * @par Lua函数原型
     * hasBaseForceSensor() -> boolean
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.hasBaseForceSensor","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":false}
     *
     */
    bool hasBaseForceSensor();

    /**
     * 设置底座力矩偏移
     *
     * @param force_offset 底座力矩偏移
     *
     * @return 成功返回0；失败返回错误码
     * AUBO_BUSY
     * AUBO_BAD_STATE
     * -AUBO_INVL_ARGUMENT
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * setBaseForceOffset(self: pyaubo_sdk.RobotConfig, arg0: List[float]) ->
     * int
     *
     * @par Lua函数原型
     * setBaseForceOffset(force_offset: table) -> nil
     *
     */
    int setBaseForceOffset(const std::vector<double> &force_offset);

    /**
     * 获取底座力矩偏移
     *
     * @return 返回底座力矩偏移
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getBaseForceOffset(self: pyaubo_sdk.RobotConfig) -> List[float]
     *
     * @par Lua函数原型
     * getBaseForceOffset() -> table
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.getBaseForceOffset","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":[]}
     *
     */
    std::vector<double> getBaseForceOffset();

    /**
     * 获取安全参数校验码 CRC32
     *
     * @return 返回安全参数校验码
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getSafetyParametersCheckSum(self: pyaubo_sdk.RobotConfig) -> int
     *
     * @par Lua函数原型
     * getSafetyParametersCheckSum() -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.getSafetyParametersCheckSum","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":2033397241}
     *
     */
    uint32_t getSafetyParametersCheckSum();

    /**
     * 发起确认安全配置参数请求:
     * 将安全配置参数写入到安全接口板flash或文件
     *
     * @param parameters 安全配置参数
     *
     * @return 成功返回0；失败返回错误码
     * AUBO_BUSY
     * AUBO_BAD_STATE
     * -AUBO_INVL_ARGUMENT
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * confirmSafetyParameters(self: pyaubo_sdk.RobotConfig, arg0:
     * arcs::common_interface::RobotSafetyParameterRange) -> int
     *
     * @par Lua函数原型
     *
     *
     */
    int confirmSafetyParameters(const RobotSafetyParameterRange &parameters);

    /**
     * 计算安全参数的 CRC32 校验值
     *
     * @return crc32
     *
     * @throws arcs::common_interface::AuboException
     *
     */
    uint32_t calcSafetyParametersCheckSum(
        const RobotSafetyParameterRange &parameters);

    /**
     * 获取关节最大位置（物理极限）
     *
     * @return 返回关节最大位置
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getJointMaxPositions(self: pyaubo_sdk.RobotConfig) -> List[float]
     *
     * @par Lua函数原型
     * getJointMaxPositions() -> table
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.getJointMaxPositions","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":[6.283185307179586,6.283185307179586,6.283185307179586,6.283185307179586,6.283185307179586,6.283185307179586]}
     *
     */
    std::vector<double> getJointMaxPositions();

    /**
     * 获取关节最小位置（物理极限）
     *
     * @return 返回关节最小位置
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getJointMinPositions(self: pyaubo_sdk.RobotConfig) -> List[float]
     *
     * @par Lua函数原型
     * getJointMinPositions() -> table
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.getJointMinPositions","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":[-6.283185307179586,-6.283185307179586,-6.283185307179586,-6.283185307179586,-6.283185307179586,-6.283185307179586]}
     *
     */
    std::vector<double> getJointMinPositions();

    /**
     * 获取关节最大速度（物理极限）
     *
     * @return 返回关节最大速度
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getJointMaxSpeeds(self: pyaubo_sdk.RobotConfig) -> List[float]
     *
     * @par Lua函数原型
     * getJointMaxSpeeds() -> table
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.getJointMaxSpeeds","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":[3.892084231947355,3.892084231947355,3.892084231947355,3.1066860685499065,3.1066860685499065,3.1066860685499065]}
     *
     */
    std::vector<double> getJointMaxSpeeds();

    /**
     * 获取关节最大加速度（物理极限）
     *
     * @return 返回关节最大加速度
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getJointMaxAccelerations(self: pyaubo_sdk.RobotConfig) -> List[float]
     *
     * @par Lua函数原型
     * getJointMaxAccelerations() -> table
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.getJointMaxAccelerations","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":[31.104877758314785,31.104877758314785,31.104877758314785,20.73625684294463,20.73625684294463,20.73625684294463]}
     *
     */
    std::vector<double> getJointMaxAccelerations();

    /**
     * 获取TCP最大速度（物理极限）
     *
     * @return 返回TCP最大速度
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getTcpMaxSpeeds(self: pyaubo_sdk.RobotConfig) -> List[float]
     *
     * @par Lua函数原型
     * getTcpMaxSpeeds() -> table
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.getTcpMaxSpeeds","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":[2.0,5.0]}
     *
     */
    std::vector<double> getTcpMaxSpeeds();

    /**
     * 获取TCP最大加速度（物理极限）
     *
     * @return 返回TCP最大加速度
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getTcpMaxAccelerations(self: pyaubo_sdk.RobotConfig) -> List[float]
     *
     * @par Lua函数原型
     * getTcpMaxAccelerations() -> table
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.getTcpMaxAccelerations","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":[10.0,10.0]}
     *
     */
    std::vector<double> getTcpMaxAccelerations();

    /**
     * 设置机器人安装姿态
     *
     * @param gravity 安装姿态
     *
     * @return 成功返回0；失败返回错误码
     * AUBO_BUSY
     * AUBO_BAD_STATE
     * -AUBO_INVL_ARGUMENT
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * setGravity(self: pyaubo_sdk.RobotConfig, arg0: List[float]) -> int
     *
     * @par Lua函数原型
     * setGravity(gravity: table) -> nil
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.setGravity","params":[[0.0,0.0,-9.87654321]],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int setGravity(const std::vector<double> &gravity);

    /**
     * 获取机器人的安装姿态
     *
     * 如果机器人底座安装了姿态传感器，则从传感器读取数据，否则按照用户设置
     *
     * @return 返回安装姿态
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getGravity(self: pyaubo_sdk.RobotConfig) -> List[float]
     *
     * @par Lua函数原型
     * getGravity() -> table
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.getGravity","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":[0.0,0.0,-9.87654321]}
     *
     */
    std::vector<double> getGravity();

    /**
     * 设置当前的TCP偏移
     *
     * TCP偏移表示形式为(x,y,z,rx,ry,rz)。
     * 其中x、y、z是工具中心点（TCP）在基坐标系下相对于法兰盘中心的位置偏移，单位是m。
     * rx、ry、rz是工具中心点（TCP）在基坐标系下相对于法兰盘中心的的姿态偏移，是ZYX欧拉角，单位是rad。
     *
     * @param offset 当前的TCP偏移,形式为(x,y,z,rx,ry,rz)
     *
     * @return 成功返回0；失败返回错误码
     * AUBO_BUSY
     * AUBO_BAD_STATE
     * -AUBO_INVL_ARGUMENT
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * setTcpOffset(self: pyaubo_sdk.RobotConfig, arg0: List[float]) -> int
     *
     * @par Lua函数原型
     * setTcpOffset(offset: table) -> nil
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.setTcpOffset","params":[[0.0,0.0,0.0,0.0,0.0,0.0]],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int setTcpOffset(const std::vector<double> &offset);

    /**
     * 获取当前的TCP偏移
     *
     * TCP偏移表示形式为(x,y,z,rx,ry,rz)。
     * 其中x、y、z是工具中心点（TCP）在基坐标系下相对于法兰盘中心的位置偏移，单位是m。
     * rx、ry、rz是工具中心点（TCP）在基坐标系下相对于法兰盘中心的的姿态偏移，是ZYX欧拉角，单位是rad。
     *
     * @return 当前的TCP偏移,形式为(x,y,z,rx,ry,rz)
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getTcpOffset(self: pyaubo_sdk.RobotConfig) -> List[float]
     *
     * @par Lua函数原型
     * getTcpOffset() -> table
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.getTcpOffset","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":[0.0,0.0,0.0,0.0,0.0,0.0]}
     *
     */
    std::vector<double> getTcpOffset();

    /**
     * 设置工具端质量、质心及惯量
     *
     * @param m 工具端质量
     * @param com 质心
     * @param inertial 惯量
     *
     * @return 成功返回0；失败返回错误码
     * AUBO_BUSY
     * AUBO_BAD_STATE
     * -AUBO_INVL_ARGUMENT
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * setToolInertial(self: pyaubo_sdk.RobotConfig, arg0: float, arg1:
     * List[float], arg2: List[float]) -> int
     *
     * @par Lua函数原型
     * setToolInertial(m: number, com: table, inertial: table) -> nil
     *
     */
    int setToolInertial(double m, const std::vector<double> &com,
                        const std::vector<double> &inertial);

    /**
     * 设置有效负载
     *
     * @param m 质量, 单位: kg
     * @param cog 重心, 单位: m, 形式为(CoGx, CoGy, CoGz)
     * @param aom 力矩轴的方向, 单位: rad, 形式为(rx, ry, rz)
     * @param inertia 惯量, 单位: kg*m^2, 形式为(Ixx, Iyy, Izz, Ixy, Ixz, Iyz)或(Ixx, Ixy, Ixz, Iyx, Iyy, Iyz, Izx, Izy, Izz)
     *
     * @return 成功返回0；失败返回错误码
     * AUBO_BUSY
     * AUBO_BAD_STATE
     * -AUBO_INVL_ARGUMENT
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * setPayload(self: pyaubo_sdk.RobotConfig, arg0: float, arg1: List[float],
     * arg2: List[float], arg3: List[float]) -> int
     *
     * @par Lua函数原型
     * setPayload(m: number, cog: table, aom: table, inertia: table) -> nil
     *
     * @par Lua示例
     * setPayload(3, {0,0,0}, {0,0,0}, {0,0,0,0,0,0,0,0,0})
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.setPayload","params":[3,[0,0,0],[0,0,0],[0,0,0,0,0,0,0,0,0]],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int setPayload(double m, const std::vector<double> &cog,
                   const std::vector<double> &aom,
                   const std::vector<double> &inertia);

    /**
     * 获取有效负载
     *
     * @return 有效负载.
     * 第一个元素表示质量, 单位: kg;
     * 第二个元素表示重心, 单位: m, 形式为(CoGx, CoGy, CoGz);
     * 第三个元素表示力矩轴的方向, 单位: rad, 形式为(rx, ry, rz);
     * 第四个元素表示惯量, 单位: kg*m^2, 形式为(Ixx, Iyy, Izz, Ixy, Ixz, Iyz)
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getPayload(self: pyaubo_sdk.RobotConfig) -> Tuple[float, List[float],
     * List[float], List[float]]
     *
     * @par Lua函数原型
     * getPayload() -> number, table, table, table
     *
     * @par Lua示例
     * m, cog, aom, inertia = getPayload()
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.getPayload","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":[3.0,[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0]]}
     *
     */
    Payload getPayload();

    /**
     * 末端位姿是否在安全范围之内
     *
     * @param pose 末端位姿
     * @return 在安全范围内返回true; 反之返回false
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * toolSpaceInRange(self: pyaubo_sdk.RobotConfig, arg0: List[float]) -> bool
     *
     * @par Lua函数原型
     * toolSpaceInRange(pose: table) -> boolean
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.toolSpaceInRange","params":[[0.58712,
     * -0.15775, 0.48703, 2.76, 0.344, 1.432]],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":true}
     *
     */
    bool toolSpaceInRange(const std::vector<double> &pose);

    /**
     * 发起固件升级请求，控制器软件将进入固件升级模式
     *
     * @param fw 固件升级路径。该路径的格式为: 固件安装包路径\#升级节点列表。
     * 其中固件安装包路径和升级节点列表以井号(#)分隔，升级节点以逗号(,)分隔。
     * 如果节点名称后带有!，则表示强制(不带版本校验)升级该节点；
     * 反之，则表示带校验版本地升级节点，
     * 即在升级该节点前，会先判断当前版本和目标版本是否相同，如果相同就不升级该节点。\n
     * 可以根据实际需求灵活设置需要升级的节点。\n
     * 例如，
     * /tmp/firmware_update-1.0.42-rc.5+2347b0d.firm\#master_mcu!,slace_mcu!,
     * base!,tool!,joint1!,joint2!,joint3!,joint4!,joint5!,joint6!
     * 表示强制升级接口板主板、接口板从板、基座、工具和6个关节（joint1至joint6）。\n
     * all表示所有的节点，例如
     * /tmp/firm_XXX.firm\#all 表示带校验版本地升级全部节点，
     * /tmp/firm_XXX.firm\#all！表示强制升级全部节点
     *
     * @return 指令下发成功返回0; 失败返回错误码。 \n
     * -AUBO_BAD_STATE: 运行时(RuntimeMachine)的当前状态不是Stopped,
     * 固件升级请求被拒绝。AUBO_BAD_STATE的值是1。 \n
     * -AUBO_TIMEOUT: 超时。AUBO_TIMEOUT的值是4。 \n
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * firmwareUpdate(self: pyaubo_sdk.RobotConfig, arg0: str) -> int
     *
     * @par Lua函数原型
     * firmwareUpdate(fw: string) -> nil
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.firmwareUpdate","params":["/tmp/firmware_update-1.0.42-rc.12+3e33eac.firm#master_mcu"],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int firmwareUpdate(const std::string &fw);

    /**
     * 获取当前的固件升级的进程
     *
     * @return 当前的固件升级进程。 \n
     * 第一个元素表示步骤名称。如果是failed，则表示固件升级失败 \n
     * 第二个元素表示升级的进度(0~1)，完成之后，返回("", 1)
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getFirmwareUpdateProcess(self: pyaubo_sdk.RobotConfig) -> Tuple[str,
     * float]
     *
     * @par Lua函数原型
     * getFirmwareUpdateProcess() -> table
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.getFirmwareUpdateProcess","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":[" ",0.0]}
     *
     */
    std::tuple<std::string, double> getFirmwareUpdateProcess();

    /**
     * 获取关节最大位置（当前正在使用的限制值）
     *
     * @return 返回关节最大位置（当前正在使用的限制值）
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getLimitJointMaxPositions(self: pyaubo_sdk.RobotConfig) -> List[float]
     *
     * @par Lua函数原型
     * getLimitJointMaxPositions() -> table
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.getLimitJointMaxPositions","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":[6.2831854820251465,6.2831854820251465,6.2831854820251465,6.2831854820251465,6.2831854820251465,6.2831854820251465]}
     *
     */
    std::vector<double> getLimitJointMaxPositions();

    /**
     * 获取关节最小位置（当前正在使用的限制值）
     *
     * @return 返回关节最小位置（当前正在使用的限制值）
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getLimitJointMinPositions(self: pyaubo_sdk.RobotConfig) -> List[float]
     *
     * @par Lua函数原型
     * getLimitJointMinPositions() -> table
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.getLimitJointMinPositions","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":[-6.2831854820251465,-6.2831854820251465,-6.2831854820251465,-6.2831854820251465,-6.2831854820251465,-6.2831854820251465]}
     *
     */
    std::vector<double> getLimitJointMinPositions();

    /**
     * 获取关节最大速度（当前正在使用的限制值）
     *
     * @return 返回关节最大速度（当前正在使用的限制值）
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getLimitJointMaxSpeeds(self: pyaubo_sdk.RobotConfig) -> List[float]
     *
     * @par Lua函数原型
     * getLimitJointMaxSpeeds() -> table
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.getLimitJointMaxSpeeds","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":[3.8920841217041016,3.8920841217041016,3.8920841217041016,3.1066861152648926,3.1066861152648926,3.1066861152648926]}
     *
     */
    std::vector<double> getLimitJointMaxSpeeds();

    /**
     * 获取关节最大加速度（当前正在使用的限制值）
     *
     * @return 返回关节最大加速度（当前正在使用的限制值）
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getLimitJointMaxAccelerations(self: pyaubo_sdk.RobotConfig) ->
     * List[float]
     *
     * @par Lua函数原型
     * getLimitJointMaxAccelerations() -> table
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.getLimitJointMaxAccelerations","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":[31.104877758314785,31.104877758314785,31.104877758314785,20.73625684294463,20.73625684294463,20.73625684294463]}
     *
     */
    std::vector<double> getLimitJointMaxAccelerations();

    /**
     * 获取TCP最大速度（当前正在使用的限制值）
     *
     * @return 返回TCP最大速度（当前正在使用的限制值）
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getLimitTcpMaxSpeed(self: pyaubo_sdk.RobotConfig) -> List[float]
     *
     * @par Lua函数原型
     * getLimitTcpMaxSpeed() -> table
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.getLimitTcpMaxSpeed","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":2.0}
     *
     */
    double getLimitTcpMaxSpeed();

    /**
     * 获取当前安全停止的类型
     *
     * @return 返回当前安全停止的类型
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.getSafeguardStopType","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":"None"}
     *
     */
    SafeguedStopType getSafeguardStopType();

    /**
     * 按位获取完整的安全停止触发源
     *
     * @return 返回所有安全停止触发源
     *
     * 安全停止的原因:
     * 手动模式下可配置安全IO触发的安全停止 - 1<<0
     * 自动模式下可配置安全IO触发的安全停止 - 1<<1
     * 控制柜SI输入触发的安全停止 - 1<<2
     * 示教器三态开关触发的安全停止 - 1<<3
     * 自动切手动触发的安全停止 - 1<<4
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.getSafeguardStopSource","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int getSafeguardStopSource();

    /**
     * 按位获取完整的机器人紧急停止触发源
     *
     * @return 返回所有机器人紧急停止触发源
     *
     * 紧急停止的原因:
     * 控制柜紧急停止按钮触发 - 1<<0
     * 示教器紧急停止按钮触发 - 1<<1
     * 手柄紧急停止按钮触发 - 1<<2
     * 固定IO紧急停止触发 - 1<<3
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotConfig.getRobotEmergencyStopSource","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int getRobotEmergencyStopSource();

protected:
    void *d_;
};
using RobotConfigPtr = std::shared_ptr<RobotConfig>;

} // namespace common_interface
} // namespace arcs
#endif // AUBO_SDK_ROBOT_CONFIG_H
