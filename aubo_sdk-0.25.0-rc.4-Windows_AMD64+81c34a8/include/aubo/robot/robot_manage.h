/** @file  robot_manage.h
 *  @brief 机器人管理接口，如上电、启动、拖动示教模式等
 */
#ifndef AUBO_SDK_ROBOT_CONTROL_INTERFACE_H
#define AUBO_SDK_ROBOT_CONTROL_INTERFACE_H

#include <vector>
#include <thread>

#include <aubo/global_config.h>
#include <aubo/type_def.h>

namespace arcs {
namespace common_interface {

class ARCS_ABI_EXPORT RobotManage
{
public:
    RobotManage();
    virtual ~RobotManage();

    /**
     * 发起机器人上电请求
     *
     * @return 成功返回0; 失败返回错误码
     * AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * poweron(self: pyaubo_sdk.RobotManage) -> int
     *
     * @par Lua函数原型
     * poweron() -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotManage.poweron","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     * @par C++示例
     * @code
     * auto rpc_cli = std::make_shared<RpcClient>();
     * auto robot_name = rpc_cli->getRobotNames().front();
     * rpc_cli->getRobotInterface(robot_name)->getRobotManage()->poweron();
     * @endcode
     *
     */
    int poweron();

    /**
     * 发起机器人启动请求
     *
     * @return 成功返回0; 失败返回错误码
     * AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * startup(self: pyaubo_sdk.RobotManage) -> int
     *
     * @par Lua函数原型
     * startup() -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotManage.startup","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     * @par C++示例
     * @code
     * auto rpc_cli = std::make_shared<RpcClient>();
     * auto robot_name = rpc_cli->getRobotNames().front();
     * rpc_cli->getRobotInterface(robot_name)->getRobotManage()->startup();
     * @endcode
     */
    int startup();

    /**
     * 发起机器人松开刹车请求
     *
     * @return 成功返回0; 失败返回错误码
     * AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * releaseRobotBrake(self: pyaubo_sdk.RobotManage) -> int
     *
     * @par Lua函数原型
     * releaseRobotBrake() -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotManage.releaseRobotBrake","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     * @par C++示例
     * @code
     * auto rpc_cli = std::make_shared<RpcClient>();
     * auto robot_name = rpc_cli->getRobotNames().front();
     * rpc_cli->getRobotInterface(robot_name)->getRobotManage()->releaseRobotBrake();
     * @endcode
     */
    int releaseRobotBrake();

    /**
     * 发起机器人刹车请求
     *
     * @return 成功返回0; 失败返回错误码
     * AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * lockRobotBrake(self: pyaubo_sdk.RobotManage) -> int
     *
     * @par Lua函数原型
     * lockRobotBrake() -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotManage.lockRobotBrake","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     * @par C++示例
     * @code
     * auto rpc_cli = std::make_shared<RpcClient>();
     * auto robot_name = rpc_cli->getRobotNames().front();
     * rpc_cli->getRobotInterface(robot_name)->getRobotManage()->lockRobotBrake();
     * @endcode
     */
    int lockRobotBrake();

    /**
     * 发起机器人断电请求
     *
     * @return 成功返回0; 失败返回错误码
     * AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * poweroff(self: pyaubo_sdk.RobotManage) -> int
     *
     * @par Lua函数原型
     * poweroff() -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotManage.poweroff","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     * @par C++示例
     * @code
     * auto rpc_cli = std::make_shared<RpcClient>();
     * auto robot_name = rpc_cli->getRobotNames().front();
     * rpc_cli->getRobotInterface(robot_name)->getRobotManage()->poweroff();
     * @endcode
     */
    int poweroff();

    /**
     * 发起机器人反向驱动请求
     *
     * @param enable
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
     * backdrive(self: pyaubo_sdk.RobotManage, arg0: bool) -> int
     *
     * @par Lua函数原型
     * backdrive(enable: boolean) -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotManage.backdrive","params":[false],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     * @par C++示例
     * @code
     * auto rpc_cli = std::make_shared<RpcClient>();
     * auto robot_name = rpc_cli->getRobotNames().front();
     * rpc_cli->getRobotInterface(robot_name)->getRobotManage()->backdrive(true);
     * @endcode
     *
     */
    int backdrive(bool enable);

    /**
     * 发起机器人自由驱动请求
     * 接口在软件版本 0.31.x 后已废弃，使用 handguideMode 接口替换
     * handguideMode({1,1,1,1,1}, {0,0,0,0,0,0})
     *
     * @param enable
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
     * freedrive(self: pyaubo_sdk.RobotManage, arg0: bool) -> int
     *
     * @par Lua函数原型
     * freedrive(enable: boolean) -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotManage.freedrive","params":[true],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     * @par C++示例
     * @code
     * auto rpc_cli = std::make_shared<RpcClient>();
     * auto robot_name = rpc_cli->getRobotNames().front();
     * rpc_cli->getRobotInterface(robot_name)->getRobotManage()->freedrive(true);
     * @endcode
     *
     */
    int freedrive(bool enable);

    /**
     * 高阶拖动示教
     *
     * @param freeAxes 可以拖动的轴 0-不能拖动 1-可以拖动
     * @param feature 如果维度为0，代表基于 TCP 坐标系拖动
     *
     * @return 成功返回0; 失败返回错误码
     * AUBO_BUSY
     * AUBO_BAD_STATE
     * -AUBO_INVL_ARGUMENT
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     */
    int handguideMode(const std::vector<int> &freeAxes,
                      const std::vector<double> &feature);

    /**
     * 退出拖动示教
     *
     * @note 暂未实现
     *
     * @return 成功返回0; 失败返回错误码
     * AUBO_BUSY
     * AUBO_BAD_STATE
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     */
    int exitHandguideMode();

    /**
     * 获取拖动示教器的状态（是否处于奇异空间）
     *
     * @note 暂未实现
     *
     * @return
     * • 0 - 正常操作.
     * • 1 - 接近奇异空间.
     * • 2 - 极其接近奇异点，将产生较大的拖动阻尼.
     *
     * @throws arcs::common_interface::AuboException
     *
     */
    int getHandguideStatus();

    /**
     * 获取拖动示教器触发源
     *
     * @note 暂未实现
     *
     * @return
     *
     * @throws arcs::common_interface::AuboException
     *
     */
    int getHandguideTrigger();

    /**
     * 获取拖动示教使能状态
     *
     * @return 使能返回true; 失能返回false
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotManage.isHandguideEnabled","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":false}
     *
     */
    bool isHandguideEnabled();

    /**
     * 发起机器人进入/退出仿真模式请求
     *
     * @param enable
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
     * setSim(self: pyaubo_sdk.RobotManage, arg0: bool) -> int
     *
     * @par Lua函数原型
     * setSim(enable: boolean) -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotManage.setSim","params":[true],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     * @par C++示例
     * @code
     * auto rpc_cli = std::make_shared<RpcClient>();
     * auto robot_name = rpc_cli->getRobotNames().front();
     * rpc_cli->getRobotInterface(robot_name)->getRobotManage()->setSim(true);
     * @endcode
     *
     */
    int setSim(bool enable);

    /**
     * 设置机器人操作模式
     *
     * @param mode 操作模式
     *
     * @return 成功返回0; 失败返回错误码
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * setOperationalMode(self: pyaubo_sdk.RobotManage, arg0:
     * arcs::common_interface::OperationalModeType) -> int
     *
     * @par Lua函数原型
     * setOperationalMode(mode: number) -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotManage.setOperationalMode","params":["Manual"],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     * @par C++示例
     * @code
     * auto rpc_cli = std::make_shared<RpcClient>();
     * auto robot_name = rpc_cli->getRobotNames().front();
     * rpc_cli->getRobotInterface(robot_name)->getRobotManage()->setOperationalMode(OperationalModeType::Automatic);
     * @endcode
     *
     */
    int setOperationalMode(OperationalModeType mode);

    /**
     * 获取机器人操作模式
     *
     * @return 机器人操作模式
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getOperationalMode(self: pyaubo_sdk.RobotManage) ->
     * arcs::common_interface::OperationalModeType
     *
     * @par Lua函数原型
     * getOperationalMode() -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotManage.getOperationalMode","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":"Manual"}
     *
     * @par C++示例
     * @code
     * auto rpc_cli = std::make_shared<RpcClient>();
     * auto robot_name = rpc_cli->getRobotNames().front();
     * OperationalModeType mode =
     * rpc_cli->getRobotInterface(robot_name)->getRobotManage()->getOperationalMode();
     * @endcode
     *
     */
    OperationalModeType getOperationalMode();

    /**
     * 获取控制模式
     *
     * @return 控制模式
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getRobotControlMode(self: pyaubo_sdk.RobotManage) ->
     * arcs::common_interface::RobotControlModeType
     *
     * @par Lua函数原型
     * getRobotControlMode() -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotManage.getRobotControlMode","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":"Position"}
     *
     * @par C++示例
     * @code
     * auto rpc_cli = std::make_shared<RpcClient>();
     * auto robot_name = rpc_cli->getRobotNames().front();
     * RobotControlModeType mode =
     * rpc_cli->getRobotInterface(robot_name)->getRobotManage()->getRobotControlMode();
     * @endcode
     */
    RobotControlModeType getRobotControlMode();

    /**
     * 是否使能了拖动示教模式
     *
     * @return 使能返回true; 反之返回false
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * isFreedriveEnabled(self: pyaubo_sdk.RobotManage) -> bool
     *
     * @par Lua函数原型
     * isFreedriveEnabled() -> boolean
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotManage.isFreedriveEnabled","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":false}
     *
     * @par C++示例
     * @code
     * auto rpc_cli = std::make_shared<RpcClient>();
     * auto robot_name = rpc_cli->getRobotNames().front();
     * bool isEnabled =
     * rpc_cli->getRobotInterface(robot_name)->getRobotManage()->isFreedriveEnabled();
     * @endcode
     *
     */
    bool isFreedriveEnabled();

    /**
     * 是否使能了反向驱动模式
     *
     * @return 使能返回true; 反之返回false
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * isBackdriveEnabled(self: pyaubo_sdk.RobotManage) -> bool
     *
     * @par Lua函数原型
     * isBackdriveEnabled() -> boolean
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotManage.isBackdriveEnabled","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":false}
     *
     * @par C++示例
     * @code
     * auto rpc_cli = std::make_shared<RpcClient>();
     * auto robot_name = rpc_cli->getRobotNames().front();
     * bool isEnabled =
     * rpc_cli->getRobotInterface(robot_name)->getRobotManage()->isBackdriveEnabled();
     * @endcode
     */
    bool isBackdriveEnabled();

    /**
     * 是否使能了仿真模式
     *
     * @return 使能返回true; 反之返回false
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * isSimulationEnabled(self: pyaubo_sdk.RobotManage) -> bool
     *
     * @par Lua函数原型
     * isSimulationEnabled() -> boolean
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotManage.isSimulationEnabled","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":true}
     *
     * @par C++示例
     * @code
     * auto rpc_cli = std::make_shared<RpcClient>();
     * auto robot_name = rpc_cli->getRobotNames().front();
     * bool isEnabled =
     * rpc_cli->getRobotInterface(robot_name)->getRobotManage()->isSimulationEnabled();
     * @endcode
     */
    bool isSimulationEnabled();

    /**
     * 清除防护停机，包括碰撞停机
     *
     * @return 成功返回0；失败返回错误码
     * AUBO_BUSY
     * AUBO_BAD_STATE
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * setUnlockProtectiveStop(self: pyaubo_sdk.RobotManage) -> int
     *
     * @par Lua函数原型
     * setUnlockProtectiveStop() -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotManage.setUnlockProtectiveStop","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     * @par C++示例
     * @code
     * auto rpc_cli = std::make_shared<RpcClient>();
     * auto robot_name = rpc_cli->getRobotNames().front();
     * rpc_cli->getRobotInterface(robot_name)->getRobotManage()->setUnlockProtectiveStop();
     * @endcode
     */
    int setUnlockProtectiveStop();

    /**
     * 重置安全接口板，一般在机器人断电之后需要重置时调用，比如机器人急停、故障等之后
     *
     * @return 成功返回0; 失败返回错误码
     * AUBO_BUSY
     * AUBO_BAD_STATE
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * restartInterfaceBoard(self: pyaubo_sdk.RobotManage) -> int
     *
     * @par Lua函数原型
     * restartInterfaceBoard() -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotManage.restartInterfaceBoard","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     * @par C++示例
     * @code
     * auto rpc_cli = std::make_shared<RpcClient>();
     * auto robot_name = rpc_cli->getRobotNames().front();
     * rpc_cli->getRobotInterface(robot_name)->getRobotManage()->restartInterfaceBoard();
     * @endcode
     */
    int restartInterfaceBoard();

    /**
     * 开始实时轨迹的记录
     *
     * @param file_name
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
    int startRecord(const std::string &file_name);

    /**
     * 停止实时记录
     *
     * @return 成功返回0；失败返回错误码
     * AUBO_BUSY
     * AUBO_BAD_STATE
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotManage.stopRecord","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int stopRecord();

    /**
     * 暂停实时记录
     *
     * @param pause
     *
     * @return 成功返回0；失败返回错误码
     * AUBO_BUSY
     * AUBO_BAD_STATE
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotManage.pauseRecord","params":[true],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int pauseRecord(bool pause);

    /**
     * 发起机器人进入/退出联动模式请求,
     * 只有操作模式为自动或者无时，才能使能联动模式
     *
     * @param enable
     *
     * @return 成功返回0; 失败返回错误码
     * AUBO_BUSY
     * AUBO_REQUEST_IGNORE
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * setLinkModeEnable(self: pyaubo_sdk.RobotManage, arg0: bool) -> int
     *
     * @par Lua函数原型
     * setLinkModeEnable(enable: boolean) -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotManage.setLinkModeEnable","params":[true],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     * @par C++示例
     * @code
     * auto rpc_cli = std::make_shared<RpcClient>();
     * auto robot_name = rpc_cli->getRobotNames().front();
     * rpc_cli->getRobotInterface(robot_name)->getRobotManage()->setLinkModeEnable(true);
     * @endcode
     *
     */
    int setLinkModeEnable(bool enable);

    /**
     * 是否使能了联动模式，联动模式下用户可以通过外部IO控制机器人（用户可以对IO的功能进行配置）
     *
     * @return 使能返回true; 反之返回false
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * isLinkModeEnabled(self: pyaubo_sdk.RobotManage) -> bool
     *
     * @par Lua函数原型
     * isLinkModeEnabled() -> boolean
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotManage.isLinkModeEnabled","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":false}
     *
     * @par C++示例
     * @code
     * auto rpc_cli = std::make_shared<RpcClient>();
     * auto robot_name = rpc_cli->getRobotNames().front();
     * bool isEnabled =
     * rpc_cli->getRobotInterface(robot_name)->getRobotManage()->isLinkModeEnabled();
     * @endcode
     */
    bool isLinkModeEnabled();

    /**
     * 手动触发生成诊断文件
     *
     * @return 指令下发成功返回0; 失败返回错误码。 \n
     * -AUBO_BAD_STATE: 运行时(RuntimeMachine)的当前状态不是Stopped,
     * 固件升级请求被拒绝。AUBO_BAD_STATE的值是1。 \n
     * -AUBO_TIMEOUT: 超时。AUBO_TIMEOUT的值是4。 \n
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * generateDiagnoseFile(self: pyaubo_sdk.RobotManage, arg0: str) -> int
     *
     * @par Lua函数原型
     * generateDiagnoseFile(reason: string) -> nil
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.RobotManage.generateDiagnoseFile","params":["reason"],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":false}
     *
     * @par C++示例
     * @code
     * auto rpc_cli = std::make_shared<RpcClient>();
     * auto robot_name = rpc_cli->getRobotNames().front();
     * bool isEnabled =
     * rpc_cli->getRobotInterface(robot_name)->getRobotManage()->generateDiagnoseFile("reason");
     * @endcode
     */
    int generateDiagnoseFile(const std::string &reason);

protected:
    void *d_;
};
using RobotManagePtr = std::shared_ptr<RobotManage>;
} // namespace common_interface
} // namespace arcs

#endif // AUBO_SDK_ROBOT_CONTROL_INTERFACE_H
