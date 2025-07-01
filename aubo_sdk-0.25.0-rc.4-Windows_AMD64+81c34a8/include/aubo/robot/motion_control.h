/** @file  motion_control.h
 *  @brief 运动控制接口
 *
 *  The robot movements are programmed as pose-to-pose movements, that is move
 *  from the current position to a new position. The path between these two
 *  positions is then automatically calculated by the robot.
 */
#ifndef AUBO_SDK_MOTION_CONTROL_INTERFACE_H
#define AUBO_SDK_MOTION_CONTROL_INTERFACE_H

#include <vector>
#include <functional>
#include <thread>

#include <aubo/global_config.h>
#include <aubo/type_def.h>

namespace arcs {
namespace common_interface {

/**
 *  pathBuffer类型
 */
enum PathBufferType
{
    PathBuffer_TOPPRA = 1,      ///< 1: toppra 时间最优路径规划
    PathBuffer_CubicSpline = 2, ///< 2: cubic_spline(录制的轨迹)

    /// 3: 关节B样条插值，最少三个点
    /// 废弃，建议用5替代，现在实际是关节空间 CUBIC_SPLINE
    PathBuffer_JointSpline = 3,

    /// 4:关节B样条插值，最少三个点，但是传入的是笛卡尔空间位姿
    /// 废弃，建议用6替代，现在实际是关节空间 CUBIC_SPLINE
    PathBuffer_JointSplineC = 4,

    ///< 5: 关节B样条插值，最少三个点
    PathBuffer_JointBSpline = 5,

    /// 6:关节B样条插值，最少三个点，但是传入的是笛卡尔空间位姿
    PathBuffer_JointBSplineC = 6,
};

/**
 *  MotionControl类
 */
class ARCS_ABI_EXPORT MotionControl
{
public:
    MotionControl();
    virtual ~MotionControl();

    /**
     * 获取等效半径，单位 m
     * moveLine/moveCircle时，末端姿态旋转的角度等效到末端位置移动
     * 可以通过 setEqradius 设置，默认为1
     *
     * @return 返回等效半径
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getEqradius(self: pyaubo_sdk.MotionControl) -> float
     *
     * @par Lua函数原型
     * getEqradius() -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.getEqradius","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":1.0}
     *
     */
    double getEqradius();

    /**
     * 设置等效半径，单位 m
     * moveLine/moveCircle时，末端姿态旋转的角度等效到末端位置移动，数值越大，姿态旋转速度越快
     *
     * @param eqradius 0表示只规划移动，姿态旋转跟随移动
     *
     * @return 成功返回0; 失败返回错误码
     * AUBO_BAD_STATE
     * AUBO_BUSY
     * -AUBO_INVL_ARGUMENT
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.setEqradius","params":[0.8],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     * @par Python函数原型
     * setEqradius(self: pyaubo_sdk.MotionControl, arg0: float) -> int
     *
     * @par Lua函数原型
     * setEqradius(eqradius: number) -> number
     */
    int setEqradius(double eqradius);

    /**
     * 动态调整机器人运行速度和加速度比例 (0., 1.]
     *
     * @param fraction 机器人运行速度和加速度比例
     * @return 成功返回0; 失败返回错误码
     * AUBO_INVL_ARGUMENT
     * AUBO_BUSY
     * AUBO_BAD_STATE
     * -AUBO_INVL_ARGUMENT
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * setSpeedFraction(self: pyaubo_sdk.MotionControl, arg0: float) -> int
     *
     * @par Lua函数原型
     * setSpeedFraction(fraction: number) -> nil
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.setSpeedFraction","params":[0.8],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int setSpeedFraction(double fraction);

    /**
     * 获取速度和加速度比例，默认为 1
     * 可以通过 setSpeedFraction 接口设置
     *
     * @return 返回速度和加速度比例
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getSpeedFraction(self: pyaubo_sdk.MotionControl) -> float
     *
     * @par Lua函数原型
     * getSpeedFraction() -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.getSpeedFraction","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":1.0}
     *
     */
    double getSpeedFraction();

    /**
     * 速度比例设置临界区，使能之后速度比例被强制设定为1. ,
     * 失能之后恢复之前的速度比例
     *
     * @param enable
     * @return 成功返回0; 失败返回错误码
     * AUBO_BAD_STATE
     * AUBO_BUSY
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.speedFractionCritical","params":[true],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int speedFractionCritical(bool enable);

    /**
     * 是否处于速度比例设置临界区
     *
     * @return 处于速度比例设置临界区返回 true; 反之返回 false
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.isSpeedFractionCritical","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":true}
     *
     */
    bool isSpeedFractionCritical();

    /**
     * 是否处交融区
     *
     * @return 处交融区返回 true; 反之返回 false
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.isBlending","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":false}
     *
     */
    bool isBlending();

    /**
     * 设置偏移的最大速度和最大加速度
     * 仅对pathOffsetSet中 type=1 有效
     * @param v 最大速度
     * @param a 最大加速度
     * @return 成功返回0；失败返回错误码
     * AUBO_BUSY
     * AUBO_BAD_STATE
     * -AUBO_INVL_ARGUMENT
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * pathOffsetLimits(self: pyaubo_sdk.MotionControl, arg0: float, arg1:
     * float)
     * -> int
     *
     * @par Lua函数原型
     * pathOffsetLimits(v: number, a: number) -> nil
     *
     */
    int pathOffsetLimits(double v, double a);

    /**
     * 设置偏移的参考坐标系
     * 仅对pathOffsetSet中 type=1 有效
     * @param ref_coord 参考坐标系 0-基坐标系 1-TCP
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * pathOffsetCoordinate(self: pyaubo_sdk.MotionControl, arg0: int) -> float
     *
     * @par Lua函数原型
     * pathOffsetCoordinate(ref_coord: number) -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.pathOffsetCoordinate","params":[0],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int pathOffsetCoordinate(int ref_coord);

    /**
     * 路径偏移使能
     *
     * @return 成功返回0; 失败返回错误码
     * AUBO_BUSY
     * AUBO_BAD_STATE
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * pathOffsetEnable(self: pyaubo_sdk.MotionControl) -> int
     *
     * @par Lua函数原型
     * pathOffsetEnable() -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.pathOffsetEnable","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int pathOffsetEnable();

    /**
     * 设置路径偏移
     *
     * @param offset 在各方向的位姿偏移
     * @param type 运动类型 0-位置规划 1-速度规划
     * @return 成功返回0; 失败返回错误码
     * AUBO_BAD_STATE
     * AUBO_BUSY
     * -AUBO_INVL_ARGUMENT
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * pathOffsetSet(self: pyaubo_sdk.MotionControl, arg0: List[float], arg1:
     * int) -> int
     *
     * @par Lua函数原型
     * pathOffsetSet(offset: table, type: number) -> nil
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.pathOffsetSet","params":[[0,0,0.01,0,0,0],0],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int pathOffsetSet(const std::vector<double> &offset, int type = 0);

    /**
     * 路径偏移失能
     *
     * @return 成功返回0; 失败返回错误码
     * AUBO_BAD_STATE
     * AUBO_BUSY
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * pathOffsetDisable(self: pyaubo_sdk.MotionControl) -> int
     *
     * @par Lua函数原型
     * pathOffsetDisable() -> nil
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.pathOffsetDisable","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int pathOffsetDisable();

    /**
     * @brief 监控轨迹偏移范围
     * @param min: 沿坐标轴负方向最大偏移量
     * @param max: 沿坐标轴负正方向最大偏移量
     * @param strategy: 达到最大偏移量后监控策略
     * 　　　　0-禁用监控
     * 　　　　1-饱和限制，即维持最大姿态
     * 　　　　2-保护停止
     * @return
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * pathOffsetSupv(self: pyaubo_sdk.MotionControl, arg0: List[float], arg1:
     * List[float], arg2: int) -> int
     *
     * @par Lua函数原型
     * pathOffsetSupv(min: table, max: table, strategy: number) -> number
     */
    int pathOffsetSupv(const std::vector<double> &min,
                       const std::vector<double> &max, int strategy);

    /**
     * 关节偏移使能
     *
     * @return 成功返回0; 失败返回错误码
     * AUBO_BAD_STATE
     * AUBO_BUSY
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * jointOffsetEnable(self: pyaubo_sdk.MotionControl) -> int
     *
     * @par Lua函数原型
     * jointOffsetEnable() -> nil
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.jointOffsetEnable","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int jointOffsetEnable();

    /**
     * 设置关节偏移
     *
     * @param offset 在各关节的位姿偏移
     * @param type
     * @return 成功返回0; 失败返回错误码
     * AUBO_BAD_STATE
     * AUBO_BUSY
     * -AUBO_INVL_ARGUMENT
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * jointOffsetSet(self: pyaubo_sdk.MotionControl, arg0: List[float], arg1:
     * int) -> int
     *
     * @par Lua函数原型
     * jointOffsetSet(offset: table, type: number) -> nil
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.jointOffsetSet","params":[[0.1,0,0,0,0],1],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int jointOffsetSet(const std::vector<double> &offset, int type = 1);

    /**
     * 关节偏移失能
     *
     * @return 成功返回0; 失败返回错误码
     * AUBO_BAD_STATE
     * AUBO_BUSY
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * jointOffsetDisable(self: pyaubo_sdk.MotionControl) -> int
     *
     * @par Lua函数原型
     * jointOffsetDisable() -> nil
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.jointOffsetDisable","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int jointOffsetDisable();

    /**
     * 获取已经入队的指令段(INST)数量，运动指令包括
     * moveJoint/moveLine/moveCircle 等运动指令以及 setPayload 等配置指令
     *
     * 指令一般会在接口宏定义里面用 _INST 指示, 比如 moveJoint
     *   _INST(MotionControl, 5, moveJoint, q, a, v, blend_radius, duration)
     *
     * @return 已经入队的指令段数量
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getQueueSize(self: pyaubo_sdk.MotionControl) -> int
     *
     * @par Lua函数原型
     * getQueueSize() -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.getQueueSize","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int getQueueSize();

    /**
     * 获取已经入队的运动规划插补点数量
     *
     * @return 已经入队的运动规划插补点数量
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getTrajectoryQueueSize(self: pyaubo_sdk.MotionControl) -> int
     *
     * @par Lua函数原型
     * getTrajectoryQueueSize() -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.getTrajectoryQueueSize","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int getTrajectoryQueueSize();

    /**
     * 获取当前正在插补的运动指令段的ID
     *
     * @return 当前正在插补的运动指令段的ID
     * @retval -1 表示轨迹队列为空 \n
     * 像movePathBuffer运动中的buffer或者规划器(moveJoint和moveLine等)里的队列都属于轨迹队列
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getExecId(self: pyaubo_sdk.MotionControl) -> int
     *
     * @par Lua函数原型
     * getExecId() -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.getExecId","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":-1}
     *
     */
    int getExecId();

    /**
     * 获取指定ID的运动指令段的预期执行时间
     *
     * @param id 运动指令段ID
     * @return 返回预期执行时间
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getDuration(self: pyaubo_sdk.MotionControl, arg0: int) -> float
     *
     * @par Lua函数原型
     * getDuration(id: number) -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.getDuration","params":[16],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0.0}
     *
     */
    double getDuration(int id);

    /**
     * 获取指定ID的运动指令段的剩余执行时间
     *
     * @param id 运动指令段ID
     * @return 返回剩余执行时间
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getMotionLeftTime(self: pyaubo_sdk.MotionControl, arg0: int) -> float
     *
     * @par Lua函数原型
     * getMotionLeftTime(id: number) -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.getMotionLeftTime","params":[16],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0.0}
     *
     */
    double getMotionLeftTime(int id);

    /**
     * StopMove is used to stop robot and external axes movements and any
     * belonging process temporarily. If the instruction StartMove is given then
     * the movement and process resumes.
     *
     * This instruction can, for example, be used in a trap routine to stop the
     * robot temporarily when an interrupt occurs.
     *
     * @param quick true: Stops the robot on the path as fast as possible.
     * Without the optional parameter \Quick, the robot stops on the path, but
     * the braking distance is longer (same as for normal Program Stop).
     *
     * @return 成功返回0; 失败返回错误码
     * AUBO_BAD_STATE
     * AUBO_BUSY
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     */
    int stopMove(bool quick, bool all_tasks);

    /**
     * StartMove is used to resume robot, external axes movement and belonging
     * process after the movement has been stopped
     *
     * • by the instruction StopMove.
     * • after execution of StorePath ... RestoPath sequence.
     * • after asynchronously raised movements errors, such as ERR_PATH_STOP or
     * specific process error after handling in the ERROR handler.
     *
     * @return 成功返回0; 失败返回错误码
     * AUBO_BAD_STATE
     * AUBO_BUSY
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.startMove","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int startMove();

    /**
     * storePath
     *
     * @param keep_sync
     * @return 成功返回0; 失败返回错误码
     * AUBO_BAD_STATE
     * AUBO_BUSY
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     */
    int storePath(bool keep_sync);

    /**
     * ClearPath (Clear Path) clears the whole motion path on the current motion
     * path level (base level or StorePath level).
     *
     * @return 成功返回0; 失败返回错误码
     * AUBO_BAD_STATE
     * AUBO_BUSY
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.clearPath","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int clearPath();

    /**
     * restoPath
     *
     * @return 成功返回0; 失败返回错误码
     * AUBO_BAD_STATE
     * AUBO_BUSY
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.restoPath","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int restoPath();

    /**
     * 获取当前运动指令段的执行进度
     *
     * @return 返回执行进度
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getProgress(self: pyaubo_sdk.MotionControl) -> float
     *
     * @par Lua函数原型
     * getProgress() -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.getProgress","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0.0}
     *
     */
    double getProgress();

    /**
     * 当工件安装在另外一台机器人的末端或者外部轴上时，指定其名字和安装位置
     *
     * @note 暂未实现
     *
     * @param module_name 控制模块名字
     * @param mounting_pose 抓取的相对位置，
     * 如果是机器人，则相对于机器人末端中心点（非TCP点）
     * @return 成功返回0; 失败返回错误码
     * AUBO_BAD_STATE
     * AUBO_BUSY
     * -AUBO_INVL_ARGUMENT
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * setWorkObjectHold(self: pyaubo_sdk.MotionControl, arg0: str, arg1:
     * List[float]) -> int
     *
     * @par Lua函数原型
     * setWorkObjectHold(module_name: string, mounting_pose: table) -> nil
     *
     */
    int setWorkObjectHold(const std::string &module_name,
                          const std::vector<double> &mounting_pose);

    /**
     * getWorkObjectHold
     *
     * @note 暂未实现
     *
     * @return
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getWorkObjectHold(self: pyaubo_sdk.MotionControl) -> Tuple[str,
     * List[float]]
     *
     * @par Lua函数原型
     * getWorkObjectHold() -> table
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.getWorkObjectHold","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":["",[]]}
     *
     */
    std::tuple<std::string, std::vector<double>> getWorkObjectHold();

    /**
     * getPauseJointPositions
     *
     * @note 获取暂停点关节位置
     * 常用于运行工程时发生碰撞后继续运动过程中(先通过resumeMoveJoint或resumeMoveLine运动到暂停位置,再恢复工程)
     *
     * @return 暂停关节位置
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getPauseJointPositions(self: pyaubo_sdk.MotionControl) -> List[float]
     *
     * @par Lua函数原型
     * getPauseJointPositions() -> table
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.getPauseJointPositions","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":[8.2321e-13,-0.200999,1.33999,0.334999,1.206,-6.39383e-12]}
     *
     */
    std::vector<double> getPauseJointPositions();

    /**
     * 设置伺服模式
     * 使用 setServoModeSelect 替代
     *
     * @param enable 是否使能
     * @return 成功返回0; 失败返回错误码
     * AUBO_BAD_STATE
     * AUBO_BUSY
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * setServoMode(self: pyaubo_sdk.MotionControl, arg0: bool) -> int
     *
     * @par Lua函数原型
     * setServoMode(enable: boolean) -> nil
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.setServoMode","params":[true],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    ARCS_DEPRECATED int setServoMode(bool enable);

    /**
     * 判断伺服模式是否使能
     * 使用 getServoModeSelect 替代
     *
     * @return 已使能返回true，反之则返回false
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * isServoModeEnabled(self: pyaubo_sdk.MotionControl) -> bool
     *
     * @par Lua函数原型
     * isServoModeEnabled() -> boolean
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.isServoModeEnabled","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":false}
     *
     */
    ARCS_DEPRECATED bool isServoModeEnabled();

    /**
     * 设置伺服运动模式
     *
     * @param mode
     * 0-退出伺服模式
     * 1-规划伺服模式
     * 2-透传模式(直接下发)
     * 3-透传模式(缓存)
     * @return
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.setServoModeSelect","params":[0],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int setServoModeSelect(int mode);

    /**
     * 获取伺服运动模式
     *
     * @return
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.getServoModeSelect","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int getServoModeSelect();

    /**
     * 关节空间伺服
     *
     * 目前可用参数只有 q 和 t;
     * @param q 关节角, 单位 rad,
     * @param a 加速度, 单位 rad/s^2,
     * @param v 速度，单位 rad/s,
     * @param t 运行时间，单位 s \n
     * t 值越大,机器臂运动越慢,反之，运动越快;
     * 该参数最优值为连续调用 servoJoint 接口的间隔时间。
     * @param lookahead_time 前瞻时间，单位 s \n
     * 指定机器臂开始减速前要运动的时长，用前瞻时间来平滑轨迹[0.03, 0.2],
     * 当 lookahead_time 小于一个控制周期时，越小则超调量越大,
     * 该参数最优值为一个控制周期。
     * @param gain 比例增益
     * 跟踪目标位置的比例增益[100, 200],
     * 用于控制运动的顺滑性和精度,
     * 比例增益越大，到达目标位置的时间越长，超调量越小。
     *
     * @return 成功返回0；失败返回错误码，正数为警告，负数为错误。
     * AUBO_BAD_STATE(1): 当前安全模式处于非 Normal、ReducedMode、Recovery 状态;
     * AUBO_BUSY(3): 上一条指令正在执行中;
     * AUBO_QUEUE_FULL(2): 轨迹队列已满;
     * -AUBO_REQUEST_IGNORE(-13): 当前处于非 servo 模式;
     * -AUBO_BAD_STATE(-1): 当前机器臂模式处于非 Running 状态;
     * -AUBO_TIMEOUT(-4): 调用接口超时;
     * -AUBO_INVL_ARGUMENT(-5): 轨迹位置超限或速度超限;
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * servoJoint(self: pyaubo_sdk.MotionControl, arg0: List[float], arg1:
     * float, arg2: float, arg3: float, arg4: float, arg5: float) -> int
     *
     * @par Lua函数原型
     * servoJoint(q: table, a: number, v: number, t: number, lookahead_time:
     * number, gain: number) -> nil
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.servoJoint","params":[[0,0,0,0,0,0],0,0,10,0,0],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":-13}
     *
     */
    int servoJoint(const std::vector<double> &q, double a, double v, double t,
                   double lookahead_time, double gain);

    /**
     * 笛卡尔空间伺服
     *
     * 目前可用参数只有 pose 和 t;
     * @param pose 位姿, 单位 m,
     * @param a 加速度, 单位 m/s^2,
     * @param v 速度，单位 m/s,
     * @param t 运行时间，单位 s \n
     * t 值越大,机器臂运动越慢,反之，运动越快;
     * 该参数最优值为连续调用 servoCartesian 接口的间隔时间。
     * @param lookahead_time 前瞻时间，单位 s \n
     * 指定机器臂开始减速前要运动的时长，用前瞻时间来平滑轨迹[0.03, 0.2],
     * 当 lookahead_time 小于一个控制周期时，越小则超调量越大,
     * 该参数最优值为一个控制周期。
     * @param gain 比例增益
     * 跟踪目标位置的比例增益[100, 200],
     * 用于控制运动的顺滑性和精度,
     * 比例增益越大，到达目标位置的时间越长，超调量越小。
     *
     * @return 成功返回0；失败返回错误码，正数为警告，负数为错误。
     * AUBO_BAD_STATE(1): 当前安全模式处于非 Normal、ReducedMode、Recovery 状态;
     * AUBO_BUSY(3): 上一条指令正在执行中;
     * AUBO_QUEUE_FULL(2): 轨迹队列已满;
     * -AUBO_REQUEST_IGNORE(-13): 当前处于非 servo 模式;
     * -AUBO_BAD_STATE(-1): 当前机器臂模式处于非 Running 状态;
     * -AUBO_TIMEOUT(-4): 调用接口超时;
     * -AUBO_INVL_ARGUMENT(-5): 轨迹位置超限或速度超限;
     * -AUBO_IK_NO_CONVERGE(-23): 逆解计算不收敛，计算出错;
     * -AUBO_IK_OUT_OF_RANGE(-24): 逆解计算超出机器人最大限制;
     * -AUBO_IK_CONFIG_DISMATCH(-25): 逆解输入配置存在错误;
     * -AUBO_IK_JACOBIAN_FAILED(-26): 逆解雅可比矩阵计算失败;
     * -AUBO_IK_NO_SOLU(-27): 目标点存在解析解，但均不满足选解条件;
     * -AUBO_IK_UNKOWN_ERROR(-28): 逆解返回未知类型错误;
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * servoCartesian(self: pyaubo_sdk.MotionControl, arg0: List[float], arg1:
     * float, arg2: float, arg3: float, arg4: float, arg5: float) -> int
     *
     * @par Lua函数原型
     * servoCartesian(pose: table, a: number, v: number, t: number,
     * lookahead_time: number, gain: number) -> nil
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.servoCartesian","params":[[0.58712,-0.15775,0.48703,2.76,0.344,1.432],0,0,10,0,0],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":-13}
     *
     */
    int servoCartesian(const std::vector<double> &pose, double a, double v,
                       double t, double lookahead_time, double gain);

    /**
     * 伺服运动（带外部轴），用于执行离线轨迹、透传用户规划轨迹等
     *
     * @param q
     * @param extq
     * @param t
     * @param smooth_scale
     * @param delay_sacle
     * @return
     */
    int servoJointWithAxes(const std::vector<double> &q,
                           const std::vector<double> &extq, double a, double v,
                           double t, double lookahead_time, double gain);

    int servoJointWithAxisGroup(const std::vector<double> &q, double a,
                                double v, double t, double lookahead_time,
                                double gain, const std::string &group_name,
                                const std::vector<double> &extq);

    /**
     * 伺服运动（带外部轴），用于执行离线轨迹、透传用户规划轨迹等
     * 与 servoJointWithAxes 区别在于接收笛卡尔空间位姿而不是关节角度
     * (由软件内部直接做逆解)
     *
     * @param pose
     * @param extq
     * @param t
     * @param smooth_scale
     * @param delay_sacle
     * @return
     */
    int servoCartesianWithAxes(const std::vector<double> &pose,
                               const std::vector<double> &extq, double a,
                               double v, double t, double lookahead_time,
                               double gain);

    int servoCartesianWithAxisGroup(const std::vector<double> &pose, double a,
                                    double v, double t, double lookahead_time,
                                    double gain, const std::string &group_name,
                                    const std::vector<double> &extq);

    /**
     * 跟踪运动，用于执行离线轨迹、透传用户规划轨迹等
     *
     * @param q
     * @param smooth_scale
     * @param delay_sacle
     * @return
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.trackJoint","params":[[0,0,0,0,0,0],0.01,0.5,1],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int trackJoint(const std::vector<double> &q, double t, double smooth_scale,
                   double delay_sacle);

    /**
     * 跟踪运动，用于执行离线轨迹、透传用户规划轨迹等
     * 与 trackJoint 区别在于接收笛卡尔空间位姿而不是关节角度
     * (由软件内部直接做逆解)
     *
     * @param pose
     * @param t
     * @param smooth_scale
     * @param delay_sacle
     * @return
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.trackCartesian","params":[[0.58712,-0.15775,0.48703,2.76,0.344,1.432],0.01,0.5,1],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int trackCartesian(const std::vector<double> &pose, double t,
                       double smooth_scale, double delay_sacle);

    /**
     * 关节空间跟随
     *
     * @note 暂未实现
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * followJoint(self: pyaubo_sdk.MotionControl, arg0: List[float]) -> int
     *
     * @par Lua函数原型
     * followJoint(q: table) -> nil
     *
     */
    int followJoint(const std::vector<double> &q);

    /**
     * 笛卡尔空间跟随
     *
     * @note 暂未实现
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * followLine(self: pyaubo_sdk.MotionControl, arg0: List[float]) -> int
     *
     * @par Lua函数原型
     * followLine(pose: table) -> nil
     *
     */
    int followLine(const std::vector<double> &pose);

    /**
     * 关节空间速度跟随
     *
     * 当机械臂还没达到目标速度的时候，给一个新的目标速度，机械臂会立刻达到新的目标速度
     *
     * @param qd 目标关节速度, 单位 rad/s
     * @param a 主轴的加速度, 单位 rad/s^2
     * @param t 函数返回所需要的时间, 单位 s \n
     * 如果 t = 0，当达到目标速度的时候，函数将返回；
     * 反之，则经过 t 时间后，函数返回，不管是否达到目标速度。\n
     * 如果没有达到目标速度，会减速到零。
     * 如果达到了目标速度就是按照目标速度匀速运动。
     * @return 成功返回0; 失败返回错误码，正数为警告，负数为错误。
     * AUBO_BAD_STATE(1): 当前安全模式处于非 Normal、ReducedMode、Recovery 状态;
     * AUBO_BUSY(3): 上一条指令正在执行中;
     * -AUBO_BAD_STATE(-1): 当前机器臂模式处于非 Running 状态;
     * -AUBO_TIMEOUT(-4): 调用接口超时;
     * -AUBO_INVL_ARGUMENT(-5): 参数数组qd的长度小于当前机器臂的自由度;
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * speedJoint(self: pyaubo_sdk.MotionControl, arg0: List[float], arg1:
     * float, arg2: float) -> int
     *
     * @par Lua函数原型
     * speedJoint(qd: table, a: number, t: number) -> nil
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.speedJoint","params":[[0.2,0,0,0,0,0],1.5,100],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int speedJoint(const std::vector<double> &qd, double a, double t);

    /**
     * 关节空间速度跟随(机械臂运行工程时发生碰撞,通过此接口移动到安全位置)
     *
     * 当机械臂还没达到目标速度的时候，给一个新的目标速度，机械臂会立刻达到新的目标速度
     *
     * @param qd 目标关节速度, 单位 rad/s
     * @param a 主轴的加速度, 单位 rad/s^2
     * @param t 函数返回所需要的时间, 单位 s
     * 如果 t = 0，当达到目标速度的时候，函数将返回；
     * 反之，则经过 t 时间后，函数返回，不管是否达到目标速度。
     * 如果没有达到目标速度，会减速到零。
     * 如果达到了目标速度就是按照目标速度匀速运动。
     * @return 成功返回0; 失败返回错误码
     * AUBO_BUSY
     * -AUBO_INVL_ARGUMENT
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * resumeSpeedJoint(self: pyaubo_sdk.MotionControl, arg0: List[float], arg1:
     * float, arg2: float) -> int
     *
     *
     * @par Lua函数原型
     * resumeSpeedJoint(q: table, a: number, t: number) -> nil
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.resumeSpeedJoint","params":[[0.2,0,0,0,0,0],1.5,100],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":-1}
     *
     */
    int resumeSpeedJoint(const std::vector<double> &qd, double a, double t);

    /**
     * 笛卡尔空间速度跟随
     *
     * 当机械臂还没达到目标速度的时候，给一个新的目标速度，机械臂会立刻达到新的目标速度
     *
     * @param xd 工具速度, 单位 m/s
     * @param a 工具位置加速度, 单位 m/s^2
     * @param t 函数返回所需要的时间, 单位 s \n
     * 如果 t = 0，当达到目标速度的时候，函数将返回；
     * 反之，则经过 t 时间后，函数返回，不管是否达到目标速度。
     * 如果没有达到目标速度，会减速到零。
     * 如果达到了目标速度就是按照目标速度匀速运动。
     * @return 成功返回0; 失败返回错误码
     * AUBO_BUSY(3): 上一条指令正在执行中;
     * AUBO_BAD_STATE(1): 当前安全模式处于非 Normal、ReducedMode、Recovery 状态;
     * -AUBO_BAD_STATE(-1): 当前机器臂模式处于非 Running 状态;
     * -AUBO_TIMEOUT(-4): 调用接口超时;
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * speedLine(self: pyaubo_sdk.MotionControl, arg0: List[float], arg1: float,
     * arg2: float) -> int
     *
     * @par Lua函数原型
     * speedLine(pose: table, a: number, t: number) -> nil
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.speedLine","params":[[0.25,0,0,0,0,0],1.2,100],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int speedLine(const std::vector<double> &xd, double a, double t);

    /**
     * 笛卡尔空间速度跟随(机械臂运行工程时发生碰撞,通过此接口移动到安全位置)
     *
     * 当机械臂还没达到目标速度的时候，给一个新的目标速度，机械臂会立刻达到新的目标速度
     *
     * @param xd 工具速度, 单位 m/s
     * @param a 工具位置加速度, 单位 m/s^2
     * @param t 函数返回所需要的时间, 单位 s \n
     * 如果 t = 0，当达到目标速度的时候，函数将返回；
     * 反之，则经过 t 时间后，函数返回，不管是否达到目标速度。
     * 如果没有达到目标速度，会减速到零。
     * 如果达到了目标速度就是按照目标速度匀速运动。
     * @return 成功返回0; 失败返回错误码
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * resumeSpeedLine(self: pyaubo_sdk.MotionControl, arg0: List[float], arg1:
     * float, arg2: float) -> int
     *
     * @par Lua函数原型
     * resumeSpeedLine(pose: table, a: number, t: number) -> nil
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.resumeSpeedLine","params":[[0.25,0,0,0,0,0],1.2,100],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":-1}
     *
     */
    int resumeSpeedLine(const std::vector<double> &xd, double a, double t);

    /**
     * 在关节空间做样条插值
     *
     * @param q 关节角度，如果传入参数维度为0，表示样条运动结束
     * @param a 加速度, 单位 rad/s^2,
     * 最大值可通过RobotConfig类中的接口getJointMaxAccelerations()来获取
     * @param v 速度， 单位 rad/s,
     * 最大值可通过RobotConfig类中的接口getJointMaxSpeeds()来获取
     * @param duration
     * @return 成功返回0; 失败返回错误码
     * AUBO_BUSY
     * AUBO_BAD_STATE
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * moveSpline(self: pyaubo_sdk.MotionControl, arg0: List[float], arg1:
     * float, arg2: float, arg3: float) -> int
     *
     * @par Lua函数原型
     * moveSpline(q: table, a: number, v: number, duration: number) -> nil
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.moveSpline","params":[[0,0,0,0,0,0],1,1,0],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int moveSpline(const std::vector<double> &q, double a, double v,
                   double duration);

    /**
     * 添加关节运动
     *
     * @param q 关节角, 单位 rad
     * @param a 加速度, 单位 rad/s^2,
     * 最大值可通过RobotConfig类中的接口getJointMaxAccelerations()来获取
     * @param v 速度， 单位 rad/s,
     * 最大值可通过RobotConfig类中的接口getJointMaxSpeeds()来获取
     * @param blend_radius 交融半径, 单位 m
     * @param duration 运行时间，单位 s \n
     * 通常当给定了速度和加速度，便能够确定轨迹的运行时间。
     * 如果想延长轨迹的运行时间，便要设置 duration 这个参数。
     * duration 可以延长轨迹运动的时间，但是不能缩短轨迹时间。\n
     * 当 duration = 0的时候，
     * 表示不指定运行时间，即没有明确完成运动的时间，将根据速度与加速度计算运行时间。
     * 如果duration不等于0，a 和 v 的值将被忽略。
     * @return 成功返回0；失败返回错误码，正数为警告，负数为错误。
     * AUBO_BAD_STATE(1): 当前安全模式处于非 Normal、ReducedMode、Recovery 状态;
     * AUBO_QUEUE_FULL(2): 规划队列已满;
     * AUBO_BUSY(3): 上一条指令正在执行中;
     * -AUBO_BAD_STATE(-1): 当前机器臂模式处于非 Running 状态;
     * -AUBO_TIMEOUT(-4): 调用接口超时;
     * -AUBO_INVL_ARGUMENT(-5): 参数数组q的长度小于当前机器臂的自由度;
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * moveJoint(self: pyaubo_sdk.MotionControl, arg0: List[float], arg1: float,
     * arg2: float, arg3: float, arg4: float) -> int
     *
     * @par Lua函数原型
     * moveJoint(q: table, a: number, v: number, blend_radius: number, duration:
     * number) -> nil
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.moveJoint","params":[[-2.05177,
     * -0.400292, 1.19625, 0.0285152, 1.57033, -2.28774],0.3,0.3,0,0],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int moveJoint(const std::vector<double> &q, double a, double v,
                  double blend_radius, double duration);

    /**
     * 机器人与外部轴同步运动
     *
     * @param group_name
     * @param q
     * @param a
     * @param v
     * @param blend_radius
     * @param duration
     * @return
     */
    int moveJointWithAxisGroup(const std::vector<double> &q, double a, double v,
                               double blend_radius, double duration,
                               const std::string &group_name,
                               const std::vector<double> &extq);

    /**
     * 通过关节运动移动到暂停点的位置
     *
     * @param q 关节角, 单位 rad
     * @param a 加速度, 单位 rad/s^2
     * @param v 速度， 单位 rad/s
     * @param duration 运行时间，单位 s \n
     * 通常当给定了速度和加速度，便能够确定轨迹的运行时间。
     * 如果想延长轨迹的运行时间，便要设置 duration 这个参数。
     * duration 可以延长轨迹运动的时间，但是不能缩短轨迹时间。\n
     * 当 duration = 0的时候，
     * 表示不指定运行时间，即没有明确完成运动的时间，将根据速度与加速度计算运行时间。
     * 如果duration不等于0，a 和 v 的值将被忽略。
     * @return 成功返回0；失败返回错误码
     * -AUBO_INVL_ARGUMENT
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * resumeMoveJoint(self: pyaubo_sdk.MotionControl, arg0: List[float], arg1:
     * float, arg2: float, arg3: float) -> int
     *
     * @par Lua函数原型
     * resumeMoveJoint(q: table, a: number, v: number, duration: number) -> nil
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.resumeMoveJoint","params":[[-2.05177,
     * -0.400292, 1.19625, 0.0285152, 1.57033, -2.28774],0.3,0.3,0],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":-1}
     */
    int resumeMoveJoint(const std::vector<double> &q, double a, double v,
                        double duration);

    /**
     * 添加直线运动
     *
     * @param pose 目标位姿
     * @param a 加速度(如果位置变化小于1mm,姿态变化大于 1e-4
     * rad,此加速度会被作为角加速度,单位 rad/s^2.否则为线加速度,单位 m/s^2)
     * 最大值可通过RobotConfig类中的接口getTcpMaxAccelerations()来获取
     * @param v 速度(如果位置变化小于1mm,姿态变化大于 1e-4
     * rad,此速度会被作为角速度,单位 rad/s.否则为线速度,单位 m/s)
     * 最大值可通过RobotConfig类中的接口getTcpMaxSpeeds()来获取
     * @param blend_radius 交融半径，单位 m，值介于0.001和1之间
     * @param duration 运行时间，单位 s \n
     * 通常当给定了速度和加速度，便能够确定轨迹的运行时间。
     * 如果想延长轨迹的运行时间，便要设置 duration 这个参数。
     * duration 可以延长轨迹运动的时间，但是不能缩短轨迹时间。\n
     * 当 duration = 0的时候，
     * 表示不指定运行时间，即没有明确完成运动的时间，将根据速度与加速度计算运行时间。
     * 如果duration不等于0，a 和 v 的值将被忽略。
     * @return 成功返回0；失败返回错误码，正数为警告，负数为错误。
     * AUBO_BAD_STATE(1): 当前安全模式处于非 Normal、ReducedMode、Recovery 状态;
     * AUBO_QUEUE_FULL(2): 轨迹队列已满;
     * AUBO_BUSY(3): 上一条指令正在执行中;
     * -AUBO_BAD_STATE(-1): 当前机器臂模式处于非 Running 状态;
     * -AUBO_TIMEOUT(-4): 调用接口超时;
     * -AUBO_INVL_ARGUMENT(-5): 参数数组 pose 的长度小于当前机器臂的自由度;
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * moveLine(self: pyaubo_sdk.MotionControl, arg0: List[float], arg1: float,
     * arg2: float, arg3: float, arg4: float) -> int
     *
     * @par Lua函数原型
     * moveLine(pose: table, a: number, v: number, blend_radius: number,
     * duration: number) -> nil
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.moveLine","params":[[0.58815,0.0532,0.62391,2.46,0.479,1.619],1.2,0.25,0,0],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int moveLine(const std::vector<double> &pose, double a, double v,
                 double blend_radius, double duration);

    /**
     * 直线运动与外部轴同步运动
     *
     * @param group_name
     * @param pose
     * @param a
     * @param v
     * @param blend_radius
     * @param duration
     * @return
     */
    int moveLineWithAxisGroup(const std::vector<double> &pose, double a,
                              double v, double blend_radius, double duration,
                              const std::string &group_name,
                              const std::vector<double> &extq);

    /**
     * 添加工艺运动
     *
     * @param pose
     * @param a
     * @param v
     * @param blend_radius
     * @return 成功返回0；失败返回错误码
     * AUBO_BAD_STATE
     * AUBO_BUSY
     * -AUBO_INVL_ARGUMENT
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.moveProcess","params":[[0.58815,0.0532,0.62391,2.46,0.479,1.619],1.2,0.25,0],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int moveProcess(const std::vector<double> &pose, double a, double v,
                    double blend_radius);

    /**
     * 通过直线运动移动到暂停点的位置
     *
     * @param pose 目标位姿
     * @param a 加速度, 单位 m/s^2
     * @param v 速度, 单位 m/s
     * @param duration 运行时间，单位 s \n
     * 通常当给定了速度和加速度，便能够确定轨迹的运行时间。
     * 如果想延长轨迹的运行时间，便要设置 duration 这个参数。
     * duration 可以延长轨迹运动的时间，但是不能缩短轨迹时间。\n
     * 当 duration = 0的时候，
     * 表示不指定运行时间，即没有明确完成运动的时间，将根据速度与加速度计算运行时间。
     * 如果duration不等于0，a 和 v 的值将被忽略。
     * @return 成功返回0；失败返回错误码
     * -AUBO_INVL_ARGUMENT
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * resumeMoveLine(self: pyaubo_sdk.MotionControl, arg0: List[float], arg1:
     * float, arg2: float, arg3: float) -> int
     *
     * @par Lua函数原型
     * resumeMoveLine(pose: table, a: number, v: number,duration: number) -> nil
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.resumeMoveLine","params":[[0.58815,0.0532,0.62391,2.46,0.479,1.619],1.2,0.25,0],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int resumeMoveLine(const std::vector<double> &pose, double a, double v,
                       double duration);

    /**
     * 添加圆弧运动
     *
     * @todo 可以由多段圆弧组成圆周运动
     *
     * @param via_pose 圆弧运动途中点的位姿
     * @param end_pose 圆弧运动结束点的位姿
     * @param a 加速度(如果via_pose与上一个路点位置变化小于1mm,姿态变化大于 1e-4
     * rad, 此加速度会被作为角加速度,单位 rad/s^2.否则为线加速度,单位 m/s^2)
     * @param v 速度(如果via_pose与上一个路点位置变化小于1mm, 姿态变化大于 1e-4
     * rad, 此速度会被作为角速度, 单位 rad / s.否则为线速度, 单位 m / s)
     * @param blend_radius 交融半径，单位: m
     * @param duration 运行时间，单位: s \n
     * 通常当给定了速度和加速度，便能够确定轨迹的运行时间。
     * 如果想延长轨迹的运行时间，便要设置 duration 这个参数。
     * duration 可以延长轨迹运动的时间，但是不能缩短轨迹时间。\n
     * 当 duration = 0 的时候，
     * 表示不指定运行时间，即没有明确完成运动的时间，将根据速度与加速度计算运行时间。
     * 如果duration不等于0，a 和 v 的值将被忽略。
     * @return 成功返回0；失败返回错误码
     * AUBO_BAD_STATE
     * AUBO_BUSY
     * -AUBO_INVL_ARGUMENT
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * moveCircle(self: pyaubo_sdk.MotionControl, arg0: List[float], arg1:
     * List[float], arg2: float, arg3: float, arg4: float, arg5: float) -> int
     *
     * @par Lua函数原型
     * moveCircle(via_pose: table, end_pose: table, a: number, v: number,
     * blend_radius: number, duration: number) -> nil
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.moveCircle","params":[[0.440164,-0.00249391,0.398658,2.45651,0,1.5708],[0.440164,0.166256,0.297599,2.45651,0,1.5708],1.2,0.25,0,0],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int moveCircle(const std::vector<double> &via_pose,
                   const std::vector<double> &end_pose, double a, double v,
                   double blend_radius, double duration);

    /**
     * moveCircle 与外部轴同步运动
     *
     * @param group_name
     * @param via_pose
     * @param end_pose
     * @param a
     * @param v
     * @param blend_radius
     * @param duration
     * @return
     */
    int moveCircleWithAxisGroup(const std::vector<double> &via_pose,
                                const std::vector<double> &end_pose, double a,
                                double v, double blend_radius, double duration,
                                const std::string &group_name,
                                const std::vector<double> &extq);

    /**
     * 设置圆弧路径模式
     *
     * @param mode 模式 \n
     * 0:工具姿态相对于圆弧路径点坐标系保持不变 \n
     * 1:姿态线性变化,绕着空间定轴转动,从起始点姿态变化到目标点姿态 \n
     * 2:从起点姿态开始经过中间点姿态,变化到目标点姿态
     * @return 成功返回0；失败返回错误码
     * AUBO_BAD_STATE
     * AUBO_BUSY
     * -AUBO_INVL_ARGUMENT
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * setCirclePathMode(self: pyaubo_sdk.MotionControl, arg0: int) -> int
     *
     * @par Lua函数原型
     * setCirclePathMode(mode: number) -> nil
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.setCirclePathMode","params":[0],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int setCirclePathMode(int mode);

    /**
     * 高级圆弧或者圆周运动
     *
     * @param param 圆周运动参数
     * @return 成功返回0；失败返回错误码
     * AUBO_BAD_STATE
     * AUBO_BUSY
     * -AUBO_INVL_ARGUMENT
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * moveCircle2(self: pyaubo_sdk.MotionControl, arg0:
     * arcs::common_interface::CircleParameters) -> int
     *
     * @par Lua函数原型
     * moveCircle2(param: table) -> nil
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.moveCircle2","params":[{"pose_via":[0.440164,-0.00249391,0.398658,2.45651,0,1.570],
     * "pose_to":[0.440164,0.166256,0.297599,2.45651,0,1.5708],"a":1.2,"v":0.25,"blend_radius":0,"duration":0,"helix":0,
     * "spiral":0,"direction":0,"loop_times":0}],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int moveCircle2(const CircleParameters &param);

    /**
     * 新建一个路径点缓存
     *
     * @param name 指定路径的名字
     * @param type 路径的类型 \n
     *   1: toppra 时间最优路径规划 \n
     *   2: cubic_spline(录制的轨迹) \n
     *   3: 关节B样条插值，最少三个点
     * @param size 缓存区大小
     * @return 新建成功返回 AUBO_OK(0)
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * pathBufferAlloc(self: pyaubo_sdk.MotionControl, arg0: str, arg1: int,
     * arg2: int) -> int
     *
     * @par Lua函数原型
     * pathBufferAlloc(name: string, type: number, size: number) -> nil
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.pathBufferAlloc","params":["rec",2,3],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int pathBufferAlloc(const std::string &name, int type, int size);

    /**
     * 向路径缓存添加路点
     *
     * @param name 路径缓存的名字
     * @param waypoints 路点
     * @return 成功返回0；失败返回错误码
     * -AUBO_INVL_ARGUMENT
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * pathBufferAppend(self: pyaubo_sdk.MotionControl, arg0: str, arg1:
     * List[List[float]]) -> int
     *
     * @par Lua函数原型
     * pathBufferAppend(name: string, waypoints: table) -> nil
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.pathBufferAppend","params":["rec",[[-0.000000,0.000009,-0.000001,0.000002,0.000002,0.000000],[-0.000001,0.000010,-0.000002,0.000000,0.000003,0.000002]]],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int pathBufferAppend(const std::string &name,
                         const std::vector<std::vector<double>> &waypoints);

    /**
     * 计算、优化等耗时操作，传入的参数相同时不会重新计算
     *
     * @param name 通过pathBufferAlloc新建的路径点缓存的名字
     * @param a 关节加速度限制,需要和机器人自由度大小相同,单位 m/s^2
     * @param v 关节速度限制,需要和机器人自由度大小相同,单位 m/s
     * @param t 时间 \n
     * pathBufferAlloc 这个接口分配内存的时候要指定类型，
     * 根据pathBufferAlloc这个接口的类型:\n
     * 类型为1时,表示运动持续时间\n
     * 类型为2时,表示采样时间间隔\n
     * 类型为3时,t参数设置为0
     * @return 成功返回0；失败返回错误码
     * -AUBO_INVL_ARGUMENT
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * pathBufferEval(self: pyaubo_sdk.MotionControl, arg0: str, arg1:
     * List[float], arg2: List[float], arg3: float) -> int
     *
     * @par Lua函数原型
     * pathBufferEval(name: string, a: table, v: table, t: number) -> nil
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.pathBufferEval","params":["rec",[1,1,1,1,1,1],[1,1,1,1,1,1],0.02],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int pathBufferEval(const std::string &name, const std::vector<double> &a,
                       const std::vector<double> &v, double t);

    /**
     * 指定名字的buffer是否有效
     *
     * buffer需要满足三个条件有效: \n
     * 1、buffer存在，已经分配过内存 \n
     * 2、传进buffer的点要大于等于buffer的大小 \n
     * 3、要执行一次pathBufferEval
     *
     * @param name buffer的名字
     * @return 有效返回true; 无效返回false
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * pathBufferValid(self: pyaubo_sdk.MotionControl, arg0: str) -> bool
     *
     * @par Lua函数原型
     * pathBufferValid(name: string) -> boolean
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.pathBufferValid","params":["rec"],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":false}
     *
     */
    bool pathBufferValid(const std::string &name);

    /**
     * 释放路径缓存
     *
     * @param name 缓存路径的名字
     * @return 成功返回0；失败返回错误码
     * -AUBO_INVL_ARGUMENT
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * pathBufferFree(self: pyaubo_sdk.MotionControl, arg0: str) -> int
     *
     * @par Lua函数原型
     * pathBufferFree(name: string) -> nil
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.pathBufferFree","params":["rec"],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":-5}
     *
     */
    int pathBufferFree(const std::string &name);

    /**
     * 关节空间路径滤波器
     *
     * @brief pathBufferFilter
     *
     * @param name 缓存路径的名称
     * @param order　滤波器阶数(一般取2)
     * @param fd　截止频率，越小越光滑，但是延迟会大(一般取3-20)
     * @param fs　离散数据的采样频率(一般取20-500)
     *
     * @return 成功返回0
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * pathBufferFree(self: pyaubo_sdk.MotionControl, arg0: str, arg1: int,
     * arg2: float, arg3: float) -> int
     *
     * @par Lua函数原型
     * pathBufferFree(name: string, order: number, fd: number, fs:number) -> nil
     */
    int pathBufferFilter(const std::string &name, int order, double fd,
                         double fs);

    /**
     * 列出所有缓存路径的名字
     *
     * @return 返回所有缓存路径的名字
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * pathBufferList(self: pyaubo_sdk.MotionControl) -> List[str]
     *
     * @par Lua函数原型
     * pathBufferList() -> table
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.pathBufferList","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":[]}
     *
     */
    std::vector<std::string> pathBufferList();

    /**
     * 执行缓存的路径
     *
     * @param name 缓存路径的名字
     * @return 成功返回0；失败返回错误码
     * -AUBO_INVL_ARGUMENT
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * movePathBuffer(self: pyaubo_sdk.MotionControl, arg0: str) -> int
     *
     * @par Lua函数原型
     * movePathBuffer(name: string) -> nil
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.movePathBuffer","params":["rec"],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int movePathBuffer(const std::string &name);

    /**
     * 相贯线接口
     *
     * @param pose由三个示教位姿组成(首先需要运动到起始点,起始点的要求:过主圆柱
     *            柱体轴心且与子圆柱体轴线平行的平面与子圆柱体在底部的交点)
     * p1:过子圆柱体轴线且与大圆柱体轴线平行的平面,与小圆柱体在左侧顶部的交点
     * p2:过子圆柱体轴线且与大圆柱体轴线平行的平面,与大圆柱体在左侧底部的交点
     * p3:过子圆柱体轴线且与大圆柱体轴线平行的平面,与大圆柱体在右侧底部的交点
     * @param v:速度
     * @param a:加速度
     * @param main_pipe_radius: 主圆柱体半径
     * @param sub_pipe_radius:  子圆柱体半径
     * @param normal_distance:  两圆柱体轴线距离
     * @param normal_alpha:     两圆柱体轴线的夹角
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
     * moveIntersection(self: pyaubo_sdk.MotionControl, arg0: List[float], arg1:
     float, arg2: float, arg3: float, arg4: float, arg5: float, arg6: float) ->
     int
     *
     * @par Lua函数原型
     * moveIntersection(poses: table, a: number, v: number,
     * main_pipe_radius:number, sub_pipe_radius: number,normal_distance:
     * number,normal_alpha: number,) -> nil
     */
    int moveIntersection(const std::vector<std::vector<double>> &poses,
                         double a, double v, double main_pipe_radius,
                         double sub_pipe_radius, double normal_distance,
                         double normal_alpha);
    /**
     * 关节空间停止运动
     *
     * @param acc 关节加速度，单位: rad/s^2
     * @return 成功返回0；失败返回错误码
     * AUBO_BUSY
     * AUBO_BAD_STATE
     * -AUBO_INVL_ARGUMENT
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * stopJoint(self: pyaubo_sdk.MotionControl, arg0: float) -> int
     *
     * @par Lua函数原型
     * stopJoint(acc: number) -> nil
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.stopJoint","params":[31],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int stopJoint(double acc);

    /**
     * 关节空间停止运动(机械臂运行工程时发生碰撞,通过resumeSpeedJoint接口移动到安全位置后需要停止时调用此接口)
     *
     * @param acc 关节加速度，单位: rad/s^2
     * @return 成功返回0；失败返回错误码
     * AUBO_BUSY
     * AUBO_BAD_STATE
     * -AUBO_INVL_ARGUMENT
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * resumeStopJoint(self: pyaubo_sdk.MotionControl, arg0: float) -> int
     *
     * @par Lua函数原型
     * resumeStopJoint(acc: number) -> nil
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.resumeStopJoint","params":[31],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":-1}
     *
     */
    int resumeStopJoint(double acc);

    /**
     * 笛卡尔空间停止运动
     *
     * @param acc 工具加速度, 单位: m/s^2
     * @param acc_rot
     * @return 成功返回0；失败返回错误码
     * AUBO_BUSY
     * AUBO_BAD_STATE
     * -AUBO_INVL_ARGUMENT
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * stopLine(self: pyaubo_sdk.MotionControl, arg0: float, arg1: float) -> int
     *
     * @par Lua函数原型
     * stopLine(acc: number, acc_rot: number) -> nil
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.stopLine","params":[10,10],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int stopLine(double acc, double acc_rot);

    /**
     * 笛卡尔空间停止运动(机械臂运行工程时发生碰撞,通过resumeSpeedLine接口移动到安全位置后需要停止时调用此接口)
     *
     * @param acc 位置加速度, 单位: m/s^2
     * @param acc_rot 姿态加速度，单位: rad/s^2
     * @return 成功返回0；失败返回错误码
     * AUBO_BUSY
     * AUBO_BAD_STATE
     * -AUBO_INVL_ARGUMENT
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * resumeStopLine(self: pyaubo_sdk.MotionControl, arg0: float, arg1: float)
     * -> int
     *
     * @par Lua函数原型
     * resumeStopLine(acc: number, acc_rot: number) -> nil
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.resumeStopLine","params":[10,10],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":-1}
     *
     */
    int resumeStopLine(double acc, double acc_rot);

    /**
     * 开始摆动: weaveStart 和 weaveEnd 的 moveLine/moveCircle 将根据 params
     * 进行摆动
     *
     * @param params Json字符串用于定义摆动参数
     * {
     *   "type": <string>, // "SINE" "SPIRAL" "TRIANGLE" "TRAPEZOIDAL"
     *   "step": <num>,
     *   "amplitude": {<num>,<num>},
     *   "hold_distance": {<num>,<num>},
     *   "angle": <num>
     *   "direction": <num>
     * }
     * @return 成功返回0；失败返回错误码
     * AUBO_BUSY
     * AUBO_BAD_STATE
     * -AUBO_INVL_ARGUMENT
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     */
    int weaveStart(const std::string &params);

    /**
     * 结束摆动
     *
     * @return 成功返回0；失败返回错误码
     * AUBO_BUSY
     * AUBO_BAD_STATE
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.weaveEnd","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int weaveEnd();

    /**
     * 设置未来路径上点的采样时间间隔
     *
     * @param sample_time 采样时间间隔 单位: m/s^2
     * @return 成功返回0；失败返回错误码
     * AUBO_BUSY
     * AUBO_BAD_STATE
     * -AUBO_INVL_ARGUMENT
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     */
    int setFuturePointSamplePeriod(double sample_time);

    /**
     * 获取未来路径上的轨迹点
     *
     * @return 路点(100ms * 10)
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.getFuturePathPointsJoint","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":[]}
     *
     */
    std::vector<std::vector<double>> getFuturePathPointsJoint();

    /**
     * 圆形传送带跟随
     *
     * @note 暂未实现
     *
     * @param encoder_id
     * 0-集成传感器
     *
     * @param center
     * @param tick_per_revo
     * @param rotate_tool
     * @return
     *
     * @throws arcs::common_interface::AuboException
     *
     */
    int conveyorTrackCircle(int encoder_id, const std::vector<double> &center,
                            int tick_per_revo, bool rotate_tool);

    /**
     * 线性传送带跟随
     *
     * @note 暂未实现
     *
     * @param encoder_id 预留
     * @param direction
     * @param tick_per_meter
     * @return
     *
     * @throws arcs::common_interface::AuboException
     *
     */
    int conveyorTrackLine(int encoder_id, const std::vector<double> &direction,
                          int tick_per_meter);

    /**
     * 终止传送带跟随
     *
     * @note 暂未实现
     *
     * @param a
     * @return
     *
     * @throws arcs::common_interface::AuboException
     *
     */
    int conveyorTrackStop(double a);

    /**
     * 螺旋线运动
     *
     * @param param 封装的参数
     * @param blend_radius
     * @param v
     * @param a
     * @param t
     * @return 成功返回0；失败返回错误码
     * AUBO_BUSY
     * AUBO_BAD_STATE
     * -AUBO_INVL_ARGUMENT
     * -AUBO_BAD_STATE
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.moveSpiral",
     * "params":[{"spiral":0.005,"helix":0.005,"angle":18.84,"plane":0,"frame":[0,0,0,0,0,0]},0,0.25,1.2,0],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int moveSpiral(const SpiralParameters &param, double blend_radius, double v,
                   double a, double t);

    /**
     * 获取前瞻段数
     *
     * @return 返回前瞻段数
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * getLookAheadSize(self: pyaubo_sdk.MotionControl) -> int
     *
     * @par Lua函数原型
     * getLookAheadSize() -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.getLookAheadSize","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":1}
     *
     */
    int getLookAheadSize();

    /**
     * 设置前瞻段数
     * 1.对于有较高速度平稳性要求的任务,如数控加工,涂胶,焊接等匀速需求,较长的前瞻段数可以提供更优的速度规划,产生的运动会更加平滑;
     * 2.对于快速响应的抓取类任务,更倾向于较短的前瞻段数,以提高反应速度,但可能因为进给的路径不够及时导致速度波动很大.
     *
     * @return 成功返回0
     *
     * @throws arcs::common_interface::AuboException
     *
     * @par Python函数原型
     * setLookAheadSize(self: pyaubo_sdk.MotionControl, arg0: int) -> int
     *
     * @par Lua函数原型
     * setLookAheadSize(eqradius: number) -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"rob1.MotionControl.setLookAheadSize","params":[1],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int setLookAheadSize(int size);

protected:
    void *d_;
};
using MotionControlPtr = std::shared_ptr<MotionControl>;
} // namespace common_interface
} // namespace arcs

#endif // AUBO_SDK_MOTION_CONTROL_INTERFACE_H
