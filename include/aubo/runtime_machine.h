/** @file  runtime_machine.h
 *  @brief 脚本解释器运行时接口，
 *  可以实现脚本解释器的暂停、脚本解释器的设置/取消断点
 */
#ifndef AUBO_SDK_RUNTIME_MACHINE_INTERFACE_H
#define AUBO_SDK_RUNTIME_MACHINE_INTERFACE_H

#include <memory>
#include <aubo/global_config.h>
#include <aubo/type_def.h>

namespace arcs {
namespace common_interface {

/**
 * The RuntimeMachine class
 */
class ARCS_ABI_EXPORT RuntimeMachine
{
public:
    RuntimeMachine();
    virtual ~RuntimeMachine();

    /**
     * 返回 task_id
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"RuntimeMachine.newTask","params":[false],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":26}
     *
     */
    int newTask(bool daemon = false);

    /**
     * 删除 task，会终止正在执行的运动
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"RuntimeMachine.deleteTask","params":[26],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int deleteTask(int tid);

    /**
     * 等待 task 自然结束
     *
     * @param tid
     * @return
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"RuntimeMachine.detachTask","params":[26],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int detachTask(int tid);

    /**
     * 判断任务是否存活
     *
     * @param tid
     * @return
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"RuntimeMachine.isTaskAlive","params":[26],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":true}
     *
     */
    bool isTaskAlive(int tid);

    /**
     * 切换当前线程，切换之后接下来的指令将被插入切换后的线程中
     *
     * @param tid
     * @return
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"RuntimeMachine.switchTask","params":[26],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int switchTask(int tid);

    /**
     * 标记记下来的指令的行号和注释
     *
     * @param lineno
     * @param comment
     * @return
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"RuntimeMachine.setLabel","params":[5,"moveJoint"],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int setLabel(int lineno, const std::string &comment);

    /**
     * 向aubo_control日志中添加注释
     * 使用 setLabel 替换
     *
     * @param tid 指令的线程ID
     * @param lineno 行号
     * @param comment 注释
     * @return
     *
     * @par Python函数原型
     * setPlanContext(self: pyaubo_sdk.RuntimeMachine, arg0: int, arg1: int,
     * arg2: str) -> int
     *
     * @par Lua函数原型
     * setPlanContext(tid: number, lineno: number, comment: string) -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"RuntimeMachine.setPlanContext","params":[26,3,"moveJoint"],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    ARCS_DEPRECATED int setPlanContext(int tid, int lineno,
                                       const std::string &comment);

    /**
     * 空操作
     *
     * @return
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"RuntimeMachine.nop","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int nop();

    /**
     * 获取耗时的接口(INST)执行状态, 如 setPersistentParameters
     *
     * @return 指令名字, 执行状态
     * 执行状态: EXECUTING/FINISHED
     *
     * @par Python函数原型
     * getExecutionStatus(self: pyaubo_sdk.RuntimeMachine) -> Tuple[str, str,
     * int]
     *
     * @par Lua函数原型
     * getExecutionStatus() -> string, string, number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"RuntimeMachine.getExecutionStatus","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":["confirmSafetyParameters","FINISHED"]}
     *
     */
    std::tuple<std::string, std::string> getExecutionStatus();
    std::tuple<std::string, std::string, int> getExecutionStatus1();

    /**
     * 跳转到指定行号
     *
     * @param lineno
     * @return
     *
     * @par Python函数原型
     * gotoLine(self: pyaubo_sdk.RuntimeMachine, arg0: int) -> int
     *
     * @par Lua函数原型
     * gotoLine(lineno: number) -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"RuntimeMachine.gotoLine","params":[10],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int gotoLine(int lineno);

    /**
     * 获取当前运行上下文
     *
     * @param tid 任务编号
     * 如果指定(不是-1)，返回对应任务的运行上下文；如果不指定(是-1)，返回正在运行的线程的运行上下文
     *
     * @return
     *
     * @par Python函数原型
     * getPlanContext(self: pyaubo_sdk.RuntimeMachine) -> Tuple[int, int, str]
     *
     * @par Lua函数原型
     * getPlanContext() -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"RuntimeMachine.getPlanContext","params":[-1],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":[-1,0,""]}
     *
     */
    std::tuple<int, int, std::string> getPlanContext(int tid = -1);

    /**
     * 获取提前运行规划器的上下文信息
     *
     * @param tid 任务编号
     * 如果指定(不是-1)，返回对应任务运行规划器的上下文信息；如果不指定(是-1)，返回正在运行的线程运行规划器的上下文信息
     *
     * @return
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"RuntimeMachine.getAdvancePlanContext","params":[-1],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":[-1,-1,""]}
     *
     */
    std::tuple<int, int, std::string> getAdvancePlanContext(int tid = -1);

    /**
     * 获取AdvanceRun的程序指针
     *
     * @return
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"RuntimeMachine.getAdvancePtr","params":[-1],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":-1}
     *
     */
    int getAdvancePtr(int tid = -1);

    /**
     * 获取机器人运动的程序指针
     *
     * @param tid 任务编号
     * 如果指定(不是-1)，返回对应任务的程序指针；如果不指定(是-1)，返回正在运行线程的程序指针
     *
     * @return
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"RuntimeMachine.getMainPtr","params":[-1],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":-1}
     *
     */
    int getMainPtr(int tid = -1);

    /**
     * 获取最近解释过的指令指针
     *
     * @param tid
     * @return
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"RuntimeMachine.getInterpPtr","params":[26],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":-1}
     *
     */
    int getInterpPtr(int tid);

    /**
     * 加载本地工程文件
     * Lua 脚本，只需要给出文件名字，不需要后缀，需要从 ${ARCS_WS}/program
     * 目录中查找
     *
     * @param program
     * @return
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"RuntimeMachine.loadProgram","params":["demo"],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int loadProgram(const std::string &program);

    /**
     * 运行已经加载的工程文件
     *
     * @return
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"RuntimeMachine.runProgram","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int runProgram();

    /**
     * 开始运行时
     *
     * @return
     *
     * @par Python函数原型
     * start(self: pyaubo_sdk.RuntimeMachine) -> int
     *
     * @par Lua函数原型
     * start() -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"RuntimeMachine.start","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int start();

    /**
     * 停止运行时即脚本运行，无法停止运行时状态为 Stopped 时的机器人运动
     *
     * 如果考虑停止机器人所有运动，可以调用 RuntimeMachine::abort 接口
     *
     * @return
     *
     * @par Python函数原型
     * stop(self: pyaubo_sdk.RuntimeMachine) -> int
     *
     * @par Lua函数原型
     * stop() -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"RuntimeMachine.stop","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int stop();

    /**
     * 终止机器人运行.
     *
     * 如果只是考虑停止运行时，可以调用 RuntimeMachine::stop 接口
     *
     * 如果脚本运行时处于 Running 状态，则终止运行时；如果运行时处于 Stopped
     * 且机器人正在移动，则停止机器人移动；如果此时力控开启了，则机器人停止力控
     *
     * @return
     *
     * @par Python函数原型
     * abort(self: pyaubo_sdk.RuntimeMachine) -> int
     *
     * @par Lua函数原型
     * abort() -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"RuntimeMachine.abort","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int abort();

    /**
     * 暂停解释器
     *
     * @return
     *
     * @par Python函数原型
     * pause(self: pyaubo_sdk.RuntimeMachine) -> int
     *
     * @par Lua函数原型
     * pause() -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"RuntimeMachine.pause","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int pause();

    /**
     * 单步运行
     *
     * @return
     *
     * @par Python函数原型
     * step(self: pyaubo_sdk.RuntimeMachine) -> int
     *
     * @par Lua函数原型
     * step() -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"RuntimeMachine.step","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int step();

    /**
     * 恢复解释器
     *
     * @return
     *
     * @par Python函数原型
     * resume(self: pyaubo_sdk.RuntimeMachine) -> int
     *
     * @par Lua函数原型
     * resume() -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"RuntimeMachine.resume","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int resume();

    /**
     * 恢复解释器之前等待恢复前之前的序列完成
     *
     * @param wait
     * @return
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"RuntimeMachine.setResumeWait","params":[true],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int setResumeWait(bool wait);

    /**
     * 获取规划器的状态
     *
     * @return
     *
     * @par Python函数原型
     * getStatus(self: pyaubo_sdk.RuntimeMachine) ->
     * arcs::common_interface::RuntimeState
     *
     * @par Lua函数原型
     * getStatus() -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"RuntimeMachine.getStatus","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":"Running"}
     *
     */
    ARCS_DEPRECATED RuntimeState getStatus();
    RuntimeState getRuntimeState();

    /**
     * 设置断点
     *
     * @param lineno
     * @return
     *
     * @par Python函数原型
     * setBreakPoint(self: pyaubo_sdk.RuntimeMachine, arg0: int) -> int
     *
     * @par Lua函数原型
     * setBreakPoint(lineno: number) -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"RuntimeMachine.setBreakPoint","params":[15],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int setBreakPoint(int lineno);

    /**
     * 移除断点
     *
     * @param lineno
     * @return
     *
     * @par Python函数原型
     * removeBreakPoint(self: pyaubo_sdk.RuntimeMachine, arg0: int) -> int
     *
     * @par Lua函数原型
     * removeBreakPoint(lineno: number) -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"RuntimeMachine.removeBreakPoint","params":[15],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int removeBreakPoint(int lineno);

    /**
     * 清除所有断点
     *
     * @return
     *
     * @par Python函数原型
     * clearBreakPoints(self: pyaubo_sdk.RuntimeMachine) -> int
     *
     * @par Lua函数原型
     * clearBreakPoints() -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"RuntimeMachine.clearBreakPoints","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int clearBreakPoints();

    /**
     * 定时器开始
     *
     * @param name
     * @return
     *
     * @par Python函数原型
     * timerStart(self: pyaubo_sdk.RuntimeMachine, arg0: str) -> int
     *
     * @par Lua函数原型
     * timerStart(name: string) -> nil
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"RuntimeMachine.timerStart","params":["timer"],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int timerStart(const std::string &name);

    /**
     * 定时器结束
     *
     * @param name
     * @return
     *
     * @par Python函数原型
     * timerStop(self: pyaubo_sdk.RuntimeMachine, arg0: str) -> int
     *
     * @par Lua函数原型
     * timerStop(name: string) -> nil
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"RuntimeMachine.timerStop","params":["timer"],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int timerStop(const std::string &name);

    /**
     * 定时器重置
     *
     * @param name
     * @return
     *
     * @par Python函数原型
     * timerReset(self: pyaubo_sdk.RuntimeMachine, arg0: str) -> int
     *
     * @par Lua函数原型
     * timerReset(name: string) -> nil
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"RuntimeMachine.timerReset","params":["timer"],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int timerReset(const std::string &name);

    /**
     * 定时器删除
     *
     * @param name
     * @return
     *
     * @par Python函数原型
     * timerDelete(self: pyaubo_sdk.RuntimeMachine, arg0: str) -> int
     *
     * @par Lua函数原型
     * timerDelete(name: string) -> nil
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"RuntimeMachine.timerDelete","params":["timer"],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int timerDelete(const std::string &name);

    /**
     * 获取定时器数值
     *
     * @param name
     * @return
     *
     * @par Python函数原型
     * getTimer(self: pyaubo_sdk.RuntimeMachine, arg0: str) -> float
     *
     * @par Lua函数原型
     * getTimer(name: string) -> number
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"RuntimeMachine.getTimer","params":["timer"],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":25.409769612}
     *
     */
    double getTimer(const std::string &name);

    /**
     * 开始配置触发
     *
     * @param distance
     * @param delay
     * @return
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"RuntimeMachine.triggBegin","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int triggBegin(double distance, double delay);

    /**
     * 终止配置触发
     *
     * @return
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"RuntimeMachine.triggEnd","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0}
     *
     */
    int triggEnd();

    /**
     * 返回自动分配的中断号
     *
     * @param distance
     * @param delay
     * @param intnum
     * @return
     */
    int triggInterrupt(double distance, double delay);

    /**
     * 获取所有的中断号列表
     *
     * @return
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"RuntimeMachine.getTriggInterrupts","params":[],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":[]}
     *
     */
    std::vector<int> getTriggInterrupts();

protected:
    void *d_;
};

using RuntimeMachinePtr = std::shared_ptr<RuntimeMachine>;

} // namespace common_interface
} // namespace arcs
#endif // AUBO_SDK_RUNTIME_MACHINE_H
