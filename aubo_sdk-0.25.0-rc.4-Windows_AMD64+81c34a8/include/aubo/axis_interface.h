/** @file axes.h
 *  @brief 外部轴接口
 */
#ifndef AUBO_SDK_AXIS_INTERFACE_H
#define AUBO_SDK_AXIS_INTERFACE_H

#include <aubo/sync_move.h>
#include <aubo/trace.h>

namespace arcs {
namespace common_interface {

/**
 * 外部轴API接口
 */
class ARCS_ABI_EXPORT AxisInterface
{
public:
    AxisInterface();
    virtual ~AxisInterface();

    /**
     * 通电
     *
     * @return
     */
    int poweronExtAxis();

    /**
     * 断电
     *
     * @return
     */
    int poweroffExtAxis();

    /**
     * 使能
     *
     * @return
     */
    int enableExtAxis();

    /**
     * 设置外部轴的安装位姿(相对于世界坐标系)
     *
     * @param pose
     * @return
     */
    int setExtAxisMountingPose(const std::vector<double> &pose);

    /**
     * 运动到指定点， 旋转或者平移
     *
     * @param pos
     * @param v
     * @param a
     * @param duration
     * @return
     */
    int moveExtJoint(double pos, double v, double a, double duration);

    /**
     * 制定目标运动速度
     *
     * @param v
     * @param a
     * @param duration
     * @return
     */
    int speedExtJoint(double v, double a, double duration);

    int stopExtJoint(double a);

    /**
     * 获取外部轴的类型 0代表是旋转 1代表平移
     *
     * @return
     */
    int getExtAxisType();

    /**
     * 获取当前外部轴的状态
     *
     * @return 当前外部轴的状态
     */
    AxisModeType getAxisModeType();

    /**
     * 获取外部轴安装位姿
     *
     * @return 外部轴安装位姿
     */
    std::vector<double> getExtAxisMountingPose();

    /**
     * 获取相对于安装坐标系的位姿，外部轴可能为变位机或者导轨
     *
     * @return 相对于安装坐标系的位姿
     */
    std::vector<double> getExtAxisPose();

    /**
     * 获取外部轴位置
     *
     * @return 外部轴位置
     */
    double getExtAxisPosition();

    /**
     * 获取外部轴运行速度
     *
     * @return 外部轴运行速度
     */
    double getExtAxisVelocity();

    /**
     * 获取外部轴运行加速度
     *
     * @return 外部轴运行加速度
     */
    double getExtAxisAcceleration();

    /**
     * 获取外部轴电流
     *
     * @return 外部轴电流
     */
    double getExtAxisCurrent();

    /**
     * 获取外部轴温度
     *
     * @return 外部轴温度
     */
    double getExtAxisTemperature();

    /**
     * 获取外部轴电压
     *
     * @return 外部轴电压
     */
    double getExtAxisBusVoltage();

    /**
     * 获取外部轴电流
     *
     * @return 外部轴电流
     */
    double getExtAxisBusCurrent();

    /**
     * 获取外部轴最大位置
     *
     * @return 外部轴最大位置
     */
    double getExtAxisMaxPosition();

    /**
     * 获取外部轴最小位置
     *
     * @return 外部轴最小位置
     */
    double getExtMinPosition();

    /**
     * 获取外部轴最大速度
     *
     * @return 外部轴最大速度
     */
    double getExtAxisMaxVelocity();

    /**
     * 获取外部轴最大加速度
     *
     * @return 外部轴最大加速度
     */
    double getExtAxisMaxAcceleration();

    /**
     * 跟踪另一个外部轴的运动(禁止运动过程中使用)
     *
     * @param target_name 目标的外部轴名字
     * @param phase 相位差
     * @param err 跟踪运行的最大误差
     * @return
     */
    int followAnotherAxis(const std::string &target_name, double phase,
                          double err);

    /**
     * @brief stopFollowAnotherAxis(禁止运动过程中使用)
     * @return
     */
    int stopFollowAnotherAxis();

protected:
    void *d_;
};
using AxisInterfacePtr = std::shared_ptr<AxisInterface>;

} // namespace common_interface
} // namespace arcs

#endif // AUBO_SDK_AXIS_INTERFACE_H
