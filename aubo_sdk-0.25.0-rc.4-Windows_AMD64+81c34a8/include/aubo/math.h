/** @file  math.h
 *  @brief 数学方法接口，如欧拉角与四元数转换、位姿的加减运算
 */
#ifndef AUBO_SDK_MATH_INTERFACE_H
#define AUBO_SDK_MATH_INTERFACE_H

#include <vector>
#include <memory>

#include <aubo/type_def.h>
#include <aubo/global_config.h>

namespace arcs {
namespace common_interface {

class ARCS_ABI_EXPORT Math
{
public:
    Math();
    virtual ~Math();

    /**
     * Pose addition
     *
     * Both arguments contain three position parameters (x, y, z) jointly called
     * P, and three rotation parameters (R_x, R_y, R_z) jointly called R. This
     * function calculates the result x_3 as the addition of the given poses as
     * follows:
     *
     * p_3.P = p_1.P + p_2.P
     * p_3.R = p_1.R * p_2.R
     *
     * 位姿相加。
     * 两个参数都包含三个位置参数（x、y、z），统称为P，
     * 以及三个旋转参数（R_x、R_y、R_z），统称为R。
     * 此函数根据以下方式计算结果 p_3，即给定位姿的相加：
     * p_3.P = p_1.P + p_2.P,
     * p_3.R = p_1.R * p_2.R
     *
     * @param p1 工具位姿1（pose）
     * @param p2 工具位姿2（pose）
     * @return Sum of position parts and product of rotation parts (pose)
     *         位置部分之和和旋转部分之积（pose）
     *
     * @par Python函数原型
     * poseAdd(self: pyaubo_sdk.Math, arg0: List[float], arg1: List[float]) ->
     * List[float]
     *
     * @par Lua函数原型
     * poseAdd(p1: table, p2: table) -> table
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"Math.poseAdd","params":[[0.2, 0.5, 0.1, 1.57,
     * 0, 0],[0.2, 0.5, 0.6, 1.57, 0, 0]],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":[0.4,1.0,0.7,3.14,-0.0,0.0]}
     *
     */
    std::vector<double> poseAdd(const std::vector<double> &p1,
                                const std::vector<double> &p2);

    /**
     * Pose subtraction
     * 位姿相减
     *
     * 两个参数都包含三个位置参数（x、y、z），统称为P，
     * 以及三个旋转参数（R_x、R_y、R_z），统称为R。
     * 此函数根据以下方式计算结果 p_3，即给定位姿的相加：
     * p_3.P = p_1.P - p_2.P,
     * p_3.R = p_1.R * p_2.R.inverse
     *
     * @param p1 工具位姿1
     * @param p2 工具位姿2
     * @return 位姿相减计算结果
     *
     * @par Python函数原型
     * poseSub(self: pyaubo_sdk.Math, arg0: List[float], arg1: List[float]) ->
     * List[float]
     *
     * @par Lua函数原型
     * poseSub(p1: table, p2: table) -> table
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"Math.poseSub","params":[[0.2, 0.5, 0.1, 1.57,
     * 0, 0],[0.2, 0.5, 0.6, 1.57, 0, 0]],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":[0.0,0.0,-0.5,0.0,-0.0,0.0]}
     *
     */
    std::vector<double> poseSub(const std::vector<double> &p1,
                                const std::vector<double> &p2);

    /**
     * 计算线性插值
     *
     * @param p1 起点的TCP位姿
     * @param p2 终点的TCP位姿
     * @param alpha 系数，
     * 当0<alpha<1，返回p1和p2两点直线的之间靠近p1端且占总路径比例为alpha的点；
     * 例如当alpha=0.3,返回的是靠近p1那端，总路径的百分之30的点；
     * 当alpha>1,返回p2；
     * 当alpha<0,返回p1；
     * @return 插值计算结果
     *
     * @par Python函数原型
     * interpolatePose(self: pyaubo_sdk.Math, arg0: List[float], arg1:
     * List[float], arg2: float) -> List[float]
     *
     * @par Lua函数原型
     * interpolatePose(p1: table, p2: table, alpha: number) -> table
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"Math.interpolatePose","params":[[0.2, 0.2,
     * 0.4, 0, 0, 0],[0.2, 0.2, 0.6, 0, 0, 0],0.5],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":[0.2,0.2,0.5,0.0,-0.0,0.0]}
     *
     */
    std::vector<double> interpolatePose(const std::vector<double> &p1,
                                        const std::vector<double> &p2,
                                        double alpha);
    /**
     * Pose transformation
     *
     * The first argument, p_from, is used to transform the second argument,
     * p_from_to, and the result is then returned. This means that the result is
     * the resulting pose, when starting at the coordinate system of p_from, and
     * then in that coordinate system moving p_from_to.
     *
     * This function can be seen in two different views. Either the function
     * transforms, that is translates and rotates, p_from_to by the parameters
     * of p_from. Or the function is used to get the resulting pose, when first
     * making a move of p_from and then from there, a move of p_from_to. If the
     * poses were regarded as transformation matrices, it would look like:
     *
     * T_world->to = T_world->from * T_from->to，
     * T_x->to = T_x->from * T_from->to
     *
     * 位姿变换
     *
     * 第一个参数 p_from 用于转换第二个参数 p_from_to，并返回结果。
     * 这意味着结果是从 p_from 的坐标系开始，
     * 然后在该坐标系中移动 p_from_to后的位姿。
     *
     * 这个函数可以从两个不同的角度来看。
     * 一种是函数将 p_from_to 根据 p_from 的参数进行转换，即平移和旋转。
     * 另一种是函数被用于获取结果姿态，先对 p_from 进行移动，然后再对 p_from_to
     * 进行移动。 如果将姿态视为转换矩阵，它看起来像是：
     *
     * T_world->to = T_world->from * T_from->to，
     * T_x->to = T_x->from * T_from->to
     *
     * 这两个方程描述了姿态变换的基本原理，根据给定的起始姿态和相对于起始姿态的姿态变化，可以计算出目标姿态。
     *
     * 举个例子，已知B相对于A的位姿、C相对于B的位姿，求C相对于A的位姿。
     * 第一个参数是B相对于A的位姿，第二个参数是C相对于B的位姿，
     * 返回值是C相对于A的位姿。
     *
     * @param pose_from 起始位姿（空间向量）
     * @param pose_from_to 相对于起始位姿的姿态变化（空间向量）
     * @return 结果位姿 (空间向量)
     *
     * @par Python函数原型
     * poseTrans(self: pyaubo_sdk.Math, arg0: List[float], arg1: List[float]) ->
     * List[float]
     *
     * @par Lua函数原型
     * poseTrans(pose_from: table, pose_from_to: table) -> table
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"Math.poseTrans","params":[[0.2, 0.5,
     * 0.1, 1.57, 0, 0],[0.2, 0.5, 0.6, 1.57, 0, 0]],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":[0.4,-0.09960164640373415,0.6004776374923573,3.14,-0.0,0.0]}
     *
     */
    std::vector<double> poseTrans(const std::vector<double> &pose_from,
                                  const std::vector<double> &pose_from_to);

    /**
     * 姿态逆变换
     *
     * 已知C相对于A的位姿、C相对于B的位姿，求B相对于A的位姿。
     * 第一个参数是C相对于A的位姿，第二个参数是C相对于B的位姿，
     * 返回值是B相对于A的位姿。
     *
     * @param pose_from 起始位姿
     * @param pose_to_from 相对于结果位姿的姿态变化
     * @return 结果位姿
     *
     * @par Python函数原型
     * poseTransInv(self: pyaubo_sdk.Math, arg0: List[float], arg1: List[float])
     * -> List[float]
     *
     * @par Lua函数原型
     * poseTransInv(pose_from: table, pose_to_from: table) -> table
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"Math.poseTransInv","params":[[0.4, -0.0996016,
     * 0.600478, 3.14, 0, 0],[0.2, 0.5, 0.6, 1.57, 0, 0]],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":[0.2,0.5000000464037341,0.10000036250764266,1.57,-0.0,0.0]}
     *
     */
    std::vector<double> poseTransInv(const std::vector<double> &pose_from,
                                     const std::vector<double> &pose_to_from);

    /**
     * Get the inverse of a pose
     *
     * 获取位姿的逆
     *
     * @param pose tool pose (spatial vector)
     *             工具位姿（空间向量）
     * @return inverse tool pose transformation (spatial vector)
     *         工具位姿的逆转换（空间向量）
     *
     * @par Python函数原型
     * poseInverse(self: pyaubo_sdk.Math, arg0: List[float]) -> List[float]
     *
     * @par Lua函数原型
     * poseInverse(pose: table) -> table
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"Math.poseInverse","params":[[0.2, 0.5,
     * 0.1, 1.57, 0, 3.14]],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":[0.19920341988726448,-0.09960155178838484,-0.5003973704832628,
     * 1.5699999989900404,-0.0015926530848129354,-3.1415913853161266]}
     *
     */
    std::vector<double> poseInverse(const std::vector<double> &pose);

    /**
     * 计算两个位姿的位置距离
     *
     * @param p1 位姿1
     * @param p2 位姿2
     * @return 两个位姿的位置距离
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"Math.poseDistance","params":[[0.1, 0.3, 0.1,
     * 0.3142, 0.0, 1.571],[0.2, 0.5, 0.6, 0, -0.172, 0.0]],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0.5477225575051661}
     *
     */
    double poseDistance(const std::vector<double> &p1,
                        const std::vector<double> &p2);

    /**
     * 计算两个位姿的轴角距离
     *
     * @param p1 位姿1
     * @param p2 位姿2
     * @return 轴角距离
     */
    double poseAngleDistance(const std::vector<double> &p1,

                             const std::vector<double> &p2);

    /**
     * 判断两个位姿是否相等
     *
     * @param p1 位姿1
     * @param p2 位姿2
     * @param eps 误差
     * @return 相等返回true，反之返回false
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"Math.poseDistance","params":[[0.1, 0.3, 0.1,
     * 0.3142, 0.0, 1.571],[0.1, 0.3, 0.1, 0.3142, 0.0, 1.5711]],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":0.0}
     *
     */
    bool poseEqual(const std::vector<double> &p1, const std::vector<double> &p2,
                   double eps = 5e-5);

    /**
     *
     * @param F_b_a_old
     * @param V_in_a
     * @param type
     * @return
     *
     * @par Python函数原型
     * transferRefFrame(self: pyaubo_sdk.Math, arg0: List[float], arg1:
     * List[float[3]], arg2: int) -> List[float]
     *
     * @par Lua函数原型
     * transferRefFrame(F_b_a_old: table, V_in_a: table, type: number) -> table
     *
     */
    std::vector<double> transferRefFrame(const std::vector<double> &F_b_a_old,
                                         const Vector3d &V_in_a, int type);

    /**
     * 姿态旋转
     *
     * @param pose
     * @param rotv
     * @return
     *
     * @par Python函数原型
     * poseRotation(self: pyaubo_sdk.Math, arg0: List[float], arg1: List[float])
     * -> List[float]
     *
     * @par Lua函数原型
     * poseRotation(pose: table, rotv: table) -> table
     *
     */
    std::vector<double> poseRotation(const std::vector<double> &pose,
                                     const std::vector<double> &rotv);

    /**
     * 欧拉角转四元数
     *
     * @param rpy 欧拉角
     * @return 四元数
     *
     * @par Python函数原型
     * rpyToQuaternion(self: pyaubo_sdk.Math, arg0: List[float]) -> List[float]
     *
     * @par Lua函数原型
     * rpyToQuaternion(rpy: table) -> table
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"Math.rpyToQuaternion","params":[[0.611, 0.785,
     * 0.960]],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":[0.834721517970497,0.07804256900772265,0.4518931575790371,0.3048637712043723]}
     *
     */
    std::vector<double> rpyToQuaternion(const std::vector<double> &rpy);

    /**
     * 四元数转欧拉角
     *
     * @param quat 四元数
     * @return 欧拉角
     *
     * @par Python函数原型
     * quaternionToRpy(self: pyaubo_sdk.Math, arg0: List[float]) -> List[float]
     *
     * @par Lua函数原型
     * quaternionToRpy(quat: table) -> table
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"Math.quaternionToRpy","params":[[0.834722,
     * 0.0780426, 0.451893, 0.304864]],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":[0.6110000520523781,0.7849996877683915,0.960000543982093]}
     *
     */
    std::vector<double> quaternionToRpy(const std::vector<double> &quat);

    /**
     * 四点法标定TCP偏移
     *
     * 找一个尖点，将机械臂工具末端点绕着尖点示教四个位置，姿态差别要大。
     * 设置完毕后即可计算出来结果。
     *
     * @param poses 四个点的位姿集合
     * @return TCP标定结果和标定结果是否有效
     *
     * @par Python函数原型
     * tcpOffsetIdentify(self: pyaubo_sdk.Math, arg0: List[List[float]]) ->
     * Tuple[List[float], int]
     *
     * @par Lua函数原型
     * tcpOffsetIdentify(poses: table) -> table
     *
     */
    ResultWithErrno tcpOffsetIdentify(
        const std::vector<std::vector<double>> &poses);

    /**
     * 三点法标定坐标系
     *
     * @param poses 三个点的位姿集合
     * @param type 类型:\n
     *      0 - oxy 原点 x轴正方向 xy平面（y轴正方向）\n
     *      1 - oxz 原点 x轴正方向 xz平面（z轴正方向）\n
     *      2 - oyz 原点 y轴正方向 yz平面（z轴正方向）\n
     *      3 - oyx 原点 y轴正方向 yx平面（x轴正方向）\n
     *      4 - ozx 原点 z轴正方向 zx平面（x轴正方向）\n
     *      5 - ozy 原点 z轴正方向 zy平面（y轴正方向）\n
     * @return 坐标系标定结果和标定结果是否有效
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"Math.calibrateCoordinate","params":[[[0.55462,0.06219,0.37175,-3.142,0.0,1.580],
     * [0.63746,0.11805,0.37175,-3.142,0.0,1.580],[0.40441,0.28489,0.37174,-3.142,0.0,1.580]],0],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":[[0.55462,0.06219,0.37175,-3.722688983883945e-05,-1.6940658945086007e-21,0.5932768162455785],0]}
     *
     */
    ResultWithErrno calibrateCoordinate(
        const std::vector<std::vector<double>> &poses, int type);

    /**
     *  根据圆弧的三个点,计算出拟合成的圆的另一半圆弧的中间点位置
     *
     *  @param p1 圆弧的起始点
     *  @param p2 圆弧的中间点
     *  @param p3 圆弧的结束点
     *  @param mode 当mode等于1的时候，表示需要对姿态进行圆弧规划；
     *  当mode等于0的时候，表示不需要对姿态进行圆弧规划
     *  @return 拟合成的圆的另一半圆弧的中间点位置和计算结果是否有效
     *
     * @par JSON-RPC请求示例
     * {"jsonrpc":"2.0","method":"Math.calculateCircleFourthPoint","params":[[0.5488696249770836,-0.1214996547187204,0.2631931199112321,-3.14159198038469,-3.673205103150083e-06,1.570796326792424],
     * [0.5488696249770835,-0.1214996547187207,0.3599720701808493,-3.14159198038469,-3.6732051029273e-06,1.570796326792423],
     * [0.5488696249770836,-0.0389996547187214,0.3599720701808496,-3.141591980384691,-3.673205102557476e-06,
     * 1.570796326792422],1],"id":1}
     *
     * @par JSON-RPC响应示例
     * {"id":1,"jsonrpc":"2.0","result":[[0.5488696249770837,-0.031860179583911546,0.27033259504604207,-3.1415919803846903,-3.67320510285378e-06,1.570796326792423],1]}
     *
     */
    ResultWithErrno calculateCircleFourthPoint(const std::vector<double> &p1,
                                               const std::vector<double> &p2,
                                               const std::vector<double> &p3,
                                               int mode);
    /**
     * @brief forceTrans:
     * 变换力和力矩的参考坐标系 force_in_b = pose_a_in_b * force_in_a
     * @param pose_a_in_b: a 坐标系在 b 坐标系的位姿
     * @param force_in_a: 力和力矩在 a 坐标系的描述
     * @return　force_in_b，力和力矩在 b 坐标系的描述
     */
    std::vector<double> forceTrans(const std::vector<double> &pose_a_in_b,
                                   const std::vector<double> &force_in_a);

    /**
     * @brief 通过距离计算工具坐标系下的位姿增量
     * @param distances: N 个距离, N >=3
     * @param position: 距离参考轨迹的保持高度
     * @param radius: 传感器中心距离末端tcp的等效半径
     * @param track_scale: 跟踪比例, 设置范围(0, 1], 1表示跟踪更快
     * @return 基于工具坐标系的位姿增量
     */
    std::vector<double> getDeltaPoseBySensorDistance(
        const std::vector<double> &distances, double position, double radius,
        double track_scale);

    /**
     * @brief changeFTFrame: 变换力和力矩的参考坐标系
     * @param pose_a_in_b: a 坐标系在 b 坐标系的位姿
     * @param ft_in_a: 作用在 a 点的力和力矩在 a 坐标系的描述
     * @return ft_in_b，作用在 b 点的力和力矩在 b 坐标系的描述
     */
    std::vector<double> deltaPoseTrans(const std::vector<double> &pose_a_in_b,
                                       const std::vector<double> &ft_in_a);

    /**
     * @brief addDeltaPose: 计算以给定速度变换单位时间后的位姿
     * @param pose_a_in_b: 当前时刻 a 相对于 b 的位姿
     * @param v_in_b: 当前时刻　a 坐标系的速度在　b 的描述
     * @return pose_in_b, 单位时间后的位姿在 b 的描述
     */
    std::vector<double> deltaPoseAdd(const std::vector<double> &pose_a_in_b,
                                     const std::vector<double> &v_in_b);

    /**
     * @brief changePoseWithXYRef: 修改 pose_tar
     * 的ｘｙ轴方向,尽量与　pose_ref　一致，
     * @param pose_tar: 需要修改的目标位姿
     * @param pose_ref: 参考位姿
     * @return 修改后的位姿，采用pose_tar的 xyz 坐标和 z 轴方向
     */
    std::vector<double> changePoseWithXYRef(
        const std::vector<double> &pose_tar,
        const std::vector<double> &pose_ref);

    /**
     * @brief homMatrixToPose: 由齐次变换矩阵得到位姿
     * @param homMatrix: 4*4 齐次变换矩阵, 输入元素采用横向排列
     * @return 对应的位姿
     */
    std::vector<double> homMatrixToPose(const std::vector<double> &homMatrix);

    /**
     * @brief poseToHomMatrix: 位姿变换得到齐次变换矩阵
     * @param pose: 输入的位姿
     * @return 输出的齐次变换矩阵,元素横向排列
     */
    std::vector<double> poseToHomMatrix(const std::vector<double> &pose);

protected:
    void *d_;
};
using MathPtr = std::shared_ptr<Math>;

} // namespace common_interface
} // namespace arcs
#endif
