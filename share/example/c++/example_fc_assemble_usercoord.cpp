#include <cstring>
#include <fstream>
#include <math.h>
#ifdef WIN32
#include <windows.h>
#endif
#include "thread"
#include <mutex>

#include "aubo_sdk/rpc.h"
#include "aubo_sdk/rtde.h"

using namespace arcs::aubo_sdk;
using namespace arcs::common_interface;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define EMBEDDED

#define INF 99999

std::mutex rtde_mtx_;
bool cond_fullfiled_ = false;
std::vector<double> tcp_pose_(6, 0.);
std::vector<double> tcp_force_(6, 0.);
// Implement blocking functionality: When the robot arm reaches the target waypoint, the program continues execution
int waitArrival(RobotInterfacePtr impl)
{
    // API call: Get the current motion command ID
    int exec_id = impl->getMotionControl()->getExecId();

    int cnt = 0;
    // Maximum retry count for getting exec_id while waiting for the robot arm to start moving
    int max_retry_count = 50;

    // Wait for the robot arm to start moving
    while (exec_id == -1) {
        if (cnt++ > max_retry_count) {
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        exec_id = impl->getMotionControl()->getExecId();
    }

    // Wait for the robot arm to finish the action
    while (exec_id != -1) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        exec_id = impl->getMotionControl()->getExecId();
    }

    return 0;
}

template <typename T>
inline std::ostream &operator<<(std::ostream &os, const std::vector<T> &list)
{
    for (size_t i = 0; i < list.size(); i++) {
        try {
            os << list.at(i);
            if (i != (list.size() - 1)) {
                os << ",";
            }
        } catch (std::exception &e) {
        }
    }
    return os;
}

inline bool equal(const std::vector<double> &q1, const std::vector<double> &q2,
                  double eps = 0.001)
{
    //    std::cout << "q1: " << q1 << std::endl;
    //    std::cout << "q2: " << q2 << std::endl;
    if (q1.size() && (q1.size() == q2.size())) {
        for (unsigned int i = 0; i < q1.size(); i++) {
            if (std::abs(q1[i] - q2[i]) > eps) {
                return false;
            }
        }
        return true;
    }
    return false;
}

void tcpSensorTest(RpcClientPtr cli)
{
    // API call: Get the robot's name
    auto robot_name = cli->getRobotNames().front();
#ifdef EMBEDDED
    // Built-in sensor
    std::vector<double> sensor_pose = { 0, 0, 0, 0, 0, 0 };
    cli->getRobotInterface(robot_name)
        ->getRobotConfig()
        ->selectTcpForceSensor("embedded");
#else
    // External KW sensor
    std::vector<double> sensor_pose = { 0, 0, 0.047, 0, 0, 0 };
    cli->getRobotInterface(robot_name)
        ->getRobotConfig()
        ->selectTcpForceSensor("kw_ftsensor");
#endif
    while (1) {
        std::cout << "------------------------------------------" << std::endl;
        auto sensor_data = cli->getRobotInterface(robot_name)
                               ->getRobotState()
                               ->getTcpForceSensors();
        std::cout << "force sensor: " << sensor_data << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        auto curr_pose =
            cli->getRobotInterface(robot_name)->getRobotState()->getTcpPose();
        auto payload =
            cli->getRobotInterface(robot_name)->getRobotConfig()->getPayload();
        std::cout << "mass: " << std::get<0>(payload)
                  << " cog: " << std::get<1>(payload) << std::endl;
        auto result =
            cli->getRobotInterface(robot_name)
                ->getRobotAlgorithm()
                ->calibrateTcpForceSensor({ sensor_data }, { curr_pose });

        std::cout << "force offset: " << std::get<0>(result) << std::endl;
        std::cout << "------------------------------------------" << std::endl;
    }
}

/**
 * @brief condFullfiled Determine whether the force control termination condition is met
 * @param timeout Time limit. If the time exceeds timeout and the condition is not met, return -1;
 *        If timeout=-1, no time limit is applied
 * @param target_pose If the target point is reached but the condition is not met, return 0;
 *        If target_pose={}, do not check the target point
 *
 * @return Returns 1 if the condition is met; returns 0 if the target is reached but the condition is not met; returns -1 if timeout;
 *         If timeout=-1 and target_pose={}, this function blocks until the condition is met
 */
int condFullfiled(double timeout, std::vector<double> target_pose)
{
    int ret = 0;
    int cnt = 0;

    while (!cond_fullfiled_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        if ((timeout > 0) && (cnt++ > timeout * 500)) {
            std::cout << "Condition wait timeout!" << std::endl;
            ret = -1;
            break;
        }
        if ((target_pose.size() == 6) && equal(target_pose, tcp_pose_)) {
            std::cout << "Robot arm has reached the target!" << std::endl;
            ret = 0;
            break;
        }
    }
    if (cond_fullfiled_) {
        // Condition met, Z direction search finished
        ret = 1;
    }
    return ret;
}

/**
 * @brief locateZ Z direction positioning
 * Determines whether the change in Z direction force exceeds a certain threshold, implemented via setCondForce
 * @param cli
 */
int locateZ(RpcClientPtr cli, const double bolt_len,
            const std::vector<double> &usercoord_in_tcp)
{
    // Force control parameters
    std::vector<double> locate_z_M = { 30.0, 30.0, 30.0, 10.0, 10.0, 10.0 };
    std::vector<double> locate_z_D = {
        300.0, 250.0, 200.0, 100.0, 100.0, 100.0
    };
    std::vector<double> locate_z_K = {
        500.0, 500.0, 500.0, 100.0, 100.0, 100.0
    };
    // Force control enabled directions
    std::vector<bool> locate_z_enable = {
        true, true, true, false, false, false
    };
    // Target force
    std::vector<double> locate_z_goal_wrench = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    // Search distance described in usercoord
    std::vector<double> seeking_distance = {
        0.0, 0.0, bolt_len, 0.0, 0.0, 0.0
    };
    // Force control coordinate system selection: base coordinate system
    std::vector<double> feature = usercoord_in_tcp;
    TaskFrameType frame_type = TaskFrameType::TOOL_FORCE;
    // Force control speed limits
    std::vector<double> speed_limits(6, 2.0);

    auto robot_name = cli->getRobotNames().front();
    // Set force control parameters
    cli->getRobotInterface(robot_name)
        ->getForceControl()
        ->setDynamicModel(locate_z_M, locate_z_D, locate_z_K);
    // Set target force
    cli->getRobotInterface(robot_name)
        ->getForceControl()
        ->setTargetForce(feature, locate_z_enable, locate_z_goal_wrench,
                         speed_limits, frame_type);
    double min_force = -INF;
    double max_force = 5.0;
    std::vector<double> select = { 0, 0., 1., 0., 0., 0. };
    std::vector<double> feature1 = usercoord_in_tcp;
    double type = static_cast<double>(TaskFrameType::TOOL_FORCE);
    double timeout = -1;
    double count = 1.;
    double outside = 1;

    std::vector<double> args;
    args.insert(args.end(), min_force);
    args.insert(args.end(), max_force);
    args.insert(args.end(), select.begin(), select.end());
    args.insert(args.end(), feature1.begin(), feature1.end());
    args.insert(args.end(), type);
    args.insert(args.end(), count);
    args.insert(args.end(), outside);
    cli->getRobotInterface(robot_name)
        ->getForceControl()
        ->setCondAdvanced("ConditionContactForceOnUsercoord", args, timeout);

    // Get current TCP position
    auto tcp_pose =
        cli->getRobotInterface(robot_name)->getRobotState()->getTcpPose();
    std::vector<double> usercoord_in_base =
        cli->getMath()->poseTrans(tcp_pose, usercoord_in_tcp);

    auto tcp_in_usercoord = cli->getMath()->poseInverse(usercoord_in_tcp);
    std::vector<double> target_pose_in_usercoord(6, 0.);
    for (int i = 0; i < 6; i++) {
        target_pose_in_usercoord[i] = tcp_in_usercoord[i] + seeking_distance[i];
    }
    auto target_pose_in_base =
        cli->getMath()->poseTrans(usercoord_in_base, target_pose_in_usercoord);
    // Linear motion
    double v = 0.0025, a = 4.0;
    cli->getRobotInterface(robot_name)
        ->getMotionControl()
        ->moveLine(target_pose_in_base, a, v, 0, 0);
    // Enable force control
    cli->getRobotInterface(robot_name)->getForceControl()->fcEnable();
    cli->getRobotInterface(robot_name)->getForceControl()->setCondActive();
    auto ret = condFullfiled(3, target_pose_in_base);
    cli->getRobotInterface(robot_name)->getMotionControl()->stopJoint(1);
    // Disable force control
    cli->getRobotInterface(robot_name)->getForceControl()->fcDisable();

    return ret;
}

/**
 * @brief insertDetect Determine whether the bolt is inserted into the hole; move forward and backward twice, if both contacts are detected, consider it inserted
 * @param cli
 * @return
 */
int insertDetect(RpcClientPtr cli, const std::vector<double> &usercoord_in_tcp)
{
    // Force control parameters
    std::vector<double> insert_detect_M = {
        30.0, 30.0, 30.0, 10.0, 10.0, 10.0
    };
    std::vector<double> insert_detect_D = { 1000.0, 1000.0, 500.0,
                                            100.0,  100.0,  100.0 };
    std::vector<double> insert_detect_K = { 500.0, 500.0, 500.0,
                                            100.0, 100.0, 100.0 };
    // Force control enabled directions
    std::vector<bool> insert_detect_enable = { true,  true,  false,
                                               false, false, false };
    // Force control coordinate system selection: base coordinate system
    std::vector<double> feature = usercoord_in_tcp;
    TaskFrameType frame_type = TaskFrameType::NONE;
    // Force control speed limits
    std::vector<double> speed_limits(6, 2.0);

    auto robot_name = cli->getRobotNames().front();
    // Set force control parameters
    cli->getRobotInterface(robot_name)
        ->getForceControl()
        ->setDynamicModel(insert_detect_M, insert_detect_D, insert_detect_K);

    // Target force
    std::vector<double> insert_detect_goal_wrench = { -8.0, 0.0, 0.0,
                                                      0.0,  0.0, 0.0 };
    // Set target force
    cli->getRobotInterface(robot_name)
        ->getForceControl()
        ->setTargetForce(feature, insert_detect_enable,
                         insert_detect_goal_wrench, speed_limits, frame_type);

    double min_force = -INF;
    double max_force = 5.0;
    std::vector<double> select = { 1., 1., 0., 0., 0., 0. };
    std::vector<double> feature1 = usercoord_in_tcp;
    double type = static_cast<double>(TaskFrameType::TOOL_FORCE);
    double timeout = -1;
    double count = 3.;
    double outside = 1;

    std::vector<double> args;
    args.insert(args.end(), min_force);
    args.insert(args.end(), max_force);
    args.insert(args.end(), select.begin(), select.end());
    args.insert(args.end(), feature1.begin(), feature1.end());
    args.insert(args.end(), type);
    args.insert(args.end(), count);
    args.insert(args.end(), outside);
    cli->getRobotInterface(robot_name)
        ->getForceControl()
        ->setCondAdvanced("ConditionContactForceOnUsercoord", args, timeout);

    // Enable force control
    cli->getRobotInterface(robot_name)->getForceControl()->fcEnable();
    cli->getRobotInterface(robot_name)->getForceControl()->setCondActive();
    auto ret = condFullfiled(1, {});
    if (ret < 0) {
        std::cout << "Insertion failed, insertion detection ended!" << std::endl;
        cli->getRobotInterface(robot_name)->getMotionControl()->stopJoint(1);
        // Disable force control
        cli->getRobotInterface(robot_name)->getForceControl()->fcDisable();
        return ret;
    } else {
        std::cout << "Insertion detection: Forward contact successful!" << std::endl;
    }

    cli->getRobotInterface(robot_name)->getMotionControl()->stopJoint(1);
    // Disable force control
    cli->getRobotInterface(robot_name)->getForceControl()->fcDisable();
    // Target force
    insert_detect_goal_wrench = { 8.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    // Start reverse contact
    cli->getRobotInterface(robot_name)
        ->getForceControl()
        ->setTargetForce(feature, insert_detect_enable,
                         insert_detect_goal_wrench, speed_limits, frame_type);
    cli->getRobotInterface(robot_name)
        ->getForceControl()
        ->setCondAdvanced("ConditionContactForceOnUsercoord", args, timeout);
    cli->getRobotInterface(robot_name)->getForceControl()->fcEnable();
    cli->getRobotInterface(robot_name)->getForceControl()->setCondActive();
    ret = condFullfiled(1, {});
    if (ret < 0) {
        std::cout << "Insertion failed, insertion detection ended!" << std::endl;
        cli->getRobotInterface(robot_name)->getMotionControl()->stopJoint(1);
        // Disable force control
        cli->getRobotInterface(robot_name)->getForceControl()->fcDisable();
        return ret;
    } else {
        std::cout << "Insertion detection: Reverse contact successful!" << std::endl;
    }
    cli->getRobotInterface(robot_name)->getMotionControl()->stopJoint(1);
    // Disable force control
    cli->getRobotInterface(robot_name)->getForceControl()->fcDisable();
    return ret;
}

/**
 * @brief locateXY Locate bolt position, move in a spiral trajectory in the XY plane
 * @param cli
 * @return
 */
int locateXY(RpcClientPtr cli, const std::vector<double> &usercoord_in_tcp)
{
    // Force control parameters
    std::vector<double> locate_xy_M = { 30.0, 30.0, 30.0, 10.0, 10.0, 10.0 };
    std::vector<double> locate_xy_D = { 1000.0, 1000.0, 500.0,
                                        100.0,  100.0,  100.0 };
    std::vector<double> locate_xy_K = {
        500.0, 500.0, 500.0, 100.0, 100.0, 100.0
    };

    // Force control enabled directions
    std::vector<bool> locate_xy_enable = {
        true, true, true, false, false, false
    };
    // Target force
    std::vector<double> locate_xy_goal_wrench = {
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    };

    // Force control coordinate system selection: tool coordinate system
    std::vector<double> feature = usercoord_in_tcp;
    TaskFrameType frame_type = TaskFrameType::TOOL_FORCE;
    // Force control speed limits
    std::vector<double> speed_limits(6, 2.0);

    auto robot_name = cli->getRobotNames().front();
    // Set force control parameters
    cli->getRobotInterface(robot_name)
        ->getForceControl()
        ->setDynamicModel(locate_xy_M, locate_xy_D, locate_xy_K);
    // Set target force
    cli->getRobotInterface(robot_name)
        ->getForceControl()
        ->setTargetForce(feature, locate_xy_enable, locate_xy_goal_wrench,
                         speed_limits, frame_type);
    double min_force = -INF;
    double max_force = 10.0;
    std::vector<double> select = { 1., 0., 0., 0., 0., 0. };
    std::vector<double> feature1 = usercoord_in_tcp;
    double type = static_cast<double>(TaskFrameType::TOOL_FORCE);
    double timeout = -1;
    double count = 1.;
    double outside = 1;
    std::vector<double> args_x;
    args_x.insert(args_x.end(), min_force);
    args_x.insert(args_x.end(), max_force);
    args_x.insert(args_x.end(), select.begin(), select.end());
    args_x.insert(args_x.end(), feature1.begin(), feature1.end());
    args_x.insert(args_x.end(), type);
    args_x.insert(args_x.end(), count);
    args_x.insert(args_x.end(), outside);
    cli->getRobotInterface(robot_name)
        ->getForceControl()
        ->setCondAdvanced("ConditionContactForceOnUsercoord", args_x, timeout);

    select = { 0., 1., 0., 0., 0., 0. };
    std::vector<double> args_y;
    args_y.insert(args_y.end(), min_force);
    args_y.insert(args_y.end(), max_force);
    args_y.insert(args_y.end(), select.begin(), select.end());
    args_y.insert(args_y.end(), feature1.begin(), feature1.end());
    args_y.insert(args_y.end(), type);
    args_y.insert(args_y.end(), count);
    args_y.insert(args_y.end(), outside);
    cli->getRobotInterface(robot_name)
        ->getForceControl()
        ->setCondAdvanced("ConditionContactForceOnUsercoord", args_y, timeout);

    SpiralParameters spiral;
    spiral.spiral = 0.001;       // Spiral step size
    spiral.helix = 0.;           // Planar spiral
    spiral.angle = 5 * 2 * M_PI; // Rotate 5 turns
    spiral.plane = 0;
    double radius = 0.001; // Spiral first circle radius

    spiral.frame =
        cli->getRobotInterface(robot_name)->getRobotState()->getTcpPose();

    // Calculate spiral trajectory center point
    spiral.frame[0] =
        std::sqrt(2) * radius / 2 + spiral.frame[0]; // Calculate spiral radius reference point
    spiral.frame[1] = std::sqrt(2) * radius / 2 + spiral.frame[1];
    double v = 0.015, a = 0.2;
    // Spiral motion
    cli->getRobotInterface(robot_name)
        ->getMotionControl()
        ->moveSpiral(spiral, 0, v, a, 0);
    // Enable force control
    cli->getRobotInterface(robot_name)->getForceControl()->fcEnable();
    // Activate termination condition
    cli->getRobotInterface(robot_name)->getForceControl()->setCondActive();
    auto ret = condFullfiled(10, {});
    cli->getRobotInterface(robot_name)->getMotionControl()->stopJoint(1);
    // Disable force control
    cli->getRobotInterface(robot_name)->getForceControl()->fcDisable();
    return ret;
}

/**
 * @brief calBoltPose Calculate bolt position, based on base coordinate system
 * @param cli
 * @param bolt_len Bolt length
 * @param max_force Maximum force
 * @param diameter Bolt diameter
 * @param user_coord User coordinate system expressed in tool coordinate system
 * @return Bolt position expressed in user coordinate system
 */
std::vector<double> calBoltPose(RpcClientPtr cli, const double bolt_len,
                                const double max_force, const double diameter,
                                const std::vector<double> &usercoord_in_tcp)
{
    auto robot_name = cli->getRobotNames().front();

    std::vector<double> tcp_pose =
        cli->getRobotInterface(robot_name)->getRobotState()->getTcpPose();
    std::vector<double> usercoord_in_base =
        cli->getMath()->poseTrans(tcp_pose, usercoord_in_tcp);
    auto tcp_force =
        cli->getRobotInterface(robot_name)->getRobotState()->getTcpForce();
    auto tcp_wrench_in_user = cli->getMath()->forceTrans(
        cli->getMath()->poseInverse(usercoord_in_base), tcp_force);
    std::cout << "tcp_wrench_in_user: " << tcp_wrench_in_user << std::endl;

    // Current tool end position in user coordinate system
    std::vector<double> bolt_pose_in_user =
        cli->getMath()->poseInverse(usercoord_in_tcp);
    if ((fabs(tcp_wrench_in_user[0]) > max_force) &&
        (fabs(tcp_wrench_in_user[1]) < max_force)) {
        // X direction force > limit, Y direction < limit, move in X direction
        bolt_pose_in_user[0] =
            bolt_pose_in_user[0] -
            (tcp_wrench_in_user[0] / fabs(tcp_wrench_in_user[0])) * diameter;
    } else if ((fabs(tcp_wrench_in_user[0]) < max_force) &&
               (fabs(tcp_wrench_in_user[1]) > max_force)) {
        // X direction force < limit, Y direction > limit, move in Y direction
        bolt_pose_in_user[1] =
            bolt_pose_in_user[1] -
            (tcp_wrench_in_user[1] / fabs(tcp_wrench_in_user[1])) * diameter;

    } else if ((fabs(tcp_wrench_in_user[0]) > max_force) &&
               (fabs(tcp_wrench_in_user[1]) > max_force)) {
        // X direction force > limit, Y direction > limit, move in XY composite direction
        double contact_force =
            std::sqrt(tcp_wrench_in_user[0] * tcp_wrench_in_user[0] +
                      tcp_wrench_in_user[1] * tcp_wrench_in_user[1]);
        bolt_pose_in_user[0] =
            bolt_pose_in_user[0] -
            (tcp_wrench_in_user[0] / contact_force) * diameter;
        bolt_pose_in_user[1] =
            bolt_pose_in_user[1] -
            (tcp_wrench_in_user[1] / contact_force) * diameter;
    } else {
        std::cout << "Failed to calculate bolt position!" << std::endl;
        return {};
    }
    bolt_pose_in_user[2] = bolt_pose_in_user[2] - bolt_len;

    return bolt_pose_in_user;
}

/**
 * @brief moveToBoltPose Move above the bolt
 * @param cli
 * @param bolt_pose_in_user Bolt position
 * @param bolt_len  Bolt length
 * @return
 */
int moveToBoltPose(RpcClientPtr cli, std::vector<double> &bolt_pose_in_user,
                   const std::vector<double> &usercoord_in_tcp)

{
    // Force control parameters
    std::vector<double> move_M = { 30.0, 30.0, 30.0, 10.0, 10.0, 10.0 };
    std::vector<double> move_D = { 300.0, 250.0, 200.0, 100.0, 100.0, 100.0 };
    std::vector<double> move_K = { 500.0, 500.0, 500.0, 100.0, 100.0, 100.0 };
    // Force control enabled directions
    std::vector<bool> move_fc_enable = {
        true, true, false, false, false, false
    };
    // Target force
    std::vector<double> move_goal_wrench = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

    // Force control coordinate system selection: base coordinate system
    std::vector<double> feature = usercoord_in_tcp;
    TaskFrameType frame_type = TaskFrameType::TOOL_FORCE;
    // Force control speed limits
    std::vector<double> speed_limits(6, 2.0);

    auto robot_name = cli->getRobotNames().front();
    // Set force control parameters
    cli->getRobotInterface(robot_name)
        ->getForceControl()
        ->setDynamicModel(move_M, move_D, move_K);
    // Set target force
    cli->getRobotInterface(robot_name)
        ->getForceControl()
        ->setTargetForce(feature, move_fc_enable, move_goal_wrench,
                         speed_limits, frame_type);

    std::vector<double> tcp_pose =
        cli->getRobotInterface(robot_name)->getRobotState()->getTcpPose();
    std::vector<double> usercoord_in_base =
        cli->getMath()->poseTrans(tcp_pose, usercoord_in_tcp);
    auto bolt_pose_in_base =
        cli->getMath()->poseTrans(usercoord_in_base, bolt_pose_in_user);
    std::cout << "current_pose: " << tcp_pose << std::endl;
    std::cout << "target_pose: " << bolt_pose_in_base << std::endl;
    double v = 0.02, a = 4.0;
    std::vector<double> trans_pose = tcp_pose;
    trans_pose[2] = bolt_pose_in_base[2];
    // Move to transition point
    cli->getRobotInterface(robot_name)
        ->getMotionControl()
        ->moveLine(trans_pose, a, v, 0, 0);
    // Call motion API first, then enable force control
    cli->getRobotInterface(robot_name)->getForceControl()->fcEnable();
    waitArrival(cli->getRobotInterface(robot_name));
    // Move above the bolt
    cli->getRobotInterface(robot_name)
        ->getMotionControl()
        ->moveLine(bolt_pose_in_base, a, v, 0, 0);
    waitArrival(cli->getRobotInterface(robot_name));
    cli->getRobotInterface(robot_name)->getMotionControl()->stopJoint(1);
    // Disable force control
    cli->getRobotInterface(robot_name)->getForceControl()->fcDisable();
    return 0;
}

int searchXY(RpcClientPtr cli, const std::vector<double> &usercoord_in_tcp)
{
    std::vector<double> search_M = { 30.0, 30.0, 30.0, 10.0, 10.0, 10.0 };
    std::vector<double> search_D = { 600.0, 500.0, 500.0, 200.0, 200.0, 200.0 };
    std::vector<double> search_K = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    std::vector<bool> search_fc_enable = {
        false, false, true, false, false, false,
    };
    std::vector<double> search_goal_wrench = { 0.0, 0.0, -8.5, 0.0, 0.0, 0.0 };

    // Force control coordinate system selection: base coordinate system
    std::vector<double> feature = usercoord_in_tcp;
    TaskFrameType frame_type = TaskFrameType::TOOL_FORCE;
    // Force control speed limits
    std::vector<double> speed_limits(6, 2.0);

    auto robot_name = cli->getRobotNames().front();

    // Set force control parameters
    cli->getRobotInterface(robot_name)
        ->getForceControl()
        ->setDynamicModel(search_M, search_D, search_K);

    // Set target force
    cli->getRobotInterface(robot_name)
        ->getForceControl()
        ->setTargetForce(feature, search_fc_enable, search_goal_wrench,
                         speed_limits, frame_type);

    // Z direction force suddenly decreases, consider the hole found
    double min_force = -INF;
    double max_force = 4.5;
    std::vector<double> select = { 0., 0., 1., 0., 0., 0. };
    std::vector<double> feature1 = usercoord_in_tcp;
    double type = static_cast<double>(TaskFrameType::TOOL_FORCE);
    double timeout = -1;
    double count = 3.;
    double outside = 0;

    std::vector<double> args;
    args.insert(args.end(), min_force);
    args.insert(args.end(), max_force);
    args.insert(args.end(), select.begin(), select.end());
    args.insert(args.end(), feature1.begin(), feature1.end());
    args.insert(args.end(), type);
    args.insert(args.end(), count);
    args.insert(args.end(), outside);
    cli->getRobotInterface(robot_name)
        ->getForceControl()
        ->setCondAdvanced("ConditionContactForceOnUsercoord", args, timeout);

    // Probe down 2mm, consider the hole found
    cli->getRobotInterface(robot_name)
        ->getForceControl()
        ->setCondDistance(0.002, -1);
    SpiralParameters spiral;
    spiral.spiral = 0.001;       // Spiral step size
    spiral.helix = 0.;           // Planar spiral
    spiral.angle = 7 * 2 * M_PI; // Rotate 5 turns
    spiral.plane = 0;
    double radius = 0.001; // Spiral first circle radius

    spiral.frame =
        cli->getRobotInterface(robot_name)->getRobotState()->getTcpPose();
    // Calculate spiral trajectory center point
    spiral.frame[0] =
        std::sqrt(2) * radius / 2 + spiral.frame[0]; // Calculate spiral radius reference point
    spiral.frame[1] = std::sqrt(2) * radius / 2 + spiral.frame[1];
    double v = 0.002, a = 0.1;
    // Spiral motion
    cli->getRobotInterface(robot_name)
        ->getMotionControl()
        ->moveSpiral(spiral, 0, v, a, 0);
    // Enable force control
    cli->getRobotInterface(robot_name)->getForceControl()->fcEnable();
    // Activate termination condition
    cli->getRobotInterface(robot_name)->getForceControl()->setCondActive();

    auto ret = condFullfiled(60, {});
    cli->getRobotInterface(robot_name)->getMotionControl()->stopJoint(1);
    // Disable force control
    cli->getRobotInterface(robot_name)->getForceControl()->fcDisable();
    return ret;
}

int insertZ(RpcClientPtr cli, const double bolt_len,
            const std::vector<double> &usercoord_in_tcp)
{
    std::vector<double> insert_M = { 30.0, 30.0, 30.0, 5.0, 5.0, 5.0 };
    std::vector<double> insert_D = { 800.0, 800.0, 600.0, 100.0, 100.0, 100.0 };
    std::vector<double> insert_K = { 1.0, 1.0, 500.0, 1.0, 1.0, 400.0 };
    std::vector<bool> insert_fc_enable = { true, true, true, true, true, true };
    std::vector<double> insert_goal_wrench = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    // Force control coordinate system selection: base coordinate system
    std::vector<double> feature = usercoord_in_tcp;
    TaskFrameType frame_type = TaskFrameType::TOOL_FORCE;
    // Force control speed limits
    std::vector<double> speed_limits(6, 2.0);

    auto robot_name = cli->getRobotNames().front();

    // Set force control parameters
    cli->getRobotInterface(robot_name)
        ->getForceControl()
        ->setDynamicModel(insert_M, insert_D, insert_K);

    // Set target force
    cli->getRobotInterface(robot_name)
        ->getForceControl()
        ->setTargetForce(feature, insert_fc_enable, insert_goal_wrench,
                         speed_limits, frame_type);

    // Calculate insertion termination position
    auto tcp_pose =
        cli->getRobotInterface(robot_name)->getRobotState()->getTcpPose();
    std::vector<double> usercoord_in_base =
        cli->getMath()->poseTrans(tcp_pose, usercoord_in_tcp);

    auto tcp_in_usercoord = cli->getMath()->poseInverse(usercoord_in_tcp);
    std::vector<double> insert_distance = { 0., 0., bolt_len, 0., 0., 0. };
    std::vector<double> target_pose_in_usercoord(6, 0.);
    for (int i = 0; i < 6; i++) {
        target_pose_in_usercoord[i] = tcp_in_usercoord[i] + insert_distance[i];
    }
    auto target_pose_in_base =
        cli->getMath()->poseTrans(usercoord_in_base, target_pose_in_usercoord);

    std::vector<double> box = { -INF, +INF, -INF, +INF, 0, bolt_len };
    // Linear motion
    double v = 0.0025, a = 4.0;
    cli->getRobotInterface(robot_name)
        ->getMotionControl()
        ->moveLine(target_pose_in_base, a, v, 0, 0);

    // Enable force control
    cli->getRobotInterface(robot_name)->getForceControl()->fcEnable();
    // Activate termination condition
    cli->getRobotInterface(robot_name)->getForceControl()->setCondActive();

    auto ret = condFullfiled(5, target_pose_in_base);

    cli->getRobotInterface(robot_name)->getMotionControl()->stopJoint(1);
    // Disable force control
    cli->getRobotInterface(robot_name)->getForceControl()->fcDisable();
    return ret;
}

int example(RpcClientPtr cli)
{
    auto robot_name = cli->getRobotNames().front();
#ifdef EMBEDDED
    // Built-in sensor
    std::vector<double> sensor_pose = { 0, 0, 0, 0, 0, 0 };
    cli->getRobotInterface(robot_name)
        ->getRobotConfig()
        ->selectTcpForceSensor("embedded");
#else
    // External KW sensor
    std::vector<double> sensor_pose = { 0, 0, 0.047, 0, 0, 0 };
    cli->getRobotInterface(robot_name)
        ->getRobotConfig()
        ->selectTcpForceSensor("kw_ftsensor");
#endif

    // Set sensor installation pose
    cli->getRobotInterface(robot_name)
        ->getRobotConfig()
        ->setTcpForceSensorPose(sensor_pose);
    // Set TCP offset
    std::vector<double> tcp_offset = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    cli->getRobotInterface(robot_name)
        ->getRobotConfig()
        ->setTcpOffset(tcp_offset);

    double mass = 0.578721;
    std::vector<double> com = { -0.00661349, 0.00102332, 0.0192072 };
    std::vector<double> force_offset = { 12.9354,  -13.0272, -31.889,
                                         0.186967, -1.06773, 0.385074 };

    // Set payload
    cli->getRobotInterface(robot_name)
        ->getRobotConfig()
        ->setPayload(mass, com, { 0. }, { 0. });

    // Set force sensor offset
    cli->getRobotInterface(robot_name)
        ->getRobotConfig()
        ->setTcpForceOffset(force_offset);
    // User coordinate system
    std::vector<double> usercoord_in_tcp = { 0, 0, 0.04, 0, 0, 0 };
    double bolt_len = 0.010;      // Bolt length 15mm
    double bolt_diameter = 0.006; // Bolt diameter 6mm

//#define TEST
#ifndef TEST
    std::vector<double> q = { 14.93 / 180 * M_PI, -24.07 / 180 * M_PI,
                              53.31 / 180 * M_PI, -11.51 / 180 * M_PI,
                              89.14 / 180 * M_PI, -73.40 / 180 * M_PI };

    cli->getRobotInterface(robot_name)
        ->getMotionControl()
        ->moveJoint(q, 20.0 / 180.0 * M_PI, 20.0 / 180.0 * M_PI, 0.0, 0.0);
    waitArrival(cli->getRobotInterface(robot_name));

    // Two retry opportunities
    for (int i = 0; i < 2; i++) {
        if (locateZ(cli, bolt_len, usercoord_in_tcp) > 0) {
            std::cout << "Bolt located, start searching for hole position in XY plane" << std::endl;
            if (searchXY(cli, usercoord_in_tcp) > 0) {
                std::cout << "Condition met, confirm insertion" << std::endl;
                if (insertDetect(cli, usercoord_in_tcp) > 0) {
                    std::cout << "Insertion detected, start insertion action" << std::endl;
                    if (insertZ(cli, bolt_len, usercoord_in_tcp) < 0) {
                        std::cout << "Insertion timeout, please check if inserted or adjust wait time"
                                  << std::endl;
                    } else {
                        std::cout << "Insertion completed, end program" << std::endl;
                    }
                    return 0;
                } else {
                    std::cout << "Not inserted, possibly outside the hole, return to start"
                              << std::endl;
                    cli->getRobotInterface(robot_name)
                        ->getMotionControl()
                        ->moveJoint(q, 20.0 / 180.0 * M_PI, 20.0 / 180.0 * M_PI,
                                    0.0, 0.0);
                    waitArrival(cli->getRobotInterface(robot_name));
                    continue;
                }
            } else {
                std::cout << "Search timeout, exit search, return to start" << std::endl;
                cli->getRobotInterface(robot_name)
                    ->getMotionControl()
                    ->moveJoint(q, 20.0 / 180.0 * M_PI, 20.0 / 180.0 * M_PI,
                                0.0, 0.0);
                waitArrival(cli->getRobotInterface(robot_name));
                continue;
            }
        } else {
            std::cout << "Search distance reached, bolt not located, confirm insertion"
                      << std::endl;
            if (insertDetect(cli, usercoord_in_tcp) > 0) {
                std::cout << "Insertion detected, start insertion action" << std::endl;
                if (insertZ(cli, 0.6, usercoord_in_tcp) < 0) {
                    std::cout << "Insertion timeout, please check if inserted or adjust wait time"
                              << std::endl;
                } else {
                    std::cout << "Insertion completed, end program" << std::endl;
                }
                return 0;
            } else {
                std::cout << "Not inserted, start XY plane bolt positioning"
                          << std::endl;
                if (locateXY(cli, usercoord_in_tcp) < 0) {
                    std::cout << "Search timeout, bolt not located, end search, return to start"
                              << std::endl;
                    cli->getRobotInterface(robot_name)
                        ->getMotionControl()
                        ->moveJoint(q, 20.0 / 180.0 * M_PI, 20.0 / 180.0 * M_PI,
                                    0.0, 0.0);
                    waitArrival(cli->getRobotInterface(robot_name));
                    continue;
                } else {
                    auto bolt_pose = calBoltPose(
                        cli, bolt_len, 10.0, bolt_diameter, usercoord_in_tcp);
                    std::cout
                        << "Bolt located, position in user coordinate system: " << bolt_pose
                        << std::endl;
                    moveToBoltPose(cli, bolt_pose, usercoord_in_tcp);
                    std::cout << "Moved above the bolt" << std::endl;
                }
            }
        }
    }
#endif
}

#define LOCAL_IP "172.16.3.68"

int main(int argc, char **argv)
{
#ifdef WIN32
    // Set Windows console output code page to UTF-8
    SetConsoleOutputCP(CP_UTF8);
#endif

    auto rpc = std::make_shared<RpcClient>();
    // API call: Set RPC timeout
    rpc->setRequestTimeout(1000);
    // API call: Connect to RPC service
    rpc->connect(LOCAL_IP, 30004);
    // API call: Login
    rpc->login("aubo", "123456");

    auto rtde_cli = std::make_shared<RtdeClient>();
    // API call: Connect to RTDE service
    rtde_cli->connect(LOCAL_IP, 30010);
    // API call: Login
    rtde_cli->login("aubo", "123456");

    // API call: Set topic, force control condition, TCP position, TCP force
    int topic = rtde_cli->setTopic(
        false,
        { "R1_fc_cond_fullfiled", "R1_actual_TCP_pose", "R1_actual_TCP_force" },
        200, 1);
    // API call: Subscribe
    rtde_cli->subscribe(topic, [](InputParser &parser) {
        std::unique_lock<std::mutex> lck(rtde_mtx_);
        cond_fullfiled_ = parser.popBool();
        tcp_pose_ = parser.popVectorDouble();
        tcp_force_ = parser.popVectorDouble();
    });
    example(rpc);
    /* Test force sensor data */
    // tcpSensorTest(rpc);

    return 0;
}
