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
// Implement blocking: The program proceeds only after the robot reaches the target waypoint
int waitArrival(RobotInterfacePtr impl)
{
    // API call: Get the current motion command ID
    int exec_id = impl->getMotionControl()->getExecId();

    int cnt = 0;
    // Maximum retry count to get exec_id while waiting for the robot to start moving
    int max_retry_count = 50;

    // Wait for the robot to start moving
    while (exec_id == -1) {
        if (cnt++ > max_retry_count) {
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        exec_id = impl->getMotionControl()->getExecId();
    }

    // Wait for the robot to finish moving
    while (exec_id != -1) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        exec_id = impl->getMotionControl()->getExecId();
    }

    return 0;
}

inline bool equal(const std::vector<double> &q1, const std::vector<double> &q2,
                  double eps = 0.001)
{
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
 * @param timeout Time limit. If time exceeds timeout and condition is not met, return -1;
 *        If timeout=-1, no time limit is applied
 * @param target_pose If the robot reaches the target point but the condition is not met, return 0;
 *        If target_pose={}, target point is not checked
 *
 * @return Returns 1 if condition is met; 0 if at position but condition not met; -1 if timeout;
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
            std::cout << "Robot is in position!" << std::endl;
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
 * Determines if the change in Z direction force exceeds a threshold using setCondForce
 * @param cli
 */
int locateZ(RpcClientPtr cli)
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
    // Search distance
    std::vector<double> seeking_distance = { 0.0, 0.0, -0.015, 0.0, 0.0, 0.0 };
    // Force control coordinate system selection: base frame
    std::vector<double> feature = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    TaskFrameType frame_type = TaskFrameType::NONE;
    // Force control speed limits
    std::vector<double> speed_limits(6, 2.0);
    // End effector force condition, INF is a very large value, [-INF,INF] means ignore monitoring for that direction
    std::vector<double> min_force = { -INF, -INF, -INF, -INF, -INF, -INF };
    std::vector<double> max_force = { INF, INF, 5.0, INF, INF, INF };

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
    // Set force monitoring condition
    double timeout = 1000;
    cli->getRobotInterface(robot_name)
        ->getForceControl()
        ->setCondForce(min_force, max_force, true, timeout);
    // Get current TCP position
    auto current_pose =
        cli->getRobotInterface(robot_name)->getRobotState()->getTcpPose();
    // Calculate search end position
    std::vector<double> target_pose(6, 0.);
    for (int i = 0; i < 6; i++) {
        target_pose[i] = current_pose[i] + seeking_distance[i];
    }

    // Linear motion
    double v = 0.0025, a = 4.0;
    cli->getRobotInterface(robot_name)
        ->getMotionControl()
        ->moveLine(target_pose, a, v, 0, 0);
    // Enable force control
    cli->getRobotInterface(robot_name)->getForceControl()->fcEnable();
    cli->getRobotInterface(robot_name)->getForceControl()->setCondActive();
    auto ret = condFullfiled(-1, target_pose);
    cli->getRobotInterface(robot_name)->getMotionControl()->stopJoint(1);
    // Disable force control
    cli->getRobotInterface(robot_name)->getForceControl()->fcDisable();

    return ret;
}

/**
 * @brief insertDetect Detect whether the bolt is inserted into the hole; move forward and backward twice, both contacts mean inserted
 * @param cli
 * @return
 */
int insertDetect(RpcClientPtr cli)
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
    // Force control coordinate system selection: base frame
    std::vector<double> feature = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
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
    double timeout = -1;
    double count = 3.;
    double outside = 1;
    std::vector<double> args;
    args.insert(args.end(), min_force);
    args.insert(args.end(), max_force);
    args.insert(args.end(), select.begin(), select.end());
    args.insert(args.end(), count);
    args.insert(args.end(), outside);
    cli->getRobotInterface(robot_name)
        ->getForceControl()
        ->setCondAdvanced("ConditionContactForce", args, timeout);

    // Enable force control
    cli->getRobotInterface(robot_name)->getForceControl()->fcEnable();
    cli->getRobotInterface(robot_name)->getForceControl()->setCondActive();
    auto ret = condFullfiled(3, {});
    if (ret < 0) {
        cli->getRobotInterface(robot_name)->getMotionControl()->stopJoint(1);
        // Disable force control
        cli->getRobotInterface(robot_name)->getForceControl()->fcDisable();
        return ret;
    } else {
        std::cout << "Insertion detection: Forward contact successful!" << std::endl;
    }
    // Target force
    insert_detect_goal_wrench = { 8.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    // Start reverse contact
    cli->getRobotInterface(robot_name)
        ->getForceControl()
        ->setTargetForce(feature, insert_detect_enable,
                         insert_detect_goal_wrench, speed_limits, frame_type);
    ret = condFullfiled(3, {});
    if (ret < 0) {
        std::cout << "Insertion failed, insertion detection finished!" << std::endl;
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
int locateXY(RpcClientPtr cli)
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

    // Force control coordinate system selection: base frame
    std::vector<double> feature = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    TaskFrameType frame_type = TaskFrameType::NONE;
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
    double max_force = 8.0;
    std::vector<double> select = { 1., 0., 0., 0., 0., 0. };
    double timeout = -1;
    double count = 1.;
    double outside = 1;
    std::vector<double> args_x;
    args_x.insert(args_x.end(), min_force);
    args_x.insert(args_x.end(), max_force);
    args_x.insert(args_x.end(), select.begin(), select.end());
    args_x.insert(args_x.end(), count);
    args_x.insert(args_x.end(), outside);
    cli->getRobotInterface(robot_name)
        ->getForceControl()
        ->setCondAdvanced("ConditionContactForce", args_x, timeout);

    select = { 0., 1., 0., 0., 0., 0. };
    std::vector<double> args_y;
    args_y.insert(args_y.end(), min_force);
    args_y.insert(args_y.end(), max_force);
    args_y.insert(args_y.end(), select.begin(), select.end());
    args_y.insert(args_y.end(), count);
    args_y.insert(args_y.end(), outside);
    cli->getRobotInterface(robot_name)
        ->getForceControl()
        ->setCondAdvanced("ConditionContactForce", args_y, timeout);

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
 * @brief calBoltPose Calculate bolt position
 * @param cli
 * @param bolt_len  Bolt length
 * @return
 */
std::vector<double> calBoltPose(RpcClientPtr cli, double bolt_len)
{
    double max_force = 8.0;
    double diameter = 0.008; // Bolt diameter
    auto robot_name = cli->getRobotNames().front();
    auto tcp_wrench =
        cli->getRobotInterface(robot_name)->getRobotState()->getTcpForce();
    std::vector<double> bolt_pose =
        cli->getRobotInterface(robot_name)->getRobotState()->getTcpPose();

    if ((fabs(tcp_wrench[0]) > max_force) &&
        (fabs(tcp_wrench[1]) < max_force)) {
        // X direction force > limit, contact, Y direction < limit, move in X direction
        bolt_pose[0] =
            bolt_pose[0] - (tcp_wrench[0] / fabs(tcp_wrench[0])) * diameter;

    }

    else if ((fabs(tcp_wrench[0]) < max_force) &&
             (fabs(tcp_wrench[1]) > max_force)) {
        // X direction force < limit, Y direction > limit, move in Y direction
        bolt_pose[1] =
            bolt_pose[1] - (tcp_wrench[1] / fabs(tcp_wrench[1])) * diameter;

    } else if ((fabs(tcp_wrench[0]) > max_force) &&
               (fabs(tcp_wrench[1]) > max_force)) {
        // X direction force > limit, Y direction > limit, move in XY composite direction
        double contact_force = std::sqrt(tcp_wrench[0] * tcp_wrench[0] +
                                         tcp_wrench[1] * tcp_wrench[1]);
        bolt_pose[0] =
            bolt_pose[0] - (tcp_wrench[0] / contact_force) * diameter;
        bolt_pose[1] =
            bolt_pose[1] - (tcp_wrench[1] / contact_force) * diameter;
    }
    bolt_pose[2] = bolt_pose[2] + bolt_len;
    return bolt_pose;
}

/**
 * @brief moveToBoltPose Move above the bolt
 * @param cli
 * @param bolt_pose Bolt position
 * @param bolt_len  Bolt length
 * @return
 */
int moveToBoltPose(RpcClientPtr cli, std::vector<double> &bolt_pose,
                   double bolt_len)

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

    // Force control coordinate system selection: base frame
    std::vector<double> feature = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    TaskFrameType frame_type = TaskFrameType::NONE;
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
    double v = 0.02, a = 4.0;
    std::vector<double> trans_pose =
        cli->getRobotInterface(robot_name)->getRobotState()->getTcpPose();
    trans_pose[2] = trans_pose[2] + bolt_len;
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
        ->moveLine(bolt_pose, a, v, 0, 0);
    waitArrival(cli->getRobotInterface(robot_name));
    cli->getRobotInterface(robot_name)->getMotionControl()->stopJoint(1);
    // Disable force control
    cli->getRobotInterface(robot_name)->getForceControl()->fcDisable();
    return 0;
}

/**
 * @brief seekZ Z direction search, move down above the bolt
 * @return
 */
/*
int seekZ(RpcClientPtr cli)
{
    std::vector<double> seek_M = { 30.0, 30.0, 30.0, 10.0, 10.0, 10.0 };
    std::vector<double> seek_D = { 300.0, 300.0, 500.0, 100.0, 100.0, 100.0 };
    std::vector<double> seek_K = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    std::vector<bool> seek_fc_enable = {
        false, false, true, false, false, false
    };
    std::vector<double> seek_goal_wrench = { 0.0, 0.0, 5.0, 0.0, 0.0, 0.0 };
    std::vector<double> seek_force_threshold = { 1.0, 1.0, 1.0, 0.1, 0.1, 0.1 };

    // Force control coordinate system selection: base frame
    std::vector<double> feature = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    TaskFrameType frame_type = TaskFrameType::NONE;
    // Force control speed limits
    std::vector<double> speed_limits(6, 2.0);

    auto robot_name = cli->getRobotNames().front();
    // Set force control parameters
    cli->getRobotInterface(robot_name)
        ->getForceControl()
        ->setDynamicModel(seek_M, seek_D, seek_K);

    // Set target force
    cli->getRobotInterface(robot_name)
        ->getForceControl()
        ->setTargetForce(feature, seek_fc_enable, seek_goal_wrench,
                         speed_limits, frame_type);

    double min_force = -INF;
    double max_force = fabs(seek_goal_wrench[2]) - 2.0;
    std::vector<double> select = { 0., 0., 1., 0., 0., 0. };
    double timeout = -1;
    double count = 5.;
    double outside = 1;
    std::vector<double> args;
    args.insert(args.end(), min_force);
    args.insert(args.end(), max_force);
    args.insert(args.end(), select.begin(), select.end());
    args.insert(args.end(), count);
    args.insert(args.end(), outside);
    cli->getRobotInterface(robot_name)
        ->getForceControl()
        ->setCondAdvanced("ConditionContactForce", args, timeout);
    cli->getRobotInterface(robot_name)->getForceControl()->fcEnable();
    // Activate termination condition
    cli->getRobotInterface(robot_name)->getForceControl()->setCondActive();
    auto ret = condFullfiled(5, {});
    if (ret < 0) {
        std::cout << "Z direction search failed, bolt not found!" << std::endl;
    } else {
        std::cout << "Z direction search successful!" << std::endl;
    }
    cli->getRobotInterface(robot_name)->getMotionControl()->stopJoint(1);
    // Disable force control
    cli->getRobotInterface(robot_name)->getForceControl()->fcDisable();
    return ret;
}
*/

int searchXY(RpcClientPtr cli)
{
    std::vector<double> search_M = { 30.0, 30.0, 30.0, 10.0, 10.0, 10.0 };
    std::vector<double> search_D = { 600.0, 500.0, 500.0, 200.0, 200.0, 200.0 };
    std::vector<double> search_K = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    std::vector<bool> search_fc_enable = {
        false, false, true, false, false, false,
    };
    std::vector<double> search_goal_wrench = { 0.0, 0.0, 8.5, 0.0, 0.0, 0.0 };

    // Force control coordinate system selection: base frame
    std::vector<double> feature = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    TaskFrameType frame_type = TaskFrameType::NONE;
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

    double min_force = -INF;
    double max_force = 4.5;
    std::vector<double> select = { 0., 0., 1., 0., 0., 0. };
    double timeout = -1;
    double count = 3.;
    double outside = 0;
    std::vector<double> args_x;
    args_x.insert(args_x.end(), min_force);
    args_x.insert(args_x.end(), max_force);
    args_x.insert(args_x.end(), select.begin(), select.end());
    args_x.insert(args_x.end(), count);
    args_x.insert(args_x.end(), outside);
    // Z direction force suddenly decreases, considered as finding the screw hole
    cli->getRobotInterface(robot_name)
        ->getForceControl()
        ->setCondAdvanced("ConditionContactForce", args_x, timeout);
    // Move down 2mm, considered as finding the screw hole
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

int insertZ(RpcClientPtr cli)
{
    std::vector<double> insert_M = { 30.0, 30.0, 30.0, 5.0, 5.0, 5.0 };
    std::vector<double> insert_D = { 600.0, 600.0, 600.0, 100.0, 100.0, 100.0 };
    std::vector<double> insert_K = { 10.0, 10.0, 500.0, 1.0, 1.0, 400.0 };
    std::vector<bool> insert_fc_enable = { true, true, true, true, true, true };
    std::vector<double> insert_goal_wrench = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    std::vector<double> insert_depth = { 0.0, 0.0, -0.01, 0.0, 0.0, 0.0 };
    // Force control coordinate system selection: base frame
    std::vector<double> feature = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    TaskFrameType frame_type = TaskFrameType::NONE;
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
    // Get current TCP position
    auto current_pose =
        cli->getRobotInterface(robot_name)->getRobotState()->getTcpPose();

    std::vector<double> cylinder_center = { current_pose[0], current_pose[1],
                                            current_pose[2] + insert_depth[2] };

    double radius_ = 1000.;
    cli->getRobotInterface(robot_name)
        ->getForceControl()
        ->setCondCylinder(cylinder_center, radius_, true, -1);
    // Calculate insertion end position
    std::vector<double> target_pose(6, 0.);
    for (int i = 0; i < 6; i++) {
        target_pose[i] = current_pose[i] + insert_depth[i];
    }
    // Linear motion
    double v = 0.0025, a = 4.0;
    cli->getRobotInterface(robot_name)
        ->getMotionControl()
        ->moveLine(target_pose, a, v, 0, 0);

    // Enable force control
    cli->getRobotInterface(robot_name)->getForceControl()->fcEnable();
    // Activate termination condition
    cli->getRobotInterface(robot_name)->getForceControl()->setCondActive();

    auto ret = condFullfiled(5, target_pose);

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
    std::vector<double> tcp_pose = { 0, 0, 0.04, 0, 0, 0 };
    cli->getRobotInterface(robot_name)
        ->getRobotConfig()
        ->setTcpOffset(tcp_pose);

    double mass = 0.542767;
    std::vector<double> com = { 0.00650296, -0.000310792, 0.0199925 };
    // Force sensor offset needs to be set according to actual situation
    std::vector<double> force_offset = { 13.0463,  -11.9162, -33.9405,
                                         0.168684, -1.07947, 0.403233 };
    // Set payload
    cli->getRobotInterface(robot_name)
        ->getRobotConfig()
        ->setPayload(mass, com, { 0. }, { 0. });

    // Set force sensor offset
    cli->getRobotInterface(robot_name)
        ->getRobotConfig()
        ->setTcpForceOffset(force_offset);

    std::vector<double> q = { -22.74 / 180 * M_PI, 1.52 / 180 * M_PI,
                              82.7 / 180 * M_PI,   -8.6 / 180 * M_PI,
                              93.42 / 180 * M_PI,  112.02 / 180 * M_PI };

    cli->getRobotInterface(robot_name)
        ->getMotionControl()
        ->moveJoint(q, 20.0 / 180.0 * M_PI, 20.0 / 180.0 * M_PI, 0.0, 0.0);
    waitArrival(cli->getRobotInterface(robot_name));

    // Two retry opportunities
    for (int i = 0; i < 2; i++) {
        if (locateZ(cli) > 0) {
            std::cout << "Bolt located, start searching for screw hole position in XY plane" << std::endl;
            if (searchXY(cli) > 0) {
                std::cout << "Condition met, confirm insertion" << std::endl;
                if (insertDetect(cli) > 0) {
                    std::cout << "Insertion detected, start insertion action" << std::endl;
                    if (insertZ(cli) < 0) {
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
            if (insertDetect(cli) > 0) {
                std::cout << "Insertion detected, start insertion action" << std::endl;
                if (insertZ(cli) < 0) {
                    std::cout << "Insertion timeout, please check if inserted or adjust wait time"
                              << std::endl;
                } else {
                    std::cout << "Insertion completed, end program" << std::endl;
                }
                return 0;
            } else {
                std::cout << "Not inserted, start XY plane bolt positioning"
                          << std::endl;
                if (locateXY(cli) < 0) {
                    std::cout << "Search timeout, bolt not located, end search, return to start"
                              << std::endl;
                    cli->getRobotInterface(robot_name)
                        ->getMotionControl()
                        ->moveJoint(q, 20.0 / 180.0 * M_PI, 20.0 / 180.0 * M_PI,
                                    0.0, 0.0);
                    waitArrival(cli->getRobotInterface(robot_name));
                    continue;
                } else {
                    auto bolt_pose = calBoltPose(cli, 0.015);
                    std::cout << "Bolt located, position: " << bolt_pose
                              << std::endl;
                    moveToBoltPose(cli, bolt_pose, 0.015);
                    std::cout << "Moved above the bolt" << std::endl;
                }
            }
        }
    }
}
