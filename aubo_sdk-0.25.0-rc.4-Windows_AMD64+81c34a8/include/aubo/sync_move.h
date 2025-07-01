/** @file  sync_move.h
 *  @brief 同步运行
 *
 * 1. Independent movements
 *   If the different task programs, and their robots, work independently, no
 *   synchronization or coordination is needed. Each task program is then
 *   written as if it was the program for a single robot system.
 *
 * 2. Semi coordinated movements
 *   Several robots can work with the same work object, without synchronized
 *   movements, as long as the work object is not moving.
 *   A positioner can move the work object when the robots are not coordinated
 *   to it, and the robots can be coordinated to the work object when it is not
 *   moving. Switching between moving the object and coordinating the robots is
 *   called semi coordinated movements.
 *
 * 3. Coordinated synchronized movements
 *   Several robots can work with the same moving work object.
 *   The positioner or robot that holds the work object and the robots that work
 *   with the work object must have synchronized movements. This means that the
 *   RAPID task programs, that handle one mechanical unit each, execute their
 *   move instructions simultaneously.
 */
#ifndef AUBO_SDK_SYNC_MOVE_INTERFACE_H
#define AUBO_SDK_SYNC_MOVE_INTERFACE_H

#include <vector>
#include <unordered_set>
#include <string>
#include <memory>
#include <aubo/global_config.h>

namespace arcs {
namespace common_interface {

typedef std::unordered_set<std::string> TaskSet;
class ARCS_ABI_EXPORT SyncMove
{
public:
    SyncMove();
    virtual ~SyncMove();

    /**
     * syncMoveOn is used to start synchronized movement mode.
     *
     * A syncMoveOn instruction will wait for the other task programs. When
     * all task programs have reached the syncMoveOn, they will continue
     * their execution in synchronized movement mode. The move instructions
     * in the different task programs are executed simultaneously, until the
     * instruction syncMoveOff is executed.
     * A stop point must be programmed before the syncMoveOn instruction.
     *
     * @param syncident
     * @param taskset
     * @return
     *
     * @par Python函数原型
     * syncMoveOn(self: pyaubo_sdk.SyncMove, arg0: str, arg1: Set[str]) -> int
     *
     * @par Lua函数原型
     * syncMoveOn(syncident: string, taskset: table) -> nil
     * @endcoe
     */
    int syncMoveOn(const std::string &syncident, const TaskSet &taskset);

    /**
     * 设置同步路径段的ID
     * In synchronized movements mode, all or none of the simultaneous move
     * instructions must be programmed with corner zones. This means that the
     * move instructions with the same ID must either all have corner zones, or
     * all have stop points. If a move instruction with a corner zone and a move
     * instruction with a stop point are synchronously executed in their
     * respective task program, an error will occur.
     *
     * Synchronously executed move instructions can have corner zones of
     * different sizes (e.g. one use z10 and one use z50).
     *
     * @param id
     * @return
     *
     * @par Python函数原型
     * syncMoveSegment(self: pyaubo_sdk.SyncMove, arg0: int) -> bool
     *
     * @par Lua函数原型
     * syncMoveSegment(id: number) -> boolean
     * @endcoe
     */
    bool syncMoveSegment(int id);

    /**
     * syncMoveOff is used to end synchronized movement mode.
     *
     * A syncMoveOff instruction will wait for the other task programs. When
     * all task programs have reached the syncMoveOff, they will continue
     * their execution in unsynchronized mode.
     * A stop point must be programmed before the syncMoveOff instruction.
     *
     * @param syncident
     * @return
     *
     * @par Python函数原型
     * syncMoveOff(self: pyaubo_sdk.SyncMove, arg0: str) -> int
     *
     * @par Lua函数原型
     * syncMoveOff(syncident: string) -> nil
     * @endcoe
     */
    int syncMoveOff(const std::string &syncident);

    /**
     * syncMoveUndo is used to turn off synchronized movements, even if not
     * all the other task programs execute the syncMoveUndo instruction.
     *
     * syncMoveUndo is intended for UNDO handlers. When the program
     * pointer is moved from the procedure, syncMoveUndo is used to turn off
     * the synchronization.
     *
     * @return
     *
     * @par Python函数原型
     * syncMoveUndo(self: pyaubo_sdk.SyncMove) -> int
     *
     * @par Lua函数原型
     * syncMoveUndo() -> nil
     * @endcoe
     */
    int syncMoveUndo();

    /**
     * waitSyncTasks used to synchronize several task programs at a special
     * point in the program.
     *
     * A waitSyncTasks instruction will wait for the other
     * task programs. When all task programs have reached the waitSyncTasks
     * instruction, they will continue their execution.
     *
     * @param syncident
     * @param taskset
     * @return
     *
     * @par Python函数原型
     * waitSyncTasks(self: pyaubo_sdk.SyncMove, arg0: str, arg1: Set[str]) ->
     * int
     *
     * @par Lua函数原型
     * waitSyncTasks(syncident: string, taskset: table) -> nil
     * @endcoe
     */
    int waitSyncTasks(const std::string &syncident, const TaskSet &taskset);

    /**
     * isSyncMoveOn is used to tell if the mechanical unit group is in synchron-
     * ized movement mode.
     *
     * A task that does not control any mechanical unit can find out if the
     * mechanical units defined in the parameter Use Mechanical Unit Group are
     * in synchronized movement mode.
     *
     * @return
     *
     * @par Python函数原型
     * isSyncMoveOn(self: pyaubo_sdk.SyncMove) -> bool
     *
     * @par Lua函数原型
     * isSyncMoveOn() -> boolean
     * @endcoe
     */
    bool isSyncMoveOn();

    /**
     *
     * @return
     *
     * @par Python函数原型
     * syncMoveSuspend(self: pyaubo_sdk.SyncMove) -> int
     *
     * @par Lua函数原型
     * syncMoveSuspend() -> nil
     * @endcoe
     */
    int syncMoveSuspend();

    /**
     *
     * @return
     *
     * @par Python函数原型
     * syncMoveResume(self: pyaubo_sdk.SyncMove) -> int
     *
     * @par Lua函数原型
     * syncMoveResume() -> nil
     * @endcoe
     */
    int syncMoveResume();

    /**
     * Add a frame with the name name initialized at the specified pose
     * expressed in the ref_frame coordinate frame. This command only adds a
     * frame to the world, it does not attach it to the ref_frame coordinate
     * frame. Use frameAttach() to attach the newly added frame to ref_frame if
     * desired.
     *
     * @param name: name of the frame to be added. The name must not be the same
     * as any existing world model object (frame, axis, or axis group),
     * otherwise an exception is thrown
     * @param pose: initial pose of the new object
     * @param ref_frame: name of the world model object whose coordinate frame
     * the pose is expressed in. If nothing is provided here, the default is the
     * robot “base” frame.
     *
     * @return
     */
    int frameAdd(const std::string &name, const std::vector<double> &pose,
                 const std::string &ref_name);

    /**
     * Attaches the child frame to the parent world model object. The relative
     * transform between the parent and child will be set such that the child
     * does not move in the world when the attachment occurs.
     *
     * The child cannot be “world”, “flange”, “tcp”, or the same as parent.
     *
     * This will fail if child or parent is not an existing frame, or this makes
     * the attachments form a closed chain.
     *
     * If being used with the MotionPlus, the parent argument can be the name of
     * an external axis or axis group.
     *
     * @param child: name of the frame to be attached. The name must not be
     * “world”, “flange”, or “tcp”.
     * @param parent: name of the object that the child frame will be attached
     * to.
     *
     * @return
     */
    int frameAttach(const std::string &child, const std::string &parent);

    /**
     * Delete all frames that have been added to the world model.
     *
     * The “world”, “base”, “flange”, and “tcp” frames cannot be deleted.
     *
     * Any frames that are attached to the deleted frames will be attached to
     * the “world” frame with new frame offsets set such that the detached
     * frames do not move in the world.
     *
     * @return
     */
    int frameDeleteAll();

    /**
     * Delete the frame named frame from the world model.
     *
     * The “world”, “base”, “flange”, and “tcp” frames cannot be deleted.
     *
     * Any frames that are attached to the deleted frame will be attached to the
     * “world” frame with new frame offsets set such that the detached frame
     * does not move in the world.
     *
     * This command will fail if the frame does not exist.
     *
     * @param name: name of the frame to be deleted
     *
     * @return
     */
    int frameDelete(const std::string &name);

    /**
     * Changes the placement of the coordinate frame named name to the new
     * placement given by pose that is defined in the ref_name coordinate frame.
     *
     * This will fail if name is “world”, “flange”, “tcp”, or if the frame does
     * not exist. Note: to move the “tcp” frame, use the set_tcp() command
     * instead.
     *
     * If being used with the MotionPlus, the ref_name argument can be the name
     * of an external axis or axis group.
     *
     * @param name: the name of the frame to move
     * @param pose: the new placement
     * @param ref_name: the coordinate frame that pose is expressed in. The
     * default value is the robot’s “base” frame.
     *
     * @return
     */
    int frameMove(const std::string &name, const std::vector<double> &pose,
                  const std::string &ref_name);

    /**
     * Get the pose of the name frame relative to the rel_frame frame but
     * expressed in the coordinates of the ref_frame frame. If ref_frame is not
     * provided, then this returns the pose of the name frame relative to and
     * expressed in the same frame as rel_frame.
     *
     * This will fail if any arguments are not an existing frame.
     *
     * If being used with MotionPlus, all three arguments can also be the names
     * of external axes or axis groups.
     *
     * @param name: name of the frame to query.
     * @param rel_frame: short for “relative frame” is the frame where the pose
     * is computed relative to
     * @param ref_frame: short for “reference frame” is the frame to express the
     * coordinates of resulting relative pose in. If this is not provided, then
     * it will default to match the value of rel_frame.
     *
     * @return The pose of the frame expressed in the ref_frame coordinates.
     */
    std::vector<double> frameGetPose(const std::string &name,
                                     const std::string &rel_frame,
                                     const std::string &ref_frame);

    /**
     * Convert pose from from_frame to to_frame.
     *
     * This will fail if either coordinate system argument is not an existing
     * frame.
     *
     * If being used with MotionPlus, all three arguments can also be the names
     * of external axes or axis groups.
     *
     * @param pose: pose to be converted
     * @param from_frame: name of reference frame at origin of old coordinate
     * system
     * @param to_frame: name of reference frame at origin of new coordinate
     * system
     *
     * @return Value of pose expressed in the coordinates of to_frame.
     */
    std::vector<double> frameConvertPose(const std::vector<double> &pose,
                                         const std::string &from_frame,
                                         const std::string &to_frame);

    /**
     * Queries for the existence of a frame by the given name.
     *
     * @param name: name of the frame to be queried.
     *
     * @return Returns true if there is a frame by the given name, false if not.
     */
    bool frameExist(const std::string &name);

    /**
     * Get the parent of the frame named name in the world model.
     *
     * If the frame is not attached to another frame, then “world” is the
     * parent.
     *
     * @return name: the frame being queried
     *
     * @return name of the parent as a string
     */
    std::string frameGetParent(const std::string &name);

    /**
     * Returns a list of immediate child object frame names. Parent-child
     * relationships are defined by wold model attachments. If being used with
     * MotionPlus, the child objects may also be an axis group or an axis.
     *
     * @param name: the name of the parent object.
     *
     * @return a list of immediate child object frame names
     */
    std::vector<std::string> frameGetChildren(const std::string &name);

    /**
     * Adds a new axis group with the given name to the world model. It is
     * placed at the given pose in the reference coordinate frame defined by
     * ref_frame.
     *
     * An axis group can only be attached to the world coordinate frame.
     *
     * Each axis group has a coordinate frame attached to its base, which can be
     * used as an argument to other world model functions by referring the name
     * of the group.
     *
     * At most 6 axis groups can be added to the world model.
     *
     * @param name: (string) Name of the axis group to add. The name cannot be
     * an empty string. Names used by world model objects (e.g., frame, axis
     * group, axis, etc.) must be unique.
     *
     * @param pose: (pose) Pose of the axis group’s base, in the reference
     * coordinate frame.
     *
     * @param ref_frame (optional): (string) Name of the reference coordinate
     * frame that pose is defined in. This can be any world model entity with a
     * coordinate system (e.g., frame, axis group, axis, etc.). The default
     * value "base" refers to the robot’s base frame.
     *
     * @return
     */
    int axisGroupAdd(const std::string &name, const std::vector<double> &pose,
                     const std::string &ref_frame);

    /**
     * Deletes the axis group with the given name from the world model.
     *
     * All attached axes are also disabled (if live) and deleted.
     *
     * This function will fail, if this axis group is under control by another
     * URScript function.
     *
     * @param name: (string) Name of the axis group to delete. Axis group with
     * such name must exist.
     *
     * @return
     */
    int axisGroupDelete(const std::string &name);

    /**
     * Adds an external axis with the given name to the axis group named
     * group_name. The axis is attached at the given pose in the reference
     * coordinate frame defined by parent when it axis position is 0. The type,
     * max velocity, max acceleration, position limits, and index of this axis
     * are defined by type, v_limit, a_limit, q_limits, and axis_index,
     * respectively.
     *
     * The pose parameter is typically obtained from a calibration process when
     * the external axis is commissioned. See here for a guide on a basic
     * routine for calibrating a single rotary axis.
     *
     * This function will fail, if this axis group is under control of another
     * URScript function.
     * This function will fail, if the kinematic chain created by the attachment
     * forms a closed chain.
     *
     * @param group_name: (string) Name of the axis group this new axis is added
     * to. The axis group would have been created using axis_group_add(). Axis
     * group with such name must exist.
     *
     * @param name: (string) Name of the new axis. The name cannot be an empty
     * string. Names used by world model objects (e.g., frame, axis group, axis,
     * etc.) must be unique.
     *
     * @param parent: (string) Name of the parent axis. If it’s empty or the
     * same as group_name, the new axis will be attached to the base of the axis
     * group. Axis with such name must exist in the axis group.
     *
     * @param pose: (pose) the zero-position pose, in the parent coordinate
     * frame, this axis will be placed and attached to. This is the pose the
     * axis will be (relative to its parent) when its axis position is 0. If
     * type is 0 (rotary), then the z axis of the frame corresponds to the axis
     * of rotation. If type is 1 (linear), then the z axis is the axis of
     * translation.
     */
    int axisGroupAddAxis(const std::string &group_name, const std::string &name,
                         const std::string &parent,
                         const std::vector<double> &pose);

    /**
     * Updates the corresponding properties of axis with name. The pose
     * parameter is typically obtained from a calibration process when the
     * external axis is commissioned. See here for a guide on a basic routine
     * for calibrating a single rotary axis.
     *
     * This function will fail, if the axis group the axis attached to is
     * already being controlled by another URScript command.
     * This function will fail, if any attached axis of the axis group is live
     * and enabled.
     *
     * @param name: (string) Name of the axis to update. Axis with such name
     * must exist.
     *
     * @param pose (optional): (pose) New zero-position pose, in the coordinate
     * frame of the parent axis (or axis group), of the axis. This is the pose
     * of the axis when its axis position is 0.
     */
    int axisGroupUpdateAxis(const std::string &name,
                            const std::vector<double> &pose);

    /**
     * Returns the index of the axis with given axis_name in the RTDE target
     * positions and actual positions arrays.
     *
     * @param axis_name: (string) Name of the axis in query.
     * Axis with such name must exist.
     *
     * @return integer: Index of the axis in the RTDE target positions and
     * actual positions arrays.
     */
    int axisGroupGetAxisIndex(const std::string &name);

    /**
     * Returns the name of the axis with the given axis_index.
     *
     * @param  axis_index: (integer) Index of the axis in query.
     * Axis with such index must exist.
     *
     * @eturn string: Name of the axis.
     */
    std::string axisGroupGetAxisName(int index);

    /**
     * Returns the current target positions of the axis group with group_name.
     * If group_name is not provided, the target positions of all external axes
     * will be returned.
     *
     * This function will fail, if the external axis bus is disabled.
     *
     * @param group_name (optional): (string) Name of the axis group in query.
     * Axis group with such name must REALLY exist.
     *
     * @return Double[]: Target positions of the involved axes, in the order of
     * their external axis indices.
     */
    std::vector<double> axisGroupGetTargetPositions(
        const std::string &group_name);

    /**
     * Returns the current actual positions of the axis group with group_name.
     * If group_name is not provided, the actual positions of all external axes
     * will be returned.
     *
     * This function will fail, if the external axis bus is disabled.
     *
     * @param group_name (optional): (string) Name of the axis group in query.
     * Axis group with such name must exist.
     *
     * @return Double[]: Actual positions of the involved axes, in the order of
     * their external axis indices.
     */
    std::vector<double> axisGroupGetActualPositions(
        const std::string &group_name);

    /**
     * Shifts the target and actual positions of the axis group group_name by
     * the given offset.
     *
     * This is a software shift that happens in the controller only, it does not
     * affect external axis drives. The shift is also applied to any streamed
     * target and actual positions published on RTDE.
     *
     * @param group_name: (string) Name of the axis group to apply the offset
     * positions to. Axis group with such name must exist.
     *
     * @param offset: (float[]) Offsets that the target and actual positions
     * should be shifted by. The size of offset must match the number of axes
     * attached to the given group.
     *
     * @return
     */
    int axisGroupOffsetPositions(const std::string &group_name,
                                 const std::vector<double> &offset);

    /**
     * Moves the axes of axis group named group_name to new positions q, using a
     * trapezoidal velocity profile. Factor a specifying the percentage of the
     * max profile accelerations out of the acceleration limits of each axes.
     * Factor v specifying the percentage of the max profile velocities out of
     * the velocity limits of each axes.
     *
     * The actual accelerations and velocities are determined by the most
     * constraining axis, so that all the axes complete the acceleration,
     * cruise, and deceleration phases at the same time.
     *
     * @param group_name: (string) Name of the axis group to move.
     * Axis group with such name must exist.
     *
     * @param q: (float[]) Target positions in rad (rotary) or in m (linear). If
     * the target exceeds the position limits, then it is set to the nearest
     * limit. The involved axes are ordered increasingly by their axis indices.
     * The size of q must match the number of axes attached to the given group.
     *
     * @param a: (float) Factor specifying the max accelerations of this move
     * out of the acceleration limits. a must be in range of (0,1].
     *
     * @param v: (float) Factor specifying the max velocities of this move out
     * of the velocity limits. v must be in range of (0,1].
     *
     * Return: n/a
     */
    int axisGroupMoveJoint(const std::string &group_name,
                           const std::vector<double> &q, double a, double v);

    /**
     * Accelerates the axes of axis group named group_name up to the target
     * velocities qd. Factor a specifying the percentage of the max
     * accelerations out of the acceleration limits of each axes. The function
     * will run for a period of t seconds.
     *
     * @param group_name: (string) Name of the external axis group to control.
     * Axis group with such name must exist.
     *
     * @param qd: (float[]) Target velocities for the axes in the axis group. If
     * the target exceeds the velocity limits, then it is set to the limit. The
     * involved axes are ordered increasingly by their axis indices. The size of
     * qd must match the number of axes attached to the given group.
     *
     * @param a: (float) Factor specifying the max accelerations of this move
     * out of the acceleration limits. a must be in range of (0,1].
     *
     * @param t (optional): (float) Duration in seconds before the function
     * returns. If t < 0, then the function will return when the target
     * velocities are reached. if t ≥ 0, then the function will return after
     * this duration, regardless of what the achieved axes velocities are.
     */
    int axisGroupSpeedJoint(const std::string &group_name,
                            const std::vector<double> &qd, double a, double t);

protected:
    void *d_;
};

using SyncMovePtr = std::shared_ptr<SyncMove>;
} // namespace common_interface
} // namespace arcs
#endif // AUBO_SDK_SYNC_MOVE_INTERFACE_H
