# Migration Notes

API changes in MoveIt! releases

## ROS Melodic

- Migration to ``tf2`` API.
- Replaced Eigen::Affine3d with Eigen::Isometry3d, which is computationally more efficient.
  Simply find-replace occurences of Affine3d:
  ``find . -iname "*.[hc]*" -print0 | xargs -0 sed -i 's#Affine3#Isometry3#g'``
- The move_group capability ``ExecuteTrajectoryServiceCapability`` has been removed in favor of the improved ``ExecuteTrajectoryActionCapability`` capability. Since Indigo, both capabilities were supported. If you still load default capabilities in your ``config/launch/move_group.launch``, you can just remove them from the capabilities parameter. The correct default capabilities will be loaded automatically.
- Deprecated method ``CurrentStateMonitor::waitForCurrentState(double wait_time)`` was finally removed.
- Renamed ``RobotState::getCollisionBodyTransforms`` to ``getCollisionBodyTransform`` as it returns a single transform only.
- Removed deprecated class MoveGroup (was renamed to MoveGroupInterface).
- KinematicsBase: Deprecated members `tip_frame_`, `search_discretization_`.
  Use `tip_frames_` and `redundant_joint_discretization_` instead.
- KinematicsBase: Deprecated `initialize(robot_description, ...)` in favour of `initialize(robot_model, ...)`.
  Adapt your kinematics plugin to directly receive a `RobotModel`. See the [KDL plugin](https://github.com/ros-planning/moveit/tree/melodic-devel/moveit_kinematics/kdl_kinematics_plugin) for an example.
- IK: Removed notion of IK attempts and redundant random seeding in RobotState::setFromIK(). Number of attempts is limited by timeout only. ([#1288](https://github.com/ros-planning/moveit/pull/1288))
  Remove parameters `kinematics_solver_attempts` from your `kinematics.yaml` config files.
- ``RDFLoader`` / ``RobotModelLoader``: removed TinyXML-based API (https://github.com/ros-planning/moveit/pull/1254)
- Deprecated `EndEffectorInteractionStyle` got removed from `RobotInteraction` (https://github.com/ros-planning/moveit/pull/1287)
  Use [the corresponding `InteractionStyle` definitions](https://github.com/ros-planning/moveit/pull/1287/files#diff-24e57a8ea7f2f2d8a63cfc31580d09ddL240) instead
- `moveit_ros_plannning` no longer depends on `moveit_ros_perception`
- The joint states of `passive` joints must be published in ROS and the CurrentStateMonitor will now wait for them as well. Their semantics dictate that they cannot be actively controlled, but they must be known to use the full robot state in collision checks. (https://github.com/ros-planning/moveit/pull/2663)
- In release 1.0.9 the macro `MOVEIT_VERSION` was renamed to `MOVEIT_VERSION_STR` in favour of `MOVEIT_VERSION` becoming a numeric version identifier suitable for a version check via: `#if MOVEIT_VERSION >= MOVEIT_VERSION_CHECK(1, 0, 9)`.

## ROS Kinetic

- In the C++ MoveGroupInterface class the ``plan()`` method returns a ``MoveItErrorCode`` object and not a boolean.
  `static_cast<bool>(mgi.plan())` can be used to achieve the old behavior.
- ``CurrentStateMonitor::waitForCurrentState(double wait_time)`` has been renamed to ``waitForCompleteState`` to better reflect the actual semantics. Additionally a new method ``waitForCurrentState(const ros::Time t = ros::Time::now())`` was added that actually waits until all joint updates are newer than ``t``.
- To avoid deadlocks, the PlanningSceneMonitor listens to its own EventQueue, monitored by an additional spinner thread.
  Providing a custom NodeHandle, a user can control which EventQueue and processing thread is used instead.
  Providing a default NodeHandle, the old behavior (using the global EventQueue) can be restored, which is however not recommended.
