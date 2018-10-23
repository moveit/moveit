# Migration Notes

API changes in MoveIt! releases

## ROS Melodic

- Migration to ``tf2`` API.
- The move_group capability ``ExecuteTrajectoryServiceCapability`` has been removed in favor of the improved ``ExecuteTrajectoryActionCapability`` capability. Since Indigo, both capabilities were supported. If you still load default capabilities in your ``config/launch/move_group.launch``, you can just remove them from the capabilities parameter. The correct default capabilities will be loaded automatically.
- Deprecated method ``CurrentStateMonitor::waitForCurrentState(double wait_time)`` was finally removed.
- Renamed ``RobotState::getCollisionBodyTransforms`` to ``getCollisionBodyTransform`` as it returns a single transform only.

## ROS Kinetic

- In the C++ MoveGroupInterface class the ``plan()`` method returns a ``MoveItErrorCode`` object and not a boolean.
  `static_cast<bool>(mgi.plan())` can be used to achieve the old behavior.
- ``CurrentStateMonitor::waitForCurrentState(double wait_time)`` has been renamed to ``waitForCompleteState`` to better reflect the actual semantics. Additionally a new method ``waitForCurrentState(const ros::Time t = ros::Time::now())`` was added that actually waits until all joint updates are newer than ``t``.
- To avoid deadlocks, the PlanningSceneMonitor listens to its own EventQueue, monitored by an additional spinner thread.
  Providing a custom NodeHandle, a user can control which EventQueue and processing thread is used instead.
  Providing a default NodeHandle, the old behavior (using the global EventQueue) can be restored, which is however not recommended.
