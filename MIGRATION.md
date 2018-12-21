# Migration Notes

API changes in MoveIt! releases

## ROS Kinetic

- In the C++ MoveGroupInterface class the ``plan()`` method returns a ``MoveItErrorCode`` object and not a boolean.
  `static_cast<bool>(mgi.plan())` can be used to achieve the old behavior.
- ``CurrentStateMonitor::waitForCurrentState(double wait_time)`` has been renamed to ``waitForCompleteState`` to better reflect the actual semantics. Additionally a new method ``waitForCurrentState(const ros::Time t = ros::Time::now())`` was added that actually waits until all joint updates are newer than ``t``.
- To avoid deadlocks, the PlanningSceneMonitor listens to its own EventQueue, monitored by an additional spinner thread.
  Providing a custom NodeHandle, a user can control which EventQueue and processing thread is used instead.
  Providing a default NodeHandle, the old behavior (using the global EventQueue) can be restored, which is however not recommended.
