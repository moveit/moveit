# Migration Notes

API changes in MoveIt! releases

## ROS Melodic

- The move_group capability ``ExecuteTrajectoryServiceCapability`` has been removed in favor of the improved ``ExecuteTrajectoryActionCapability`` capability. Since Indigo, both capabilities were supported. If you still load default capabilities in your ``config/launch/move_group.launch``, you can just remove them from the capabilities parameter. The correct default capabilities will be loaded automatically.

## ROS Kinetic

- In the C++ MoveGroup class the ``plan()`` method returns a ``MoveItErrorCode`` object and not a boolean.
