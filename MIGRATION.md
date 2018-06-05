# Migration Notes

API changes in MoveIt! releases

## ROS Kinetic

- In the C++ MoveGroupInterface class the ``plan()`` method returns a ``MoveItErrorCode`` object and not a boolean.
  `static_cast<bool>(mgi.plan())` can be used to achieve the old behavior.
