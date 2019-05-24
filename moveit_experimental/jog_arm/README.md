## Jog Arm

#### Quick Start Guide

Clone https://github.com/ros-industrial/universal_robot.git        (kinetic-devel branch is fine)

`roslaunch ur_gazebo ur5.launch`

`roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true`

`roslaunch ur5_moveit_config moveit_rviz.launch`

In RViz, "plan and execute" a motion to a non-singular position (not all zero joint angles) that is not close to a joint limit.

`rosservice call /controller_manager/switch_controller "start_controllers:`
`- 'joint_group_position_controller'`
`stop_controllers:`
`- 'arm_controller'`
`strictness: 2"`

`roslaunch jog_arm spacenav_cpp.launch`

`rostopic pub -r 100 /jog_arm_server/delta_jog_cmds geometry_msgs/TwistStamped "header: auto`
`twist:`
`  linear:`
`    x: 0.0`
`    y: 0.01`
`    z: -0.01`
`  angular:`
`    x: 0.0`
`    y: 0.0`
`    z: 0.0"`

If you see a warning about "close to singularity", try changing the direction of motion.

#### Running Tests

Run tests from the jog\_arm folder with `catkin run_tests --no-deps --this`

