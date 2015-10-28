# Overview

This package provides plugins of base class `moveit_controller_manager::MoveItControllerManager` and a new plugin base class for `moveit_controller_manager::MoveItControllerHandle` allocators.
The allocator class is necessary because `moveit_controller_manager::MoveItControllerHandle` need a name passed to constructor.

# moveit_ros_control_interface::MoveItControllerManager
This plugin intefaces a single ros_control-driven node in the namespace given in the `~ros_control_namespace` ROS parameter.
It polls all controller via the `list_controllers` and passes their properties to MoveIt!.
The polling is throttled to 1 Hertz.

## Handle plugins
The actual handle creation is delegated to allocator plugin of base class `moveit_ros_control_interface::ControllerHandleAllocator`.
These plugins should be registered with lookup names that match the corresponding controller types.

Currently plugins for `position_controllers/JointTrajectoryController`, `velocity_controllers/JointTrajectoryController` and `effort_controllers/JointTrajectoryController` are available, which simple wrap `moveit_simple_controller_manager::FollowJointTrajectoryControllerHandle` instances.

## Controller switching
Moveit! can decide which controllers have to be started and stopped.
Since only controller names with registered allocator plugins in are handed over to MoveIt!, this implementation takes care of stopping other controllers based on their claimed resources and the resources for the to-be-started controlles.

## Namespaces
All controller names get prefixed by the namespace of the ros_control node.
For this to work the controller names should not contain slashes. This is a strict requirement if the ros_control  namespace is `/`.

# moveit_ros_control_interface::MoveItMultiControllerManager

This plugin does not need further configuration. It polls the ROS master for services and identifies ros_control nodes automatically.
It spawns `moveit_ros_control_interface::MoveItControllerManager` instances with their namespace and takes cares of proper delegation.
