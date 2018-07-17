# Jog Arm

### Quick Usage Instructions:

Adjust the parameters in ./config/jog_settings.yaml for your particular robot. More on that below. Launch your robot in MoveIt! Then,

```
roslaunch moveit_experimental jog_with_spacenav.launch
```

or

```
roslaunch moveit_experimental jog_with_xbox.launch
```

### Features:

jog_arm is useful for sending real-time commands to a manipulator. This could be for teleoperation or for compliance. Here is a video of teleoperation:

https://drive.google.com/file/d/1OZOEpct3XdnNgujEinPzNmAPvyObIz1T/view

And a video of opening a door with compliance. Note, the start pose and door radius were not hard-coded. The robot is adjusting its trajectory in real-time to minimize radial force and torques.

https://drive.google.com/file/d/1cgxnZ7ylCTpulb6jw2BHLopYDr-WwFZm/view

### Teleoperation devices:

The package comes with launch files for an XBox-style controller and a 3Dconnexion SpaceNavigator.

https://www.logitechg.com/en-us/product/f710-wireless-gamepad

https://www.3dconnexion.com/products/spacemouse.html

### Parameter Tuning:

We recommend using velocity controllers for smooth motion, if possible.

Here are some guidelines for tuning the parameters in ./config/jog_settings.yaml, although every type of robot is different.

**gazebo**: set to true if running a Gazebo simulation. It requires slightly different trajectory messages.

**collision_check**: set to true to enable collision checking.

**command_in_topic**: name of the topic for incoming commands, e.g. from an XBox controller

**command_frame**: name of the tf frame for incoming commands. You could set it to an end-effector frame so it changes orientation with the robot, or to a fixed frame.

**incoming_command_timeout**: for safety, stop jogging when fresh commands haven't been received for this many seconds.

**joint_topic**: where does the arm driver publish the joint names/values? Typically 'joint_states'.

**move_group_name**: name of the MoveIt! group of interest. Must be a serial chain since the Jacobian is used.

**singularity_threshold**: how sensitive is singularity detection? Larger ==> Can jog closer to a singularity before slowing.

**hard_stop_singularity_threshold**: stop when the arm is very close to singularity. Should be > singularity_threshold parameter. Larger ==> Can be closer to a singularity before stopping.

**command_out_topic**: name of the outgoing trajectory_msgs::JointTrajectory topic. Your robot driver should be subscribed to this.

**planning_frame**: the MoveIt! planning frame. Often 'base_link' or another link at the base of the robot.

**low_pass_filter_coeff**: a larger coefficient yields smoother velocity profiles, but more lag.

**pub_period**: define the rate of outgoing joint commands. Typically in the 60-125 Hz range.

**scale/linear**: incoming joystick commands are typically in the range [-1, 1]. Use this value to scale them to something reasonable for your robot. This defines the max linear velocity in meters per pub_period. max_velocity = linear_scale/pub_period, units are [m/s].

**scale/angular**: incoming joystick commands are typically in the range [-1, 1]. Use this value to scale them to something reasonable for your robot. This defines the max angular velocity in radians per pub_period. max_velocity = angular_scale/pub_period, units are [rad/s].

**warning_topic**: publish true to this topic if jogging halts due to collision/joint limit/velocity limit/singularity.