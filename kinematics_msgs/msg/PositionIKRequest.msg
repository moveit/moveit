# A Position IK request message
# The name of the link for which we are computing IK
string ik_link_name

# The (stamped) pose of the link
geometry_msgs/PoseStamped pose_stamped

# A RobotState consisting of hint/seed positions for the IK computation. 
# These may be used to seed the IK search. 
# The seed state MUST contain state for all joints to be used by the IK solver
# to compute IK. The list of joints that the IK solver deals with can be found using
# the kinematics_msgs/GetKinematicSolverInfo
moveit_msgs/RobotState ik_seed_state

# Additional state information can be provided here to specify the starting positions 
# of other joints/links on the robot.
moveit_msgs/RobotState robot_state
