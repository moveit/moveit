# The model id for which this path has been generated
string model_id

# The representation of the path contains position values for all the joints that are moving along the path
RobotTrajectory trajectory

# The robot state is used to obtain positions for all/some of the joints of the robot. 
# It is used by the path display node to determine the positions of the joints that are not specified in the joint path message above. 
# If the robot state message contains joint position information for joints that are also mentioned in the joint path message, the positions in the joint path message will overwrite the positions specified in the robot state message. 
RobotState trajectory_start
