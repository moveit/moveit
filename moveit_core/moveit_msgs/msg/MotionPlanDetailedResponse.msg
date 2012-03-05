# The starting state considered for the robot solution path
RobotState trajectory_start

# Multiple solution paths are reported, each reflecting intermediate steps in the trajectory processing

# The list of reported trajectories
RobotTrajectory[] trajectory

# Description of the reported trajectories (name of processing step)
string[] description

# The amount of time spent computing a particular step in motion plan computation 
duration[] processing_time
