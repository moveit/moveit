# std header
std_msgs/Header header

# group name
string group_name

# gripper/tool offset 
# (in the frame of the end effector
# i.e. the last frame in the arm)
geometry_msgs/Pose tool_frame_offset

# workspace samples (These are points where you want the tool to be)
WorkspacePoint[] points

# True only if workspace points have been sampled on a grid
bool ordered

# Store information about how the workspace has been sampled here
# These parameters are relevant only if "ordered" is true
moveit_msgs/WorkspaceParameters parameters
float64 position_resolution
geometry_msgs/Quaternion[] orientations

