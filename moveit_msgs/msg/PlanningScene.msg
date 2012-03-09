# name of planning scene
string name

#full robot state
RobotState robot_state

#additional frames for duplicating tf
geometry_msgs/TransformStamped[] fixed_frame_transforms

#full allowed collision matrix
AllowedCollisionMatrix allowed_collision_matrix

# all link paddings
LinkPadding[] link_padding

# all link scales
LinkScale[] link_scale

# attached collision objects
AttachedCollisionObject[] attached_collision_objects

# Colors for the attached collision_objects
std_msgs/ColorRGBA[] attached_collision_objects_colors

# the collision map
PlanningSceneWorld world
