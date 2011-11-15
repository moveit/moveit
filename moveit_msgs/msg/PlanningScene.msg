#full robot state
RobotState robot_state

#additional frames for duplicating tf
geometry_msgs/TransformStamped[] fixed_frame_transforms

#full allowed collision matrix
AllowedCollisionMatrix allowed_collision_matrix

#all link paddings
LinkPadding[] link_padding

#collision objects
CollisionObject[] collision_objects
AttachedCollisionObject[] attached_collision_objects

#the collision map
CollisionMap collision_map
