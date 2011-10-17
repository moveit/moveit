# The constraint is to maintain visibility to a point on the following link 
string target_link

# The offset (in the link frame) for the target point we want to maintain visible
geometry_msgs/Vector3 target_point_offset

# The local pose of the frame in which visibility is to be maintained
# The frame id should represent the robot link to which the sensor is attached
# The visual axis of the sensor is assumed to be along the X axis of this frame
geometry_msgs/PoseStamped sensor_pose

# The deviation (in radians) that will be tolerated
# Constraint error will be measured as the solid angle between the 
# X axis of the frame defined above and the vector between the origin 
# of the frame defined above and the target location
float64 absolute_tolerance

# A weighting factor for this constraint (denotes relative importance to other constraints. Closer to zero means less important)
float64 weight
