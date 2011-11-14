# The constraint is useful to maintain visibility to a disc in a particular frame.
# This constraint does NOT enforce minimum or maximum distances between the sensor
# and the target, nor does it enforce the target to be in the field of view of
# the sensor. A PositionConstraint can (and probably should) be used for such purposes.
# This constraint only requires that a particular visibility cone originating from
# the sensor and "looking" towards the target is not obstructed by the robot itself.

# The radius of the disc that should be maintained visible 
float64 target_radius

# The pose of the disc; as the robot moves, the pose of the disc may change as well
geometry_msgs/PoseStamped target_pose

# From the sensor origin towards the target, the disc forms a visibility cone
# This cone is approximated using many sides. For example, when using 4 sides, 
# that in fact makes the visibility region be a pyramid.
# This value should always be 3 or more.
int32 cone_sides

# The local pose of the frame in which visibility is to be maintained
# The frame id should represent the robot link to which the sensor is attached
# The visual axis of the sensor is assumed to be along the X axis of this frame
geometry_msgs/PoseStamped sensor_pose

# A weighting factor for this constraint (denotes relative importance to other constraints. Closer to zero means less important)
float64 weight
