# This message contains the definition of a position constraint.

# The robot link this constraint refers to
string link_name

# The offset (in the link frame) for the target point on the link we are planning for
geometry_msgs/Vector3 target_point_offset

# The shape of the bounded region that constrains the position of the end-effector
# This region is always centered at the position defined above
shape_msgs/Shape constraint_region_shape

# The orientation of the bounded region that constrains the position of the end-effector
# This allows the specification of non-axis aligned constraints
geometry_msgs/PoseStamped constraint_region_pose

# A weighting factor for this constraint (denotes relative importance to other constraints. Closer to zero means less important)
float64 weight
