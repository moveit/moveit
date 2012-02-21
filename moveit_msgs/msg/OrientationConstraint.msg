# This message contains the definition of an orientation constraint.

# The robot link this constraint refers to
string link_name

# The desired orientation of the robot link specified as a quaternion
geometry_msgs/QuaternionStamped orientation

# optional axis-angle error tolerances specified
float64 absolute_x_axis_tolerance
float64 absolute_y_axis_tolerance
float64 absolute_z_axis_tolerance

# A weighting factor for this constraint (denotes relative importance to other constraints. Closer to zero means less important)
float64 weight
