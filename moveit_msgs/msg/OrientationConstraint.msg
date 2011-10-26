# This message contains the definition of an orientation constraint.

# The robot link this constraint refers to
string link_name

# The desired orientation of the robot link specified as a quaternion
geometry_msgs/QuaternionStamped orientation

# optional RPY error tolerances specified if 
float64 absolute_roll_tolerance
float64 absolute_pitch_tolerance
float64 absolute_yaw_tolerance

# A weighting factor for this constraint (denotes relative importance to other constraints. Closer to zero means less important)
float64 weight
