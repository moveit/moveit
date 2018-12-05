# This is a message to hold data to jog by specifying joint
# displacement. You only need to set relative displacement to joint
# angles (or displacements for linear joints).

# header message. You must set frame_id to define the reference
# coordinate system of the displacament
Header header

# Name list of the joints. You don't need to specify all joint of the
# robot. Joint names are case-sensitive.
string[] joint_names

# Relative displacement of the joints to jog. The order must be
# identical to joint_names. Unit is in radian for revolutive joints,
# meter for linear joints.
float64[] deltas
