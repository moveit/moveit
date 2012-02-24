# Standard ROS header contains information 
# about the frame in which this 
# contact is specified
Header header

# Position of the contact point
geometry_msgs/Point position

# Normal corresponding to the contact point
geometry_msgs/Vector3 normal 

# Depth of contact point
float64 depth

# Name of the first body that is in contact
# This could be a link or a namespace that represents a body
string contact_body_1
uint32 body_type_1

# Name of the second body that is in contact
# This could be a link or a namespace that represents a body
string contact_body_2
uint32 body_type_2

uint32 ROBOT_LINK=0
uint32 WORLD_OBJECT=1
uint32 ROBOT_ATTACHED=2
