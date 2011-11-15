# a header, used for interpreting the poses
Header header

# the id of the object
string id

#the shapes associated with the object
Shape[] shapes

#the poses associated with the shapes - will be transformed using the header
geometry_msgs/Pose[] poses

# Puts the object into the environment
#or updates the object if already added
byte ADD=0

# Removes the object from the environment entirely
byte REMOVE=1

# Operation to be performed
byte operation
