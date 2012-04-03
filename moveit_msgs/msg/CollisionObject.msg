# a header, used for interpreting the poses
Header header

# the id of the object
string id

# the shapes associated with the object
shape_msgs/Shape[] shapes

# the poses associated with the shapes - will be transformed using the header
geometry_msgs/Pose[] poses

# the static shapes associated with the object
shape_msgs/StaticShape[] static_shapes

# Puts the object into the environment
#or updates the object if already added
byte ADD=0

# Removes the object from the environment entirely (everything that mages the specified id)
byte REMOVE=1

# Operation to be performed
byte operation
