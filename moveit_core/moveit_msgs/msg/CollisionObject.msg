# a header, used for interpreting the poses
Header header

# the id of the object
string id

# The padding used for filtering points near the object.
# This does not affect collision checking for the object.  
# Set to negative to get zero padding.
float32 padding

#the shapes associated with the object
Shape[] shapes

#the poses associated with the shapes - will be transformed using the header
geometry_msgs/Pose[] poses


#Puts the object into the environment
#or updates the object if already added
byte ADD=0

#Removes the object from the environment entirely
byte REMOVE=1

#Only valid within the context of a CollisionAttachedObject message
#Will be ignored if sent with an CollisionObject message
#Takes an attached object, detaches from the attached link
#But adds back in as regular object
byte DETACH_AND_ADD_AS_OBJECT=2

#Only valid within the context of a CollisionAttachedObject message
#Will be ignored if sent with an CollisionObject message
#Takes current object in the environment and removes it as
#a regular object
byte ATTACH_AND_REMOVE_AS_OBJECT=3

# Operation to be performed
byte operation
