# Informs that a planar table has been detected at a given location

# The pose gives you the transform that take you to the coordinate system
# of the table, with the origin somewhere in the table plane and the 
# z axis normal to the plane
geometry_msgs/PoseStamped pose

# These values give you the observed extents of the table, along x and y,
# in the table's own coordinate system (above)
# there is no guarantee that the origin of the table coordinate system is
# inside the boundary defined by these values. 
float32 x_min
float32 x_max
float32 y_min
float32 y_max

# There is no guarantee that the table does NOT extend further than these 
# values; this is just as far as we've observed it.


# Newer table definition as triangle mesh of convex hull (relative to pose)
shape_msgs/Mesh convex_hull
