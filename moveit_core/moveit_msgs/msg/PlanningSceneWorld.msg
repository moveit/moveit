#collision objects
CollisionObject[] collision_objects

#the collision map
CollisionMap collision_map

# Optional colors for objects; the last element in the array 
# is the color of the collision_map (the colors
# array has length 1 more than the collision_objects array).
# To specify the color only for the collision_map, include one
# single element in the colors array)
std_msgs/ColorRGBA[] colors
