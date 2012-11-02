# Point cloud
sensor_msgs/PointCloud2 cloud

# Indices for the region of interest
int32[] mask

# One of the corresponding 2D images, if applicable
sensor_msgs/Image image

# The disparity image, if applicable
sensor_msgs/Image disparity_image

# Camera info for the camera that took the image
sensor_msgs/CameraInfo cam_info

# a 3D region of interest for grasp planning
geometry_msgs/PoseStamped  roi_box_pose
geometry_msgs/Vector3 	   roi_box_dims
