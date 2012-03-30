# Contains all the information from one run of the tabletop detection node

# The information for the plane that has been detected
Table table

# The raw clusters detected in the scan 
sensor_msgs/PointCloud[] clusters

# The list of potential models that have been detected for each cluster
# An empty list will be returned for a cluster that has no recognition results at all
moveit_manipulation_msgs/DatabaseModelPoseList[] models

# For each cluster, the index of the list of models that was fit to that cluster
# keep in mind that multiple raw clusters can correspond to a single fit
int32[] cluster_model_indices

# Whether the detection has succeeded or failed
int32 NO_CLOUD_RECEIVED = 1
int32 NO_TABLE = 2
int32 OTHER_ERROR = 3
int32 SUCCESS = 4
int32 result
