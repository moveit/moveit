# Informs that a specific model from the Model Database has been 
# identified at a certain location

# the database id of the model
int32 model_id

# the pose that it can be found in
geometry_msgs/PoseStamped pose

# a measure of the confidence level in this detection result
float32 confidence

# the name of the object detector that generated this detection result
string detector_name
