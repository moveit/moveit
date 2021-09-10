
#include <moveit/ompl_interface/mrx_custom/parameterization/joint_pose_model_space_factory.h>
#include <moveit/ompl_interface/mrx_custom/parameterization/joint_pose_model_space.h>

ompl_interface::JointPoseModelStateSpaceFactory::JointPoseModelStateSpaceFactory() : ModelBasedStateSpaceFactory()
{
  type_ = JointPoseModelStateSpace::PARAMETERIZATION_TYPE;
}

int ompl_interface::JointPoseModelStateSpaceFactory::canRepresentProblem(
    const std::string& /*group*/, const moveit_msgs::MotionPlanRequest& req,
    const moveit::core::RobotModelConstPtr& /*robot_model*/) const
{
  ROS_INFO("JointPoseModelStateSpaceFactory::canRepresentProblem. req.planner_id=%s", req.planner_id.c_str());
  if (req.planner_id == "geometric::InformedBiTRRT")
  {
    ROS_INFO("planner_id=%s. Use JointPoseModelStateSpace.", req.planner_id.c_str());
    return 500;
  }

  return 0;
}

ompl_interface::ModelBasedStateSpacePtr ompl_interface::JointPoseModelStateSpaceFactory::allocStateSpace(
    const ModelBasedStateSpaceSpecification& space_spec) const
{
  return std::make_shared<JointPoseModelStateSpace>(space_spec);
}
