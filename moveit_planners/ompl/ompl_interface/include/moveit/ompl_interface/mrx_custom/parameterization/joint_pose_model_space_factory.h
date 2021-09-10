#pragma once

#include <moveit/ompl_interface/parameterization/model_based_state_space_factory.h>

namespace ompl_interface
{
class JointPoseModelStateSpaceFactory : public ModelBasedStateSpaceFactory
{
public:
  JointPoseModelStateSpaceFactory();

  int canRepresentProblem(const std::string& group, const moveit_msgs::MotionPlanRequest& req,
                          const moveit::core::RobotModelConstPtr& robot_model) const override;

protected:
  ModelBasedStateSpacePtr allocStateSpace(const ModelBasedStateSpaceSpecification& space_spec) const override;
};
}  // namespace ompl_interface
