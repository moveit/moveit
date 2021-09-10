#pragma once

#include "moveit/ompl_interface/parameterization/work_space/pose_model_state_space.h"

namespace ompl_interface
{
OMPL_CLASS_FORWARD(JointPoseStateSpace);
class JointPoseStateSpace : public PoseModelStateSpace
{
public:
  static const std::string PARAMETERIZATION_TYPE;

  JointPoseStateSpace(const ModelBasedStateSpaceSpecification& spec);

  const std::string& getParameterizationType() const override
  {
    return PARAMETERIZATION_TYPE;
  }

  double getMaximumExtent() const override;

  double getMeasure() const override;

  double distance(const ompl::base::State* state1, const ompl::base::State* state2) const override;

  double distanceJoint(const ompl::base::State* state1, const ompl::base::State* state2) const;

  double distancePosition(const ompl::base::State* state1, const ompl::base::State* state2) const;

  void interpolate(const ompl::base::State* from, const ompl::base::State* to, const double t,
                   ompl::base::State* state) const override;
};
}  // namespace ompl_interface
