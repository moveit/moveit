#pragma once

#include <ompl/util/ProlateHyperspheroid.h>

#include "moveit/ompl_interface/parameterization/work_space/pose_model_state_space.h"

namespace ompl_interface
{
OMPL_CLASS_FORWARD(JointPoseModelStateSpace);
OMPL_CLASS_FORWARD(EllipsoidalSampler);

class JointPoseModelStateSpace : public PoseModelStateSpace
{
public:
  static const std::string PARAMETERIZATION_TYPE;

  JointPoseModelStateSpace(const ModelBasedStateSpaceSpecification& spec);

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

class EllipsoidalSampler
{
public:
  EllipsoidalSampler(const unsigned int n, const std::vector<double>& focus1, const std::vector<double>& focus2,
                     JointPoseModelStateSpacePtr space);

  void setTraverseDiameter(const double diameter);
  void sampleUniform(ompl::base::State* state);

private:
  JointPoseModelStateSpacePtr space_;
  ompl::base::StateSamplerPtr base_sampler_;
  ompl::ProlateHyperspheroidPtr phs_ptr_;
  ompl::RNG rng_;
  double traverse_diameter_;
};
}  // namespace ompl_interface
