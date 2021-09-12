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
  JointPoseModelStateSpace(const ModelBasedStateSpacePtr& space_ptr);

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
  EllipsoidalSampler(const unsigned int n, const std::vector<double>& start_point,
                     const std::vector<double>& goal_point, JointPoseModelStateSpacePtr space);

  void setTraverseDiameter(const double diameter);
  void sampleUniform(ompl::base::State* state);
  double getPathLength(ompl::base::State* state) const;

  const std::vector<double>& getStartPoint() const;
  const std::vector<double>& getGoalPoint() const;

  double distanceFromStartPoint(const std::vector<double>& point) const;
  double distanceFromGoalPoint(const std::vector<double>& point) const;

private:
  JointPoseModelStateSpacePtr space_;
  ompl::base::StateSamplerPtr base_sampler_;
  ompl::ProlateHyperspheroidPtr phs_ptr_;
  ompl::RNG rng_;
  double traverse_diameter_;
  std::vector<double> start_point_;
  std::vector<double> goal_point_;
};
}  // namespace ompl_interface
