
#include "moveit/ompl_interface/mrx_custom/parameterization/joint_pose_space.h"

namespace ompl_interface
{
constexpr char LOGNAME[] = "joint_pose_state_space";
}  // namespace ompl_interface

const std::string ompl_interface::JointPoseStateSpace::PARAMETERIZATION_TYPE = "JointPose";

double ompl_interface::JointPoseStateSpace::getMaximumExtent() const
{
  return spec_.joint_model_group_->getMaximumExtentL2(spec_.joint_bounds_);
}

double ompl_interface::JointPoseStateSpace::getMeasure() const
{
  return spec_.joint_model_group_->getMaximumExtentL2(spec_.joint_bounds_);
}

double ompl_interface::JointPoseStateSpace::distance(const ompl::base::State* state1,
                                                     const ompl::base::State* state2) const
{
  if (distance_function_)
    return distance_function_(state1, state2);
  else
    return distanceJoint(state1, state2);
}

double ompl_interface::JointPoseStateSpace::distanceJoint(const ompl::base::State* state1,
                                                          const ompl::base::State* state2) const
{
  return spec_.joint_model_group_->distanceL2(state1->as<StateType>()->values, state2->as<StateType>()->values);
}

double ompl_interface::JointPoseStateSpace::distancePosition(const ompl::base::State* state1,
                                                             const ompl::base::State* state2) const
{
  std::vector<double> positions1;
  std::vector<double> positions2;

  if (!copyPositionsToReals(positions1, state1))
  {
    ROS_ERROR("Copy position failed when getting distance!!");
    return std::numeric_limits<double>::max();
  }

  if (!copyPositionsToReals(positions2, state2))
  {
    ROS_ERROR("Copy position failed when getting distance!!");
    return std::numeric_limits<double>::max();
  }

  Eigen::Map<const Eigen::VectorXd> vec1(positions1.data(), positions1.size());
  Eigen::Map<const Eigen::VectorXd> vec2(positions2.data(), positions2.size());

  return (vec1 - vec2).norm();
}

void ompl_interface::JointPoseStateSpace::interpolate(const ompl::base::State* from, const ompl::base::State* to,
                                                      const double t, ompl::base::State* state) const
{
  ompl_interface::ModelBasedStateSpace::interpolate(from, to, t, state);

  if (!computeStateFK(state))
    ROS_ERROR("computeStateFK failed when interpolate!!");
}

ompl_interface::EllipsoidalSampler::EllipsoidalSampler(const unsigned int n, const std::vector<double>& focus1,
                                                       const std::vector<double>& focus2, JointPoseStateSpacePtr space)
  : space_(std::move(space))
  , base_sampler_(space_->allocStateSampler())
  , phs_ptr_(new ompl::ProlateHyperspheroid(n, focus1.data(), focus2.data()))
  , traverse_diameter_(-1.0)
{
}

void ompl_interface::EllipsoidalSampler::setTraverseDiameter(const double diameter)
{
  assert(diameter > 0);
  phs_ptr_->setTransverseDiameter(diameter);
  traverse_diameter_ = diameter;
}

void ompl_interface::EllipsoidalSampler::sampleUniform(ompl::base::State* state)
{
  base_sampler_->sampleUniform(state);

  // If no traverse_diameter updates, just use base_sampler
  if (traverse_diameter_ < 0)
    return;

  std::vector<double> informedVector(phs_ptr_->getPhsDimension());

  // Sample positions in the ellipsoid
  rng_.uniformProlateHyperspheroid(phs_ptr_, &informedVector[0]);

  // Update positions
  space_->copyPositionsFromReals(state, informedVector);
}
