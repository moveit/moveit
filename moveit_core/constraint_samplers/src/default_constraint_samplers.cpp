/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#include <moveit/constraint_samplers/default_constraint_samplers.h>
#include <set>
#include <cassert>
#include <eigen_conversions/eigen_msg.h>
#include <boost/bind.hpp>

namespace constraint_samplers
{
bool JointConstraintSampler::configure(const moveit_msgs::Constraints& constr)
{
  // construct the constraints
  std::vector<kinematic_constraints::JointConstraint> jc;
  for (std::size_t i = 0; i < constr.joint_constraints.size(); ++i)
  {
    kinematic_constraints::JointConstraint j(scene_->getRobotModel());
    if (j.configure(constr.joint_constraints[i]))
      jc.push_back(j);
  }

  return jc.empty() ? false : configure(jc);
}

bool JointConstraintSampler::configure(const std::vector<kinematic_constraints::JointConstraint>& jc)
{
  clear();

  if (!jmg_)
  {
    ROS_ERROR_NAMED("constraint_samplers", "NULL group specified for constraint sampler");
    return false;
  }

  // find and keep the constraints that operate on the group we sample
  // also keep bounds for joints as convenient
  std::map<std::string, JointInfo> bound_data;
  bool some_valid_constraint = false;
  for (std::size_t i = 0; i < jc.size(); ++i)
  {
    if (!jc[i].enabled())
      continue;

    const robot_model::JointModel* jm = jc[i].getJointModel();
    if (!jmg_->hasJointModel(jm->getName()))
      continue;

    some_valid_constraint = true;

    const robot_model::VariableBounds& joint_bounds = jm->getVariableBounds(jc[i].getJointVariableName());
    JointInfo ji;
    std::map<std::string, JointInfo>::iterator it = bound_data.find(jc[i].getJointVariableName());
    if (it != bound_data.end())
      ji = it->second;
    else
      ji.index_ = jmg_->getVariableGroupIndex(jc[i].getJointVariableName());
    ji.potentiallyAdjustMinMaxBounds(
        std::max(joint_bounds.min_position_, jc[i].getDesiredJointPosition() - jc[i].getJointToleranceBelow()),
        std::min(joint_bounds.max_position_, jc[i].getDesiredJointPosition() + jc[i].getJointToleranceAbove()));

    ROS_DEBUG_NAMED("constraint_samplers", "Bounds for %s JointConstraint are %g %g",
                    jc[i].getJointVariableName().c_str(), ji.min_bound_, ji.max_bound_);

    if (ji.min_bound_ > ji.max_bound_ + std::numeric_limits<double>::epsilon())
    {
      std::stringstream cs;
      jc[i].print(cs);
      ROS_ERROR_NAMED("constraint_samplers",
                      "The constraints for joint '%s' are such that "
                      "there are no possible values for the joint: min_bound: %g, max_bound: %g. Failing.\n",
                      jm->getName().c_str(), ji.min_bound_, ji.max_bound_);
      clear();
      return false;
    }
    bound_data[jc[i].getJointVariableName()] = ji;
  }

  if (!some_valid_constraint)
  {
    ROS_WARN_NAMED("constraint_samplers", "No valid joint constraints");
    return false;
  }

  for (std::map<std::string, JointInfo>::iterator it = bound_data.begin(); it != bound_data.end(); ++it)
    bounds_.push_back(it->second);

  // get a separate list of joints that are not bounded; we will sample these randomly
  const std::vector<const robot_model::JointModel*>& joints = jmg_->getJointModels();
  for (std::size_t i = 0; i < joints.size(); ++i)
    if (bound_data.find(joints[i]->getName()) == bound_data.end() && joints[i]->getVariableCount() > 0 &&
        joints[i]->getMimic() == nullptr)
    {
      // check if all the vars of the joint are found in bound_data instead
      const std::vector<std::string>& vars = joints[i]->getVariableNames();
      if (vars.size() > 1)
      {
        bool all_found = true;
        for (std::size_t j = 0; j < vars.size(); ++j)
          if (bound_data.find(vars[j]) == bound_data.end())
          {
            all_found = false;
            break;
          }
        if (all_found)
          continue;
      }
      unbounded_.push_back(joints[i]);
      // Get the first variable name of this joint and find its index position in the planning group
      uindex_.push_back(jmg_->getVariableGroupIndex(vars[0]));
    }
  values_.resize(jmg_->getVariableCount());
  is_valid_ = true;
  return true;
}

bool JointConstraintSampler::sample(robot_state::RobotState& state,
                                    const robot_state::RobotState& /* reference_state */,
                                    unsigned int /* max_attempts */)
{
  if (!is_valid_)
  {
    ROS_WARN_NAMED("constraint_samplers", "JointConstraintSampler not configured, won't sample");
    return false;
  }

  // sample the unbounded joints first (in case some joint variables are bounded)
  std::vector<double> v;
  for (std::size_t i = 0; i < unbounded_.size(); ++i)
  {
    v.resize(unbounded_[i]->getVariableCount());
    unbounded_[i]->getVariableRandomPositions(random_number_generator_, &v[0]);
    for (std::size_t j = 0; j < v.size(); ++j)
      values_[uindex_[i] + j] = v[j];
  }

  // enforce the constraints for the constrained components (could be all of them)
  for (std::size_t i = 0; i < bounds_.size(); ++i)
    values_[bounds_[i].index_] = random_number_generator_.uniformReal(bounds_[i].min_bound_, bounds_[i].max_bound_);

  state.setJointGroupPositions(jmg_, values_);

  // we are always successful
  return true;
}

bool JointConstraintSampler::project(robot_state::RobotState& state, unsigned int max_attempts)
{
  return sample(state, state, max_attempts);
}

void JointConstraintSampler::clear()
{
  ConstraintSampler::clear();
  bounds_.clear();
  unbounded_.clear();
  uindex_.clear();
  values_.clear();
}

IKSamplingPose::IKSamplingPose()
{
}

IKSamplingPose::IKSamplingPose(const kinematic_constraints::PositionConstraint& pc)
  : position_constraint_(new kinematic_constraints::PositionConstraint(pc))
{
}

IKSamplingPose::IKSamplingPose(const kinematic_constraints::OrientationConstraint& oc)
  : orientation_constraint_(new kinematic_constraints::OrientationConstraint(oc))
{
}

IKSamplingPose::IKSamplingPose(const kinematic_constraints::PositionConstraint& pc,
                               const kinematic_constraints::OrientationConstraint& oc)
  : position_constraint_(new kinematic_constraints::PositionConstraint(pc))
  , orientation_constraint_(new kinematic_constraints::OrientationConstraint(oc))
{
}

IKSamplingPose::IKSamplingPose(const kinematic_constraints::PositionConstraintPtr& pc) : position_constraint_(pc)
{
}

IKSamplingPose::IKSamplingPose(const kinematic_constraints::OrientationConstraintPtr& oc) : orientation_constraint_(oc)
{
}

IKSamplingPose::IKSamplingPose(const kinematic_constraints::PositionConstraintPtr& pc,
                               const kinematic_constraints::OrientationConstraintPtr& oc)
  : position_constraint_(pc), orientation_constraint_(oc)
{
}

void IKConstraintSampler::clear()
{
  ConstraintSampler::clear();
  kb_.reset();
  ik_frame_ = "";
  transform_ik_ = false;
  eef_to_ik_tip_transform_ = Eigen::Affine3d::Identity();
  need_eef_to_ik_tip_transform_ = false;
}

bool IKConstraintSampler::configure(const IKSamplingPose& sp)
{
  clear();
  if (!sp.position_constraint_ && !sp.orientation_constraint_)
    return false;
  if ((!sp.orientation_constraint_ && !sp.position_constraint_->enabled()) ||
      (!sp.position_constraint_ && !sp.orientation_constraint_->enabled()) ||
      (sp.position_constraint_ && sp.orientation_constraint_ && !sp.position_constraint_->enabled() &&
       !sp.orientation_constraint_->enabled()))
  {
    ROS_WARN_NAMED("constraint_samplers", "No enabled constraints in sampling pose");
    return false;
  }

  sampling_pose_ = sp;
  ik_timeout_ = jmg_->getDefaultIKTimeout();
  if (sampling_pose_.position_constraint_ && sampling_pose_.orientation_constraint_)
    if (sampling_pose_.position_constraint_->getLinkModel()->getName() !=
        sampling_pose_.orientation_constraint_->getLinkModel()->getName())
    {
      ROS_ERROR_NAMED("constraint_samplers",
                      "Position and orientation constraints need to be specified for the same link "
                      "in order to use IK-based sampling");
      return false;
    }

  if (sampling_pose_.position_constraint_ && sampling_pose_.position_constraint_->mobileReferenceFrame())
    frame_depends_.push_back(sampling_pose_.position_constraint_->getReferenceFrame());
  if (sampling_pose_.orientation_constraint_ && sampling_pose_.orientation_constraint_->mobileReferenceFrame())
    frame_depends_.push_back(sampling_pose_.orientation_constraint_->getReferenceFrame());
  kb_ = jmg_->getSolverInstance();
  if (!kb_)
  {
    ROS_WARN_NAMED("constraint_samplers", "No solver instance in setup");
    is_valid_ = false;
    return false;
  }
  is_valid_ = loadIKSolver();
  return is_valid_;
}

bool IKConstraintSampler::configure(const moveit_msgs::Constraints& constr)
{
  for (std::size_t p = 0; p < constr.position_constraints.size(); ++p)
    for (std::size_t o = 0; o < constr.orientation_constraints.size(); ++o)
      if (constr.position_constraints[p].link_name == constr.orientation_constraints[o].link_name)
      {
        kinematic_constraints::PositionConstraintPtr pc(
            new kinematic_constraints::PositionConstraint(scene_->getRobotModel()));
        kinematic_constraints::OrientationConstraintPtr oc(
            new kinematic_constraints::OrientationConstraint(scene_->getRobotModel()));
        if (pc->configure(constr.position_constraints[p], scene_->getTransforms()) &&
            oc->configure(constr.orientation_constraints[o], scene_->getTransforms()))
          return configure(IKSamplingPose(pc, oc));
      }

  for (std::size_t p = 0; p < constr.position_constraints.size(); ++p)
  {
    kinematic_constraints::PositionConstraintPtr pc(
        new kinematic_constraints::PositionConstraint(scene_->getRobotModel()));
    if (pc->configure(constr.position_constraints[p], scene_->getTransforms()))
      return configure(IKSamplingPose(pc));
  }

  for (std::size_t o = 0; o < constr.orientation_constraints.size(); ++o)
  {
    kinematic_constraints::OrientationConstraintPtr oc(
        new kinematic_constraints::OrientationConstraint(scene_->getRobotModel()));
    if (oc->configure(constr.orientation_constraints[o], scene_->getTransforms()))
      return configure(IKSamplingPose(oc));
  }
  return false;
}

double IKConstraintSampler::getSamplingVolume() const
{
  double v = 1.0;
  if (sampling_pose_.position_constraint_)
  {
    const std::vector<bodies::BodyPtr>& b = sampling_pose_.position_constraint_->getConstraintRegions();
    double vol = 0;
    for (std::size_t i = 0; i < b.size(); ++i)
      vol += b[i]->computeVolume();
    if (!b.empty())
      v *= vol;
  }

  if (sampling_pose_.orientation_constraint_)
    v *= sampling_pose_.orientation_constraint_->getXAxisTolerance() *
         sampling_pose_.orientation_constraint_->getYAxisTolerance() *
         sampling_pose_.orientation_constraint_->getZAxisTolerance();
  return v;
}

const std::string& IKConstraintSampler::getLinkName() const
{
  if (sampling_pose_.orientation_constraint_)
    return sampling_pose_.orientation_constraint_->getLinkModel()->getName();
  return sampling_pose_.position_constraint_->getLinkModel()->getName();
}

bool IKConstraintSampler::loadIKSolver()
{
  if (!kb_)
  {
    ROS_ERROR_NAMED("constraint_samplers", "No IK solver");
    return false;
  }

  // check if we need to transform the request into the coordinate frame expected by IK
  ik_frame_ = kb_->getBaseFrame();
  transform_ik_ = !robot_state::Transforms::sameFrame(ik_frame_, jmg_->getParentModel().getModelFrame());
  if (!ik_frame_.empty() && ik_frame_[0] == '/')
    ik_frame_.erase(ik_frame_.begin());
  if (transform_ik_)
    if (!jmg_->getParentModel().hasLinkModel(ik_frame_))
    {
      ROS_ERROR_NAMED("constraint_samplers",
                      "The IK solver expects requests in frame '%s' but this frame is not known to the sampler. "
                      "Ignoring transformation (IK may fail)",
                      ik_frame_.c_str());
      transform_ik_ = false;
    }

  // check if IK is performed for the desired link
  bool wrong_link = false;
  if (sampling_pose_.position_constraint_)
  {
    const moveit::core::LinkModel* lm = sampling_pose_.position_constraint_->getLinkModel();
    if (!moveit::core::Transforms::sameFrame(kb_->getTipFrame(), lm->getName()))
    {
      wrong_link = true;
      const moveit::core::LinkTransformMap& fixed_links = lm->getAssociatedFixedTransforms();
      for (moveit::core::LinkTransformMap::const_iterator it = fixed_links.begin(); it != fixed_links.end(); ++it)
        if (moveit::core::Transforms::sameFrame(it->first->getName(), kb_->getTipFrame()))
        {
          eef_to_ik_tip_transform_ = it->second;
          need_eef_to_ik_tip_transform_ = true;
          wrong_link = false;
          break;
        }
    }
  }

  if (!wrong_link && sampling_pose_.orientation_constraint_)
  {
    const moveit::core::LinkModel* lm = sampling_pose_.orientation_constraint_->getLinkModel();
    if (!robot_state::Transforms::sameFrame(kb_->getTipFrame(), lm->getName()))
    {
      wrong_link = true;
      const moveit::core::LinkTransformMap& fixed_links = lm->getAssociatedFixedTransforms();
      for (moveit::core::LinkTransformMap::const_iterator it = fixed_links.begin(); it != fixed_links.end(); ++it)
        if (moveit::core::Transforms::sameFrame(it->first->getName(), kb_->getTipFrame()))
        {
          eef_to_ik_tip_transform_ = it->second;
          need_eef_to_ik_tip_transform_ = true;
          wrong_link = false;
          break;
        }
    }
  }

  if (wrong_link)
  {
    ROS_ERROR_NAMED("constraint_samplers",
                    "IK cannot be performed for link '%s'. The solver can report IK solutions for link '%s'.",
                    sampling_pose_.position_constraint_ ?
                        sampling_pose_.position_constraint_->getLinkModel()->getName().c_str() :
                        sampling_pose_.orientation_constraint_->getLinkModel()->getName().c_str(),
                    kb_->getTipFrame().c_str());
    return false;
  }
  return true;
}

bool IKConstraintSampler::samplePose(Eigen::Vector3d& pos, Eigen::Quaterniond& quat, const robot_state::RobotState& ks,
                                     unsigned int max_attempts)
{
  if (ks.dirtyLinkTransforms())
  {
    // samplePose below requires accurate transforms
    ROS_ERROR_NAMED("constraint_samplers",
                    "IKConstraintSampler received dirty robot state, but valid transforms are required. "
                    "Failing.");
    return false;
  }

  if (sampling_pose_.position_constraint_)
  {
    const std::vector<bodies::BodyPtr>& b = sampling_pose_.position_constraint_->getConstraintRegions();
    if (!b.empty())
    {
      bool found = false;
      std::size_t k = random_number_generator_.uniformInteger(0, b.size() - 1);
      for (std::size_t i = 0; i < b.size(); ++i)
        if (b[(i + k) % b.size()]->samplePointInside(random_number_generator_, max_attempts, pos))
        {
          found = true;
          break;
        }
      if (!found)
      {
        ROS_ERROR_NAMED("constraint_samplers", "Unable to sample a point inside the constraint region");
        return false;
      }
    }
    else
    {
      ROS_ERROR_NAMED("constraint_samplers", "Unable to sample a point inside the constraint region. "
                                             "Constraint region is empty when it should not be.");
      return false;
    }

    // if this constraint is with respect a mobile frame, we need to convert this rotation to the root frame of the
    // model
    if (sampling_pose_.position_constraint_->mobileReferenceFrame())
      pos = ks.getFrameTransform(sampling_pose_.position_constraint_->getReferenceFrame()) * pos;
  }
  else
  {
    // do FK for rand state
    robot_state::RobotState tempState(ks);
    tempState.setToRandomPositions(jmg_);
    pos = tempState.getGlobalLinkTransform(sampling_pose_.orientation_constraint_->getLinkModel()).translation();
  }

  if (sampling_pose_.orientation_constraint_)
  {
    // sample a rotation matrix within the allowed bounds
    double angle_x =
        2.0 * (random_number_generator_.uniform01() - 0.5) *
        (sampling_pose_.orientation_constraint_->getXAxisTolerance() - std::numeric_limits<double>::epsilon());
    double angle_y =
        2.0 * (random_number_generator_.uniform01() - 0.5) *
        (sampling_pose_.orientation_constraint_->getYAxisTolerance() - std::numeric_limits<double>::epsilon());
    double angle_z =
        2.0 * (random_number_generator_.uniform01() - 0.5) *
        (sampling_pose_.orientation_constraint_->getZAxisTolerance() - std::numeric_limits<double>::epsilon());
    Eigen::Affine3d diff(Eigen::AngleAxisd(angle_x, Eigen::Vector3d::UnitX()) *
                         Eigen::AngleAxisd(angle_y, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(angle_z, Eigen::Vector3d::UnitZ()));
    Eigen::Affine3d reqr(sampling_pose_.orientation_constraint_->getDesiredRotationMatrix() * diff.linear());
    quat = Eigen::Quaterniond(reqr.linear());

    // if this constraint is with respect a mobile frame, we need to convert this rotation to the root frame of the
    // model
    if (sampling_pose_.orientation_constraint_->mobileReferenceFrame())
    {
      const Eigen::Affine3d& t = ks.getFrameTransform(sampling_pose_.orientation_constraint_->getReferenceFrame());
      Eigen::Affine3d rt(t.linear() * quat.toRotationMatrix());
      quat = Eigen::Quaterniond(rt.linear());
    }
  }
  else
  {
    // sample a random orientation
    double q[4];
    random_number_generator_.quaternion(q);
    quat = Eigen::Quaterniond(q[3], q[0], q[1], q[2]);
  }

  // if there is an offset, we need to undo the induced rotation in the sampled transform origin (point)
  if (sampling_pose_.position_constraint_ && sampling_pose_.position_constraint_->hasLinkOffset())
    // the rotation matrix that corresponds to the desired orientation
    pos = pos - quat.toRotationMatrix() * sampling_pose_.position_constraint_->getLinkOffset();

  return true;
}

namespace
{
void samplingIkCallbackFnAdapter(robot_state::RobotState* state, const robot_model::JointModelGroup* jmg,
                                 const robot_state::GroupStateValidityCallbackFn& constraint,
                                 const geometry_msgs::Pose& /*unused*/, const std::vector<double>& ik_sol,
                                 moveit_msgs::MoveItErrorCodes& error_code)
{
  const std::vector<unsigned int>& bij = jmg->getKinematicsSolverJointBijection();
  std::vector<double> solution(bij.size());
  for (std::size_t i = 0; i < bij.size(); ++i)
    solution[i] = ik_sol[bij[i]];
  if (constraint(state, jmg, &solution[0]))
    error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  else
    error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
}
}

bool IKConstraintSampler::sample(robot_state::RobotState& state, const robot_state::RobotState& reference_state,
                                 unsigned int max_attempts)
{
  return sampleHelper(state, reference_state, max_attempts, false);
}

bool IKConstraintSampler::sampleHelper(robot_state::RobotState& state, const robot_state::RobotState& reference_state,
                                       unsigned int max_attempts, bool project)
{
  if (!is_valid_)
  {
    ROS_WARN_NAMED("constraint_samplers", "IKConstraintSampler not configured, won't sample");
    return false;
  }

  kinematics::KinematicsBase::IKCallbackFn adapted_ik_validity_callback;
  if (group_state_validity_callback_)
    adapted_ik_validity_callback =
        boost::bind(&samplingIkCallbackFnAdapter, &state, jmg_, group_state_validity_callback_, _1, _2, _3);

  for (unsigned int a = 0; a < max_attempts; ++a)
  {
    // sample a point in the constraint region
    Eigen::Vector3d point;
    Eigen::Quaterniond quat;
    if (!samplePose(point, quat, reference_state, max_attempts))
    {
      if (verbose_)
        ROS_INFO_NAMED("constraint_samplers", "IK constraint sampler was unable to produce a pose to run IK for");
      return false;
    }

    // we now have the transform we wish to perform IK for, in the planning frame
    if (transform_ik_)
    {
      // we need to convert this transform to the frame expected by the IK solver
      // both the planning frame and the frame for the IK are assumed to be robot links
      Eigen::Affine3d ikq(Eigen::Translation3d(point) * quat.toRotationMatrix());
      ikq = reference_state.getFrameTransform(ik_frame_).inverse(Eigen::Isometry) * ikq;
      point = ikq.translation();
      quat = Eigen::Quaterniond(ikq.linear());
    }

    if (need_eef_to_ik_tip_transform_)
    {
      // After sampling the pose needs to be transformed to the ik chain tip
      Eigen::Affine3d ikq(Eigen::Translation3d(point) * quat.toRotationMatrix());
      ikq = ikq * eef_to_ik_tip_transform_;
      point = ikq.translation();
      quat = Eigen::Quaterniond(ikq.linear());
    }

    geometry_msgs::Pose ik_query;
    ik_query.position.x = point.x();
    ik_query.position.y = point.y();
    ik_query.position.z = point.z();
    ik_query.orientation.x = quat.x();
    ik_query.orientation.y = quat.y();
    ik_query.orientation.z = quat.z();
    ik_query.orientation.w = quat.w();

    if (callIK(ik_query, adapted_ik_validity_callback, ik_timeout_, state, project && a == 0))
      return true;
  }
  return false;
}

bool IKConstraintSampler::project(robot_state::RobotState& state, unsigned int max_attempts)
{
  return sampleHelper(state, state, max_attempts, true);
}

bool IKConstraintSampler::validate(robot_state::RobotState& state) const
{
  state.update();
  return (!sampling_pose_.orientation_constraint_ ||
          sampling_pose_.orientation_constraint_->decide(state, verbose_).satisfied) &&
         (!sampling_pose_.position_constraint_ ||
          sampling_pose_.position_constraint_->decide(state, verbose_).satisfied);
}

bool IKConstraintSampler::callIK(const geometry_msgs::Pose& ik_query,
                                 const kinematics::KinematicsBase::IKCallbackFn& adapted_ik_validity_callback,
                                 double timeout, robot_state::RobotState& state, bool use_as_seed)
{
  const std::vector<unsigned int>& ik_joint_bijection = jmg_->getKinematicsSolverJointBijection();
  std::vector<double> seed(ik_joint_bijection.size(), 0.0);
  std::vector<double> vals;

  if (use_as_seed)
    state.copyJointGroupPositions(jmg_, vals);
  else
    // sample a seed value
    jmg_->getVariableRandomPositions(random_number_generator_, vals);

  assert(vals.size() == ik_joint_bijection.size());
  for (std::size_t i = 0; i < ik_joint_bijection.size(); ++i)
    seed[i] = vals[ik_joint_bijection[i]];

  std::vector<double> ik_sol;
  moveit_msgs::MoveItErrorCodes error;

  if (adapted_ik_validity_callback ?
          kb_->searchPositionIK(ik_query, seed, timeout, ik_sol, adapted_ik_validity_callback, error) :
          kb_->searchPositionIK(ik_query, seed, timeout, ik_sol, error))
  {
    assert(ik_sol.size() == ik_joint_bijection.size());
    std::vector<double> solution(ik_joint_bijection.size());
    for (std::size_t i = 0; i < ik_joint_bijection.size(); ++i)
      solution[ik_joint_bijection[i]] = ik_sol[i];
    state.setJointGroupPositions(jmg_, solution);

    return validate(state);
  }
  else
  {
    if (error.val != moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION &&
        error.val != moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE &&
        error.val != moveit_msgs::MoveItErrorCodes::TIMED_OUT)
      ROS_ERROR_NAMED("constraint_samplers", "IK solver failed with error %d", error.val);
    else if (verbose_)
      ROS_INFO_NAMED("constraint_samplers", "IK failed");
  }
  return false;
}

}  // end of namespace constraint_samplers