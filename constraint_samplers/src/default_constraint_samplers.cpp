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
*   * Neither the name of the Willow Garage nor the names of its
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

bool constraint_samplers::JointConstraintSampler::configure(const moveit_msgs::Constraints &constr)
{  
  // construct the constraints
  std::vector<kinematic_constraints::JointConstraint> jc;
  for (std::size_t i = 0 ; i < constr.joint_constraints.size() ; ++i)
  {
    kinematic_constraints::JointConstraint j(scene_->getKinematicModel(), scene_->getTransforms());
    if (j.configure(constr.joint_constraints[i]))
      jc.push_back(j);
  }
  
  return jc.empty() ? false : setup(jc);
}

bool constraint_samplers::JointConstraintSampler::setup(const std::vector<kinematic_constraints::JointConstraint> &jc)
{
  if (!jmg_)
  {
    logError("NULL group specified for constraint sampler");
    return false;
  }
  
  // find and keep the constraints that operate on the group we sample
  // also keep bounds for joints as convenient
  const std::map<std::string, unsigned int> &vim = jmg_->getJointVariablesIndexMap();
  std::set<std::string> bounded;
  for (std::size_t i = 0 ; i < jc.size() ; ++i)
  {
    if (!jc[i].enabled())
      continue;
    
    const kinematic_model::JointModel *jm = jc[i].getJointModel();
    if (!jmg_->hasJointModel(jm->getName()))
      continue;
    
    std::pair<double, double> bounds;
    jm->getVariableBounds(jc[i].getJointVariableName(), bounds);

    bounds.first = std::max(bounds.first, jc[i].getDesiredJointPosition() - jc[i].getJointToleranceBelow());
    bounds.second = std::min(bounds.second, jc[i].getDesiredJointPosition() + jc[i].getJointToleranceAbove());
    if (bounds.first > bounds.second)
    {
      std::stringstream cs; jc[i].print(cs);
      logWarn("The constraints for joint '%s' are such that there are no possible values for this joint. Ignoring constraint: %s\n", jm->getName().c_str(), cs.str().c_str());
      continue;
    }
    if (jm->getVariableCount() == 1)
      bounded.insert(jm->getName());
    bounds_.push_back(bounds);
    index_.push_back(vim.find(jc[i].getJointVariableName())->second);
  }
  
  // get a separate list of joints that are not bounded; we will sample these randomly
  const std::vector<const kinematic_model::JointModel*> &joints = jmg_->getJointModels();
  for (std::size_t i = 0 ; i < joints.size() ; ++i)
    if (bounded.find(joints[i]->getName()) == bounded.end())
    {
      unbounded_.push_back(joints[i]);
      uindex_.push_back(vim.find(joints[i]->getName())->second);
    } 
  values_.resize(jmg_->getVariableCount());
  return true;
}

bool constraint_samplers::JointConstraintSampler::sample(kinematic_state::JointStateGroup *jsg, const kinematic_state::KinematicState & /* ks */,
                                                         unsigned int /* max_attempts */)
{
  // sample the unbounded joints first (in case some joint variables are bounded)
  for (std::size_t i = 0 ; i < unbounded_.size() ; ++i)
  {
    std::vector<double> v;
    unbounded_[i]->getVariableRandomValues(random_number_generator_, v);
    for (std::size_t j = 0 ; j < v.size() ; ++j)
      values_[uindex_[i] + j] = v[j];
  }

  // enforce the constraints for the constrained components (could be all of them)
  for (std::size_t i = 0 ; i < bounds_.size() ; ++i)
    values_[index_[i]] = random_number_generator_.uniformReal(bounds_[i].first, bounds_[i].second);

  jsg->setVariableValues(values_);


  // we are always successful
  return true;
}

constraint_samplers::IKSamplingPose::IKSamplingPose(void)
{
}

constraint_samplers::IKSamplingPose::IKSamplingPose(const kinematic_constraints::PositionConstraint &pc) : position_constraint_(new kinematic_constraints::PositionConstraint(pc))
{
}

constraint_samplers::IKSamplingPose::IKSamplingPose(const kinematic_constraints::OrientationConstraint &oc) : orientation_constraint_(new kinematic_constraints::OrientationConstraint(oc))
{
}

constraint_samplers::IKSamplingPose::IKSamplingPose(const kinematic_constraints::PositionConstraint &pc, const kinematic_constraints::OrientationConstraint &oc) : 
  position_constraint_(new kinematic_constraints::PositionConstraint(pc)), orientation_constraint_(new kinematic_constraints::OrientationConstraint(oc))
{
}

constraint_samplers::IKSamplingPose::IKSamplingPose(const boost::shared_ptr<kinematic_constraints::PositionConstraint> &pc) : position_constraint_(pc)
{
}

constraint_samplers::IKSamplingPose::IKSamplingPose(const boost::shared_ptr<kinematic_constraints::OrientationConstraint> &oc) : orientation_constraint_(oc)
{
}

constraint_samplers::IKSamplingPose::IKSamplingPose(const boost::shared_ptr<kinematic_constraints::PositionConstraint> &pc, const boost::shared_ptr<kinematic_constraints::OrientationConstraint> &oc) : position_constraint_(pc), orientation_constraint_(oc)
{
}

bool constraint_samplers::IKConstraintSampler::setup(const IKSamplingPose &sp)
{
  sampling_pose_ = sp;
  ik_timeout_ = 0.5;
  if (sampling_pose_.position_constraint_ && sampling_pose_.orientation_constraint_)
    if (sampling_pose_.position_constraint_->getLinkModel()->getName() != sampling_pose_.orientation_constraint_->getLinkModel()->getName())
    {
      logError("Position and orientation constraints need to be specified for the same link in order to use IK-based sampling");
      return false;
    }
  if (sampling_pose_.position_constraint_ && sampling_pose_.position_constraint_->mobileReferenceFrame())
    frame_depends_.push_back(sampling_pose_.position_constraint_->getReferenceFrame());
  if (sampling_pose_.orientation_constraint_ && sampling_pose_.orientation_constraint_->mobileReferenceFrame())
    frame_depends_.push_back(sampling_pose_.orientation_constraint_->getReferenceFrame());
  ik_alloc_ = jmg_->getSolverAllocators().first;
  return true;
}

bool constraint_samplers::IKConstraintSampler::configure(const moveit_msgs::Constraints &constr)
{
  for (std::size_t p = 0 ; p < constr.position_constraints.size() ; ++p)
    for (std::size_t o = 0 ; o < constr.orientation_constraints.size() ; ++o)
      if (constr.position_constraints[p].link_name == constr.orientation_constraints[o].link_name)
      {
        boost::shared_ptr<kinematic_constraints::PositionConstraint> pc(new kinematic_constraints::PositionConstraint(scene_->getKinematicModel(), scene_->getTransforms()));
        boost::shared_ptr<kinematic_constraints::OrientationConstraint> oc(new kinematic_constraints::OrientationConstraint(scene_->getKinematicModel(), scene_->getTransforms()));
        if (pc->configure(constr.position_constraints[p]) && oc->configure(constr.orientation_constraints[o]))
          return setup(IKSamplingPose(pc, oc));
      }
  
  for (std::size_t p = 0 ; p < constr.position_constraints.size() ; ++p)
  {   
    boost::shared_ptr<kinematic_constraints::PositionConstraint> pc(new kinematic_constraints::PositionConstraint(scene_->getKinematicModel(), scene_->getTransforms()));
    if (pc->configure(constr.position_constraints[p]))
      return setup(IKSamplingPose(pc));
  }
  
  for (std::size_t o = 0 ; o < constr.orientation_constraints.size() ; ++o)
  {            
    boost::shared_ptr<kinematic_constraints::OrientationConstraint> oc(new kinematic_constraints::OrientationConstraint(scene_->getKinematicModel(), scene_->getTransforms()));
      if (oc->configure(constr.orientation_constraints[o]))
        return setup(IKSamplingPose(oc));
  }
  return false;
}

double constraint_samplers::IKConstraintSampler::getSamplingVolume(void) const
{
  double v = 1.0;
  if (sampling_pose_.position_constraint_)
  {
    const std::vector<bodies::BodyPtr> &b = sampling_pose_.position_constraint_->getConstraintRegions();
    double vol = 0;
    for (std::size_t i = 0 ; i < b.size() ; ++i)
      vol += b[i]->computeVolume();
    if (!b.empty())
      v *= vol;
  }
  
  if (sampling_pose_.orientation_constraint_)
    v *= sampling_pose_.orientation_constraint_->getXAxisTolerance() * sampling_pose_.orientation_constraint_->getYAxisTolerance() * sampling_pose_.orientation_constraint_->getZAxisTolerance();
  return v;
}

const std::string& constraint_samplers::IKConstraintSampler::getLinkName(void) const
{
  if (sampling_pose_.orientation_constraint_)
    return sampling_pose_.orientation_constraint_->getLinkModel()->getName();
  return sampling_pose_.position_constraint_->getLinkModel()->getName();    
}

bool constraint_samplers::IKConstraintSampler::loadIKSolver(void)
{  
  if (kb_)
    return true;

  if (!ik_alloc_)
  {
    logError("No IK allocator specified");
    return false;
  }
  
  // allocate the solver
  kb_ = ik_alloc_(jmg_);
  if (!kb_)
  {
    logError("Failed to allocate IK solver for group '%s'", jmg_->getName().c_str());
    return false;
  }
  else
    logDebug("IKConstraintSampler successfully loaded IK solver");
  
  // the ik solver must cover the same joints as the group
  const std::vector<std::string> &ik_jnames = kb_->getJointNames();
  const std::map<std::string, unsigned int> &g_map = jmg_->getJointVariablesIndexMap();
  
  if (ik_jnames.size() != g_map.size())
  {
    logError("Group '%s' does not have the same set of joints as the employed IK solver", jmg_->getName().c_str());
    return false;
  }
  
  // compute a mapping between the group state and the IK solution
  ik_joint_bijection_.clear();    
  for (std::size_t i = 0 ; i < ik_jnames.size() ; ++i)
  {
    std::map<std::string, unsigned int>::const_iterator it = g_map.find(ik_jnames[i]);
    if (it == g_map.end())
    {
      logError("IK solver computes joint values for joint '%s' but group '%s' does not contain such a joint.", ik_jnames[i].c_str(), jmg_->getName().c_str());
      return false;
    }
    const kinematic_model::JointModel *jm = jmg_->getJointModel(ik_jnames[i]);
    for (unsigned int k = 0 ; k < jm->getVariableCount() ; ++k)
      ik_joint_bijection_.push_back(it->second + k);
  }
  
  // check if we need to transform the request into the coordinate frame expected by IK
  ik_frame_ = kb_->getBaseFrame();
  transform_ik_ = ik_frame_ != jmg_->getParentModel()->getModelFrame();
  if (transform_ik_)
    if (!jmg_->getParentModel()->hasLinkModel(ik_frame_))
    {
      logError("The IK solver expects requests in frame '%s' but this frame is not known to the sampler. Ignoring transformation (IK may fail)", ik_frame_.c_str());
      transform_ik_ = false;
    }
  
  // check if IK is performed for the desired link
  bool wrong_link = false;
  if (sampling_pose_.position_constraint_)
  {
    if (kb_->getTipFrame() != sampling_pose_.position_constraint_->getLinkModel()->getName())
      wrong_link = true;
  }
  else
    if (kb_->getTipFrame() != sampling_pose_.orientation_constraint_->getLinkModel()->getName())
      wrong_link = true;
  if (wrong_link)
  {
    logError("IK cannot be performed for link '%s'. The solver can report IK solutions for link '%s'.", 
             sampling_pose_.position_constraint_ ? sampling_pose_.position_constraint_->getLinkModel()->getName().c_str() : sampling_pose_.orientation_constraint_->getLinkModel()->getName().c_str(), kb_->getTipFrame().c_str());
    return false;
  }
  
  return true;
}

bool constraint_samplers::IKConstraintSampler::samplePose(Eigen::Vector3d &pos, Eigen::Quaterniond &quat,
                                                          const kinematic_state::KinematicState &ks,
                                                          unsigned int max_attempts)
{  
  if (sampling_pose_.position_constraint_)
  {
    const std::vector<bodies::BodyPtr> &b = sampling_pose_.position_constraint_->getConstraintRegions();
    if (!b.empty())
    {
      bool found = false;
      std::size_t k = random_number_generator_.uniformInteger(0, b.size() - 1);
      for (std::size_t i = 0 ; i < b.size() ; ++i)
        if (b[(i+k) % b.size()]->samplePointInside(random_number_generator_, max_attempts, pos))
        {
          found = true;
          break;
        }
      if (!found)
      {   
        logError("Unable to sample a point inside the constraint region");
        return false;
      }
    }
    else
    {   
      logError("Unable to sample a point inside the constraint region. Constraint region is empty when it should not be.");
      return false;
    }
    
    // if this constraint is with respect a mobile frame, we need to convert this rotation to the root frame of the model
    if (sampling_pose_.position_constraint_->mobileReferenceFrame())
    {
      const kinematic_state::LinkState *ls = ks.getLinkState(sampling_pose_.position_constraint_->getReferenceFrame());
      pos = ls->getGlobalLinkTransform() * pos;
    }
  }
  else
  {
    // do FK for rand state
    kinematic_state::KinematicState tempState(ks);
    kinematic_state::JointStateGroup *tmp = tempState.getJointStateGroup(jmg_->getName());
    if (tmp)
    {
      tmp->setToRandomValues();
      pos = tempState.getLinkState(sampling_pose_.orientation_constraint_->getLinkModel()->getName())->getGlobalLinkTransform().translation();
    }
    else
    {
      logError("Passed a JointStateGroup for a mismatching JointModelGroup");
      return false;
    }
  }
  
  if (sampling_pose_.orientation_constraint_)
  {
    // sample a rotation matrix within the allowed bounds
    double angle_x = 2.0 * (random_number_generator_.uniform01() - 0.5) * sampling_pose_.orientation_constraint_->getXAxisTolerance();
    double angle_y = 2.0 * (random_number_generator_.uniform01() - 0.5) * sampling_pose_.orientation_constraint_->getYAxisTolerance();
    double angle_z = 2.0 * (random_number_generator_.uniform01() - 0.5) * sampling_pose_.orientation_constraint_->getZAxisTolerance();
    Eigen::Affine3d diff(Eigen::AngleAxisd(angle_x, Eigen::Vector3d::UnitX())
                         * Eigen::AngleAxisd(angle_y, Eigen::Vector3d::UnitY())
                         * Eigen::AngleAxisd(angle_z, Eigen::Vector3d::UnitZ()));
    Eigen::Affine3d reqr(sampling_pose_.orientation_constraint_->getDesiredRotationMatrix() * diff.rotation());
    quat = Eigen::Quaterniond(reqr.rotation());

    // if this constraint is with respect a mobile frame, we need to convert this rotation to the root frame of the model
    if (sampling_pose_.orientation_constraint_->mobileReferenceFrame())
    {
      const kinematic_state::LinkState *ls = ks.getLinkState(sampling_pose_.orientation_constraint_->getReferenceFrame());
      Eigen::Affine3d rt(ls->getGlobalLinkTransform().rotation() * quat.toRotationMatrix());
      quat = Eigen::Quaterniond(rt.rotation());
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
  
  // we now have the transform we wish to perform IK for, in the planning frame
  
  if (transform_ik_)
  {
    // we need to convert this transform to the frame expected by the IK solver
    // both the planning frame and the frame for the IK are assumed to be robot links
    Eigen::Affine3d ikq(Eigen::Translation3d(pos) * quat.toRotationMatrix());
    
    const kinematic_state::LinkState *ls = ks.getLinkState(ik_frame_);
    ikq = ls->getGlobalLinkTransform().inverse() * ikq;
    
    pos = ikq.translation();
    quat = Eigen::Quaterniond(ikq.rotation());
  }
  
  return true;
}

bool constraint_samplers::IKConstraintSampler::sample(kinematic_state::JointStateGroup *jsg, const kinematic_state::KinematicState &ks, unsigned int max_attempts)
{
  // make sure we at least have a chance of sampling using IK; we need at least some kind of constraint
  if (!sampling_pose_.position_constraint_ && !sampling_pose_.orientation_constraint_)
    return false;

  // load an IK solver if we need to 
  if (!loadIKSolver())
  {
    kb_.reset();
    return false;
  }

  for (unsigned int a = 0 ; a < max_attempts ; ++a)
  {
    // sample a point in the constraint region
    Eigen::Vector3d point;    
    Eigen::Quaterniond quat;
    if (!samplePose(point, quat, ks, max_attempts))
      return false;

    geometry_msgs::Pose ik_query;
    ik_query.position.x = point.x();
    ik_query.position.y = point.y();
    ik_query.position.z = point.z();
    ik_query.orientation.x = quat.x();
    ik_query.orientation.y = quat.y();
    ik_query.orientation.z = quat.z();
    ik_query.orientation.w = quat.w();        
    
    if (callIK(ik_query, ik_timeout_, jsg))
      return true; 
  }
  return false;
}

bool constraint_samplers::IKConstraintSampler::callIK(const geometry_msgs::Pose &ik_query, double timeout, kinematic_state::JointStateGroup *jsg)
{
  // sample a seed value
  std::vector<double> vals;
  jmg_->getVariableRandomValues(random_number_generator_, vals);
  assert(vals.size() == ik_joint_bijection_.size());
  std::vector<double> seed(ik_joint_bijection_.size(), 0.0);
  for (std::size_t i = 0 ; i < ik_joint_bijection_.size() ; ++i)
    seed[ik_joint_bijection_[i]] = vals[i];
  
  std::vector<double> ik_sol;
  moveit_msgs::MoveItErrorCodes error;
 
  if (kb_->searchPositionIK(ik_query, seed, timeout, ik_sol, error))
  {
    assert(ik_sol.size() == ik_joint_bijection_.size());
    std::vector<double> solution(ik_joint_bijection_.size());
    for (std::size_t i = 0 ; i < ik_joint_bijection_.size() ; ++i)
      solution[i] = ik_sol[ik_joint_bijection_[i]];
    jsg->setVariableValues(solution);
    assert((!sampling_pose_.orientation_constraint_ || sampling_pose_.orientation_constraint_->decide(*jsg->getKinematicState(), false).satisfied) && 
	   (!sampling_pose_.position_constraint_ || sampling_pose_.position_constraint_->decide(*jsg->getKinematicState(), false).satisfied));
    return true;
  }
  else
    if (error.val != moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION)
      logError("IK solver failed with error %d", error.val);
  return false;
}
