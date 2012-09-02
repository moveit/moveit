/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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

/* Author: Ioan Sucan, E. Gil Jones, Sachin Chitta */

#include "planning_models/kinematic_state.h"
#include <eigen_conversions/eigen_msg.h>
#include <ros/console.h>

planning_models::KinematicState::JointStateGroup::JointStateGroup(planning_models::KinematicState *state,
                                                                  const planning_models::KinematicModel::JointModelGroup *jmg) :
  kinematic_state_(state), joint_model_group_(jmg)
{
  const std::vector<const KinematicModel::JointModel*>& joint_model_vector = jmg->getJointModels();
  for (std::size_t i = 0; i < joint_model_vector.size() ; ++i)
  {
    if (!kinematic_state_->hasJointState(joint_model_vector[i]->getName()))
    {
      ROS_ERROR_STREAM("No joint state for group joint name " << joint_model_vector[i]->getName());
      continue;
    }
    JointState* js = kinematic_state_->getJointState(joint_model_vector[i]->getName());
    joint_state_vector_.push_back(js);
    joint_state_map_[joint_model_vector[i]->getName()] = js;
  }
  const std::vector<const KinematicModel::LinkModel*>& link_model_vector = jmg->getUpdatedLinkModels();
  for (unsigned int i = 0; i < link_model_vector.size(); i++)
  {
    if (!kinematic_state_->hasLinkState(link_model_vector[i]->getName()))
    {
      ROS_ERROR_STREAM("No link state for link joint name " << link_model_vector[i]->getName());
      continue;
    }
    LinkState* ls = kinematic_state_->getLinkState(link_model_vector[i]->getName());
    updated_links_.push_back(ls);
  }
  
  const std::vector<const KinematicModel::JointModel*>& joint_root_vector = jmg->getJointRoots();
  for (std::size_t i = 0; i < joint_root_vector.size(); ++i)
  {
    JointState* js = kinematic_state_->getJointState(joint_root_vector[i]->getName());
    if (js)
      joint_roots_.push_back(js);
  }
}

planning_models::KinematicState::JointStateGroup::~JointStateGroup(void)
{
}

random_numbers::RandomNumberGenerator& planning_models::KinematicState::JointStateGroup::getRandomNumberGenerator(void)
{
  if (!rng_)
    rng_.reset(new random_numbers::RandomNumberGenerator());
  return *rng_;
}

bool planning_models::KinematicState::JointStateGroup::hasJointState(const std::string &joint) const
{
  return joint_state_map_.find(joint) != joint_state_map_.end();
}

bool planning_models::KinematicState::JointStateGroup::updatesLinkState(const std::string& link) const
{
  for (std::size_t i = 0 ; i < updated_links_.size() ; ++i)
    if (updated_links_[i]->getName() == link)
      return true;
  return false;
}

bool planning_models::KinematicState::JointStateGroup::setStateValues(const std::vector<double> &joint_state_values)
{
  if (joint_state_values.size() != getVariableCount())
  {
    ROS_ERROR("Incorrect variable count specified for array of joint values. Expected %u but got %u values",
              getVariableCount(), (int)joint_state_values.size());
    return false;
  }
  
  unsigned int value_counter = 0;
  for(unsigned int i = 0; i < joint_state_vector_.size(); i++)
  {
    unsigned int dim = joint_state_vector_[i]->getVariableCount();
    if (dim != 0)
    {
      joint_state_vector_[i]->setVariableValues(&joint_state_values[value_counter]);
      value_counter += dim;
    }
  }
  updateLinkTransforms();
  return true;
}

void planning_models::KinematicState::JointStateGroup::setStateValues(const std::map<std::string, double>& joint_state_map)
{
  for(unsigned int i = 0; i < joint_state_vector_.size(); ++i)
    joint_state_vector_[i]->setVariableValues(joint_state_map);
  updateLinkTransforms();
}

void planning_models::KinematicState::JointStateGroup::setStateValues(const sensor_msgs::JointState& js)
{
  std::map<std::string, double> v;
  for (std::size_t i = 0 ; i < js.name.size() ; ++i)
    v[js.name[i]] = js.position[i];
  setStateValues(v);
}
                    
void planning_models::KinematicState::JointStateGroup::updateLinkTransforms(void)
{
  for(unsigned int i = 0; i < updated_links_.size(); ++i)
    updated_links_[i]->computeTransform();
}

void planning_models::KinematicState::JointStateGroup::copyFrom(const JointStateGroup *other_jsg)
{
  const std::vector<JointState*> &ojsv = other_jsg->getJointStateVector();
  for (std::size_t i = 0 ; i < ojsv.size() ; ++i)
    joint_state_vector_[i]->setVariableValues(ojsv[i]->getVariableValues());
  updateLinkTransforms();
}

void planning_models::KinematicState::JointStateGroup::setToDefaultValues(void)
{
  std::map<std::string, double> default_joint_values;
  for (std::size_t i = 0  ; i < joint_state_vector_.size() ; ++i)
    joint_state_vector_[i]->getJointModel()->getDefaultValues(default_joint_values);
  setStateValues(default_joint_values);
}

bool planning_models::KinematicState::JointStateGroup::setToDefaultState(const std::string &name)
{
  std::map<std::string, double> default_joint_values;
  if (!joint_model_group_->getDefaultValues(name, default_joint_values))
    return false;
  setStateValues(default_joint_values);
  return true;
}

void planning_models::KinematicState::JointStateGroup::setToRandomValues(void)
{
  random_numbers::RandomNumberGenerator &rng = getRandomNumberGenerator();
  std::vector<double> random_joint_states;
  joint_model_group_->getRandomValues(rng, random_joint_states);
  setStateValues(random_joint_states);
}

void planning_models::KinematicState::JointStateGroup::getGroupStateValues(std::vector<double>& joint_state_values) const
{
  joint_state_values.clear();
  for(unsigned int i = 0; i < joint_state_vector_.size(); i++)
  {
    const std::vector<double> &jv = joint_state_vector_[i]->getVariableValues();
    joint_state_values.insert(joint_state_values.end(), jv.begin(), jv.end());
  }
}

bool planning_models::KinematicState::JointStateGroup::satisfiesBounds(void) const
{
  for (std::size_t i = 0 ; i < joint_state_vector_.size() ; ++i)
    if (!joint_state_vector_[i]->satisfiesBounds())
      return false;
  return true;
}

void planning_models::KinematicState::JointStateGroup::enforceBounds(void)
{
  for (std::size_t i = 0 ; i < joint_state_vector_.size() ; ++i)
    joint_state_vector_[i]->enforceBounds();
  updateLinkTransforms();
}

double planning_models::KinematicState::JointStateGroup::distance(const JointStateGroup *other) const
{
  double d = 0.0;
  for (std::size_t i = 0 ; i < joint_state_vector_.size() ; ++i)
    d += joint_state_vector_[i]->distance(other->joint_state_vector_[i]) * joint_state_vector_[i]->getJointModel()->getDistanceFactor();
  return d;
}

void planning_models::KinematicState::JointStateGroup::interpolate(const JointStateGroup *to, const double t, JointStateGroup *dest) const
{
  for (std::size_t i = 0 ; i < joint_state_vector_.size() ; ++i)
    joint_state_vector_[i]->interpolate(to->joint_state_vector_[i], t, dest->joint_state_vector_[i]);
  dest->updateLinkTransforms();
}

void planning_models::KinematicState::JointStateGroup::getGroupStateValues(std::map<std::string,double>& joint_state_values) const
{
  joint_state_values.clear();
  for (std::size_t i = 0; i < joint_state_vector_.size(); ++i)
  {
    const std::vector<double> &jsv = joint_state_vector_[i]->getVariableValues();
    const std::vector<std::string> &jsn = joint_state_vector_[i]->getVariableNames();
    for (std::size_t j = 0 ; j < jsv.size(); ++j)
      joint_state_values[jsn[j]] = jsv[j];
  }
}

planning_models::KinematicState::JointState* planning_models::KinematicState::JointStateGroup::getJointState(const std::string &name) const
{
  std::map<std::string, JointState*>::const_iterator it = joint_state_map_.find(name);
  if (it == joint_state_map_.end())
  {
    ROS_ERROR("Joint '%s' not found", name.c_str());
    return NULL;
  }
  else
    return it->second;
}

bool planning_models::KinematicState::JointStateGroup::setFromIK(const geometry_msgs::Pose &pose, double timeout)
{
  const kinematics::KinematicsBaseConstPtr& solver = joint_model_group_->getSolverInstance();
  if (!solver)
    return false;
  return setFromIK(pose, solver->getTipFrame(), timeout);
}

bool planning_models::KinematicState::JointStateGroup::setFromIK(const geometry_msgs::Pose &pose, const std::string &tip, double timeout)
{
  Eigen::Affine3d mat;
  tf::poseMsgToEigen(pose, mat);
  return setFromIK(mat, tip, timeout);
}

bool planning_models::KinematicState::JointStateGroup::setFromIK(const Eigen::Affine3d &pose, double timeout)
{ 
  const kinematics::KinematicsBaseConstPtr& solver = joint_model_group_->getSolverInstance();
  if (!solver)
    return false;
  return setFromIK(pose, solver->getTipFrame(), timeout);
}

bool planning_models::KinematicState::JointStateGroup::setFromIK(const Eigen::Affine3d &pose_in, const std::string &tip_in, double timeout)
{
  const kinematics::KinematicsBaseConstPtr& solver = joint_model_group_->getSolverInstance();
  if (!solver)
    return false;

  Eigen::Affine3d pose = pose_in;
  std::string tip = tip_in;
  
  // bring the pose to the frame of the IK solver
  const std::string &ik_frame = solver->getBaseFrame();
  if (ik_frame != joint_model_group_->getParentModel()->getModelFrame())
  {
    const LinkState *ls = getParentState()->getLinkState(ik_frame);
    if (!ls)
      return false;
    pose = ls->getGlobalLinkTransform().inverse() * pose;
  }

  // see if the tip frame can be transformed via fixed transforms to the frame known to the IK solver
  const std::string &tip_frame = solver->getTipFrame();
  if (tip != tip_frame)
  {
    if (getParentState()->hasAttachedBody(tip))
    {
      const AttachedBody *ab = getParentState()->getAttachedBody(tip);
      const EigenSTL::vector_Affine3d &ab_trans = ab->getFixedTransforms();
      if (ab_trans.size() != 1)
      {
        ROS_ERROR("Cannot use an attached body with multiple geometries as a reference frame.");
        return false;
      }
      tip = ab->getAttachedLinkName();
      pose = pose * ab_trans[0].inverse();
    }
    if (tip != tip_frame)
    {
      const KinematicModel::LinkModel *lm = joint_model_group_->getParentModel()->getLinkModel(tip);
      if (!lm)
        return false;
      const KinematicModel::LinkModel::AssociatedFixedTransformMap &fixed_links = lm->getAssociatedFixedTransforms();
      for (std::map<const KinematicModel::LinkModel*, Eigen::Affine3d>::const_iterator it = fixed_links.begin() ; it != fixed_links.end() ; ++it)
        if (it->first->getName() == tip_frame)
        {
          tip = tip_frame;
          pose = pose * it->second;
          break;
        }
    }
  }
  
  if (tip != tip_frame)
  {
    ROS_ERROR_STREAM("Cannot compute IK for tip reference frame " << tip);
    return false;    
  }
  
  // sample a seed
  random_numbers::RandomNumberGenerator &rng = getRandomNumberGenerator();
  std::vector<double> seed;
  joint_model_group_->getRandomValues(rng, seed);

  Eigen::Quaterniond quat(pose.rotation());
  Eigen::Vector3d point(pose.translation());
  geometry_msgs::Pose ik_query;
  ik_query.position.x = point.x();
  ik_query.position.y = point.y();
  ik_query.position.z = point.z();
  ik_query.orientation.x = quat.x();
  ik_query.orientation.y = quat.y();
  ik_query.orientation.z = quat.z();
  ik_query.orientation.w = quat.w();
  
  // compute the IK solution
  std::vector<double> ik_sol;
  moveit_msgs::MoveItErrorCodes error;
  if (solver->searchPositionIK(ik_query, seed, timeout, ik_sol, error))
  {
    const std::vector<unsigned int> &bij = joint_model_group_->getKinematicsSolverJointBijection();
    std::vector<double> solution(bij.size());
    for (std::size_t i = 0 ; i < bij.size() ; ++i)
      solution[i] = ik_sol[bij[i]];
    setStateValues(solution);
    return true;
  }
  else
    return false;
}

bool planning_models::KinematicState::JointStateGroup::getJacobian(const std::string &link_name,
								   const Eigen::Vector3d &reference_point_position, 
								   Eigen::MatrixXd& jacobian) const
{
  if(!joint_model_group_->isChain())
  {
    ROS_ERROR("Will compute Jacobian only for a chain");
    return false;
  }
  if(!joint_model_group_->isUpdatedLink(link_name))
  {
    ROS_ERROR("Link name does not exist in this chain or is not a child for this chain");
    return false;
  }

  const planning_models::KinematicModel::JointModel* root_joint_model = (joint_model_group_->getJointRoots())[0];
  ROS_DEBUG_STREAM("ROOT_LINK" << root_joint_model->getParentLinkModel()->getName());
  const planning_models::KinematicState::LinkState *root_link_state = kinematic_state_->getLinkState(root_joint_model->getParentLinkModel()->getName());
  Eigen::Affine3d reference_transform = root_link_state ? root_link_state->getGlobalLinkTransform() : kinematic_state_->getRootTransform();
  reference_transform = reference_transform.inverse();
  jacobian = Eigen::MatrixXd::Zero(6, joint_model_group_->getVariableCount());

  const planning_models::KinematicState::LinkState *link_state = kinematic_state_->getLinkState(link_name);
  Eigen::Affine3d link_transform = reference_transform*link_state->getGlobalLinkTransform();
  Eigen::Vector3d point_transform = link_transform*reference_point_position;

  ROS_DEBUG("Point from reference origin expressed in world coordinates: %f %f %f",
            point_transform.x(),
            point_transform.y(),
            point_transform.z());

  Eigen::Vector3d joint_axis;
  Eigen::Affine3d joint_transform;

  while(link_state)
  {
    ROS_DEBUG("Link: %s, %f %f %f",link_state->getName().c_str(),
             link_state->getGlobalLinkTransform().translation().x(),
             link_state->getGlobalLinkTransform().translation().y(),
             link_state->getGlobalLinkTransform().translation().z());    
    ROS_DEBUG("Joint: %s",link_state->getParentJointState()->getName().c_str());

    if(joint_model_group_->isActiveDOF(link_state->getParentJointState()->getJointModel()->getName()))
    {
      if(link_state->getParentJointState()->getJointModel()->getType() == planning_models::KinematicModel::JointModel::REVOLUTE)
	{
	  unsigned int joint_index = joint_model_group_->getJointVariablesIndexMap().find(link_state->getParentJointState()->getJointModel()->getName())->second;
	  joint_transform = reference_transform*link_state->getGlobalLinkTransform();
	  joint_axis = joint_transform.rotation()*(dynamic_cast<const planning_models::KinematicModel::RevoluteJointModel*>(link_state->getParentJointState()->getJointModel()))->getAxis();
	  jacobian.block<3,1>(0,joint_index) = joint_axis.cross(point_transform - joint_transform.translation());
	  jacobian.block<3,1>(3,joint_index) = joint_axis;
	}
      if(link_state->getParentJointState()->getJointModel()->getType() == planning_models::KinematicModel::JointModel::PRISMATIC)
	{
	  unsigned int joint_index = joint_model_group_->getJointVariablesIndexMap().find(link_state->getParentJointState()->getJointModel()->getName())->second;
	  joint_transform = reference_transform*link_state->getGlobalLinkTransform();
	  joint_axis = joint_transform*(dynamic_cast<const planning_models::KinematicModel::PrismaticJointModel*>(link_state->getParentJointState()->getJointModel()))->getAxis();
	  jacobian.block<3,1>(0,joint_index) = joint_axis;
	}
      if(link_state->getParentJointState()->getJointModel()->getType() == planning_models::KinematicModel::JointModel::PLANAR)
	{
	  unsigned int joint_index = joint_model_group_->getJointVariablesIndexMap().find(link_state->getParentJointState()->getJointModel()->getName())->second;
	  joint_transform = reference_transform*link_state->getGlobalLinkTransform();
	  joint_axis = joint_transform*Eigen::Vector3d(1.0,0.0,0.0);
	  jacobian.block<3,1>(0,joint_index) = joint_axis;
	  joint_axis = joint_transform*Eigen::Vector3d(0.0,1.0,0.0);
	  jacobian.block<3,1>(0,joint_index+1) = joint_axis;
	  joint_axis = joint_transform*Eigen::Vector3d(0.0,0.0,1.0);
	  jacobian.block<3,1>(0,joint_index+2) = joint_axis.cross(point_transform - joint_transform.translation());
	  jacobian.block<3,1>(3,joint_index+2) = joint_axis;
	}
    }
    if(link_state->getParentJointState()->getJointModel() == root_joint_model)
      break;
    link_state = link_state->getParentLinkState();
  }
  return true;
}
