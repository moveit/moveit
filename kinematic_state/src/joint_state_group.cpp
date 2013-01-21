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

#include <moveit/kinematic_state/kinematic_state.h>
#include <eigen_conversions/eigen_msg.h>
#include <boost/bind.hpp>

#include <Eigen/SVD>

kinematic_state::JointStateGroup::JointStateGroup(KinematicState *state,
                                                  const kinematic_model::JointModelGroup *jmg) :
  kinematic_state_(state), joint_model_group_(jmg)
{
  const std::vector<const kinematic_model::JointModel*>& joint_model_vector = jmg->getJointModels();
  for (std::size_t i = 0; i < joint_model_vector.size() ; ++i)
  {
    if (!kinematic_state_->hasJointState(joint_model_vector[i]->getName()))
    {
      logError("No joint state for group joint name '%s'", joint_model_vector[i]->getName().c_str());
      continue;
    }
    JointState* js = kinematic_state_->getJointState(joint_model_vector[i]->getName());
    joint_state_vector_.push_back(js);
    joint_state_map_[joint_model_vector[i]->getName()] = js;
  }
  const std::vector<const kinematic_model::LinkModel*>& link_model_vector = jmg->getUpdatedLinkModels();
  for (unsigned int i = 0; i < link_model_vector.size(); i++)
  {
    if (!kinematic_state_->hasLinkState(link_model_vector[i]->getName()))
    {
      logError("No link state for link joint name '%s'", link_model_vector[i]->getName().c_str());
      continue;
    }
    LinkState* ls = kinematic_state_->getLinkState(link_model_vector[i]->getName());
    updated_links_.push_back(ls);
  }
  
  const std::vector<const kinematic_model::JointModel*>& joint_root_vector = jmg->getJointRoots();
  for (std::size_t i = 0; i < joint_root_vector.size(); ++i)
  {
    JointState* js = kinematic_state_->getJointState(joint_root_vector[i]->getName());
    if (js)
      joint_roots_.push_back(js);
  }
}

kinematic_state::JointStateGroup::~JointStateGroup(void)
{
}

random_numbers::RandomNumberGenerator& kinematic_state::JointStateGroup::getRandomNumberGenerator(void)
{
  if (!rng_)
    rng_.reset(new random_numbers::RandomNumberGenerator());
  return *rng_;
}

bool kinematic_state::JointStateGroup::hasJointState(const std::string &joint) const
{
  return joint_state_map_.find(joint) != joint_state_map_.end();
}

bool kinematic_state::JointStateGroup::updatesLinkState(const std::string& link) const
{
  for (std::size_t i = 0 ; i < updated_links_.size() ; ++i)
    if (updated_links_[i]->getName() == link)
      return true;
  return false;
}

bool kinematic_state::JointStateGroup::setVariableValues(const std::vector<double> &joint_state_values)
{
  if (joint_state_values.size() != getVariableCount())
  {
    logError("Incorrect variable count specified for array of joint values. Expected %u but got %u values",
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

bool kinematic_state::JointStateGroup::setVariableValues(const Eigen::VectorXd &joint_state_values)
{
  std::vector<double> values;
  for (std::size_t i = 0; i < joint_state_values.rows(); i++)
  {
    values.push_back(joint_state_values(i));
  }
  setVariableValues(values);
}

void kinematic_state::JointStateGroup::setVariableValues(const std::map<std::string, double>& joint_state_map)
{
  for(unsigned int i = 0; i < joint_state_vector_.size(); ++i)
    joint_state_vector_[i]->setVariableValues(joint_state_map);
  updateLinkTransforms();
}

void kinematic_state::JointStateGroup::setVariableValues(const sensor_msgs::JointState& js)
{
  std::map<std::string, double> v;
  for (std::size_t i = 0 ; i < js.name.size() ; ++i)
    v[js.name[i]] = js.position[i];
  setVariableValues(v);
}
                    
void kinematic_state::JointStateGroup::updateLinkTransforms(void)
{
  for(unsigned int i = 0; i < updated_links_.size(); ++i)
    updated_links_[i]->computeTransform();
}

kinematic_state::JointStateGroup& kinematic_state::JointStateGroup::operator=(const JointStateGroup &other)
{
  if (this != &other)
    copyFrom(other);
  return *this;
}

void kinematic_state::JointStateGroup::copyFrom(const JointStateGroup &other_jsg)
{
  const std::vector<JointState*> &ojsv = other_jsg.getJointStateVector();
  for (std::size_t i = 0 ; i < ojsv.size() ; ++i)
    joint_state_vector_[i]->setVariableValues(ojsv[i]->getVariableValues());
  updateLinkTransforms();
}

void kinematic_state::JointStateGroup::setToDefaultValues(void)
{
  std::map<std::string, double> default_joint_values;
  for (std::size_t i = 0  ; i < joint_state_vector_.size() ; ++i)
    joint_state_vector_[i]->getJointModel()->getVariableDefaultValues(default_joint_values);
  setVariableValues(default_joint_values);
}

bool kinematic_state::JointStateGroup::setToDefaultState(const std::string &name)
{
  std::map<std::string, double> default_joint_values;
  if (!joint_model_group_->getVariableDefaultValues(name, default_joint_values))
    return false;
  setVariableValues(default_joint_values);
  return true;
}

void kinematic_state::JointStateGroup::setToRandomValues(void)
{
  random_numbers::RandomNumberGenerator &rng = getRandomNumberGenerator();
  std::vector<double> random_joint_states;
  joint_model_group_->getVariableRandomValues(rng, random_joint_states);
  setVariableValues(random_joint_states);
}

void kinematic_state::JointStateGroup::setToRandomValuesNearBy(const std::vector<double> &near, 
                                                               const std::map<kinematic_model::JointModel::JointType, double> &distance_map)
{
  random_numbers::RandomNumberGenerator &rng = getRandomNumberGenerator();
  std::vector<double> variable_values;
  joint_model_group_->getVariableRandomValuesNearBy(rng, variable_values, near, distance_map);
  setVariableValues(variable_values);
}

void kinematic_state::JointStateGroup::setToRandomValuesNearBy(const std::vector<double> &near, 
                                                               const std::vector<double> &distances)
{
  random_numbers::RandomNumberGenerator &rng = getRandomNumberGenerator();
  std::vector<double> variable_values;
  joint_model_group_->getVariableRandomValuesNearBy(rng, variable_values, near, distances);
  setVariableValues(variable_values);
}

void kinematic_state::JointStateGroup::getVariableValues(std::vector<double>& joint_state_values) const
{
  joint_state_values.clear();
  for(unsigned int i = 0; i < joint_state_vector_.size(); i++)
  {
    const std::vector<double> &jv = joint_state_vector_[i]->getVariableValues();
    joint_state_values.insert(joint_state_values.end(), jv.begin(), jv.end());
  }
}

void kinematic_state::JointStateGroup::getVariableValues(Eigen::VectorXd& joint_state_values) const
{
  joint_state_values.resize(getVariableCount());
  unsigned int count = 0;
  for(unsigned int i = 0; i < joint_state_vector_.size(); i++)
  {
    const std::vector<double> &jv = joint_state_vector_[i]->getVariableValues();
    for(unsigned int j = 0; j < jv.size(); j++)
    {
      joint_state_values(count++) = jv[j];
    }
  }
}

bool kinematic_state::JointStateGroup::satisfiesBounds(double margin) const
{
  for (std::size_t i = 0 ; i < joint_state_vector_.size() ; ++i)
    if (!joint_state_vector_[i]->satisfiesBounds(margin))
      return false;
  return true;
}

void kinematic_state::JointStateGroup::enforceBounds(void)
{
  for (std::size_t i = 0 ; i < joint_state_vector_.size() ; ++i)
    joint_state_vector_[i]->enforceBounds();
  updateLinkTransforms();
}

double kinematic_state::JointStateGroup::infinityNormDistance(const JointStateGroup *other) const
{
  if (joint_state_vector_.empty())
    return 0.0;
  double max_d = joint_state_vector_[0]->distance(other->joint_state_vector_[0]);
  for (std::size_t i = 1 ; i < joint_state_vector_.size() ; ++i)
  {
    double d = joint_state_vector_[i]->distance(other->joint_state_vector_[i]);
    if (d > max_d)
      max_d = d;
  }
  return max_d;
}

double kinematic_state::JointStateGroup::distance(const JointStateGroup *other) const
{
  double d = 0.0;
  for (std::size_t i = 0 ; i < joint_state_vector_.size() ; ++i)
    d += joint_state_vector_[i]->distance(other->joint_state_vector_[i]) * joint_state_vector_[i]->getJointModel()->getDistanceFactor();
  return d;
}

void kinematic_state::JointStateGroup::interpolate(const JointStateGroup *to, const double t, JointStateGroup *dest) const
{
  for (std::size_t i = 0 ; i < joint_state_vector_.size() ; ++i)
    joint_state_vector_[i]->interpolate(to->joint_state_vector_[i], t, dest->joint_state_vector_[i]);
  dest->updateLinkTransforms();
}

void kinematic_state::JointStateGroup::getVariableValues(std::map<std::string, double>& joint_state_values) const
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

kinematic_state::JointState* kinematic_state::JointStateGroup::getJointState(const std::string &name) const
{
  std::map<std::string, JointState*>::const_iterator it = joint_state_map_.find(name);
  if (it == joint_state_map_.end())
  {
    logError("Joint '%s' not found", name.c_str());
    return NULL;
  }
  else
    return it->second;
}

bool kinematic_state::JointStateGroup::setFromIK(const geometry_msgs::Pose &pose, unsigned int attempts, double timeout, const StateValidityCallbackFn &constraint)
{
  const kinematics::KinematicsBaseConstPtr& solver = joint_model_group_->getSolverInstance();
  if (!solver)
  {
    logError("No kinematics solver instantiated for this group");    
    return false;
  }  
  return setFromIK(pose, solver->getTipFrame(), attempts, timeout, constraint);
}

bool kinematic_state::JointStateGroup::setFromIK(const geometry_msgs::Pose &pose, const std::string &tip, unsigned int attempts, double timeout, const StateValidityCallbackFn &constraint)
{
  Eigen::Affine3d mat;
  tf::poseMsgToEigen(pose, mat);
  return setFromIK(mat, tip,  attempts, timeout, constraint);
}

bool kinematic_state::JointStateGroup::setFromIK(const Eigen::Affine3d &pose, unsigned int attempts, double timeout, const StateValidityCallbackFn &constraint)
{ 
  const kinematics::KinematicsBaseConstPtr& solver = joint_model_group_->getSolverInstance();
  if (!solver)
  {
    logError("No kinematics solver instantiated for this group");    
    return false;
  }  
  static std::vector<double> consistency_limits;  
  return setFromIK(pose, solver->getTipFrame(), consistency_limits, attempts, timeout, constraint);  
}

bool kinematic_state::JointStateGroup::setFromIK(const Eigen::Affine3d &pose_in, const std::string &tip_in, unsigned int attempts, double timeout, const StateValidityCallbackFn &constraint)
{
  static std::vector<double> consistency_limits;  
  return setFromIK(pose_in, tip_in, consistency_limits, attempts, timeout, constraint);  
}

bool kinematic_state::JointStateGroup::setFromIK(const Eigen::Affine3d &pose_in, const std::string &tip_in, const std::vector<double> &consistency_limits, unsigned int attempts, double timeout, const StateValidityCallbackFn &constraint)
{
  const kinematics::KinematicsBaseConstPtr& solver = joint_model_group_->getSolverInstance();
  if (!solver)
  {
    logError("No kinematics solver instantiated for this group");    
    return false;
  }  

  Eigen::Affine3d pose = pose_in;
  std::string tip = tip_in;
  
  // bring the pose to the frame of the IK solver
  const std::string &ik_frame = solver->getBaseFrame();
  if (ik_frame != joint_model_group_->getParentModel()->getModelFrame())
  {
    const LinkState *ls = kinematic_state_->getLinkState(ik_frame);
    if (!ls)
      return false;
    pose = ls->getGlobalLinkTransform().inverse() * pose;
  }

    // see if the tip frame can be transformed via fixed transforms to the frame known to the IK solver
  const std::string &tip_frame = solver->getTipFrame();
  if (tip != tip_frame)
  {
    if (kinematic_state_->hasAttachedBody(tip))
    {
      const AttachedBody *ab = kinematic_state_->getAttachedBody(tip);
      const EigenSTL::vector_Affine3d &ab_trans = ab->getFixedTransforms();
      if (ab_trans.size() != 1)
      {
        logError("Cannot use an attached body with multiple geometries as a reference frame.");
        return false;
      }
      tip = ab->getAttachedLinkName();
      pose = pose * ab_trans[0].inverse();
    }
    if (tip != tip_frame)
    {
      const kinematic_model::LinkModel *lm = joint_model_group_->getParentModel()->getLinkModel(tip);
      if (!lm)
        return false;
      const kinematic_model::LinkModel::AssociatedFixedTransformMap &fixed_links = lm->getAssociatedFixedTransforms();
      for (std::map<const kinematic_model::LinkModel*, Eigen::Affine3d>::const_iterator it = fixed_links.begin() ; it != fixed_links.end() ; ++it)
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
    logError("Cannot compute IK for tip reference frame '%s'", tip.c_str());
    return false;    
  }

  // if no timeout has been specified, use the default one
  if (timeout < std::numeric_limits<double>::epsilon())
    timeout = joint_model_group_->getDefaultIKTimeout();
  
  if (attempts == 0)
    attempts = joint_model_group_->getDefaultIKAttempts();
  
  const std::vector<unsigned int> &bij = joint_model_group_->getKinematicsSolverJointBijection();
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
  
  kinematics::KinematicsBase::IKCallbackFn ik_callback_fn;
  if (constraint)
    ik_callback_fn = boost::bind(&JointStateGroup::ikCallbackFnAdapter, this, constraint, _1, _2, _3);
  
  bool first_seed = true;
  for (unsigned int st = 0 ; st < attempts ; ++st)
  {
    std::vector<double> seed(bij.size());
    
    // the first seed is the initial state
    if (first_seed)
    {
      first_seed = false;
      std::vector<double> initial_values;
      getVariableValues(initial_values);
      for (std::size_t i = 0 ; i < bij.size() ; ++i)
        seed[bij[i]] = initial_values[i];
    }
    else
    {
      // sample a random seed
      random_numbers::RandomNumberGenerator &rng = getRandomNumberGenerator();
      std::vector<double> random_values;
      joint_model_group_->getVariableRandomValues(rng, random_values);
      for (std::size_t i = 0 ; i < bij.size() ; ++i)
        seed[bij[i]] = random_values[i];
    }
    
    // compute the IK solution
    std::vector<double> ik_sol;
    moveit_msgs::MoveItErrorCodes error;
    if (ik_callback_fn ?
        solver->searchPositionIK(ik_query, seed, timeout, consistency_limits, ik_sol, ik_callback_fn, error) : 
        solver->searchPositionIK(ik_query, seed, timeout, consistency_limits, ik_sol, error))
    {
      std::vector<double> solution(bij.size());
      for (std::size_t i = 0 ; i < bij.size() ; ++i)
        solution[i] = ik_sol[bij[i]];
      setVariableValues(solution);
      return true;
    }
  }
  return false;
}

bool kinematic_state::JointStateGroup::setFromIK(const std::vector<Eigen::Affine3d> &poses_in, 
                                                 const std::vector<std::string> &tips_in, 
                                                 unsigned int attempts, 
                                                 double timeout, 
                                                 const StateValidityCallbackFn &constraint)
{
  static const std::vector<std::vector<double> > consistency_limits;
  return setFromIK(poses_in, tips_in, consistency_limits, attempts, timeout, constraint);  
}

bool kinematic_state::JointStateGroup::setFromIK(const std::vector<Eigen::Affine3d> &poses_in, 
                                                 const std::vector<std::string> &tips_in, 
                                                 const std::vector<std::vector<double> > &consistency_limits,
                                                 unsigned int attempts, 
                                                 double timeout, 
                                                 const StateValidityCallbackFn &constraint)
{
  if (poses_in.size() == 1 && tips_in.size() == 1 && consistency_limits.size() <= 1)
  {
    if (consistency_limits.empty())
      return setFromIK(poses_in[0], tips_in[0], attempts, timeout, constraint);
    else
      return setFromIK(poses_in[0], tips_in[0], consistency_limits[0], attempts, timeout, constraint);    
  }
  
  const std::vector<std::string>& sub_group_names = joint_model_group_->getSubgroupNames();
  
  if (poses_in.size() != sub_group_names.size())
  {
    logError("Number of poses must be the same as number of sub-groups");
    return false;
  }
  
  if (tips_in.size() != sub_group_names.size())
  {
    logError("Number of tip names must be the same as number of sub-groups");
    return false;
  }

  if (!consistency_limits.empty() && consistency_limits.size() != sub_group_names.size())
  {
    logError("Number of consistency limit vectors must be the same as number of sub-groups");
    return false;
  }
    
  if (!consistency_limits.empty())
  {
    for (std::size_t i = 0 ; i < consistency_limits.size() ; ++i)
    {
      if (consistency_limits[i].size() != joint_model_group_->getParentModel()->getJointModelGroup(sub_group_names[i])->getVariableCount())
      {
        logError("Number of joints in consistency_limits is %u but it should be should be %u", (unsigned int)i,
                 joint_model_group_->getParentModel()->getJointModelGroup(sub_group_names[i])->getVariableCount());
        return false;        
      }      
    }    
  }
  
  std::vector<kinematics::KinematicsBaseConstPtr> solvers;   
  for(std::size_t i = 0; i < poses_in.size() ; ++i)
  {
    kinematics::KinematicsBaseConstPtr solver = joint_model_group_->getParentModel()->getJointModelGroup(sub_group_names[i])->getSolverInstance();   
    if (!solver)
    {
      logError("Could not find solver for %s", sub_group_names[i].c_str());      
      return false;
    }
    solvers.push_back(solver);
  }
  
  std::vector<Eigen::Affine3d> transformed_poses = poses_in;
  std::vector<std::string> tip_names = tips_in;  

  for(std::size_t i = 0 ; i < poses_in.size() ; ++i)
  {    
    Eigen::Affine3d pose = poses_in[i];
    std::string tip = tips_in[i];
  
    // bring the pose to the frame of the IK solver
    const std::string &ik_frame = solvers[i]->getBaseFrame();
    if (ik_frame != joint_model_group_->getParentModel()->getModelFrame())
    {
      const LinkState *ls = kinematic_state_->getLinkState(ik_frame);
      if (!ls)
        return false;
      pose = ls->getGlobalLinkTransform().inverse() * pose;
    }

    // see if the tip frame can be transformed via fixed transforms to the frame known to the IK solver
    const std::string &tip_frame = solvers[i]->getTipFrame();
    if (tip != tip_frame)
    {
      if (kinematic_state_->hasAttachedBody(tip))
      {
        const AttachedBody *ab = kinematic_state_->getAttachedBody(tip);
        const EigenSTL::vector_Affine3d &ab_trans = ab->getFixedTransforms();
        if (ab_trans.size() != 1)
        {
          logError("Cannot use an attached body with multiple geometries as a reference frame.");
          return false;
        }
        tip = ab->getAttachedLinkName();
        pose = pose * ab_trans[0].inverse();
      }
      if (tip != tip_frame)
      {
        const kinematic_model::LinkModel *lm = joint_model_group_->getParentModel()->getLinkModel(tip);
        if (!lm)
          return false;
        const kinematic_model::LinkModel::AssociatedFixedTransformMap &fixed_links = lm->getAssociatedFixedTransforms();
        for (std::map<const kinematic_model::LinkModel*, Eigen::Affine3d>::const_iterator it = fixed_links.begin() ; it != fixed_links.end() ; ++it)
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
      logError("Cannot compute IK for tip reference frame '%s'", tip.c_str());
      return false;    
    }
    transformed_poses[i] = pose;
    tip_names[i] = tip;   
  }
    
  std::vector<geometry_msgs::Pose> ik_queries(poses_in.size());
  kinematics::KinematicsBase::IKCallbackFn ik_callback_fn;
  if (constraint)
    ik_callback_fn = boost::bind(&JointStateGroup::ikCallbackFnAdapter, this, constraint, _1, _2, _3);
  
  for(std::size_t i = 0; i < transformed_poses.size(); ++i)
  {    
    Eigen::Quaterniond quat(transformed_poses[i].rotation());
    Eigen::Vector3d point(transformed_poses[i].translation());
    ik_queries[i].position.x = point.x();
    ik_queries[i].position.y = point.y();
    ik_queries[i].position.z = point.z();
    ik_queries[i].orientation.x = quat.x();
    ik_queries[i].orientation.y = quat.y();
    ik_queries[i].orientation.z = quat.z();
    ik_queries[i].orientation.w = quat.w();
  }

  if (attempts == 0)
    attempts = joint_model_group_->getDefaultIKAttempts();

  bool first_seed = true;  
  for (unsigned int st = 0 ; st < attempts ; ++st)
  {    
    bool found_solution = true;    
    for(std::size_t sg = 0; sg < sub_group_names.size(); ++sg)
    {      
      kinematic_state::JointStateGroup* joint_state_group = getKinematicState()->getJointStateGroup(sub_group_names[sg]);      
      const std::vector<unsigned int>& bij = joint_state_group->getJointModelGroup()->getKinematicsSolverJointBijection();
      std::vector<double> seed(bij.size());
       // the first seed is the initial state
      if (first_seed)
      {
        if(sg == sub_group_names.size()-1)
          first_seed = false;
        std::vector<double> initial_values;
        joint_state_group->getVariableValues(initial_values);
        for (std::size_t i = 0 ; i < bij.size() ; ++i)
          seed[bij[i]] = initial_values[i];
      }
      else
      {
        // sample a random seed
        random_numbers::RandomNumberGenerator &rng = getRandomNumberGenerator();
        std::vector<double> random_values;
        joint_state_group->getJointModelGroup()->getVariableRandomValues(rng, random_values);
        for (std::size_t i = 0 ; i < bij.size() ; ++i)
          seed[bij[i]] = random_values[i];
      }
    
      // compute the IK solution
      std::vector<double> ik_sol;
      moveit_msgs::MoveItErrorCodes error;
      if (!consistency_limits.empty() ? 
         solvers[sg]->searchPositionIK(ik_queries[sg], seed, timeout < std::numeric_limits<double>::epsilon() ? joint_state_group->getDefaultIKTimeout() : timeout,
                                       consistency_limits[sg], ik_sol, error) :
         solvers[sg]->searchPositionIK(ik_queries[sg], seed,  timeout < std::numeric_limits<double>::epsilon() ? joint_state_group->getDefaultIKTimeout() : timeout,
                                       ik_sol, error))
      {
        std::vector<double> solution(bij.size());
        for (std::size_t i = 0 ; i < bij.size() ; ++i)
          solution[i] = ik_sol[bij[i]];
        joint_state_group->setVariableValues(solution);
      }
      else
      {
        found_solution = false;        
        break;
      }      
      if(found_solution && sg == (sub_group_names.size() - 1))
      {
        std::vector<double> full_solution;
        getVariableValues(full_solution);
        if(constraint ? constraint(this, full_solution) : true)
        {
          logDebug("Got solution");
          return true;
        }        
      }  
      logDebug("Attempt: %d of %d", st, attempts);      
    }    
  }
  return false;
}

void kinematic_state::JointStateGroup::computeJointVelocity(Eigen::VectorXd &qdot, const Eigen::VectorXd &twist, const std::string &tip, const SecondaryTaskFn &st) const
{
  //Get the Jacobian of the group at the current configuration
  Eigen::MatrixXd J(6, getVariableCount());
  Eigen::Vector3d reference_point(0.0, 0.0, 0.0);
  getJacobian(tip, reference_point, J);

  //Rotate the jacobian to the end-effector frame
  Eigen::Affine3d eMb = getKinematicState()->getLinkState(tip)->getGlobalLinkTransform().inverse();
  Eigen::MatrixXd eWb = Eigen::ArrayXXd::Zero(6, 6);
  eWb.block(0, 0, 3, 3) = eMb.matrix().block(0, 0, 3, 3);
  eWb.block(3, 3, 3, 3) = eMb.matrix().block(0, 0, 3, 3);
  J = eWb * J;

  //Do the Jacobian moore-penrose pseudo-inverse
  Eigen::JacobiSVD<Eigen::MatrixXd> svdOfJ(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::MatrixXd U = svdOfJ.matrixU();
  const Eigen::MatrixXd V = svdOfJ.matrixV();
  const Eigen::VectorXd S = svdOfJ.singularValues();

  Eigen::VectorXd Sinv = S;
  static const double pinvtoler = std::numeric_limits<float>::epsilon();
  double maxsv = 0.0 ;
  for (std::size_t i = 0; i < S.rows(); ++i)
    if (fabs(S(i)) > maxsv) maxsv = fabs(S(i));
  for (std::size_t i = 0; i < S.rows(); ++i)
  {
    //Those singular values smaller than a percentage of the maximum singular value are removed
    if ( fabs(S(i)) > maxsv * pinvtoler )
      Sinv(i) = 1.0 / S(i);
    else Sinv(i) = 0.0;
  }
  Eigen::MatrixXd Jinv = ( V * Sinv.asDiagonal() * U.transpose() );

  //Compute joint velocity
  qdot = Jinv * twist;

  //Project the secondary task
  if (st)
  {
    Eigen::VectorXd cost_vector = Eigen::VectorXd::Zero(qdot.rows());
    st(this, cost_vector);
    qdot += (Eigen::MatrixXd::Identity(qdot.rows(), qdot.rows()) - Jinv * J) * cost_vector;
  }
}

bool kinematic_state::JointStateGroup::setFromDiffIK(const Eigen::VectorXd &twist, const std::string &tip, double dt, const SecondaryTaskFn &st)
{
  Eigen::VectorXd qdot;
  computeJointVelocity(qdot, twist, tip, st);
  
  //Integrate qdot for dt
  Eigen::VectorXd q(getVariableCount());
  getVariableValues(q);
  q = q + dt * qdot;

  setVariableValues(q);
  enforceBounds();

  return true;
}

bool kinematic_state::JointStateGroup::setFromDiffIK(const geometry_msgs::Twist &twist, const std::string &tip, double dt, const SecondaryTaskFn &st)
{
  Eigen::Matrix<double, 6, 1> t;
  tf::twistMsgToEigen(twist, t);

  return setFromDiffIK(t, tip, dt, st);
}

bool kinematic_state::JointStateGroup::avoidJointLimitsSecondaryTask(const kinematic_state::JointStateGroup *joint_state_group, Eigen::VectorXd &stvector,
                                                                            double activation_threshold, double gain) const
{
  //Get current joint values (q)
  Eigen::VectorXd q;
  joint_state_group->getVariableValues(q);

  //Get joint lower and upper limits (qmin and qmax)
  const std::vector<moveit_msgs::JointLimits> &qlimits = joint_state_group->getJointModelGroup()->getVariableLimits();
  Eigen::VectorXd qmin(qlimits.size());
  Eigen::VectorXd qmax(qlimits.size());
  Eigen::VectorXd qrange(qlimits.size());
  stvector.resize(qlimits.size());
  stvector = Eigen::ArrayXd::Zero(qlimits.size());

  for (std::size_t i = 0; i < qlimits.size(); ++i)
  {
    qmin(i) = qlimits[i].min_position;
    qmax(i) = qlimits[i].max_position;
    qrange(i) = qmax(i) - qmin(i);

    //Fill in stvector with the gradient of a joint limit avoidance cost function
    const std::vector<const kinematic_model::JointModel*> joint_models = joint_state_group->getJointModelGroup()->getJointModels();
    if (qrange(i) == 0)
    {
      //If the joint range is zero do not compute the cost
      stvector(i) = 0;
    }
    else if (joint_models[i]->getType() == kinematic_model::JointModel::REVOLUTE)
    {
      //If the joint is continuous do not compute the cost
      const kinematic_model::RevoluteJointModel *rjoint = static_cast<const kinematic_model::RevoluteJointModel*>(joint_models[i]);
      if (rjoint->isContinuous())
        stvector(i) = 0;
    }
    else
    {
      if (q(i) > (qmax(i) - qrange(i) * activation_threshold))
      {
        stvector(i) = -gain * (q(i) - (qmax(i) - qrange(i) * activation_threshold)) / qrange(i);
      }
      else if (q(i) < (qmin(i) + qrange(i) * activation_threshold))
      {
        stvector(i) = -gain * (q(i) - (qmin(i) + qrange(i) * activation_threshold)) / qrange(i);
      }
    }
  }

  return true;
}

namespace
{

void addTrajectoryPoint(const kinematic_state::KinematicState *state,
                        const std::vector<const kinematic_model::JointModel*> &onedof,
                        const std::vector<const kinematic_model::JointModel*> &mdof,
                        moveit_msgs::RobotTrajectory &traj)
{ 
  if (!onedof.empty())
  { 
    traj.joint_trajectory.points.resize(traj.joint_trajectory.points.size() + 1);
    traj.joint_trajectory.points.back().positions.resize(onedof.size());
    for (std::size_t j = 0 ; j < onedof.size() ; ++j)
      traj.joint_trajectory.points.back().positions[j] = state->getJointState(onedof[j]->getName())->getVariableValues()[0];
    traj.joint_trajectory.points.back().time_from_start = ros::Duration(0.0);
  }
  if (!mdof.empty())
  {
    traj.multi_dof_joint_trajectory.points.resize(traj.multi_dof_joint_trajectory.points.size() + 1);
    traj.multi_dof_joint_trajectory.points.back().poses.resize(mdof.size());
    for (std::size_t j = 0 ; j < mdof.size() ; ++j)
    {
      tf::poseEigenToMsg(state->getJointState(mdof[j]->getName())->getVariableTransform(),
                         traj.multi_dof_joint_trajectory.points.back().poses[j]);
    }
    traj.multi_dof_joint_trajectory.points.back().time_from_start = ros::Duration(0.0);
  }
}

}

double kinematic_state::JointStateGroup::computeCartesianPath(moveit_msgs::RobotTrajectory &traj, const std::string &link_name, const Eigen::Vector3d &direction, double distance, 
                                                              double max_step, const StateValidityCallbackFn &validCallback)
{
  const LinkState *link_state = kinematic_state_->getLinkState(link_name);
  if (!link_state)
    return 0.0;
  Eigen::Affine3d start_pose = link_state->getGlobalLinkTransform();
  unsigned int steps = 1 + (unsigned int)floor(distance / max_step);
  
  const std::vector<const kinematic_model::JointModel*> &jnt = joint_model_group_->getJointModels();
  std::vector<const kinematic_model::JointModel*> onedof;
  std::vector<const kinematic_model::JointModel*> mdof;
  traj.joint_trajectory.header.frame_id = kinematic_state_->getKinematicModel()->getModelFrame();
  traj.joint_trajectory.joint_names.clear();
  traj.multi_dof_joint_trajectory.joint_names.clear();
  traj.multi_dof_joint_trajectory.child_frame_ids.clear();
  for (std::size_t i = 0 ; i < jnt.size() ; ++i)
    if (jnt[i]->getVariableCount() == 1)
    {
      traj.joint_trajectory.joint_names.push_back(jnt[i]->getName());
      onedof.push_back(jnt[i]);
    }
    else
    {
      traj.multi_dof_joint_trajectory.joint_names.push_back(jnt[i]->getName());
      traj.multi_dof_joint_trajectory.frame_ids.push_back(traj.joint_trajectory.header.frame_id);
      traj.multi_dof_joint_trajectory.child_frame_ids.push_back(jnt[i]->getChildLinkModel()->getName());
      mdof.push_back(jnt[i]);
    }
  
  addTrajectoryPoint(kinematic_state_, onedof, mdof, traj);

  double last_valid_distance = 0.0;
  for (unsigned int i = 1; i <= steps ; ++i)
  {
    double d = distance * (double)i / (double)steps;
    Eigen::Affine3d pose = start_pose;
    pose.translation() = pose.translation() + direction * d;
    if (setFromIK(pose, link_name, 1, 0.0, validCallback)) 
      addTrajectoryPoint(kinematic_state_, onedof, mdof, traj);
    else
      break;
    last_valid_distance = d;
  }
  return last_valid_distance;
}

void kinematic_state::JointStateGroup::ikCallbackFnAdapter(const StateValidityCallbackFn &constraint,
                                                           const geometry_msgs::Pose &, const std::vector<double> &ik_sol, moveit_msgs::MoveItErrorCodes &error_code)
{  
  const std::vector<unsigned int> &bij = joint_model_group_->getKinematicsSolverJointBijection();
  std::vector<double> solution(bij.size());
  for (std::size_t i = 0 ; i < bij.size() ; ++i)
    solution[i] = ik_sol[bij[i]];
  if (constraint(this, solution))
    error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  else
    error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
}

bool kinematic_state::JointStateGroup::getJacobian(const std::string &link_name,
                                                   const Eigen::Vector3d &reference_point_position, 
                                                   Eigen::MatrixXd& jacobian) const
{
  if (!joint_model_group_->isChain())
  {
    logError("The group '%s' is not a chain. Cannot compute Jacobian", joint_model_group_->getName().c_str());
    return false;
  }
  if (!joint_model_group_->isLinkUpdated(link_name))
  {
    logError("Link name '%s' does not exist in the chain '%s' or is not a child for this chain", link_name.c_str(), joint_model_group_->getName().c_str());
    return false;
  }
  
  const kinematic_model::JointModel* root_joint_model = (joint_model_group_->getJointRoots())[0];
  const kinematic_state::LinkState *root_link_state = kinematic_state_->getLinkState(root_joint_model->getParentLinkModel()->getName());
  Eigen::Affine3d reference_transform = root_link_state ? root_link_state->getGlobalLinkTransform() : kinematic_state_->getRootTransform();
  reference_transform = reference_transform.inverse();
  jacobian = Eigen::MatrixXd::Zero(6, joint_model_group_->getVariableCount());
  
  const kinematic_state::LinkState *link_state = kinematic_state_->getLinkState(link_name);
  Eigen::Affine3d link_transform = reference_transform*link_state->getGlobalLinkTransform();
  Eigen::Vector3d point_transform = link_transform*reference_point_position;
  
  logDebug("Point from reference origin expressed in world coordinates: %f %f %f",
           point_transform.x(),
           point_transform.y(),
           point_transform.z());
  
  Eigen::Vector3d joint_axis;
  Eigen::Affine3d joint_transform;
  
  while (link_state)
  {
    /*
    logDebug("Link: %s, %f %f %f",link_state->getName().c_str(),
             link_state->getGlobalLinkTransform().translation().x(),
             link_state->getGlobalLinkTransform().translation().y(),
             link_state->getGlobalLinkTransform().translation().z());    
    logDebug("Joint: %s",link_state->getParentJointState()->getName().c_str());
    */
    
    if (joint_model_group_->isActiveDOF(link_state->getParentJointState()->getJointModel()->getName()))
    {
      if (link_state->getParentJointState()->getJointModel()->getType() == kinematic_model::JointModel::REVOLUTE)
      {
        unsigned int joint_index = joint_model_group_->getJointVariablesIndexMap().find(link_state->getParentJointState()->getJointModel()->getName())->second;
        joint_transform = reference_transform*link_state->getGlobalLinkTransform();
        joint_axis = joint_transform.rotation()*(static_cast<const kinematic_model::RevoluteJointModel*>(link_state->getParentJointState()->getJointModel()))->getAxis();
        jacobian.block<3,1>(0,joint_index) = joint_axis.cross(point_transform - joint_transform.translation());
        jacobian.block<3,1>(3,joint_index) = joint_axis;
      }
      if (link_state->getParentJointState()->getJointModel()->getType() == kinematic_model::JointModel::PRISMATIC)
      {
        unsigned int joint_index = joint_model_group_->getJointVariablesIndexMap().find(link_state->getParentJointState()->getJointModel()->getName())->second;
        joint_transform = reference_transform*link_state->getGlobalLinkTransform();
        joint_axis = joint_transform*(static_cast<const kinematic_model::PrismaticJointModel*>(link_state->getParentJointState()->getJointModel()))->getAxis();
        jacobian.block<3,1>(0,joint_index) = joint_axis;
      }
      if (link_state->getParentJointState()->getJointModel()->getType() == kinematic_model::JointModel::PLANAR)
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
    if (link_state->getParentJointState()->getJointModel() == root_joint_model)
      break;
    link_state = link_state->getParentLinkState();
  }
  return true;
}
