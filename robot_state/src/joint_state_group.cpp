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

#include <moveit/robot_state/robot_state.h>
#include <eigen_conversions/eigen_msg.h>
#include <boost/bind.hpp>
#include <boost/math/constants/constants.hpp>
#include <algorithm>
#include <Eigen/SVD>

robot_state::JointStateGroup::JointStateGroup(RobotState *state,
                                              const robot_model::JointModelGroup *jmg) :
  kinematic_state_(state), joint_model_group_(jmg)
{
  const std::vector<const robot_model::JointModel*>& joint_model_vector = jmg->getJointModels();
  for (std::size_t i = 0; i < joint_model_vector.size() ; ++i)
  {
    assert(kinematic_state_->hasJointState(joint_model_vector[i]->getName()));
    JointState* js = kinematic_state_->getJointState(joint_model_vector[i]->getName());
    joint_state_vector_.push_back(js);
    joint_state_map_[joint_model_vector[i]->getName()] = js;
  }
  const std::vector<const robot_model::LinkModel*>& link_model_vector = jmg->getUpdatedLinkModels();
  for (unsigned int i = 0; i < link_model_vector.size(); i++)
  {
    assert(kinematic_state_->hasLinkState(link_model_vector[i]->getName()));
    LinkState* ls = kinematic_state_->getLinkState(link_model_vector[i]->getName());
    updated_links_.push_back(ls);
  }

  const std::vector<const robot_model::JointModel*>& joint_root_vector = jmg->getJointRoots();
  for (std::size_t i = 0; i < joint_root_vector.size(); ++i)
  {
    JointState* js = kinematic_state_->getJointState(joint_root_vector[i]->getName());
    if (js)
      joint_roots_.push_back(js);
  }
}

robot_state::JointStateGroup::~JointStateGroup()
{
}

random_numbers::RandomNumberGenerator& robot_state::JointStateGroup::getRandomNumberGenerator()
{
  if (!rng_)
    rng_.reset(new random_numbers::RandomNumberGenerator());
  return *rng_;
}

bool robot_state::JointStateGroup::hasJointState(const std::string &joint) const
{
  return joint_state_map_.find(joint) != joint_state_map_.end();
}

bool robot_state::JointStateGroup::setVariableValues(const std::vector<double> &joint_state_values)
{
  if (joint_state_values.size() != getVariableCount())
  {
    logError("JointStateGroup: Incorrect variable count specified for array of joint values. Expected %u but got %u values in group '%s'",
             getVariableCount(), (int)joint_state_values.size(), joint_model_group_->getName().c_str() );
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

bool robot_state::JointStateGroup::setVariableValues(const Eigen::VectorXd &joint_state_values)
{
  std::vector<double> values(joint_state_values.rows());
  for (std::size_t i = 0; i < joint_state_values.rows(); i++)
    values[i] =  joint_state_values(i);
  setVariableValues(values);
}

void robot_state::JointStateGroup::setVariableValues(const std::map<std::string, double>& joint_state_map)
{
  for(unsigned int i = 0; i < joint_state_vector_.size(); ++i)
    joint_state_vector_[i]->setVariableValues(joint_state_map);
  updateLinkTransforms();
}

void robot_state::JointStateGroup::setVariableValues(const sensor_msgs::JointState& js)
{
  std::map<std::string, double> v;
  for (std::size_t i = 0 ; i < js.name.size() ; ++i)
    v[js.name[i]] = js.position[i];
  setVariableValues(v);
}

void robot_state::JointStateGroup::updateLinkTransforms()
{
  for(unsigned int i = 0; i < updated_links_.size(); ++i)
    updated_links_[i]->computeTransform();
}

robot_state::JointStateGroup& robot_state::JointStateGroup::operator=(const JointStateGroup &other)
{
  if (this != &other)
    copyFrom(other);
  return *this;
}

void robot_state::JointStateGroup::copyFrom(const JointStateGroup &other_jsg)
{
  const std::vector<JointState*> &ojsv = other_jsg.getJointStateVector();
  for (std::size_t i = 0 ; i < ojsv.size() ; ++i)
    joint_state_vector_[i]->setVariableValues(ojsv[i]->getVariableValues());
  updateLinkTransforms();
}

void robot_state::JointStateGroup::setToDefaultValues()
{
  std::map<std::string, double> default_joint_values;
  for (std::size_t i = 0  ; i < joint_state_vector_.size() ; ++i)
    joint_state_vector_[i]->getJointModel()->getVariableDefaultValues(default_joint_values);
  setVariableValues(default_joint_values);
}

bool robot_state::JointStateGroup::setToDefaultState(const std::string &name)
{
  std::map<std::string, double> default_joint_values;
  if (!joint_model_group_->getVariableDefaultValues(name, default_joint_values))
    return false;
  setVariableValues(default_joint_values);
  return true;
}

void robot_state::JointStateGroup::setToRandomValues()
{
  random_numbers::RandomNumberGenerator &rng = getRandomNumberGenerator();
  std::vector<double> random_joint_states;
  joint_model_group_->getVariableRandomValues(rng, random_joint_states);
  setVariableValues(random_joint_states);
}

void robot_state::JointStateGroup::setToRandomValuesNearBy(const std::vector<double> &near,
                                                           const std::map<robot_model::JointModel::JointType, double> &distance_map)
{
  random_numbers::RandomNumberGenerator &rng = getRandomNumberGenerator();
  std::vector<double> variable_values;
  joint_model_group_->getVariableRandomValuesNearBy(rng, variable_values, near, distance_map);
  setVariableValues(variable_values);
}

void robot_state::JointStateGroup::setToRandomValuesNearBy(const std::vector<double> &near,
                                                           const std::vector<double> &distances)
{
  random_numbers::RandomNumberGenerator &rng = getRandomNumberGenerator();
  std::vector<double> variable_values;
  joint_model_group_->getVariableRandomValuesNearBy(rng, variable_values, near, distances);
  setVariableValues(variable_values);
}

void robot_state::JointStateGroup::getVariableValues(std::vector<double>& joint_state_values) const
{
  joint_state_values.clear();
  for(unsigned int i = 0; i < joint_state_vector_.size(); i++)
  {
    const std::vector<double> &jv = joint_state_vector_[i]->getVariableValues();
    joint_state_values.insert(joint_state_values.end(), jv.begin(), jv.end());
  }
}

void robot_state::JointStateGroup::getVariableValues(Eigen::VectorXd& joint_state_values) const
{
  joint_state_values.resize(getVariableCount());
  unsigned int count = 0;
  for(unsigned int i = 0; i < joint_state_vector_.size(); i++)
  {
    const std::vector<double> &jv = joint_state_vector_[i]->getVariableValues();
    for (std::size_t j = 0; j < jv.size() ; ++j)
      joint_state_values(count++) = jv[j];
  }
}

bool robot_state::JointStateGroup::satisfiesBounds(double margin) const
{
  for (std::size_t i = 0 ; i < joint_state_vector_.size() ; ++i)
    if (!joint_state_vector_[i]->satisfiesBounds(margin))
      return false;
  return true;
}

void robot_state::JointStateGroup::enforceBounds()
{
  for (std::size_t i = 0 ; i < joint_state_vector_.size() ; ++i)
    joint_state_vector_[i]->enforceBounds();
  updateLinkTransforms();
}

double robot_state::JointStateGroup::infinityNormDistance(const JointStateGroup *other) const
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

double robot_state::JointStateGroup::distance(const JointStateGroup *other) const
{
  double d = 0.0;
  for (std::size_t i = 0 ; i < joint_state_vector_.size() ; ++i)
    d += joint_state_vector_[i]->distance(other->joint_state_vector_[i]) * joint_state_vector_[i]->getJointModel()->getDistanceFactor();
  return d;
}

void robot_state::JointStateGroup::interpolate(const JointStateGroup *to, const double t, JointStateGroup *dest) const
{
  for (std::size_t i = 0 ; i < joint_state_vector_.size() ; ++i)
    joint_state_vector_[i]->interpolate(to->joint_state_vector_[i], t, dest->joint_state_vector_[i]);
  dest->updateLinkTransforms();
}

void robot_state::JointStateGroup::getVariableValues(std::map<std::string, double>& joint_state_values) const
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

robot_state::JointState* robot_state::JointStateGroup::getJointState(const std::string &name) const
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

bool robot_state::JointStateGroup::setFromIK(const geometry_msgs::Pose &pose, unsigned int attempts, double timeout, const StateValidityCallbackFn &constraint, const kinematics::KinematicsQueryOptions &options)
{
  const kinematics::KinematicsBaseConstPtr& solver = joint_model_group_->getSolverInstance();
  if (!solver)
  {
    logError("No kinematics solver instantiated for this group");
    return false;
  }
  return setFromIK(pose, solver->getTipFrame(), attempts, timeout, constraint, options);
}

bool robot_state::JointStateGroup::setFromIK(const geometry_msgs::Pose &pose, unsigned int attempts, double timeout, const kinematics::KinematicsQueryOptions &options)
{
  const kinematics::KinematicsBaseConstPtr& solver = joint_model_group_->getSolverInstance();
  if (!solver)
  {
    logError("No kinematics solver instantiated for this group");
    return false;
  }
  static StateValidityCallbackFn constraint = StateValidityCallbackFn();
  return setFromIK(pose, solver->getTipFrame(), attempts, timeout, constraint, options);
}

bool robot_state::JointStateGroup::setFromIK(const geometry_msgs::Pose &pose, const std::string &tip, unsigned int attempts, double timeout, const StateValidityCallbackFn &constraint, const kinematics::KinematicsQueryOptions &options)
{
  Eigen::Affine3d mat;
  tf::poseMsgToEigen(pose, mat);
  return setFromIK(mat, tip,  attempts, timeout, constraint, options);
}

bool robot_state::JointStateGroup::setFromIK(const Eigen::Affine3d &pose, unsigned int attempts, double timeout, const StateValidityCallbackFn &constraint, const kinematics::KinematicsQueryOptions &options)
{
  const kinematics::KinematicsBaseConstPtr& solver = joint_model_group_->getSolverInstance();
  if (!solver)
  {
    logError("No kinematics solver instantiated for this group");
    return false;
  }
  static std::vector<double> consistency_limits;
  return setFromIK(pose, solver->getTipFrame(), consistency_limits, attempts, timeout, constraint, options);
}

bool robot_state::JointStateGroup::setFromIK(const Eigen::Affine3d &pose_in, const std::string &tip_in, unsigned int attempts, double timeout, const StateValidityCallbackFn &constraint, const kinematics::KinematicsQueryOptions &options)
{
  static std::vector<double> consistency_limits;
  return setFromIK(pose_in, tip_in, consistency_limits, attempts, timeout, constraint, options);
}

bool robot_state::JointStateGroup::setFromIK(const Eigen::Affine3d &pose_in, const std::string &tip_in, unsigned int attempts, double timeout, const kinematics::KinematicsQueryOptions &options)
{
  static std::vector<double> consistency_limits;
  static StateValidityCallbackFn constraint = StateValidityCallbackFn();
  return setFromIK(pose_in, tip_in, consistency_limits, attempts, timeout, constraint, options);
}

bool robot_state::JointStateGroup::setFromIK(const Eigen::Affine3d &pose_in, unsigned int attempts, double timeout, const kinematics::KinematicsQueryOptions &options)
{
  static std::vector<double> consistency_limits;
  static StateValidityCallbackFn constraint = StateValidityCallbackFn();
  const kinematics::KinematicsBaseConstPtr& solver = joint_model_group_->getSolverInstance();
  if (!solver)
  {
    logError("No kinematics solver instantiated for this group");
    return false;
  }
  return setFromIK(pose_in, solver->getTipFrame(), consistency_limits, attempts, timeout, constraint, options);
}

bool robot_state::JointStateGroup::setFromIK(const Eigen::Affine3d &pose_in, const std::string &tip_in, const std::vector<double> &consistency_limits, unsigned int attempts, double timeout, const StateValidityCallbackFn &constraint, const kinematics::KinematicsQueryOptions &options)
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
      const robot_model::LinkModel *lm = joint_model_group_->getParentModel()->getLinkModel(tip);
      if (!lm)
        return false;
      const robot_model::LinkModel::AssociatedFixedTransformMap &fixed_links = lm->getAssociatedFixedTransforms();
      for (std::map<const robot_model::LinkModel*, Eigen::Affine3d>::const_iterator it = fixed_links.begin() ; it != fixed_links.end() ; ++it)
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
  std::vector<double> initial_values;
  getVariableValues(initial_values);
  for (unsigned int st = 0 ; st < attempts ; ++st)
  {
    std::vector<double> seed(bij.size());

    // the first seed is the initial state
    if (first_seed)
    {
      first_seed = false;
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

      if (options.lock_redundant_joints)
      {
        std::vector<unsigned int> red_joints;
        solver->getRedundantJoints(red_joints);
        if (!red_joints.empty())
        {
          for(std::size_t i = 0 ; i < red_joints.size(); ++i)
          {
            seed[bij[red_joints[i]]] = initial_values[red_joints[i]];
          }
        }
      }
    }

    // compute the IK solution
    std::vector<double> ik_sol;
    moveit_msgs::MoveItErrorCodes error;
    if (ik_callback_fn ?
        solver->searchPositionIK(ik_query, seed, timeout, consistency_limits, ik_sol, ik_callback_fn, error, options) :
        solver->searchPositionIK(ik_query, seed, timeout, consistency_limits, ik_sol, error, options))
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

bool robot_state::JointStateGroup::setFromIK(const EigenSTL::vector_Affine3d &poses_in,
                                             const std::vector<std::string> &tips_in,
                                             unsigned int attempts,
                                             double timeout,
                                             const StateValidityCallbackFn &constraint,
                                             const kinematics::KinematicsQueryOptions &options)
{
  static const std::vector<std::vector<double> > consistency_limits;
  return setFromIK(poses_in, tips_in, consistency_limits, attempts, timeout, constraint, options);
}

bool robot_state::JointStateGroup::setFromIK(const EigenSTL::vector_Affine3d &poses_in,
                                             const std::vector<std::string> &tips_in,
                                             const std::vector<std::vector<double> > &consistency_limits,
                                             unsigned int attempts,
                                             double timeout,
                                             const StateValidityCallbackFn &constraint,
                                             const kinematics::KinematicsQueryOptions &options)
{
  if (poses_in.size() == 1 && tips_in.size() == 1 && consistency_limits.size() <= 1)
  {
    if (consistency_limits.empty())
      return setFromIK(poses_in[0], tips_in[0], attempts, timeout, constraint, options);
    else
      return setFromIK(poses_in[0], tips_in[0], consistency_limits[0], attempts, timeout, constraint, options);
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

  EigenSTL::vector_Affine3d transformed_poses = poses_in;
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
        const robot_model::LinkModel *lm = joint_model_group_->getParentModel()->getLinkModel(tip);
        if (!lm)
          return false;
        const robot_model::LinkModel::AssociatedFixedTransformMap &fixed_links = lm->getAssociatedFixedTransforms();
        for (std::map<const robot_model::LinkModel*, Eigen::Affine3d>::const_iterator it = fixed_links.begin() ; it != fixed_links.end() ; ++it)
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
      robot_state::JointStateGroup* joint_state_group = getRobotState()->getJointStateGroup(sub_group_names[sg]);
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
          logDebug("Found IK solution");
          return true;
        }
      }
      logDebug("IK attempt: %d of %d", st, attempts);
    }
  }
  return false;
}

void robot_state::JointStateGroup::computeJointVelocity(Eigen::VectorXd &qdot, const Eigen::VectorXd &twist, const std::string &tip, const SecondaryTaskFn &st) const
{
  //Get the Jacobian of the group at the current configuration
  Eigen::MatrixXd J(6, getVariableCount());
  Eigen::Vector3d reference_point(0.0, 0.0, 0.0);
  getJacobian(tip, reference_point, J);

  //Rotate the jacobian to the end-effector frame
  Eigen::Affine3d eMb = getRobotState()->getLinkState(tip)->getGlobalLinkTransform().inverse();
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

bool robot_state::JointStateGroup::setFromDiffIK(const Eigen::VectorXd &twist, const std::string &tip, double dt, const StateValidityCallbackFn &constraint, const SecondaryTaskFn &st)
{
  Eigen::VectorXd qdot;
  computeJointVelocity(qdot, twist, tip, st);
  return integrateJointVelocity(qdot, dt, constraint);
}

bool robot_state::JointStateGroup::setFromDiffIK(const geometry_msgs::Twist &twist, const std::string &tip, double dt, const StateValidityCallbackFn &constraint, const SecondaryTaskFn &st)
{
  Eigen::Matrix<double, 6, 1> t;
  tf::twistMsgToEigen(twist, t);
  return setFromDiffIK(t, tip, dt, constraint, st);
}

bool robot_state::JointStateGroup::integrateJointVelocity(const Eigen::VectorXd &qdot, double dt, const StateValidityCallbackFn &constraint)
{
  Eigen::VectorXd q(getVariableCount());
  getVariableValues(q);
  q = q + dt * qdot;
  setVariableValues(q);
  enforceBounds();

  if (constraint)
  {
    std::vector<double> values;
    getVariableValues(values);
    return constraint(this, values);
  }
  else
    return true;
}

bool robot_state::JointStateGroup::avoidJointLimitsSecondaryTask(const robot_state::JointStateGroup *joint_state_group, Eigen::VectorXd &stvector,
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
    const std::vector<const robot_model::JointModel*> joint_models = joint_state_group->getJointModelGroup()->getJointModels();
    if (qrange(i) == 0)
    {
      //If the joint range is zero do not compute the cost
      stvector(i) = 0;
    }
    else if (joint_models[i]->getType() == robot_model::JointModel::REVOLUTE)
    {
      //If the joint is continuous do not compute the cost
      const robot_model::RevoluteJointModel *rjoint = static_cast<const robot_model::RevoluteJointModel*>(joint_models[i]);
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

double robot_state::JointStateGroup::computeCartesianPath(std::vector<RobotStatePtr> &traj, const std::string &link_name, const Eigen::Vector3d &direction, bool global_reference_frame,
                                                          double distance, double max_step, double jump_threshold, const StateValidityCallbackFn &validCallback, const kinematics::KinematicsQueryOptions &options)
{
  const LinkState *link_state = kinematic_state_->getLinkState(link_name);
  if (!link_state)
    return 0.0;

  //this is the Cartesian pose we start from, and have to move in the direction indicated
  const Eigen::Affine3d &start_pose = link_state->getGlobalLinkTransform();

  //the direction can be in the local reference frame (in which case we rotate it)
  const Eigen::Vector3d &rotated_direction = global_reference_frame ? direction : start_pose.rotation() * direction;

  //The target pose is built by applying a translation to the start pose for the desired direction and distance
  Eigen::Affine3d target_pose = start_pose;
  target_pose.translation() += rotated_direction * distance;

  //call computeCartesianPath for the computed target pose in the global reference frame
  return (distance * computeCartesianPath(traj, link_name, target_pose, true, max_step, jump_threshold, validCallback, options));
}

double robot_state::JointStateGroup::computeCartesianPath(std::vector<RobotStatePtr> &traj, const std::string &link_name, const Eigen::Affine3d &target, bool global_reference_frame,
                                                          double max_step, double jump_threshold, const StateValidityCallbackFn &validCallback, const kinematics::KinematicsQueryOptions &options)
{
  const LinkState *link_state = kinematic_state_->getLinkState(link_name);
  if (!link_state)
    return 0.0;

  const std::vector<const robot_model::JointModel*> &jnt = joint_model_group_->getJointModels();

  // make sure that continuous joints wrap, but remember how much we wrapped them
  std::map<std::string, double> upd_continuous_joints;
  for (std::size_t i = 0 ; i < jnt.size() ; ++i)
    if (jnt[i]->getType() == robot_model::JointModel::REVOLUTE)
    {
      if (static_cast<const robot_model::RevoluteJointModel*>(jnt[i])->isContinuous())
      {
        double initial = joint_state_vector_[i]->getVariableValues()[0];
        joint_state_vector_[i]->enforceBounds();
        double after = joint_state_vector_[i]->getVariableValues()[0];
        if (fabs(initial - after) > std::numeric_limits<double>::epsilon())
          upd_continuous_joints[joint_state_vector_[i]->getName()] = initial - after;
      }
    }

  // this is the Cartesian pose we start from, and we move in the direction indicated
  Eigen::Affine3d start_pose = link_state->getGlobalLinkTransform();

  // the target can be in the local reference frame (in which case we rotate it)
  Eigen::Affine3d rotated_target = global_reference_frame ? target : start_pose * target;

  bool test_joint_space_jump = jump_threshold > 0.0;

  // decide how many steps we will need for this trajectory
  double distance = (rotated_target.translation() - start_pose.translation()).norm();
  unsigned int steps = (test_joint_space_jump ? 5 : 1) + (unsigned int)floor(distance / max_step);

  traj.clear();
  traj.push_back(RobotStatePtr(new RobotState(*kinematic_state_)));

  std::vector<std::vector<double> > previous_values(joint_state_vector_.size());
  std::vector<double> dist_vector;
  double total_dist = 0.0;

  if (test_joint_space_jump) // the joint values we start with
    for (std::size_t k = 0 ; k < joint_state_vector_.size() ; ++k)
      previous_values[k] = joint_state_vector_[k]->getVariableValues();

  double last_valid_percentage = 0.0;
  Eigen::Quaterniond start_quaternion(start_pose.rotation());
  Eigen::Quaterniond target_quaternion(rotated_target.rotation());
  for (unsigned int i = 1; i <= steps ; ++i)
  {
    double percentage = (double)i / (double)steps;

    Eigen::Affine3d pose(start_quaternion.slerp(percentage, target_quaternion));
    pose.translation() = percentage * rotated_target.translation() + (1 - percentage) * start_pose.translation();

    if (setFromIK(pose, link_name, 1, 0.0, validCallback, options))
    {
      traj.push_back(RobotStatePtr(new RobotState(*kinematic_state_)));

      // compute the distance to the previous point (infinity norm)
      if (test_joint_space_jump)
      {
        double dist_prev_point = 0.0;
        for (std::size_t k = 0 ; k < joint_state_vector_.size() ; ++k)
        {
          double d_k = jnt[k]->distance(joint_state_vector_[k]->getVariableValues(), previous_values[k]);
          if (dist_prev_point < 0.0 || dist_prev_point < d_k)
            dist_prev_point = d_k;
          previous_values[k] = joint_state_vector_[k]->getVariableValues();
        }
        dist_vector.push_back(dist_prev_point);
        total_dist += dist_prev_point;
      }
    }
    else
      break;
    last_valid_percentage = percentage;
  }

  if (test_joint_space_jump)
  {
    // compute the average distance between the states we looked at
    double thres = jump_threshold * (total_dist / (double)dist_vector.size());
    for (std::size_t i = 0 ; i < dist_vector.size() ; ++i)
      if (dist_vector[i] > thres)
      {
        logDebug("Truncating Cartesian path due to detected jump in joint-space distance");
        last_valid_percentage = (double)i / (double)steps;
        traj.resize(i);
        break;
      }
  }

  return last_valid_percentage;
}

double robot_state::JointStateGroup::computeCartesianPath(std::vector<RobotStatePtr> &traj, const std::string &link_name, const EigenSTL::vector_Affine3d &waypoints,
                                                          bool global_reference_frame, double max_step, double jump_threshold, const StateValidityCallbackFn &validCallback, const kinematics::KinematicsQueryOptions &options)
{
  double percentage_solved = 0.0;
  for (std::size_t i = 0; i < waypoints.size(); ++i)
  {
    std::vector<RobotStatePtr> waypoint_traj;
    double wp_percentage_solved = computeCartesianPath(waypoint_traj, link_name, waypoints[i], global_reference_frame, max_step, jump_threshold, validCallback);
    if (fabs(wp_percentage_solved - 1.0) < std::numeric_limits<double>::epsilon())
    {
      percentage_solved = (double)(i + 1) / (double)waypoints.size();
      traj.insert(traj.end(), waypoint_traj.begin(), waypoint_traj.end());
    }
    else
    {
      percentage_solved += wp_percentage_solved / (double)waypoints.size();
      traj.insert(traj.end(), waypoint_traj.begin(), waypoint_traj.end());
      break;
    }
  }

  return percentage_solved;
}

void robot_state::JointStateGroup::ikCallbackFnAdapter(const StateValidityCallbackFn &constraint,
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

bool robot_state::JointStateGroup::getJacobian(const std::string &link_name,
                                               const Eigen::Vector3d &reference_point_position,
                                               Eigen::MatrixXd& jacobian,
                                               bool use_quaternion_representation) const
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

  const robot_model::JointModel* root_joint_model = (joint_model_group_->getJointRoots())[0];
  const robot_state::LinkState *root_link_state = kinematic_state_->getLinkState(root_joint_model->getParentLinkModel()->getName());
  Eigen::Affine3d reference_transform = root_link_state ? root_link_state->getGlobalLinkTransform() : kinematic_state_->getRootTransform();
  reference_transform = reference_transform.inverse();
  int rows = use_quaternion_representation ? 7 : 6;
  int columns = joint_model_group_->getVariableCount();
  jacobian = Eigen::MatrixXd::Zero(rows, columns);

  const robot_state::LinkState *link_state = kinematic_state_->getLinkState(link_name);
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
        unsigned int joint_index = joint_model_group_->getJointVariablesIndexMap().find(link_state->getParentJointState()->getJointModel()->getName())->second;
        double multiplier = 1.0; // to account for mimic joints
        if (link_state->getParentJointState()->getJointModel()->getMimic())
        {
          joint_index = joint_model_group_->getJointVariablesIndexMap().find(link_state->getParentJointState()->getJointModel()->getMimic()->getName())->second;
          multiplier = link_state->getParentJointState()->getJointModel()->getMimicFactor();
        }
      if (link_state->getParentJointState()->getJointModel()->getType() == robot_model::JointModel::REVOLUTE)
      {
        joint_transform = reference_transform*link_state->getGlobalLinkTransform();
        joint_axis = joint_transform.rotation()*(static_cast<const robot_model::RevoluteJointModel*>(link_state->getParentJointState()->getJointModel()))->getAxis();
        jacobian.block<3,1>(0,joint_index) = jacobian.block<3,1>(0,joint_index) + multiplier * joint_axis.cross(point_transform - joint_transform.translation());
        jacobian.block<3,1>(3,joint_index) = jacobian.block<3,1>(3,joint_index) + multiplier * joint_axis;
      }
      if (link_state->getParentJointState()->getJointModel()->getType() == robot_model::JointModel::PRISMATIC)
      {
        joint_transform = reference_transform*link_state->getGlobalLinkTransform();
        joint_axis = joint_transform*(static_cast<const robot_model::PrismaticJointModel*>(link_state->getParentJointState()->getJointModel()))->getAxis();
        jacobian.block<3,1>(0,joint_index) = jacobian.block<3,1>(0,joint_index) + multiplier * joint_axis;
      }
      if (link_state->getParentJointState()->getJointModel()->getType() == robot_model::JointModel::PLANAR)
      {
        joint_transform = reference_transform*link_state->getGlobalLinkTransform();
        joint_axis = joint_transform*Eigen::Vector3d(1.0,0.0,0.0);
        jacobian.block<3,1>(0,joint_index) = jacobian.block<3,1>(0,joint_index) + multiplier * joint_axis;
        joint_axis = joint_transform*Eigen::Vector3d(0.0,1.0,0.0);
        jacobian.block<3,1>(0,joint_index+1) = jacobian.block<3,1>(0,joint_index+1) + multiplier * joint_axis;
        joint_axis = joint_transform*Eigen::Vector3d(0.0,0.0,1.0);
        jacobian.block<3,1>(0,joint_index+2) = jacobian.block<3,1>(0,joint_index+2) + multiplier * joint_axis.cross(point_transform - joint_transform.translation());
        jacobian.block<3,1>(3,joint_index+2) = jacobian.block<3,1>(3,joint_index+2) + multiplier * joint_axis;
      }
    }
    if (link_state->getParentJointState()->getJointModel() == root_joint_model)
      break;
    link_state = link_state->getParentLinkState();
  }
  if (use_quaternion_representation) { // Quaternion representation
    // From "Advanced Dynamics and Motion Simulation" by Paul Mitiguy
    // d/dt ( [w] ) = 1/2 * [ -x -y -z ]  * [ omega_1 ]
    //        [x]           [  w -z  y ]    [ omega_2 ]
    //        [y]           [  z  w -x ]    [ omega_3 ]
    //        [z]           [ -y  x  w ]
    Eigen::Quaterniond q(link_transform.rotation());
    double w = q.w(), x = q.x(), y = q.y(), z = q.z();
    Eigen::MatrixXd quaternion_update_matrix(4,3);
    quaternion_update_matrix << -x, -y, -z,
                                 w, -z,  y,
                                 z,  w, -x,
                                -y,  x,  w;
    jacobian.block(3,0,4,columns) = 0.5*quaternion_update_matrix*jacobian.block(3,0, 3, columns);
  }
  return true;
}

std::pair<double,int> robot_state::JointStateGroup::getMinDistanceToBounds() const
{
  double distance = std::numeric_limits<double>::max();
  int index = -1;
  for(std::size_t i=0; i < joint_state_vector_.size(); ++i)
  {
    if(joint_state_vector_[i]->getType() == robot_model::JointModel::REVOLUTE)
    {
      const robot_model::RevoluteJointModel* revolute_model = dynamic_cast<const robot_model::RevoluteJointModel*> (joint_state_vector_[i]->getJointModel());
      if(revolute_model->isContinuous())
        continue;
    }
    if(joint_state_vector_[i]->getType() == robot_model::JointModel::PLANAR)
    {
      const std::vector<std::pair<double, double> >& planar_bounds = joint_state_vector_[i]->getVariableBounds();
      if(planar_bounds[0].first == -std::numeric_limits<double>::max() || planar_bounds[0].second == std::numeric_limits<double>::max() ||
         planar_bounds[1].first == -std::numeric_limits<double>::max() || planar_bounds[1].second == std::numeric_limits<double>::max() ||
         planar_bounds[2].first == -boost::math::constants::pi<double>() || planar_bounds[2].second == boost::math::constants::pi<double>())
        continue;
    }
    if(joint_state_vector_[i]->getType() == robot_model::JointModel::FLOATING)
    {
      //Joint limits are not well-defined for floating joints
      continue;
    }

    const std::vector<double>& joint_values = joint_state_vector_[i]->getVariableValues();
    const std::vector<std::pair<double, double> >& bounds = joint_state_vector_[i]->getVariableBounds();
    std::vector<double> lower_bounds, upper_bounds;
    for(std::size_t j=0; j < bounds.size(); ++j)
    {
      lower_bounds.push_back(bounds[j].first);
      upper_bounds.push_back(bounds[j].second);
    }
    double new_distance = joint_state_vector_[i]->getJointModel()->distance(joint_values, lower_bounds);
    if(new_distance < distance)
    {
      index = i;
      distance = new_distance;
    }
    new_distance = joint_state_vector_[i]->getJointModel()->distance(joint_values, upper_bounds);
    if(new_distance < distance)
    {
      index = i;
      distance = new_distance;
    }
  }
  return std::pair<double,int>(distance,index);
}
