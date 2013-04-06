/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
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

/* Author: Ioan Sucan, Sachin Chitta */

#include <moveit/ompl_interface/parameterization/work_space/pose_model_state_space.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/tools/debug/Profiler.h>

const std::string ompl_interface::PoseModelStateSpace::PARAMETERIZATION_TYPE = "PoseModel";

ompl_interface::PoseModelStateSpace::PoseModelStateSpace(const ModelBasedStateSpaceSpecification &spec) : ModelBasedStateSpace(spec)
{
  const std::pair<robot_model::SolverAllocatorFn, robot_model::SolverAllocatorMapFn>& slv = spec.joint_model_group_->getSolverAllocators();
  if (slv.first)
    poses_.push_back(PoseComponent(spec.joint_model_group_));
  else
    if (!slv.second.empty())
    {
      for (std::map<const robot_model::JointModelGroup*, robot_model::SolverAllocatorFn>::const_iterator it = slv.second.begin() ; it != slv.second.end() ; ++it)
        poses_.push_back(PoseComponent(it->first));
    }
  if (poses_.empty())
    logError("No kinematics solvers specified. Unable to construct a PoseModelStateSpace");
  constructSpaceFromPoses();
}

ompl_interface::PoseModelStateSpace::~PoseModelStateSpace()
{
}

void ompl_interface::PoseModelStateSpace::constructSpaceFromPoses()
{
  std::sort(poses_.begin(), poses_.end());  
  for (unsigned int i = 0 ; i < jointSubspaceCount_ ; ++i)
    weights_[i] = 0;
  for (std::size_t i = 0 ; i < poses_.size() ; ++i)
    addSubspace(poses_[i].state_space_, 1.0);  
  setName(getJointModelGroupName() + "_" + PARAMETERIZATION_TYPE);
  lock();
}

double ompl_interface::PoseModelStateSpace::distance(const ompl::base::State *state1, const ompl::base::State *state2) const
{
  double total = 0;
  for (unsigned int i = jointSubspaceCount_ ; i < componentCount_ ; ++i)
    total += weights_[i] * components_[i]->distance(state1->as<StateType>()->components[i], state2->as<StateType>()->components[i]);
  return total;
}

double ompl_interface::PoseModelStateSpace::getMaximumExtent() const
{
  double total = 0.0;
  for (unsigned int i = jointSubspaceCount_ ; i < componentCount_ ; ++i)
    total += weights_[i] * components_[i]->getMaximumExtent();
  return total;
}

ompl::base::State* ompl_interface::PoseModelStateSpace::allocState() const
{
  StateType *state = new StateType();
  allocStateComponents(state);
  return state;
}

void ompl_interface::PoseModelStateSpace::freeState(ompl::base::State *state) const
{
  ModelBasedStateSpace::freeState(state);
}

void ompl_interface::PoseModelStateSpace::copyState(ompl::base::State *destination, const ompl::base::State *source) const
{
  // copy the state data
  ModelBasedStateSpace::copyState(destination, source);
  
  // compute additional stuff if needed
  computeStateK(destination);
}

void ompl_interface::PoseModelStateSpace::sanityChecks() const
{
  ModelBasedStateSpace::sanityChecks(std::numeric_limits<double>::epsilon(), std::numeric_limits<float>::epsilon(), ~ompl::base::StateSpace::STATESPACE_TRIANGLE_INEQUALITY);
}

void ompl_interface::PoseModelStateSpace::interpolate(const ompl::base::State *from, const ompl::base::State *to, const double t, ompl::base::State *state) const
{    
  // we want to interpolate in Cartesian space; we do not have a guarantee that from and to
  // have their poses computed, but this is very unlikely to happen (depends how the planner gets its input states)
  ModelBasedStateSpace::interpolate(from, to, t, state);
  /*
  std::cout << "X\n";
  printState(from, std::cout);
  printState(to, std::cout);
  printState(state, std::cout);
  */
  // after interpolation we cannot be sure about the joint values (we use them as seed only)
  // so we recompute IK
  state->as<StateType>()->setJointsComputed(false);
  state->as<StateType>()->setPoseComputed(true);
  computeStateIK(state);
}

void ompl_interface::PoseModelStateSpace::setPlanningVolume(double minX, double maxX, double minY, double maxY, double minZ, double maxZ)
{
  ModelBasedStateSpace::setPlanningVolume(minX, maxX, minY, maxY, minZ, maxZ);
  ompl::base::RealVectorBounds b(3);
  b.low[0] = minX; b.low[1] = minY; b.low[2] = minZ;
  b.high[0] = maxX; b.high[1] = maxY; b.high[2] = maxZ;
  for (unsigned int i = jointSubspaceCount_ ; i < componentCount_ ; ++i)
    components_[i]->as<ompl::base::SE3StateSpace>()->setBounds(b);
}

ompl_interface::PoseModelStateSpace::PoseComponent::PoseComponent(const robot_model::JointModelGroup *subgroup) :
  subgroup_(subgroup), kinematics_solver_(subgroup->getSolverAllocators().first(subgroup))
{
  state_space_.reset(new ompl::base::SE3StateSpace());
  state_space_->setName(subgroup_->getName() + "_Workspace");
  fk_link_.resize(1, kinematics_solver_->getTipFrame());      
  joint_names_ = kinematics_solver_->getJointNames();
  joint_val_count_.resize(joint_names_.size());
  for (std::size_t i = 0 ; i < joint_names_.size() ; ++i)
    joint_val_count_[i] = subgroup_->getJointModel(joint_names_[i])->getVariableCount();
  variable_count_ = subgroup_->getVariableCount();
}

bool ompl_interface::PoseModelStateSpace::PoseComponent::computeStateFK(const ompl::base::StateSpace *full_state_space, ompl::base::State *full_state, ompl::base::State *state_part) const
{  
  // read the values from the joint state by name, in the order expected by the kinematics solver
  std::vector<double> values(variable_count_);
  unsigned int vindex = 0;
  for (std::size_t i = 0 ; i < joint_names_.size() ; ++i)
  {
    const double *v = full_state_space->getValueAddressAtName(full_state, joint_names_[i]);
    for (unsigned int j = 0 ; j < joint_val_count_[i] ; ++j)
      values[vindex++] = v[j];
  }
  
  // compute forward kinematics for the link of interest
  std::vector<geometry_msgs::Pose> poses;
  if (!kinematics_solver_->getPositionFK(fk_link_, values, poses))
    return false;
  
  // copy the resulting data to the desired location in the state
  ompl::base::SE3StateSpace::StateType *se3_state = state_part->as<ompl::base::SE3StateSpace::StateType>();
  se3_state->setXYZ(poses[0].position.x, poses[0].position.y, poses[0].position.z);
  ompl::base::SO3StateSpace::StateType &so3_state = se3_state->rotation();
  so3_state.x = poses[0].orientation.x;
  so3_state.y = poses[0].orientation.y;
  so3_state.z = poses[0].orientation.z;
  so3_state.w = poses[0].orientation.w;
  
  return true;
}

bool ompl_interface::PoseModelStateSpace::PoseComponent::computeStateIK(const ompl::base::StateSpace *full_state_space, ompl::base::State *full_state, ompl::base::State *state) const
{
  // read the values from the joint state by name, in the order expected by the kinematics solver; use these as the seed
  std::vector<double> seed_values(variable_count_);
  std::vector<double*> jaddr(joint_names_.size());
  unsigned int vindex = 0;
  for (std::size_t i = 0 ; i < joint_names_.size() ; ++i)
  {
    double *v = jaddr[i] = full_state_space->getValueAddressAtName(full_state, joint_names_[i]);
    for (unsigned int j = 0 ; j < joint_val_count_[i] ; ++j)
      seed_values[vindex++] = v[j];
  }
  /*
  std::cout << "seed: ";
  for (std::size_t i = 0 ; i < seed_values.size() ; ++i)
    std::cout << seed_values[i] << " ";
  std::cout << std::endl;
  */
    
  // construct the pose
  geometry_msgs::Pose pose;
  const ompl::base::SE3StateSpace::StateType *se3_state = state->as<ompl::base::SE3StateSpace::StateType>();
  pose.position.x = se3_state->getX();
  pose.position.y = se3_state->getY();
  pose.position.z = se3_state->getZ();
  const ompl::base::SO3StateSpace::StateType &so3_state = se3_state->rotation();
  pose.orientation.x = so3_state.x;
  pose.orientation.y = so3_state.y;
  pose.orientation.z = so3_state.z;
  pose.orientation.w = so3_state.w;
  
  // run IK
  std::vector<double> solution;
  moveit_msgs::MoveItErrorCodes dummy;
  if (!kinematics_solver_->getPositionIK(pose, seed_values, solution, dummy))
    return false;
  
  // copy solution to the joint state 
  vindex = 0;
  for (std::size_t i = 0 ; i < jaddr.size() ; ++i)
    for (unsigned int j = 0 ; j < joint_val_count_[i] ; ++j)
      jaddr[i][j] = solution[vindex++];
  
  return true;      
}

bool ompl_interface::PoseModelStateSpace::computeStateFK(ompl::base::State *state) const
{
  if (state->as<StateType>()->poseComputed())
    return true;
  for (std::size_t i = 0 ; i < poses_.size() ; ++i)
    if (!poses_[i].computeStateFK(this, state, state->as<StateType>()->components[componentCount_ - i - 1]))
    {
      state->as<StateType>()->markInvalid();
      return false;
    }
  state->as<StateType>()->setPoseComputed(true);
  return true;
}

bool ompl_interface::PoseModelStateSpace::computeStateIK(ompl::base::State *state) const
{  
  if (state->as<StateType>()->jointsComputed())
    return true;
  for (std::size_t i = 0 ; i < poses_.size() ; ++i)
    if (!poses_[i].computeStateIK(this, state, state->as<StateType>()->components[componentCount_ - i - 1]))
    {
      state->as<StateType>()->markInvalid();
      return false;      
    }
  state->as<StateType>()->setJointsComputed(true);
  return true;
}

bool ompl_interface::PoseModelStateSpace::computeStateK(ompl::base::State *state) const
{
  if (state->as<StateType>()->jointsComputed() && !state->as<StateType>()->poseComputed())
    return computeStateFK(state);
  if (!state->as<StateType>()->jointsComputed() && state->as<StateType>()->poseComputed())
    return computeStateIK(state);
  if (state->as<StateType>()->jointsComputed() && state->as<StateType>()->poseComputed())
    return true;
  state->as<StateType>()->markInvalid();
  return false;
} 

void ompl_interface::PoseModelStateSpace::afterStateSample(ompl::base::State *sample) const
{
  ModelBasedStateSpace::afterStateSample(sample); 
  sample->as<StateType>()->setJointsComputed(true);
  sample->as<StateType>()->setPoseComputed(false);  
  computeStateFK(sample);
  /*
  std::cout << "SAMPLE:\n";
  printState(sample, std::cout);
  std::cout << "---------- SAMPLE\n"; */
}

void ompl_interface::PoseModelStateSpace::copyToOMPLState(ompl::base::State *state, const robot_state::JointStateGroup* jsg) const
{
  ModelBasedStateSpace::copyToOMPLState(state, jsg);
  state->as<StateType>()->setJointsComputed(true);  
  state->as<StateType>()->setPoseComputed(false);
  computeStateFK(state);
  /*
  std::cout << "COPY STATE IN:\n";
  printState(state, std::cout);
  std::cout << "---------- COPY STATE IN\n"; */
}
