//Software License Agreement (BSD License)

//Copyright (c) 2011, Willow Garage, Inc.
//All rights reserved.

//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions
//are met:

// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of Willow Garage, Inc. nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.

//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//POSSIBILITY OF SUCH DAMAGE.

#include <kinematics_constraint_aware/kinematics_solver_constraint_aware.h>
#include <boost/bind.hpp>
#include <kinematic_constraints/kinematic_constraint.h>

namespace kinematics_constraint_aware 
{

KinematicsSolverConstraintAware::KinematicsSolverConstraintAware(const kinematics::KinematicsBasePtr& solver,
                                                                 const planning_models::KinematicModelConstPtr& kmodel,
                                                                 const std::string& group_name) :
  group_name_(group_name),
  kmodel_(kmodel),
  active_(false)
{
  if(group_name.empty()) {
    ROS_INFO_STREAM("Must have non-empty group");
    return;
  }

  solver_map_[group_name_] = solver;
  state_ = new planning_models::KinematicState(kmodel_);
  
  const planning_models::KinematicModel::JointModelGroup* joint_model_group = kmodel_->getJointModelGroup(group_name);
  if(joint_model_group == NULL) {
    ROS_WARN_STREAM("No joint group " << group_name);
    return;
  }
  const srdf::Model::Group& config =
    kmodel_->getJointModelGroupConfigMap().at(group_name);
  if(config.chains_.empty()) {
    ROS_WARN_STREAM("Group does not have base/tip config definition, can't solve ik");
    return;
  }
  
  if(!solver_map_[group_name]->initialize(group_name,
                                          config.chains_[0].first,
                                          config.chains_[0].second,
                                          .025)) {
    return;
  } 
  
  tip_frame_map_[group_name] = solver_map_[group_name]->getTipFrame();
  base_frame_map_[group_name] = solver_map_[group_name]->getBaseFrame();
  
  const planning_models::KinematicModel::LinkModel* end_effector_link = kmodel_->getLinkModel(solver_map_[group_name_]->getTipFrame());
  end_effector_collision_links_[group_name] = kmodel_->getChildLinkModelNames(end_effector_link);
  if(end_effector_collision_links_[group_name].size() >= 1) {
    end_effector_collision_links_[group_name].erase(end_effector_collision_links_[group_name].begin());
  }

  active_ = true;
}

KinematicsSolverConstraintAware::KinematicsSolverConstraintAware(const std::map<std::string, kinematics::KinematicsBasePtr>& solver_map,
                                                                 const planning_models::KinematicModelConstPtr& kmodel,
                                                                 const std::string& group_name) :
  group_name_(group_name),
  kmodel_(kmodel),
  active_(false),
  solver_map_(solver_map)
{
  if(group_name.empty()) {
    ROS_INFO_STREAM("Must have non-empty group");
    return;
  }

  const planning_models::KinematicModel::JointModelGroup* joint_model_group = kmodel_->getJointModelGroup(group_name);
  if(joint_model_group == NULL) {
    ROS_WARN_STREAM("No joint group " << group_name);
    return;
  }

  const srdf::Model::Group& config =
    kmodel_->getJointModelGroupConfigMap().at(group_name);

  for(unsigned int i = 0; i < config.subgroups_.size(); i++) {
    ROS_DEBUG_STREAM("Building for subgroup " << config.subgroups_[i]);
    const srdf::Model::Group& sub_group_config =
      kmodel_->getJointModelGroupConfigMap().at(config.subgroups_[i]);
    ROS_DEBUG_STREAM("Building for subgroup " << sub_group_config.name_);
    if(sub_group_config.chains_.size() > 0) {
      if(solver_map_.find(sub_group_config.name_) == solver_map_.end()) {
        ROS_WARN_STREAM("No solver specified for subgroup with chains " << sub_group_config.name_);
        return;
      }
      if(!solver_map_[sub_group_config.name_]->initialize(sub_group_config.name_,
                                                          sub_group_config.chains_[0].first,
                                                          sub_group_config.chains_[0].second,
                                                          .1)) {
        ROS_WARN_STREAM("Initialization for subgroup " << sub_group_config.name_ << " failed");
        return;
      }
      tip_frame_map_[sub_group_config.name_] = solver_map_[sub_group_config.name_]->getTipFrame();
      base_frame_map_[sub_group_config.name_] = solver_map_[sub_group_config.name_]->getBaseFrame();
      const planning_models::KinematicModel::LinkModel* end_effector_link = 
        kmodel_->getLinkModel(solver_map_[sub_group_config.name_]->getTipFrame());
      end_effector_collision_links_[sub_group_config.name_] = kmodel_->getChildLinkModelNames(end_effector_link);
      if(end_effector_collision_links_[sub_group_config.name_].size() >= 1) {
        end_effector_collision_links_[sub_group_config.name_].erase(end_effector_collision_links_[sub_group_config.name_].begin());
      }
    }
  }
  
  active_ = true;
}

bool KinematicsSolverConstraintAware::getPositionFK(const planning_models::KinematicState* robot_state,
                                                    const std::vector<std::string>& link_names,
                                                    std::vector<geometry_msgs::Pose> &poses)
{
  poses.resize(link_names.size());

  for(unsigned int i = 0; i < link_names.size(); i++) {
    const planning_models::KinematicState::LinkState* ls = robot_state->getLinkState(link_names[i]);
    if(ls == NULL) return false;
    planning_models::msgFromPose(ls->getGlobalLinkTransform(), poses[i]);
  }
  return true;
}

bool KinematicsSolverConstraintAware::getPositionIK(const geometry_msgs::Pose &pose,
                                                    const planning_models::KinematicState* seed_state,
                                                    const planning_scene::PlanningSceneConstPtr& scene,
                                                    sensor_msgs::JointState& solution,
                                                    moveit_msgs::MoveItErrorCodes& error_code)
{
  if(solver_map_.size() > 1) {
    ROS_WARN_STREAM("No single position ik for multi-group");
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
    return false;
  }
  std::map<std::string, double> seed_state_map;
  seed_state->getStateValues(seed_state_map);
  std::vector<double> seed_state_vector(solver_map_[group_name_]->getJointNames().size());
  for(unsigned int i = 0; i < solver_map_[group_name_]->getJointNames().size(); i++) {
    seed_state_vector[i] = seed_state_map[solver_map_[group_name_]->getJointNames()[i]];
  } 

  std::vector<double> sol;
  bool ik_valid = solver_map_[group_name_]->getPositionIK(pose,
                                                          seed_state_vector,
                                                          sol,
                                                          error_code);
  if(ik_valid) {
    solution.name = solver_map_[group_name_]->getJointNames();
    solution.position = sol;
  } else {
    solution.name.clear();
    solution.position.clear();
  }
  return ik_valid;
}

bool KinematicsSolverConstraintAware::getPositionIK(const std::map<std::string, geometry_msgs::Pose> &poses,
                                                    const planning_models::KinematicState* seed_state,
                                                    const planning_scene::PlanningSceneConstPtr& scene,
                                                    sensor_msgs::JointState& solution,
                                                    moveit_msgs::MoveItErrorCodes& error_code)
{
  if(solver_map_.size() > 1) {
    ROS_WARN_STREAM("No multi-pose ik for single group");
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
    return false;
  }
  std::map<std::string, double> seed_state_map;
  seed_state->getStateValues(seed_state_map);

  for(std::map<std::string, geometry_msgs::Pose>::const_iterator it = poses.begin();
      it != poses.end(); 
      it++) {
    std::map<std::string, kinematics::KinematicsBasePtr>::iterator solver_it = solver_map_.find(it->first);
    if(solver_it == solver_map_.end()) {
      ROS_WARN_STREAM("No solver named " << it->first);
      error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
      return false;      
    }
    kinematics::KinematicsBasePtr& solver = solver_it->second;
    std::vector<double> seed_state_vector(solver->getJointNames().size());
    for(unsigned int i = 0; i < solver->getJointNames().size(); i++) {
      seed_state_vector[i] = seed_state_map[solver->getJointNames()[i]];
    }
    std::vector<double> sol;
    bool ik_valid = solver->getPositionIK(it->second,
                                          seed_state_vector,
                                          sol,
                                          error_code);
    if(ik_valid) {
      solution.name.insert(solution.name.end(),
                           solver->getJointNames().begin(),
                           solver->getJointNames().end());
      solution.position.insert(solution.position.end(),
                               sol.begin(),
                               sol.end());
    } else {
      solution.name.clear();
      solution.position.clear();
      return ik_valid;
    }
  }
  return true;
}

bool KinematicsSolverConstraintAware::findConstraintAwareSolution(const geometry_msgs::Pose& pose,
                                                                  const moveit_msgs::Constraints& constraints,
                                                                  const planning_models::KinematicState* seed_state,
                                                                  const planning_scene::PlanningSceneConstPtr& scene,
                                                                  sensor_msgs::JointState& solution,
                                                                  moveit_msgs::MoveItErrorCodes& error_code, 
                                                                  const bool& do_initial_pose_check,
                                                                  const bool& use_unpadded_robot)
{
  return findConstraintAwareSolution(pose,
                                     constraints,
                                     seed_state,
                                     scene,
                                     scene->getAllowedCollisionMatrix(),
                                     solution, 
                                     error_code,
                                     do_initial_pose_check,
                                     use_unpadded_robot);
}

bool KinematicsSolverConstraintAware::findConstraintAwareSolution(const geometry_msgs::Pose& pose,
                                                                  const moveit_msgs::Constraints& constraints,
                                                                  const planning_models::KinematicState* seed_state,
                                                                  const planning_scene::PlanningSceneConstPtr& scene,
                                                                  const collision_detection::AllowedCollisionMatrix& acm,
                                                                  sensor_msgs::JointState& solution,
                                                                  moveit_msgs::MoveItErrorCodes& error_code, 
                                                                  const bool& do_initial_pose_check,
                                                                  const bool& use_unpadded_robot)
{
  if(solver_map_.size() > 1) {
    ROS_WARN_STREAM("No single position ik for multi-group");
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
    return false;
  }

  do_initial_pose_check_ = do_initial_pose_check;
  constraints_ = constraints;
  planning_scene_ = scene;
  acm_ = acm;
  use_unpadded_robot_ = use_unpadded_robot;

  last_initial_pose_check_collision_result_ = collision_detection::CollisionResult();
  
  state_ = new planning_models::KinematicState(*seed_state);

  //TODO - need better way to do this
  std::map<std::string, double> seed_state_map;
  seed_state->getStateValues(seed_state_map);

  std::vector<double> seed_state_vector(solver_map_[group_name_]->getJointNames().size());
  for(unsigned int i = 0; i < solver_map_[group_name_]->getJointNames().size(); i++) {
    seed_state_vector[i] = seed_state_map[solver_map_[group_name_]->getJointNames()[i]];
  } 
  
  std::vector<double> sol;
  bool ik_valid 
    = solver_map_[group_name_]->searchPositionIK(pose,
                                                 seed_state_vector,
                                                 1.0,
                                                 sol,
                                                 boost::bind(&KinematicsSolverConstraintAware::initialPoseCheck, this, _1, _2, _3),
                                                 boost::bind(&KinematicsSolverConstraintAware::collisionCheck, this, _1, _2, _3),
                                                 error_code);
  if(ik_valid) {
    solution.name = solver_map_[group_name_]->getJointNames();
    solution.position = sol;
  } else {
    solution.name.clear();
    solution.position.clear();
  }
  delete state_;
  return ik_valid;
}

bool KinematicsSolverConstraintAware::findConstraintAwareSolution(const std::map<std::string, geometry_msgs::Pose>& poses,
                                                                  const std::map<std::string, unsigned int>& redundancies,
                                                                  const moveit_msgs::Constraints& constraints,
                                                                  const planning_models::KinematicState* seed_state,
                                                                  const planning_scene::PlanningSceneConstPtr& scene,
                                                                  sensor_msgs::JointState& solution,
                                                                  moveit_msgs::MoveItErrorCodes& error_code, 
                                                                  const bool& do_initial_pose_check,
                                                                  const bool& use_unpadded_robot)
{
  return findConstraintAwareSolution(poses, 
                                     redundancies,
                                     constraints,
                                     seed_state,
                                     scene,
                                     scene->getAllowedCollisionMatrix(),
                                     solution,
                                     error_code,
                                     do_initial_pose_check,
                                     use_unpadded_robot);
}

bool KinematicsSolverConstraintAware::findConstraintAwareSolution(const std::map<std::string, geometry_msgs::Pose>& poses,
                                                                  const std::map<std::string, unsigned int>& redundancies,
                                                                  const moveit_msgs::Constraints& constraints,
                                                                  const planning_models::KinematicState* seed_state,
                                                                  const planning_scene::PlanningSceneConstPtr& scene,
                                                                  const collision_detection::AllowedCollisionMatrix& acm,
                                                                  sensor_msgs::JointState& solution,
                                                                  moveit_msgs::MoveItErrorCodes& error_code, 
                                                                  const bool& do_initial_pose_check,
                                                                  const bool& use_unpadded_robot)
{
  if(solver_map_.size() == 1) {
    ROS_WARN_STREAM("No single position ik for multi-group");
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
    return false;
  }

  do_initial_pose_check_ = do_initial_pose_check;
  constraints_ = constraints;
  planning_scene_ = scene;
  acm_ = acm;
  use_unpadded_robot_ = use_unpadded_robot;

  last_initial_pose_check_collision_result_ = collision_detection::CollisionResult();

  state_ = new planning_models::KinematicState(*seed_state);

  std::map<std::string, double> seed_state_map;
  seed_state->getStateValues(seed_state_map);

  if(do_initial_pose_check) {
    if(!multiGroupInitialPoseCheck(seed_state, poses)) {
      error_code.val = error_code.IK_LINK_IN_COLLISION;
      delete state_;
      return false;
    }
    ROS_DEBUG_STREAM("Passing intial pose check");
  }

  std::map<std::string, int> positive_increments;
  std::map<std::string, int> negative_increments;
  std::map<std::string, double> start_angles;
  std::map<std::string, double> dis_angles;
  std::map<std::string, int> current_counts;
  std::list<std::string> names_list;

  for(std::map<std::string, geometry_msgs::Pose>::const_iterator it = poses.begin();
      it != poses.end(); 
      it++) {
    names_list.push_back(it->first);
    std::map<std::string, kinematics::KinematicsBasePtr>::iterator solver_it = solver_map_.find(it->first);
    if(solver_it == solver_map_.end()) {
      ROS_WARN_STREAM("No solver named " << it->first);
      error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
      delete state_;
      return false;      
    }
    kinematics::KinematicsBasePtr& solver = solver_it->second;

    //building necessary stuff
    if(redundancies.find(it->first) == redundancies.end()) {
      ROS_WARN_STREAM("No redundancy specified for group " << it->first);
      delete state_;
      return false;
    }

    unsigned int redundancy_index = redundancies.at(it->first);

    if(redundancy_index >= solver->getJointNames().size()) {
      ROS_WARN_STREAM("Redundancy value of " << redundancy_index 
                      << " for group " << it->first << " larger than joints size " << solver->getJointNames().size());
      delete state_;
      return false;
    }

    std::string redundancy_name = solver->getJointNames()[redundancy_index];
    const planning_models::KinematicModel::JointModel* jm = planning_scene_->getKinematicModel()->getJointModel(redundancy_name);
    
    std::pair<double, double> bounds;
    jm->getVariableBounds(redundancy_name, bounds);

    ROS_DEBUG_STREAM("Name " << redundancy_name << " bounds " << bounds.first << " " << bounds.second << " start " << seed_state_map[redundancy_name]);

    dis_angles[it->first] = solver->getSearchDiscretization();
    start_angles[it->first] =  seed_state_map[redundancy_name]; 
    positive_increments[it->first] = (int)((bounds.second-seed_state_map[redundancy_name])/dis_angles[it->first]);
    negative_increments[it->first] = (int)((seed_state_map[redundancy_name] - bounds.first)/dis_angles[it->first]);
    ROS_DEBUG_STREAM("Dis " << dis_angles[it->first] << " pos " << positive_increments[it->first] << " neg " << negative_increments[it->first]);
    current_counts[it->first] = 0;
  }

  std::list<std::string> fixed_list = names_list;
  std::vector<collision_detection::AllowedCollisionMatrix> disabled_acms(fixed_list.size(), acm_);

  unsigned int index = 0;
  //no need to do this for the first 
  for(std::list<std::string>::iterator it = names_list.begin();
      it != names_list.end();
      it++, index++) {
    const std::vector<std::string>& link_names = solver_map_.at(*it)->getLinkNames();
    for(unsigned int i = 0; i < index; i++) {
      for(unsigned int j = 0; j < link_names.size(); j++) {
        disabled_acms[i].setDefaultEntry(link_names[j], true);
        disabled_acms[i].setEntry(link_names[j], true);
      }
    }
  }

  std::list<std::string> search_list(1, fixed_list.front());
  fixed_list.pop_front();
  ROS_DEBUG_STREAM("First pose for " << search_list.front() << " is " << poses.at(search_list.back()).position.x 
                  << " " << poses.at(search_list.back()).position.y
                  << " " << poses.at(search_list.back()).position.z);

  bool make_new_seed = true;
  std::vector<double> seed_state_vector;
  std::list<double> redundancy_positions;
  unsigned int count = 0;
  while(1) {
    kinematics::KinematicsBasePtr& solver = solver_map_.at(search_list.back());
    
    if(make_new_seed) {
      ROS_DEBUG_STREAM("Making new seed for " << search_list.back());
      seed_state_vector.resize(solver->getJointNames().size());
      for(unsigned int i = 0; i < solver->getJointNames().size(); i++) {
        seed_state_vector[i] = seed_state_map[solver->getJointNames()[i]];
      } 
      make_new_seed = false;
    }

    seed_state_vector[redundancies.at(search_list.back())] 
      = start_angles[search_list.back()]+ dis_angles[search_list.back()]*current_counts[search_list.back()];

    if(redundancy_positions.size() < search_list.size()) {
      redundancy_positions.push_back(seed_state_vector[redundancies.at(search_list.back())]); 
    } else {
      redundancy_positions.back() = (seed_state_vector[redundancies.at(search_list.back())]); 
    }
    std::list<double>::iterator dit = redundancy_positions.begin();
    ROS_DEBUG_STREAM("Trying");
    for(std::list<std::string>::iterator it = search_list.begin();
        it != search_list.end();
        it++, dit++) {
      ROS_DEBUG_STREAM("Group " << (*it) << " position " << (*dit));
    }
    ROS_DEBUG_STREAM("\n");
     
    ROS_DEBUG_STREAM("Count is " << current_counts[search_list.back()] << " seed state " << 
                    start_angles[search_list.back()]+ dis_angles[search_list.back()]*current_counts[search_list.back()]);
    
    std::vector<double> sol;
    count++;
    bool ik_valid = solver->getPositionIK(poses.at(search_list.back()),
                                          seed_state_vector,
                                          sol,
                                          error_code);
    if(ik_valid) {
      std::map<std::string, double> sol_values;
      for(unsigned int i = 0; i < solver->getJointNames().size(); i++) {
        sol_values[solver->getJointNames()[i]] = sol[i];
      }
      state_->setStateValues(sol_values);
      collision_detection::CollisionRequest req;
      collision_detection::CollisionResult res;
      if(!use_unpadded_robot_) {
        planning_scene_->checkCollision(req, res, *state_, disabled_acms[search_list.size()-1]);
      } else {
        planning_scene_->checkCollisionUnpadded(req, res, *state_, disabled_acms[search_list.size()-1]);
      }
      if(!res.collision) {
        if(fixed_list.empty()) {
          //valid values for everything
          break;
        }
        ROS_DEBUG_STREAM("Have valid ik for " << search_list.back() << " pushing back " << fixed_list.front());
        search_list.push_back(fixed_list.front());
        make_new_seed = true;
        fixed_list.pop_front();
        continue;
      } else {
        ROS_DEBUG_STREAM("Stuff is in collision");
      }
    } else {
      ROS_DEBUG_STREAM("Ik not valid " << error_code.val);
    }
    

    while(1) {
      bool ok = getCount(current_counts[search_list.back()], positive_increments[search_list.back()], -negative_increments[search_list.back()]);
      if(ok) {
        break;
      }
      ROS_DEBUG_STREAM("Count not ok for " << search_list.back());
      //reset current count
      make_new_seed = true;
      std::string bad_ok = search_list.back();
      current_counts[search_list.back()] = 0;
      search_list.pop_back();
      redundancy_positions.pop_back();
      if(search_list.empty()) {
        ROS_DEBUG_STREAM("No ik solution");
        error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
        ROS_INFO_STREAM("Count is " << count);
        delete state_;
        return false;
      }
      fixed_list.push_back(bad_ok);
    }
  }
  ROS_DEBUG_STREAM("Returning ok");
  state_->getStateValues(solution);
  delete state_;
  return true;
}                                          

bool KinematicsSolverConstraintAware::findConsistentConstraintAwareSolution(const geometry_msgs::Pose& pose,
                                                                            const moveit_msgs::Constraints& constraints,
                                                                            const planning_models::KinematicState* seed_state,
                                                                            const planning_scene::PlanningSceneConstPtr& scene,
                                                                             sensor_msgs::JointState& solution,
                                                                            moveit_msgs::MoveItErrorCodes& error_code, 
                                                                            const unsigned int& redundancy,
                                                                            const double& max_consistency,
                                                                            const bool& do_initial_pose_check,
                                                                            const bool& use_unpadded_robot)
{
  return findConsistentConstraintAwareSolution(pose,
                                               constraints,
                                               seed_state,
                                               scene,
                                               scene->getAllowedCollisionMatrix(),
                                               solution,
                                               error_code,
                                               redundancy,
                                               max_consistency,
                                               do_initial_pose_check,
                                               use_unpadded_robot);
}
 
bool KinematicsSolverConstraintAware::findConsistentConstraintAwareSolution(const geometry_msgs::Pose& pose,
                                                                            const moveit_msgs::Constraints& constraints,
                                                                            const planning_models::KinematicState* seed_state,
                                                                            const planning_scene::PlanningSceneConstPtr& scene,
                                                                            const collision_detection::AllowedCollisionMatrix& acm,
                                                                            sensor_msgs::JointState& solution,
                                                                            moveit_msgs::MoveItErrorCodes& error_code, 
                                                                            const unsigned int& redundancy,
                                                                            const double& max_consistency,
                                                                            const bool& do_initial_pose_check,
                                                                            const bool& use_unpadded_robot)
{

  if(solver_map_.size() > 1) {
    ROS_WARN_STREAM("No single position ik for multi-group");
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
    return false;
  }


  do_initial_pose_check_ = do_initial_pose_check;
  planning_scene_ = scene;
  constraints_ = constraints;
  acm_ = acm;
  use_unpadded_robot_ = use_unpadded_robot;

  last_initial_pose_check_collision_result_ = collision_detection::CollisionResult();
  
  std::map<std::string, double> seed_state_map;
  seed_state->getStateValues(seed_state_map);
  state_ = new planning_models::KinematicState(*seed_state);

  double init = 0.0;
  std::vector<double> seed_state_vector(solver_map_[group_name_]->getJointNames().size());
  for(unsigned int i = 0; i < solver_map_[group_name_]->getJointNames().size(); i++) {
    seed_state_vector[i] = seed_state_map[solver_map_[group_name_]->getJointNames()[i]];
    if(i == redundancy) {
      ROS_INFO_STREAM("Seed state value " << seed_state_vector[i]);
      init = seed_state_vector[i];
    }
  } 
  
  std::vector<double> sol;
  bool ik_valid = solver_map_[group_name_]->searchPositionIK(pose,
                                                             seed_state_vector,
                                                             1.0,
                                                             redundancy,
                                                             max_consistency,
                                                             sol,
                                                             boost::bind(&KinematicsSolverConstraintAware::initialPoseCheck, this, _1, _2, _3),
                                                             boost::bind(&KinematicsSolverConstraintAware::collisionCheck, this, _1, _2, _3),
                                                             error_code);
  if(ik_valid) {
    solution.name = solver_map_[group_name_]->getJointNames();
    ROS_INFO_STREAM("After fact " << solution.position[redundancy] << " diff " << init-solution.position[redundancy] << " max " << max_consistency); 
    solution.position = sol;
  } else {
    solution.name.clear();
    solution.position.clear();
  }
  delete state_;
  return ik_valid;
}

std::string KinematicsSolverConstraintAware::getCollisionDetectedString(const collision_detection::CollisionResult& res)
{
  std::stringstream ret;
  if(res.contacts.empty()) {
    ret << "No contacts.";
  }
  for(collision_detection::CollisionResult::ContactMap::const_iterator it = res.contacts.begin();
      it != res.contacts.end();
      it++) {
    ret << "Collision between " << it->first.first << " and " << it->first.second << "\n";
  }
  return ret.str();
}

void KinematicsSolverConstraintAware::collisionCheck(const geometry_msgs::Pose &ik_pose,
                                                     const std::vector<double> &ik_solution,
                                                     moveit_msgs::MoveItErrorCodes &error_code)
{
  bool use_acm = false;

  planning_models::KinematicState::JointStateGroup* jsg;
  if(!current_subgroup_.empty()) {
    jsg = state_->getJointStateGroup(current_subgroup_);
    use_acm = true;
  } else {
    jsg = state_->getJointStateGroup(group_name_);
  }
  jsg->setStateValues(ik_solution);
  error_code.val = error_code.SUCCESS;
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  //req.contacts = true;
  //req.max_contacts = 1;
  if(!use_unpadded_robot_) {
    if(use_acm) { 
      planning_scene_->checkCollision(req, res, *state_, current_acm_);
    } else {
      planning_scene_->checkCollision(req, res, *state_);
    }
  } else {
    if(use_acm) { 
      planning_scene_->checkCollisionUnpadded(req, res, *state_, current_acm_);
    } else {
      planning_scene_->checkCollisionUnpadded(req, res, *state_, acm_);
    }
  }
  if(res.collision) {
    ROS_DEBUG_STREAM_NAMED("kinematics_collision", getCollisionDetectedString(res));
    error_code.val = error_code.COLLISION_CONSTRAINTS_VIOLATED;
    ROS_DEBUG_STREAM("Collision constraints violated");
  } else if(!kinematic_constraints::doesKinematicStateObeyConstraints(*state_, 
                                                                      planning_scene_->getTransforms(),
                                                                      constraints_, 
                                                                      false)) {
    error_code.val = error_code.GOAL_CONSTRAINTS_VIOLATED;
  }
}

void KinematicsSolverConstraintAware::initialPoseCheck(const geometry_msgs::Pose &ik_pose,
                                                       const std::vector<double> &ik_solution,
                                                       moveit_msgs::MoveItErrorCodes &error_code)
{
  if(!do_initial_pose_check_) {
    error_code.val = error_code.SUCCESS;
    return;
  }
  std::string kinematic_frame_id = solver_map_[group_name_]->getBaseFrame();
  std::string planning_frame_id = planning_scene_->getPlanningFrame();
  //TODO - should be a check that we can actually transform, better transform library
  Eigen::Affine3d cur;
  planning_models::poseFromMsg(ik_pose, cur);
  Eigen::Affine3d nt;
  planning_scene_->getTransforms()->transformPose(*state_, kinematic_frame_id, cur, nt);
  state_->updateStateWithLinkAt(solver_map_[group_name_]->getTipFrame(), nt);

  //disabling all collision for arm links
  collision_detection::AllowedCollisionMatrix acm = acm_;
  for(unsigned int i = 0; i < solver_map_[group_name_]->getLinkNames().size(); i++) {
    acm.setDefaultEntry(solver_map_[group_name_]->getLinkNames()[i], true);
    acm.setEntry(solver_map_[group_name_]->getLinkNames()[i], true);
  }
 
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  //last_initial_pose_check_collision_result_ = collision_detection::CollisionResult();
  //req.contacts = true;
  //req.max_contacts = 1;
  //planning_scene_->checkCollision(req, last_initial_pose_check_collision_result_, *state_, acm);
  planning_scene_->checkCollision(req, res, *state_, acm);
  if(res.collision) {
    //req.verbose = true;
    //planning_scene_->checkCollision(req, res, *state_, acm);
    //ROS_INFO_STREAM("Contacts size is " << res.contacts.size());
    //ROS_INFO_STREAM_NAMED("kinematics_collisions", getCollisionDetectedString(last_initial_pose_check_collision_result_));
    error_code.val = error_code.IK_LINK_IN_COLLISION;
    ROS_DEBUG_STREAM("Initial pose check failing");
  } else {
    error_code.val = error_code.SUCCESS;
  }
}

bool KinematicsSolverConstraintAware::interpolateIKDirectional(const geometry_msgs::Pose& start_pose,
                                                               const Eigen::Vector3d& direction,
                                                               const double& distance,
                                                               const moveit_msgs::Constraints& constraints,
                                                               const planning_models::KinematicState* seed_state,
                                                               const planning_scene::PlanningSceneConstPtr& scene,
                                                               moveit_msgs::MoveItErrorCodes& error_code, 
                                                               trajectory_msgs::JointTrajectory& traj,
                                                               const unsigned int& redundancy,
                                                               const double& max_consistency,
                                                               const bool& reverse, 
                                                               const bool& premultiply,
                                                               const unsigned int& num_points,
                                                               const ros::Duration& total_dur,
                                                               const bool& do_initial_pose_check,
                                                               const bool& use_unpadded_robot)
{
  return interpolateIKDirectional(start_pose,
                                  direction,
                                  distance,
                                  constraints,
                                  seed_state,
                                  scene,
                                  scene->getAllowedCollisionMatrix(),
                                  error_code,
                                  traj,
                                  redundancy,
                                  max_consistency,
                                  reverse,
                                  premultiply,
                                  num_points,
                                  total_dur,
                                  do_initial_pose_check,
                                  use_unpadded_robot);
}

bool KinematicsSolverConstraintAware::interpolateIKDirectional(const geometry_msgs::Pose& start_pose,
                                                               const Eigen::Vector3d& direction,
                                                               const double& distance,
                                                               const moveit_msgs::Constraints& constraints,
                                                               const planning_models::KinematicState* seed_state,
                                                               const planning_scene::PlanningSceneConstPtr& scene,
                                                               const collision_detection::AllowedCollisionMatrix& acm,
                                                               moveit_msgs::MoveItErrorCodes& error_code, 
                                                               trajectory_msgs::JointTrajectory& traj,
                                                               const unsigned int& redundancy,
                                                               const double& max_consistency,
                                                               const bool& reverse, 
                                                               const bool& premultiply,
                                                               const unsigned int& num_points,
                                                               const ros::Duration& total_dur,
                                                               const bool& do_initial_pose_check,
                                                               const bool& use_unpadded_robot)
{
  trajectory_msgs::JointTrajectory ret_traj;
  ret_traj.joint_names = solver_map_[group_name_]->getJointNames();
  ret_traj.points.resize(num_points);

  Eigen::Affine3d first_pose;
  planning_models::poseFromMsg(start_pose, first_pose);

  planning_models::KinematicState cont_state(*seed_state);

  for(unsigned int i = 1; i <= num_points; i++) {

    unsigned int ind;
    if(reverse) {
      ind = num_points-i;
    } else {
      ind = i-1;
    }

    //assumes that the axis is aligned
    Eigen::Affine3d trans(Eigen::Translation3d(direction*((i-1)*1.0)*fabs(distance/(num_points*1.0)))*Eigen::Quaterniond(1.0,0.0,0.0,0.0));
    Eigen::Affine3d mult_trans;
    if(premultiply) {
      mult_trans = trans*first_pose;
    } else {
      mult_trans = first_pose*trans;
    }
    geometry_msgs::Pose trans_pose;
    planning_models::msgFromPose(mult_trans, trans_pose);

    sensor_msgs::JointState solution;
    moveit_msgs::MoveItErrorCodes temp_error_code;
    if(findConsistentConstraintAwareSolution(trans_pose,
                                             constraints,
                                             &cont_state,
                                             scene,
                                             acm,
                                             solution,
                                             temp_error_code,
                                             redundancy,
                                             max_consistency,
                                             do_initial_pose_check,
                                             use_unpadded_robot)) {
      cont_state.setStateValues(solution);
      ret_traj.points[ind].positions = solution.position;
      ret_traj.points[ind].time_from_start = ros::Duration((ind*1.0)*total_dur.toSec()/(num_points*1.0));
      ROS_INFO_STREAM("Point " << ind << " redundancy " << solution.position[redundancy]);
    } else {
      ROS_DEBUG_STREAM("Point " << i << " of " << num_points << " infeasible " << temp_error_code.val);
      ROS_DEBUG_STREAM("Point x y z " << trans_pose.position.x << " " << trans_pose.position.y << " " << trans_pose.position.z);
      return false;
    }
  }
  traj = ret_traj;
  return true;
}

bool KinematicsSolverConstraintAware::multiGroupInitialPoseCheck(const planning_models::KinematicState* seed_state,
                                                                 const std::map<std::string, geometry_msgs::Pose>& poses) {

  std::string planning_frame_id = planning_scene_->getPlanningFrame();
  collision_detection::AllowedCollisionMatrix acm = acm_;
  
  std::vector<std::string> arm_links;

  for(std::map<std::string, geometry_msgs::Pose>::const_iterator it = poses.begin();
      it != poses.end(); 
      it++) {
    std::map<std::string, kinematics::KinematicsBasePtr>::iterator solver_it = solver_map_.find(it->first);
    if(solver_it == solver_map_.end()) {
      ROS_WARN_STREAM("No solver named " << it->first);
      return false;      
    }
    std::string kinematic_frame_id = solver_it->second->getBaseFrame();
    //TODO - should be a check that we can actually transform, better transform library
    Eigen::Affine3d cur;
    planning_models::poseFromMsg(it->second, cur);
    Eigen::Affine3d nt;
    planning_scene_->getTransforms()->transformPose(*state_, kinematic_frame_id, cur, nt);
    state_->updateStateWithLinkAt(solver_it->second->getTipFrame(), nt);

    //disabling all collision for arm links
    for(unsigned int i = 0; i < solver_it->second->getLinkNames().size(); i++) { 
      ROS_DEBUG_STREAM("Disabling collisions with link " << solver_it->second->getLinkNames()[i]);
      arm_links.push_back(solver_it->second->getLinkNames()[i]);
      acm.setDefaultEntry(solver_it->second->getLinkNames()[i], true);
      acm.setEntry(solver_it->second->getLinkNames()[i], true);
    }
  }
  
  collision_detection::AllowedCollisionMatrix other_arms = acm;
  //need to re-enable collisions for all end-effector links
  for(unsigned int i = 0; i < end_effector_link_vector_.size(); i++) {
    other_arms.setEntry(end_effector_link_vector_[i], arm_links[i], false);
  }
  
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  planning_scene_->checkCollision(req, res, *state_, acm);
  if(res.collision) {
    ROS_INFO_STREAM("Multi-arm initial pose check failing");
    return false;
  } 

  std::map<std::string, double> seed_state_map;
  seed_state->getStateValues(seed_state_map);

  do_initial_pose_check_ = false;
  current_acm_ = other_arms;
 
  //now we run an individual search for each group with all arm collision checks disabled, but with end effectors in the scene
  std::vector<double> seed_state_vector;
  std::map<std::string, double> solution_map;
  for(std::map<std::string, geometry_msgs::Pose>::const_iterator it = poses.begin();
      it != poses.end(); 
      it++) {
    current_acm_ = other_arms;
    const std::vector<std::string>& end_effector_links = end_effector_collision_links_.at(it->first);
    //now we want to turn off collisions between this group's gripper and other arms
    for(unsigned int i = 0; i < end_effector_links.size(); i++) {
      current_acm_.setEntry(end_effector_links[i], arm_links, true);
    }
    if(solver_map_.find(it->first) == solver_map_.end()) {
      ROS_WARN_STREAM("No solver for pose " << it->first);
      return false;
    }
    kinematics::KinematicsBasePtr& solver = solver_map_.at(it->first);
    seed_state_vector.resize(solver->getJointNames().size());
    for(unsigned int i = 0; i < solver->getJointNames().size(); i++) {
      seed_state_vector[i] = seed_state_map[solver->getJointNames()[i]];
    } 
    current_subgroup_ = it->first;
    std::vector<double> sol;
    moveit_msgs::MoveItErrorCodes error_code;
    bool ik_valid = solver->searchPositionIK(it->second,
                                             seed_state_vector,
                                             1.0,
                                             sol,
                                             boost::bind(&KinematicsSolverConstraintAware::initialPoseCheck, this, _1, _2, _3),
                                             boost::bind(&KinematicsSolverConstraintAware::collisionCheck, this, _1, _2, _3),
                                             error_code);

    if(!ik_valid) {
      ROS_INFO_STREAM("No valid ik solutions given end effector positions for group " << it->first);
      return false;
    }
    // for(unsigned int i = 0; i < solver->getJointNames().size(); i++) {
    //   solution_map
    //     }
  }
  return true;
}
                                                                 
}


