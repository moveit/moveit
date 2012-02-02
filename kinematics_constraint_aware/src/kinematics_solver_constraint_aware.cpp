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

KinematicsSolverConstraintAware::KinematicsSolverConstraintAware(boost::shared_ptr<kinematics::KinematicsBase>& solver,
                                                                 const planning_models::KinematicModelConstPtr& kmodel,
                                                                 const std::string& group_name) :
  kmodel_(kmodel),
  active_(false),
  kinematics_solver_(solver)
{
  if(group_name.empty()) {
    ROS_INFO_STREAM("Must have non-empty group");
    return;
  }

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
  
  if(kinematics_solver_->initialize(group_name,
                                    config.chains_[0].first,
                                    config.chains_[0].second,
                                    .025)) {
  } else {
    return;
  }

  const planning_models::KinematicModel::LinkModel* end_effector_link = kmodel_->getLinkModel(kinematics_solver_->getTipFrame());
  end_effector_collision_links_ = kmodel_->getChildLinkModelNames(end_effector_link);

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
  std::map<std::string, double> seed_state_map;
  seed_state->getStateValues(seed_state_map);
  std::vector<double> seed_state_vector(kinematics_solver_->getJointNames().size());
  for(unsigned int i = 0; i < kinematics_solver_->getJointNames().size(); i++) {
    seed_state_vector[i] = seed_state_map[kinematics_solver_->getJointNames()[i]];
  } 

  std::vector<double> sol;
  bool ik_valid = kinematics_solver_->getPositionIK(pose,
                                                    seed_state_vector,
                                                    sol,
                                                    error_code);
  if(ik_valid) {
    solution.name = kinematics_solver_->getJointNames();
    solution.position = sol;
  } else {
    solution.name.clear();
    solution.position.clear();
  }
  return ik_valid;
}

bool KinematicsSolverConstraintAware::findConstraintAwareSolution(const geometry_msgs::Pose& pose,
                                                                  const moveit_msgs::Constraints& constraints,
                                                                  const planning_models::KinematicState* seed_state,
                                                                  const planning_scene::PlanningSceneConstPtr& scene,
                                                                  sensor_msgs::JointState& solution,
                                                                  moveit_msgs::MoveItErrorCodes& error_code, 
                                                                  const bool& do_initial_pose_check)
{
  do_initial_pose_check_ = do_initial_pose_check;
  constraints_ = constraints;
  planning_scene_ = scene;

  last_initial_pose_check_collision_result_ = collision_detection::CollisionResult();
  
  //TODO - need better way to do this
  std::map<std::string, double> seed_state_map;
  seed_state->getStateValues(seed_state_map);
  state_->setStateValues(seed_state_map);
  
  std::vector<double> seed_state_vector(kinematics_solver_->getJointNames().size());
  for(unsigned int i = 0; i < kinematics_solver_->getJointNames().size(); i++) {
    seed_state_vector[i] = seed_state_map[kinematics_solver_->getJointNames()[i]];
  } 
  
  std::vector<double> sol;
  bool ik_valid = kinematics_solver_->searchPositionIK(pose,
                                                       seed_state_vector,
                                                       1.0,
                                                       sol,
                                                       boost::bind(&KinematicsSolverConstraintAware::initialPoseCheck, this, _1, _2, _3),
                                                       boost::bind(&KinematicsSolverConstraintAware::collisionCheck, this, _1, _2, _3),
                                                       error_code);
  if(ik_valid) {
    solution.name = kinematics_solver_->getJointNames();
    solution.position = sol;
  } else {
    solution.name.clear();
    solution.position.clear();
  }
  return ik_valid;
}

bool KinematicsSolverConstraintAware::findConsistentConstraintAwareSolution(const geometry_msgs::Pose& pose,
                                                                            const moveit_msgs::Constraints& constraints,
                                                                            const planning_models::KinematicState* seed_state,
                                                                            const planning_scene::PlanningSceneConstPtr& scene,
                                                                            sensor_msgs::JointState& solution,
                                                                            moveit_msgs::MoveItErrorCodes& error_code, 
                                                                            const unsigned int& redundancy,
                                                                            const double& max_consistency,
                                                                            const bool& do_initial_pose_check)
{
  do_initial_pose_check_ = do_initial_pose_check;
  planning_scene_ = scene;
  constraints_ = constraints;

  last_initial_pose_check_collision_result_ = collision_detection::CollisionResult();
  
  std::map<std::string, double> seed_state_map;
  seed_state->getStateValues(seed_state_map);
  state_->setStateValues(seed_state_map);
  
  std::vector<double> seed_state_vector(kinematics_solver_->getJointNames().size());
  for(unsigned int i = 0; i < kinematics_solver_->getJointNames().size(); i++) {
    seed_state_vector[i] = seed_state_map[kinematics_solver_->getJointNames()[i]];
  } 
  
  std::vector<double> sol;
  bool ik_valid = kinematics_solver_->searchPositionIK(pose,
                                                       seed_state_vector,
                                                       1.0,
                                                       redundancy,
                                                       max_consistency,
                                                       sol,
                                                       boost::bind(&KinematicsSolverConstraintAware::initialPoseCheck, this, _1, _2, _3),
                                                       boost::bind(&KinematicsSolverConstraintAware::collisionCheck, this, _1, _2, _3),
                                                       error_code);
  if(ik_valid) {
    solution.name = kinematics_solver_->getJointNames();
    solution.position = sol;
  } else {
    solution.name.clear();
    solution.position.clear();
  }
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
  std::map<std::string, double> joint_values;
  for(unsigned int i=0; i < kinematics_solver_->getJointNames().size(); i++) {
    joint_values[kinematics_solver_->getJointNames()[i]] = ik_solution[i];
  }
  
  state_->setStateValues(joint_values);
  error_code.val = error_code.SUCCESS;
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  //req.contacts = true;
  //req.max_contacts = 1;
  planning_scene_->checkCollision(req, res, *state_);
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
  std::string kinematic_frame_id = kinematics_solver_->getBaseFrame();
  std::string planning_frame_id = planning_scene_->getPlanningFrame();
  //TODO - should be a check that we can actually transform, better transform library
  Eigen::Affine3d cur;
  planning_models::poseFromMsg(ik_pose, cur);
  Eigen::Affine3d nt;
  planning_scene_->getTransforms()->transformPose(*state_, kinematic_frame_id, cur, nt);
  state_->updateStateWithLinkAt(kinematics_solver_->getTipFrame(), nt);

  //disabling all collision for arm links
  collision_detection::AllowedCollisionMatrix acm = planning_scene_->getAllowedCollisionMatrix();
  for(unsigned int i = 0; i < kinematics_solver_->getLinkNames().size(); i++) {
    acm.setDefaultEntry(kinematics_solver_->getLinkNames()[i], true);
    acm.setEntry(kinematics_solver_->getLinkNames()[i], true);
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
                                                               const bool& do_initial_pose_check)
{
  trajectory_msgs::JointTrajectory ret_traj;
  ret_traj.joint_names = kinematics_solver_->getJointNames();
  ret_traj.points.resize(num_points);

  Eigen::Affine3d first_pose;
  planning_models::poseFromMsg(start_pose, first_pose);

  for(unsigned int i = 1; i <= num_points; i++) {
    int val;
    if(reverse) {
      val = num_points-i;
    } else {
      val = i;
    }

    //assumes that the axis is aligned
    Eigen::Affine3d trans(Eigen::Translation3d(direction*val*fabs(distance/(num_points*1.0)))*Eigen::Quaterniond(1.0,0.0,0.0,0.0));
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
                                             seed_state,
                                             scene,
                                             solution,
                                             temp_error_code,
                                             redundancy,
                                             max_consistency,
                                             do_initial_pose_check)) {
      ret_traj.points[i-1].positions = solution.position;
      ret_traj.points[i-1].time_from_start = ros::Duration((i*1.0)*total_dur.toSec()/(num_points*1.0));
    } else {
      return false;
    }
  }
  traj = ret_traj;
  return true;
}

}


