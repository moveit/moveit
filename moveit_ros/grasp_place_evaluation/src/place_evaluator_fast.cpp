/*********************************************************************
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

// Author(s): E. Gil Jones

#include <grasp_place_evaluation/place_evaluator_fast.h>
#include <kinematics_constraint_aware/kinematics_solver_constraint_aware.h>

namespace grasp_place_evaluation {

PlaceEvaluatorFast::PlaceEvaluatorFast(const planning_models::KinematicModelConstPtr& kmodel,
                                       const std::map<std::string, kinematics::KinematicsBasePtr>& solver_map)
  : PlaceEvaluator(), 
    InterpolationEvaluator(kmodel, solver_map)
{  
}

void PlaceEvaluatorFast::testPlaceLocations(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                            const planning_models::KinematicState* seed_state,
                                            const moveit_manipulation_msgs::PlaceGoal &place_goal, 
                                            const geometry_msgs::Vector3& retreat_direction,
                                            const std::vector<geometry_msgs::PoseStamped>& place_locations,
                                            PlaceExecutionInfoVector &execution_info,
                                            bool return_on_first_hit)
{
  ros::WallTime start = ros::WallTime::now();

  std::map<unsigned int, unsigned int> outcome_count;

  planning_models::KinematicState state(*seed_state);

  std::map<std::string, double> planning_scene_state_values;
  state.getStateValues(planning_scene_state_values);

  std::string end_effector_group = planning_scene->getKinematicModel()->getJointModelGroup(place_goal.arm_name)->getAttachedEndEffectorGroupName();
  // links are ordered by the order seen by depth-first. This means that for a chain the last link is the tip of the chain
  std::string tip_link = planning_scene->getKinematicModel()->getJointModelGroup(place_goal.arm_name)->getLinkModelNames().back();
  
  std::vector<std::string> end_effector_links = planning_scene->getKinematicModel()->getJointModelGroup(end_effector_group)->getLinkModelNames();
  std::vector<std::string> arm_links = planning_scene->getKinematicModel()->getJointModelGroup(place_goal.arm_name)->getLinkModelNames();

  collision_detection::AllowedCollisionMatrix original_acm = planning_scene->getAllowedCollisionMatrix();
  collision_detection::AllowedCollisionMatrix group_disable_acm = original_acm;//planning_scene->disableCollisionsForNonUpdatedLinks(place_goal.arm_name);

  collision_detection::AllowedCollisionMatrix object_disable_acm = group_disable_acm;
  object_disable_acm.setEntry(place_goal.collision_object_name, end_effector_links, true); 
  collision_detection::AllowedCollisionMatrix object_support_disable_acm = object_disable_acm;
  if(place_goal.allow_gripper_support_collision)
  {
    if(place_goal.collision_support_surface_name == "\"all\"")
    {
      for(unsigned int i = 0; i < end_effector_links.size(); i++){
	object_support_disable_acm.setDefaultEntry(end_effector_links[i], true);
      }
    } else {
      object_support_disable_acm.setEntry(place_goal.collision_support_surface_name, end_effector_links, true); 
    }
  }
  object_support_disable_acm.setEntry(place_goal.collision_support_surface_name, place_goal.collision_object_name, true); 

  collision_detection::AllowedCollisionMatrix object_all_arm_disable_acm = object_disable_acm;
  collision_detection::AllowedCollisionMatrix object_support_all_arm_disable_acm = object_support_disable_acm;
  collision_detection::AllowedCollisionMatrix group_all_arm_disable_acm = group_disable_acm;

  //turning off collisions for the arm associated with this end effector
  for(unsigned int i = 0; i < arm_links.size(); i++) {
    object_all_arm_disable_acm.setDefaultEntry(arm_links[i], true);
    object_support_all_arm_disable_acm.setDefaultEntry(arm_links[i], true);
    group_all_arm_disable_acm.setDefaultEntry(arm_links[i], true);
  }

  execution_info.place_locations_ = place_locations;
  execution_info.place_goal_ = place_goal;
  execution_info.resize(place_locations.size());

  Eigen::Vector3d approach_dir;
  approach_dir.x() = place_goal.approach.direction.vector.x;
  approach_dir.y() = place_goal.approach.direction.vector.y;
  approach_dir.z() = place_goal.approach.direction.vector.z;
  approach_dir.normalize();
  Eigen::Translation3d distance_approach_dir(approach_dir*fabs(place_goal.approach.desired_distance));
  Eigen::Affine3d approach_trans(distance_approach_dir*Eigen::Quaterniond::Identity());

  Eigen::Vector3d retreat_dir(retreat_direction.x,retreat_direction.y, retreat_direction.z);
  retreat_dir.normalize();
  Eigen::Translation3d distance_retreat_dir(retreat_dir*fabs(place_goal.desired_retreat_distance));
  Eigen::Affine3d retreat_trans(distance_retreat_dir*Eigen::Quaterniond::Identity());

  std::map<std::string, double> planning_scene_state_values_post_place = planning_scene_state_values;
  std::map<std::string, double> post_place_joint_vals;    
  std::map<std::string, double> place_joint_vals;
  for(unsigned int j = 0; j < place_goal.grasp.pre_grasp_posture.name.size(); j++) {
    planning_scene_state_values_post_place[place_goal.grasp.pre_grasp_posture.name[j]] = place_goal.grasp.pre_grasp_posture.position[j];
    post_place_joint_vals[place_goal.grasp.pre_grasp_posture.name[j]] = place_goal.grasp.pre_grasp_posture.position[j];
    place_joint_vals[place_goal.grasp.pre_grasp_posture.name[j]] = planning_scene_state_values[place_goal.grasp.pre_grasp_posture.name[j]];
  }

  std_msgs::Header world_header;
  world_header.frame_id = planning_scene->getPlanningFrame();
  std::vector<std::string> joint_names = planning_scene->getKinematicModel()->getJointModelGroup(place_goal.arm_name)->getJointModelNames();

  //now this is place specific
  for(unsigned int i = 0; i < place_locations.size(); i++) {

    if(execution_info[i].result_.result_code != 0) {
      ROS_DEBUG_STREAM("Assuming place " << i << " already evaluated");
      continue;
    }

    // ------------- CHECKING PLACE POSE ------------------

    //using the grasp posture
    state.setStateValues(post_place_joint_vals);
    
    Eigen::Affine3d place_pose;
    planning_models::poseFromMsg(place_locations[i].pose, place_pose);
    planning_scene->getTransforms()->transformPose(state,
                                                   place_locations[i].header.frame_id,
                                                   place_pose,
                                                   execution_info[i].place_pose_);
    Eigen::Affine3d grasp_pose;
    planning_models::poseFromMsg(place_goal.grasp.grasp_pose, grasp_pose);
    execution_info[i].place_pose_ = execution_info[i].place_pose_*grasp_pose;
    
    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;
    //cm->setAlteredAllowedCollisionMatrix(object_support_all_arm_disable_acm);
    planning_scene->checkCollisionUnpadded(req, res, state, object_support_all_arm_disable_acm);
    if(res.collision) {
      ROS_DEBUG_STREAM("Place in collision");
      execution_info[i].result_.result_code = moveit_manipulation_msgs::PlaceLocationResult::PLACE_IN_COLLISION;
      outcome_count[moveit_manipulation_msgs::PlaceLocationResult::PLACE_IN_COLLISION]++;
      continue;
    } 

    // ------------- CHECKING APPROACH POSE ------------------

    state.setStateValues(planning_scene_state_values);
    execution_info[i].preplace_pose_ = approach_trans*execution_info[i].place_pose_;
    state.updateStateWithLinkAt(tip_link, execution_info[i].preplace_pose_);

    //now we do the place approach pose, not allowing anything different 
    //acm->setAlteredAllowedCollisionMatrix(group_all_arm_disable_acm);
    res = collision_detection::CollisionResult();
    planning_scene->checkCollision(req, res, state, group_all_arm_disable_acm);
    if(res.collision) {
      ROS_DEBUG_STREAM("Preplace in collision");
      execution_info[i].result_.result_code = moveit_manipulation_msgs::PlaceLocationResult::PREPLACE_IN_COLLISION;
      outcome_count[moveit_manipulation_msgs::PlaceLocationResult::PREPLACE_IN_COLLISION]++;
      continue;
    }

    // --------------- CHECKING RELEASE POSE -------------------
    
    moveit_msgs::AttachedCollisionObject att_obj;
    att_obj.link_name = planning_scene->getKinematicModel()->getJointModelGroup(end_effector_group)->getEndEffectorParentGroup().second;
    att_obj.object.operation = moveit_msgs::CollisionObject::REMOVE;
    att_obj.object.id = place_goal.collision_object_name;

    execution_info[i].detached_object_diff_scene_ = planning_scene->diff();
    execution_info[i].detached_object_diff_scene_->getCurrentState().updateStateWithLinkAt(tip_link,execution_info[i].place_pose_);
    execution_info[i].detached_object_diff_scene_->processAttachedCollisionObjectMsg(att_obj);
    
    execution_info[i].detached_object_diff_scene_->getCurrentState().setStateValues(planning_scene_state_values_post_place);
    execution_info[i].retreat_pose_ = execution_info[i].place_pose_*retreat_trans;
    execution_info[i].detached_object_diff_scene_->getCurrentState().updateStateWithLinkAt(tip_link, execution_info[i].retreat_pose_);

    res = collision_detection::CollisionResult();
    planning_scene->checkCollision(req, res, execution_info[i].detached_object_diff_scene_->getCurrentState(), group_all_arm_disable_acm);
    if(res.collision) {
      ROS_DEBUG_STREAM("Retreat in collision");
      execution_info[i].result_.result_code = moveit_manipulation_msgs::PlaceLocationResult::RETREAT_IN_COLLISION;
      outcome_count[moveit_manipulation_msgs::PlaceLocationResult::RETREAT_IN_COLLISION]++;
      continue; 
    }

    state.setStateValues(planning_scene_state_values_post_place);
    
    Eigen::Affine3d base_link_world_pose =
      state.getLinkState(planning_scene->getKinematicModel()->getJointModelGroup(place_goal.arm_name)->getLinkModelNames().front())->getGlobalLinkTransform();
    Eigen::Affine3d base_link_place_pose_e = base_link_world_pose.inverse()*execution_info[i].place_pose_;
    geometry_msgs::Pose base_link_place_pose;
    planning_models::msgFromPose(base_link_place_pose_e, base_link_place_pose);
    
    moveit_msgs::Constraints emp;
    sensor_msgs::JointState solution;
    moveit_msgs::MoveItErrorCodes error_code;
    //checking unpadded
    if(!constraint_aware_solver_map_[place_goal.arm_name]->findConstraintAwareSolution(base_link_place_pose,
                                                                                       emp,
                                                                                       &state,
                                                                                       planning_scene,
                                                                                       object_support_disable_acm,
                                                                                       solution,
                                                                                       error_code,
                                                                                       false,
                                                                                       true)) {
      ROS_DEBUG_STREAM("Place out of reach");
      execution_info[i].result_.result_code = moveit_manipulation_msgs::PlaceLocationResult::PLACE_OUT_OF_REACH;
      outcome_count[moveit_manipulation_msgs::PlaceLocationResult::PLACE_OUT_OF_REACH]++;
      continue;
    }

    // ------------- CHECKING INTERPOLATED IK FROM PREPLACE TO PLACE ------------------        

    std::map<std::string, double> ik_map_place = place_joint_vals;
    std::map<std::string, double> ik_map_post_place = post_place_joint_vals;
    for(unsigned int j = 0; j < joint_names.size(); j++) {
      ik_map_place[joint_names[j]] = solution.position[j];
      ik_map_post_place[joint_names[j]] = solution.position[j];
    } 
    state.setStateValues(ik_map_place);      

    execution_info[i].approach_trajectory_.joint_names = joint_names;
    //now we need to do interpolated ik
    if(!getInterpolatedIK(place_goal.arm_name,
                          planning_scene,
                          object_support_disable_acm,
                          place_goal.path_constraints,
                          base_link_place_pose,
                          approach_dir,
                          place_goal.approach.desired_distance,
                          true,
                          true,
                          true,
                          &state,
                          execution_info[i].approach_trajectory_)) {
      ROS_DEBUG_STREAM("No interpolated IK for pre-place to place");
      execution_info[i].result_.result_code = moveit_manipulation_msgs::PlaceLocationResult::PLACE_UNFEASIBLE;
      outcome_count[moveit_manipulation_msgs::PlaceLocationResult::PLACE_UNFEASIBLE]++;
      continue;
    }

    // ------------- CHECKING INTERPOLATED IK FROM PLACE TO RETREAT ------------------        

    execution_info[i].detached_object_diff_scene_->getCurrentState().setStateValues(ik_map_post_place);
    execution_info[i].retreat_trajectory_.joint_names = joint_names;
    if(!getInterpolatedIK(place_goal.arm_name,
                          execution_info[i].detached_object_diff_scene_,
                          object_support_disable_acm,
                          place_goal.path_constraints,
                          base_link_place_pose,
                          retreat_dir,
                          place_goal.desired_retreat_distance,
                          false,
                          false,
                          true,
                          &execution_info[i].detached_object_diff_scene_->getCurrentState(),
                          execution_info[i].retreat_trajectory_)) {
      ROS_DEBUG_STREAM("No interpolated IK for place to retreat");
      execution_info[i].result_.result_code = moveit_manipulation_msgs::PlaceLocationResult::RETREAT_UNFEASIBLE;
      outcome_count[moveit_manipulation_msgs::PlaceLocationResult::RETREAT_UNFEASIBLE]++;
      continue;
    }

    // ------------- CHECKING PREPLACE OK FOR PLANNING ------------------            

    if(execution_info[i].approach_trajectory_.points.empty()) {
      ROS_WARN_STREAM("No result code and no points in approach trajectory");
      continue;
    }
    
    std::map<std::string, double> pre_place_ik = place_joint_vals;
    for(unsigned int j = 0; j < joint_names.size(); j++) {
      pre_place_ik[joint_names[j]] = execution_info[i].approach_trajectory_.points[0].positions[j];
    } 
    state.setStateValues(pre_place_ik);
    res = collision_detection::CollisionResult();
    planning_scene->checkCollision(req, res, state, group_disable_acm);
    if(res.collision) {
      ROS_DEBUG_STREAM("Final pre-place check failed");
      execution_info[i].result_.result_code = moveit_manipulation_msgs::PlaceLocationResult::PREPLACE_OUT_OF_REACH;
      outcome_count[moveit_manipulation_msgs::PlaceLocationResult::PREPLACE_OUT_OF_REACH]++;
      continue;
    }
    
    // ------------- CHECKING RETREAT OK FOR PLANNING ------------------            

    if(execution_info[i].retreat_trajectory_.points.empty()) {
      ROS_WARN_STREAM("No result code and no points in retreat trajectory");
      continue;
    }    
    std::map<std::string, double> retreat_ik = post_place_joint_vals;
    for(unsigned int j = 0; j < joint_names.size(); j++) {
      retreat_ik[joint_names[j]] = execution_info[i].retreat_trajectory_.points.back().positions[j];
    } 
    execution_info[i].detached_object_diff_scene_->getCurrentState().setStateValues(retreat_ik);
    execution_info[i].detached_object_diff_scene_->checkCollision(req, res, execution_info[i].detached_object_diff_scene_->getCurrentState());
    res = collision_detection::CollisionResult();
    if(res.collision) {
      ROS_DEBUG_STREAM("Final retreat check failed");
      execution_info[i].result_.result_code = moveit_manipulation_msgs::PlaceLocationResult::RETREAT_OUT_OF_REACH;
      outcome_count[moveit_manipulation_msgs::PlaceLocationResult::RETREAT_OUT_OF_REACH]++;
      continue;
    } else {
      ROS_INFO_STREAM("Everything successful");
      execution_info[i].result_.result_code = moveit_manipulation_msgs::PlaceLocationResult::SUCCESS;
      execution_info.resize(i+1);
      outcome_count[moveit_manipulation_msgs::PlaceLocationResult::SUCCESS]++;
      if(return_on_first_hit) {
        break;
      }
    }
  }
  for(std::map<unsigned int, unsigned int>::iterator it = outcome_count.begin();
      it != outcome_count.end();
      it++) {
    ROS_INFO_STREAM("Outcome " << it->first << " count " << it->second);
  }
}
    
} //namespace grasp_place_evaluation
