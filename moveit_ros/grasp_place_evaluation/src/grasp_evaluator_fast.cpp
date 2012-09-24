/*********************************************************************
*
*  Copyright (c) 2009, Willow Garage, Inc.
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

#include <grasp_place_evaluation/grasp_evaluator_fast.h>

using moveit_manipulation_msgs::GraspResult;
using moveit_msgs::MoveItErrorCodes;

namespace grasp_place_evaluation {

GraspEvaluatorFast::GraspEvaluatorFast(const planning_models::KinematicModelConstPtr& kmodel,
                                       const std::map<std::string, kinematics::KinematicsBasePtr>& solver_map)
  : GraspEvaluator(), 
    InterpolationEvaluator(kmodel, solver_map)
{  
}

void GraspEvaluatorFast::testGrasps(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                    const planning_models::KinematicState* seed_state,
                                    const moveit_manipulation_msgs::PickupGoal& pickup_goal,
                                    const geometry_msgs::Vector3& approach_direction,
                                    const std::vector<moveit_manipulation_msgs::Grasp>& grasps,
                                    GraspExecutionInfoVector &execution_info,
                                    bool return_on_first_hit) 
  
{
  ros::WallTime start = ros::WallTime::now();

  std::map<unsigned int, unsigned int> outcome_count;
    
  planning_models::KinematicState state(*seed_state);

  std::map<std::string, double> planning_scene_state_values;
  state.getStateValues(planning_scene_state_values);

  std::string end_effector_group = planning_scene->getKinematicModel()->getJointModelGroup(pickup_goal.arm_name)->getAttachedEndEffectorGroupName();
  // links are ordered by the order seen by depth-first. This means that for a chain the last link is the tip of the chain
  std::string tip_link = planning_scene->getKinematicModel()->getJointModelGroup(pickup_goal.arm_name)->getLinkModelNames().back();
  
  std::vector<std::string> end_effector_links = planning_scene->getKinematicModel()->getJointModelGroup(end_effector_group)->getLinkModelNames();
  std::vector<std::string> arm_links = planning_scene->getKinematicModel()->getJointModelGroup(pickup_goal.arm_name)->getLinkModelNames();
  
  collision_detection::AllowedCollisionMatrix original_acm = planning_scene->getAllowedCollisionMatrix();
  collision_detection::AllowedCollisionMatrix group_disable_acm = original_acm;//planning_scene->disableCollisionsForNonUpdatedLinks(pickup_goal.arm_name);

  collision_detection::AllowedCollisionMatrix object_disable_acm = group_disable_acm;
  object_disable_acm.setEntry(pickup_goal.collision_object_name, end_effector_links, true); 
  collision_detection::AllowedCollisionMatrix object_support_disable_acm = object_disable_acm;
  if(!pickup_goal.collision_support_surface_name.empty()) {
    //will come into play when object is attached
    object_support_disable_acm.setEntry(pickup_goal.collision_support_surface_name, pickup_goal.collision_object_name, true); 
    if(pickup_goal.allow_gripper_support_collision)
    {
      if(pickup_goal.collision_support_surface_name == "\"all\"")
      {
        for(unsigned int i = 0; i < end_effector_links.size(); i++){
          object_support_disable_acm.setDefaultEntry(end_effector_links[i], true);
        }
      } else {
        object_support_disable_acm.setEntry(pickup_goal.collision_support_surface_name, end_effector_links, true); 
      }
    }
  }
  collision_detection::AllowedCollisionMatrix object_all_arm_disable_acm = object_disable_acm;
  collision_detection::AllowedCollisionMatrix object_support_all_arm_disable_acm = object_support_disable_acm;
  collision_detection::AllowedCollisionMatrix group_all_arm_disable_acm = group_disable_acm;

  //turning off collisions for the arm associated with this end effector
  for(unsigned int i = 0; i < arm_links.size(); i++) {
    object_all_arm_disable_acm.setDefaultEntry(arm_links[i], true);
    object_support_all_arm_disable_acm.setDefaultEntry(arm_links[i], true);
    group_all_arm_disable_acm.setDefaultEntry(arm_links[i], true);
  }
  
  //first we apply link padding for grasp check
  //cm->applyLinkPaddingToCollisionSpace(linkPaddingForGrasp(pickup_goal));
    
  //setup that's not grasp specific  
  std_msgs::Header target_header;
  target_header.frame_id = pickup_goal.target.reference_frame_id;

  bool in_object_frame = false;
  Eigen::Affine3d obj_pose(Eigen::Affine3d::Identity());

  if(pickup_goal.target.reference_frame_id == pickup_goal.collision_object_name) {
    moveit_msgs::CollisionObject co;
    if(!planning_scene->getCollisionObjectMsg(pickup_goal.collision_object_name, 
                                              co)) {
      ROS_WARN_STREAM("Don't appear to have object " << pickup_goal.collision_object_name 
                      << " in planning scene, so can't tell where it is to pick it up");
      return;
    }
    in_object_frame = true;
    //ROS_DEBUG_STREAM("In object frame is true " << co.poses[0]);
    //TODO - Use potential pose?
    //Eigen::Affine3d potential_pose;
    //planning_models::poseFromMsg(pickup_goal.target.potential_models[0].pose.pose, potential_pose);
    // planning_scene->getTransforms()->transformPose(state,
    //                                                pickup_goal.target.potential_models[0].pose.header.frame_id,
    //                                                potential_pose,
    //                                                obj_pose);
    if(co.mesh_poses.size() > 0) {
      planning_models::poseFromMsg(co.mesh_poses[0], obj_pose);
    } else if(co.primitive_poses.size() > 0) {
      planning_models::poseFromMsg(co.primitive_poses[0], obj_pose);
    } else {
      ROS_ERROR_STREAM("No poses for object");
      return;
    }
  } 
  
  //assumes that whatever is in there is valid
  unsigned int current_size = execution_info.size();
  execution_info.pickup_goal_ = pickup_goal;
  execution_info.grasps_ = grasps;
  execution_info.resize(grasps.size());
  for(unsigned int i = current_size; i < execution_info.size(); i++) {
    execution_info[i].result_.result_code = 0;
  }

  Eigen::Vector3d pregrasp_dir(approach_direction.x,approach_direction.y,approach_direction.z);
  //tf::vector3MsgToTF(doNegate(handDescription().approachDirection(pickup_goal.arm_name)), pregrasp_dir);
  pregrasp_dir.normalize();

  Eigen::Vector3d lift_dir;
  lift_dir.x() = pickup_goal.lift.direction.vector.x;
  lift_dir.y() = pickup_goal.lift.direction.vector.y;
  lift_dir.z() = pickup_goal.lift.direction.vector.z;
  lift_dir.normalize();
  Eigen::Translation3d distance_lift_dir(lift_dir*fabs(pickup_goal.lift.desired_distance));
  Eigen::Affine3d lift_trans(distance_lift_dir*Eigen::Quaterniond::Identity());

  std_msgs::Header world_header;
  world_header.frame_id = planning_scene->getPlanningFrame();
  std::vector<std::string> joint_names = planning_scene->getKinematicModel()->getJointModelGroup(pickup_goal.arm_name)->getJointModelNames();

  std::vector<Eigen::Affine3d> grasp_poses(grasps.size());

  //now this is grasp specific
  for(unsigned int i = 0; i < grasps.size(); i++) {
    ros::WallTime now = ros::WallTime::now();

    if(execution_info[i].result_.result_code != 0) {
      ROS_DEBUG_STREAM("Assuming grasp " << i << " already evaluated");
      continue;
    }

    // ------------- CHECKING GRASP POSE ------------------

    std::map<std::string, double> pre_grasp_joint_vals;    
    for(unsigned int j = 0; j < grasps[i].pre_grasp_posture.name.size(); j++) {
      pre_grasp_joint_vals[grasps[i].pre_grasp_posture.name[j]] = grasps[i].pre_grasp_posture.position[j];
    }
    std::map<std::string, double> grasp_joint_vals;    
    for(unsigned int j = 0; j < grasps[i].grasp_posture.name.size(); j++) {
      grasp_joint_vals[grasps[i].grasp_posture.name[j]] = grasps[i].grasp_posture.position[j];
    }
    //check whether the grasp pose is ok (only checking hand, not arms)
    //using pre-grasp posture, cause grasp_posture only matters for closing the gripper
    state.setStateValues(pre_grasp_joint_vals);

    if(!in_object_frame) {
      Eigen::Affine3d grasp_pose;
      planning_models::poseFromMsg(grasps[i].grasp_pose, grasp_pose);
      planning_scene->getTransforms()->transformPose(state,
                                                     target_header.frame_id,
                                                     grasp_pose,
                                                     grasp_poses[i]);
    } else {
      Eigen::Affine3d gp;
      planning_models::poseFromMsg(grasps[i].grasp_pose, gp);
      grasp_poses[i] = obj_pose*gp;
      geometry_msgs::Pose pp;
      planning_models::msgFromPose(grasp_poses[i], pp);
      ROS_DEBUG_STREAM("Grasp pose is " << pp);
    }
    moveit_msgs::AttachedCollisionObject att_obj;
    att_obj.link_name = planning_scene->getKinematicModel()->getJointModelGroup(end_effector_group)->getEndEffectorParentGroup().second;
    att_obj.object.operation = moveit_msgs::CollisionObject::ADD;
    att_obj.object.id = pickup_goal.collision_object_name;
    att_obj.touch_links = end_effector_links;

    execution_info[i].attached_object_diff_scene_ = planning_scene->diff();
    //need to make sure that the fingers are in the grasped pose in order to do the attach
    execution_info[i].attached_object_diff_scene_->getCurrentState().setStateValues(grasp_joint_vals);
    execution_info[i].attached_object_diff_scene_->getCurrentState().updateStateWithLinkAt(tip_link,grasp_poses[i]);
    execution_info[i].attached_object_diff_scene_->processAttachedCollisionObjectMsg(att_obj);
    execution_info[i].attached_object_diff_scene_->getCurrentState().updateStateWithLinkAt(tip_link,grasp_poses[i]);

    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;
    //cm->setAlteredAllowedCollisionMatrix(object_support_all_arm_disable_acm);
    //req.verbose = true;
    execution_info[i].grasp_pose_ = grasp_poses[i];
    //ros::WallTime before_coll = ros::WallTime::now();    
    execution_info[i].attached_object_diff_scene_->checkCollision(req, res, 
                                                                  execution_info[i].attached_object_diff_scene_->getCurrentState(), 
                                                                  object_support_all_arm_disable_acm);
    //ROS_INFO_STREAM("First coll check took " << (ros::WallTime::now()-before_coll));
    // ros::WallTime second_coll = ros::WallTime::now();    
    // execution_info[i].attached_object_diff_scene_->checkCollision(req, res, 
    //                                                               execution_info[i].attached_object_diff_scene_->getCurrentState(), 
    //                                                               object_support_all_arm_disable_acm);
    // ROS_INFO_STREAM("Second coll check took " << (ros::WallTime::now()-second_coll));

    //req.verbose = false;
    if(res.collision) {
      execution_info[i].result_.result_code = GraspResult::GRASP_IN_COLLISION;
      outcome_count[GraspResult::GRASP_IN_COLLISION]++;
      //ROS_INFO_STREAM("Full eval for grasp in collision took " << (ros::WallTime::now()-now));
      continue;
    } 

    // ------------- CHECKING PREGRASP POSE ------------------    

    Eigen::Translation3d distance_pregrasp_dir(pregrasp_dir*fabs(grasps[0].desired_approach_distance));    
    Eigen::Affine3d pre_grasp_trans(distance_pregrasp_dir*Eigen::Quaterniond::Identity());
    Eigen::Affine3d pre_grasp_pose = grasp_poses[i]*pre_grasp_trans;
    state.updateStateWithLinkAt(tip_link,pre_grasp_pose);

    res = collision_detection::CollisionResult();
    //cm->setAlteredAllowedCollisionMatrix(group_all_arm_disable_acm);
    planning_scene->checkCollision(req,res,state,
                                   group_all_arm_disable_acm);
    execution_info[i].pregrasp_pose_ = pre_grasp_pose;
    if(res.collision) {
      ROS_DEBUG_STREAM("Pre-grasp in collision");
      execution_info[i].result_.result_code = GraspResult::PREGRASP_IN_COLLISION;
      outcome_count[GraspResult::PREGRASP_IN_COLLISION]++;
      continue;
    }

    // ------------- CHECKING LIFT POSE ------------------    

    execution_info[i].attached_object_diff_scene_->getCurrentState().setStateValues(grasp_joint_vals);
    
    Eigen::Affine3d lift_pose = lift_trans*grasp_poses[i];
    execution_info[i].attached_object_diff_scene_->getCurrentState().updateStateWithLinkAt(tip_link,lift_pose);
    
    res = collision_detection::CollisionResult();
    execution_info[i].attached_object_diff_scene_->checkCollision(req,res, execution_info[i].attached_object_diff_scene_->getCurrentState(),
                                                                  object_support_all_arm_disable_acm);
    execution_info[i].lift_pose_ = lift_pose;
    if(res.collision) {
      ROS_DEBUG_STREAM("Lift in collision");
      execution_info[i].result_.result_code = GraspResult::LIFT_IN_COLLISION;
      outcome_count[GraspResult::LIFT_IN_COLLISION]++;
      continue;
    }

    // ------------- CHECKING IK FOR GRASP ------------------        

    //getting back to original state for seed
    state.setStateValues(planning_scene_state_values);

    //adjusting planning scene state for pre-grasp
    std::map<std::string, double> pre_grasp_values;
    for(unsigned int j = 0; j < grasps[i].pre_grasp_posture.name.size(); j++) {
      pre_grasp_values[grasps[i].pre_grasp_posture.name[j]] = grasps[i].pre_grasp_posture.position[j];
    }
    state.setStateValues(pre_grasp_values);
    
    //now call ik for grasp
    
    // links in the groups are ordered by how depth-first visits them. For a chain this means the first link is the base & the last link is the tip
    Eigen::Affine3d base_link_world_pose = 
      state.getLinkState(planning_scene->getKinematicModel()->getJointModelGroup(pickup_goal.arm_name)->getLinkModelNames().front())->getGlobalLinkTransform();

    Eigen::Affine3d base_link_grasp_pose_e = base_link_world_pose.inverse()*grasp_poses[i];
    geometry_msgs::Pose base_link_grasp_pose;
    planning_models::msgFromPose(base_link_grasp_pose_e, base_link_grasp_pose);
    
    moveit_msgs::Constraints emp;
    sensor_msgs::JointState solution;
    moveit_msgs::MoveItErrorCodes error_code;
    ROS_DEBUG_STREAM("X y z " << base_link_grasp_pose.position.x << " " 
                     << base_link_grasp_pose.position.y << " " 
                     << base_link_grasp_pose.position.z);
    if(!constraint_aware_solver_map_[pickup_goal.arm_name]->findConstraintAwareSolution(base_link_grasp_pose,
                                                                                        emp,
                                                                                        &state,
                                                                                        planning_scene,
                                                                                        object_support_disable_acm,
                                                                                        solution,
                                                                                        error_code,
                                                                                        false,
                                                                                        true)) {
      ROS_DEBUG_STREAM("Grasp out of reach");
      execution_info[i].result_.result_code = GraspResult::GRASP_OUT_OF_REACH;
      outcome_count[GraspResult::GRASP_OUT_OF_REACH]++;
      continue;
    }

    // ------------- CHECKING INTERPOLATED IK FROM PREGRASP TO GRASP ------------------        
      
    std::map<std::string, double> ik_map_pre_grasp;
    std::map<std::string, double> ik_map_grasp;
    for(unsigned int j = 0; j < joint_names.size(); j++) {
      ik_map_pre_grasp[joint_names[j]] = solution.position[j];
    } 
    ik_map_grasp = ik_map_pre_grasp;
    
    for(unsigned int j = 0; j < grasps[i].pre_grasp_posture.name.size(); j++) {
      ik_map_pre_grasp[grasps[i].pre_grasp_posture.name[j]] = grasps[i].pre_grasp_posture.position[j];
    }
    for(unsigned int j = 0; j < grasps[i].grasp_posture.name.size(); j++) {
      ik_map_grasp[grasps[i].grasp_posture.name[j]] = grasps[i].grasp_posture.position[j];
    }
    
    ROS_DEBUG_STREAM("Original ik redundancy pose " << ik_map_pre_grasp["r_upper_arm_roll_joint"]);

    state.setStateValues(ik_map_pre_grasp);      
    execution_info[i].approach_trajectory_.joint_names = joint_names;
    //now we need to do interpolated ik
    if(!getInterpolatedIK(pickup_goal.arm_name,
                          planning_scene,
                          object_support_disable_acm,
                          pickup_goal.path_constraints,
                          base_link_grasp_pose,
                          pregrasp_dir,
                          grasps[i].desired_approach_distance,
                          true,
                          false,
                          true,
                          &state,
                          execution_info[i].approach_trajectory_)) {
      ROS_DEBUG_STREAM("No interpolated IK for pre-grasp to grasp");
      execution_info[i].result_.result_code = GraspResult::PREGRASP_UNFEASIBLE;
      outcome_count[GraspResult::PREGRASP_UNFEASIBLE]++;
      continue;
    }
    
    std::stringstream s;
    s << "Last approach point is ";
    for(unsigned int k = 0; k < execution_info[i].approach_trajectory_.points.back().positions.size(); k++) {
      s << execution_info[i].approach_trajectory_.points.back().positions[k] << " "; 
    }
    ROS_DEBUG_STREAM(s.str());
    
    // ------------- CHECKING INTERPOLATED IK FROM GRASP TO LIFT ------------------        o

    execution_info[i].attached_object_diff_scene_->getCurrentState().setStateValues(ik_map_grasp);
    execution_info[i].lift_trajectory_.joint_names = joint_names;
    //TODO - figure out if we need to muck with allowed collision matrix for the diff scene
    if(!getInterpolatedIK(pickup_goal.arm_name,
                          execution_info[i].attached_object_diff_scene_,
                          object_support_disable_acm,
                          pickup_goal.path_constraints,
                          base_link_grasp_pose,
                          lift_dir,
                          pickup_goal.lift.desired_distance,
                          false,
                          true,
                          true,
                          &execution_info[i].attached_object_diff_scene_->getCurrentState(),
                          execution_info[i].lift_trajectory_)) {
      ROS_DEBUG_STREAM("No interpolated IK for grasp to lift");
      execution_info[i].result_.result_code = GraspResult::LIFT_UNFEASIBLE;
      outcome_count[GraspResult::LIFT_UNFEASIBLE]++;
      continue;
    }

    std::stringstream s2;
    s2 << "First lift point is ";
    for(unsigned int k = 0; k < execution_info[i].lift_trajectory_.points.front().positions.size(); k++) {
      s2 << execution_info[i].lift_trajectory_.points.front().positions[k] << " "; 
    }
    ROS_DEBUG_STREAM(s2.str());
    
    // ------------- CHECKING PREGRASP OK FOR PLANNING ------------------            
    if(execution_info[i].approach_trajectory_.points.empty()) {
      ROS_WARN_STREAM("No result code and no points in approach trajectory");
      continue;
    }

    std::map<std::string, double> pre_grasp_ik = ik_map_pre_grasp;
    for(unsigned int j = 0; j < joint_names.size(); j++) {
      pre_grasp_ik[joint_names[j]] = execution_info[i].approach_trajectory_.points[0].positions[j];
    } 
    state.setStateValues(pre_grasp_ik);
    //the start of the approach needs to be collision-free according to the default collision matrix
    //cm->setAlteredAllowedCollisionMatrix(group_disable_acm);
    //req.verbose = true;
    res = collision_detection::CollisionResult();
    planning_scene->checkCollision(req, res, state, group_disable_acm);
    if(res.collision) {
      ROS_DEBUG_STREAM("Final pre-grasp check failed");
      execution_info[i].result_.result_code = GraspResult::PREGRASP_OUT_OF_REACH;
      outcome_count[GraspResult::PREGRASP_OUT_OF_REACH]++;
      continue;
    } 
    
    // ------------- CHECKING LIFT OK FOR PLANNING ------------------            

    if(execution_info[i].lift_trajectory_.points.empty()) {
      ROS_WARN_STREAM("No result code and no points in lift trajectory");
      continue;
    }    

    std::map<std::string, double> lift_ik = ik_map_pre_grasp;
    for(unsigned int j = 0; j < joint_names.size(); j++) {
      lift_ik[joint_names[j]] = execution_info[i].lift_trajectory_.points.back().positions[j];
    } 
    execution_info[i].attached_object_diff_scene_->getCurrentState().setStateValues(lift_ik);
    res = collision_detection::CollisionResult();
    execution_info[i].attached_object_diff_scene_->checkCollision(req, res, execution_info[i].attached_object_diff_scene_->getCurrentState());
    if(res.collision) {
      ROS_DEBUG_STREAM("Final lift check failed");
      execution_info[i].result_.result_code = GraspResult::LIFT_OUT_OF_REACH;
      outcome_count[GraspResult::LIFT_OUT_OF_REACH]++;
      continue;
    } else {
      ROS_INFO_STREAM("Everything successful");
      execution_info[i].result_.result_code = GraspResult::SUCCESS;
      execution_info.resize(i+1);
      outcome_count[GraspResult::SUCCESS]++;
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
  ROS_INFO_STREAM("Took " << (ros::WallTime::now()-start).toSec());
  return;
}

} //namespace grasp_place_evaluation
