/*********************************************************************
*
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
*
* Author: Sachin Chitta
*********************************************************************/

#include <pick_place_planner/pick_place_planner.h>

namespace pick_place_planner
{

void PickPlacePlanner::PickPlacePlanner(const std::string &freespace_planning_plugin_name,
                                        const std::string &contact_planning_plugin_name)
{
  freespace_planning_pipeline_.reset(new planning_pipeline::PlanningPipeline(kinematic_model_,
                                                                             freespace_planning_plugin_name,
                                                                             contact_planning_plugin_name));
  contact_planner_.reset(new interpolation_planners::InterpolationPlanner(kinematic_model_,
                                                                          contact_planning_plugin_name));
}

void PickPlacePlanner::planPick(const moveit_manipulation_msgs::PickupGoal &pickup_goal,
                                const planning_scene::PlanningSceneConstPtr &planning_scene,
                                std::vector<pick_place_planner::Plan> &pick_plans,
                                bool return_on_first_solution) const
{
  /**
   * A pick = (1) Move to pre-grasp (2) Move to grasp (3) Lift
   * Set of operations 
   * For each grasp:
   * (1) Check grasp pose, pre-grasp pose, lift pose
   * (2) Plan back from grasp to pre-grasp with unpadded robot
   * (3) Plan for lift with unpadded robot
   * For each valid pick place:
   * Plan for freespace motion to pre-grasp with padded robot
   */
  if(manipulation_groups_.find(pickup_goal.arm_name) == manipulation_groups_.end())
  {
    //Need to set error code
    return;    
  }

  pick_place_planner::ManipulationGroupPtr manipulation_group = manipulation_groups_.find(pickup_goal.arm_name)->second;
  pick_place_planner::PickGoal pick_goal(pickup_goal,manipulation_group);
  pick_plans.resize(pick_goal.num_grasps_);  

  for(unsigned int i=0; i < pick_goal.num_grasps_; ++i)
  { 
    pick_place_planner::Plan pick_plan(pickup_goal);    
    pick_goal.computeGoalsForGrasp(i);
        
    if(!pick_goal.checkPoses(i,pick_plans[i]))
    {     
      continue;
    }
    
    if(!planContact(pick_goal.pre_grasp_pose[i],pick_goal.grasp_pose[i],planning_scene,pick_plans[i]))
    {
      continue;
    }
        
    if(!planContact(pick_goal.grasp_pose[i],pick_goal.lift_pose[i],planning_scene,pick_plans[i]))
    {
      continue;
    }
    
    if(return_on_first_solution)
      break;
  }

  if(!planFreeSpace(pick_goal,pick_plans[i]))
    return false;

  return true;        
}

void PickPlacePlanner::planPlace(const moveit_manipulation_msgs::PlaceGoal &place_goal,
                                 const planning_scene::PlanningSceneConstPtr &planning_scene,
                                 pick_place_planner::PlanPtr &place_plan,
                                 bool return_on_first_solution) const
{

}


PickPlacePlanner::PickGoal::PickGoal(const moveit_manipulation_msgs::PickupGoal &pickup_goal,
                                     const planning_scene::PlanningSceneConstPtr &planning_scene,
                                     const pick_place_planner::ManipulationGroupConstPtr &manipulation_group)
{
  num_grasps_ = pickup_goal.desired_grasps.size();  
  
  grasp_pose_.resize(num_grasps_);
  pre_grasp_pose_.resize(num_grasps_);
  lift_pose_.resize(num_grasps_);
  for(unsigned int i=0; i < grasp_pose_.size(); ++i)
  {
    grasp_pose_[i].resize(group_names_.size());
    pre_grasp_pose_[i].resize(group_names_.size());
    lift_pose_[i].resize(group_names_.size());    
  }

  pre_grasp_direction_ = manipulation_group.pre_grasp_direction;
    
  Eigen::Translation3d pre_grasp_translation(pre_grasp_direction_*fabs(pickup_goal.desired_grasps[i].desired_approach_distance));
  Eigen::Affine3d pre_grasp_transform(pre_grasp_translation*Eigen::Quaterniond::Identity());
  pre_grasp_transform_ = pre_grasp_transform;  

  Eigen::Vector3d lift_dir;
  lift_dir.x() = pickup_goal.lift.direction.vector.x;
  lift_dir.y() = pickup_goal.lift.direction.vector.y;
  lift_dir.z() = pickup_goal.lift.direction.vector.z;
  lift_dir.normalize();
  planning_scene_->getTransforms()->transformVector(planning_scene_->getCurrentState(),
                                                    pickup_goal.target.reference_frame_id,
                                                    lift_dir,
                                                    lift_dir);
  
  Eigen::Translation3d distance_lift_dir(lift_dir*fabs(pickup_goal.lift.desired_distance));
  Eigen::Affine3d lift_trans(distance_lift_dir*Eigen::Quaterniond::Identity());
  lift_transform_ = lift_transform;
  reference_frame_id_ = pickup_goal.target.reference_frame_id;
  grasps_ = pickup_goal.desired_grasps;  
  planning_scene_diff_ = planning_scene->diff();  


  moveit_msgs::AttachedCollisionObject attached_object = manipulation_group->getAttachedBodyMsg(pickup_goal.collision_object_name)
  attached_object_diff_ = planning_scene->diff();  
  attached_object_diff_->processAttachedCollisionObjectMsg(attached_object);
  allow_gripper_support_collision_ = pickup_goal.allow_gripper_support_collision;  

  grasp_acm_ = planning_scene_->getAllowedCollisionMatrix();
  lift_acm_ = grasp_acm_;
}

void PickPlacePlanner::PickGoal::computeGoalsForGrasp(unsigned int index)
{
  // Convert everything into planning scene frame
  for(unsigned int i=0; i < group_names_.size(); ++i)
  {
    Eigen::Affine3d grasp_pose;
    planning_models::poseFromMsg(grasps_[index].grasp_poses[i],grasp_pose);    
    planning_scene_->getTransforms()->transformPose(planning_scene_->getCurrentState(),
                                                    reference_frame_id_,
                                                    grasp_pose,
                                                    grasp_pose_[index][i]);

    pre_grasp_pose_[index][i] = grasp_pose*pre_grasp_transform_; //assume this is in gripper frame
    lift_pose_[index][i] = lift_transform_*grasp_pose;    
  }  
}

bool PickPlacePlanner::checkPoses(pick_place_planner::PickGoal &pick_goal,
                                  unsigned int index,
                                  pick_place_planner::PickPlacePlanner &plan)
{
  //Check the grasp pose
  collision_detection::CollisionMatrix allowed_collision_matrix_for_grasp = getAllowedCollisionMatrix();  
  if(!checkEndEffectorPose(grasp_pose_planning_scene_diff_,
                           grasp_pose_[index],
                           grasp_posture_[index],
                           allowed_collision_matrix_for_grasp))
  {
      pick_plan.setStatus(pick_place_planner::GRASP_POSE_IN_CONTACT);          
      return false;      
  }

  collision_detection::CollisionMatrix allowed_collision_matrix_for_pre_grasp = getAllowedCollisionMatrix();  
  if(!checkEndEffectorPose(grasp_pose_planning_scene_diff_,
                           grasp_pose_[index],
                           grasp_posture_[index],
                           allowed_collision_matrix_for_pre_grasp))
  {
      pick_plan.setStatus(pick_place_planner::PRE_GRASP_POSE_IN_CONTACT);          
      return false;      
  }
  
  collision_detection::CollisionMatrix allowed_collision_matrix_for_lift = getAllowedCollisionMatrix();  
  if(!checkEndEffectorPose(grasp_pose_planning_scene_diff_,
                           grasp_pose_[index],
                           grasp_posture_[index],
                           allowed_collision_matrix_for_lift))
  {
      pick_plan.setStatus(pick_place_planner::LIFT_IN_CONTACT);
      return false;      
  }
  return true;  
}

bool PickPlacePlanner::PickGoal::checkEndEffectorPose(planning_scene::PlanningSceneConstPtr &planning_scene,
                                                      const std::vector<Eigen::Affine3d> &poses,
                                                      const std::map<std::string, double> &end_effector_posture,
                                                      const collision_detection::AllowedCollisionMatrix &acm)
{
  planning_scene->getCurrentState().setStateValues(end_effector_posture);
  for(unsigned int i=0; i < poses.size(); ++i)
  {
    planning_scene->getCurrentState().updateStateWithLinkAt(group_names_[i],poses[i]);
  }
  
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResponse collision_response;
  planning_scene->checkCollision(collision_request,
                                 collision_response,
                                 planning_scene->getCurrentState(),
                                 acm);
  if(collision_response.collision)
    return false;

  return true;  
}

PickPlacePlanner::Plan::Plan(const moveit_msgs::PickupGoal &pickup_goal,
                             const planning_scene::PlanningSceneConstPtr &planning_scene):success(false)
{
  for(unsigned int i=0; i < PICK_NUM_PHASES; ++i)
  {
    robot_trajectories_.push_back(planning_scene->getKinematicModel()->getJointModelGroup(pickup_goal.arm_name)->getRobotTrajectoryMsg());    
  }
  trajectory_types_.push_back(pick_place_planner::PickPlacePlanner::JOINT_TRAJECTORY);
  trajectory_types_.push_back(pick_place_planner::PickPlacePlanner::JOINT_TRAJECTORY);
  trajectory_types_.push_back(pick_place_planner::PickPlacePlanner::JOINT_TRAJECTORY);

  control_modes_.push_back(pick_place_planner::PickPlacePlanner::FREESPACE_TRAJECTORY);
  control_modes_.push_back(pick_place_planner::PickPlacePlanner::CONTACT_TRAJECTORY);
  control_modes_.push_back(pick_place_planner::PickPlacePlanner::CONTACT_TRAJECTORY);
}


PickPlacePlanner::Plan::Plan(const moveit_msgs::PlaceGoal &place_goal,
                             const planning_scene::PlanningSceneConstPtr &planning_scene):success_(false)
{
  for(unsigned int i=0; i < PLACE_NUM_PHASES; ++i)
  {
    robot_trajectories_.push_back(planning_scene->getKinematicModel()->getJointModelGroup(place_goal.arm_name)->getRobotTrajectoryMsg());    
  }
  trajectory_types_.push_back(pick_place_planner::PickPlacePlanner::JOINT_TRAJECTORY);
  trajectory_types_.push_back(pick_place_planner::PickPlacePlanner::JOINT_TRAJECTORY);
  trajectory_types_.push_back(pick_place_planner::PickPlacePlanner::JOINT_TRAJECTORY);

  control_modes_.push_back(pick_place_planner::PickPlacePlanner::FREESPACE_TRAJECTORY);
  control_modes_.push_back(pick_place_planner::PickPlacePlanner::CONTACT_TRAJECTORY);
  control_modes_.push_back(pick_place_planner::PickPlacePlanner::CONTACT_TRAJECTORY);
}



}
