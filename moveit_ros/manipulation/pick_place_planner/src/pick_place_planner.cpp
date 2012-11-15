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

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <pick_place_planner/pick_place_planner.h>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit_manipulation_msgs/PickupGoal.h>
#include <moveit_manipulation_msgs/PlaceGoal.h>

namespace pick_place_planner
{

PickPlacePlanner::PickPlacePlanner(
        const kinematic_model::KinematicModelConstPtr& kinematic_model,
        const std::string &freespace_planning_plugin_name,
        const std::string &contact_planning_plugin_name)
{
    kinematic_model_ = kinematic_model;
    freespace_planning_pipeline_.reset(new planning_pipeline::PlanningPipeline(
                                           kinematic_model_, freespace_planning_plugin_name));

    contact_planning_pipeline_.reset(new planning_pipeline::PlanningPipeline(
                                         kinematic_model_, contact_planning_plugin_name));
}

bool PickPlacePlanner::addManipulationGroup(const std::string &arm_name, const std::string &end_effector_name)
{
    boost::shared_ptr<ManipulationGroup> manip_group = boost::make_shared<ManipulationGroup>(kinematic_model_, arm_name);
    manipulation_groups_[arm_name] = manip_group;
    return true;
}

bool PickPlacePlanner::planPick(const moveit_manipulation_msgs::PickupGoal &pickup_goal,
                                const planning_scene::PlanningSceneConstPtr &planning_scene,
                                std::vector<PickPlacePlanner::Plan> &pick_plans,
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
        ROS_ERROR("No manipulation group for arm %s", pickup_goal.arm_name.c_str());
        return false;
    }
    ROS_INFO("Planning pick with manipulation group %s", pickup_goal.arm_name.c_str());

    pick_place_planner::ManipulationGroupPtr manipulation_group = manipulation_groups_.find(pickup_goal.arm_name)->second;
    PickPlacePlanner::PickGoal pick_goal(pickup_goal, planning_scene, manipulation_group);
    ROS_INFO("Checking %d grasps", pick_goal.num_grasps_);
    pick_plans.resize(pick_goal.num_grasps_);

    bool success = false;
    for(unsigned int i=0; i < pick_goal.num_grasps_; ++i)
    {
        pick_plans[i].initializeForPick(pickup_goal, planning_scene);

        /* check IK for pre-grasp, grasp, and lift
    if(!pick_goal.checkPoses(i, pick_plans[i]))
    {
      continue;
    }
    */

        /* Plan from pre-grasp pose to grasp pose using interpolated IK
    if(!planContact(pick_goal.pre_grasp_pose[i], pick_goal.grasp_pose[i], planning_scene, pick_plans[i]))
    {
      continue;
    }
    */

        /* Plan from grasp pose to lift pose using interpolated IK
    if(!planContact(pick_goal.grasp_pose[i], pick_goal.lift_pose[i], planning_scene, pick_plans[i]))
    {
      continue;
    }
    */

        /* Plan from start pose to pre-grasp pose
    if(!planFreeSpace(pick_goal,pick_plans[i]))
      return false;
    */

        /* if we got this far, then we have a plan for this grasp */
        success = true;
        if(return_on_first_solution)
            break;
    }

    return success;
}

bool PickPlacePlanner::planPlace(const moveit_manipulation_msgs::PlaceGoal &place_goal,
                                 const planning_scene::PlanningSceneConstPtr &planning_scene,
                                 std::vector<PickPlacePlanner::Plan> &pick_plans,
                                 bool return_on_first_solution) const
{
    return true;
}


PickPlacePlanner::PickGoal::PickGoal(const moveit_manipulation_msgs::PickupGoal &pickup_goal,
                                     const planning_scene::PlanningSceneConstPtr &planning_scene,
                                     const pick_place_planner::ManipulationGroupConstPtr &manipulation_group)
{
    pickup_goal_ = pickup_goal;
    planning_scene_ = planning_scene;
    manipulation_group_ = manipulation_group;

    if(pickup_goal.multi_end_effector_desired_grasps.size() > 0)
    {
        /* this pickup goal uses multiple end effector grasps */
        num_grasps_ = pickup_goal.multi_end_effector_desired_grasps.size();

        /* fill in the pre-grasp/grasp/lift poses for each end effector */
        grasp_pose_.resize(num_grasps_);

        for(unsigned int grasp_i = 0; grasp_i < num_grasps_; grasp_i++)
        {
            const moveit_manipulation_msgs::MultiEndEffectorGrasp& multi_eff_grasp = pickup_goal.multi_end_effector_desired_grasps[grasp_i];
            if(multi_eff_grasp.end_effectors.size() != multi_eff_grasp.grasps.size())
            {
                ROS_ERROR("Multi end effector grasp lists %zu effectors but only %zu grasps",
                          multi_eff_grasp.end_effectors.size(), multi_eff_grasp.grasps.size());
                continue;
            }

            for(unsigned int eff_i = 0; eff_i < multi_eff_grasp.end_effectors.size(); ++eff_i)
            {
                const std::string& eff_name = multi_eff_grasp.end_effectors[eff_i];
                tf::poseMsgToEigen(multi_eff_grasp.grasps[eff_i].grasp_pose, grasp_pose_[grasp_i][eff_name]);
            }
        }
    }
    else
    {
        /* this pickup goal uses only a single end effector */
        /* FIXME - not implemented yet */
        ROS_ERROR("Single end effector grasp specification not yet implemented");
        num_grasps_ = 0;
    }

    pre_grasp_pose_.resize(num_grasps_);
    lift_pose_.resize(num_grasps_);

    Eigen::Vector3d lift_dir;
    lift_dir.x() = pickup_goal.lift.direction.vector.x;
    lift_dir.y() = pickup_goal.lift.direction.vector.y;
    lift_dir.z() = pickup_goal.lift.direction.vector.z;
    lift_dir.normalize();
    planning_scene->getTransforms()->transformVector3(
                planning_scene->getCurrentState(),
                pickup_goal.target.reference_frame_id,
                lift_dir,
                lift_dir);

    Eigen::Translation3d distance_lift_dir(lift_dir * fabs(pickup_goal.lift.desired_distance));
    lift_transform_ = distance_lift_dir*Eigen::Quaterniond::Identity();
    reference_frame_id_ = pickup_goal.target.reference_frame_id;
    planning_scene_diff_ = planning_scene->diff();

    attached_object_diff_ = planning_scene->diff();
    if(!pickup_goal.collision_object_name.empty())
    {
        moveit_msgs::AttachedCollisionObject attached_object = manipulation_group->getAttachedBodyMsg(pickup_goal.collision_object_name);
        attached_object_diff_->processAttachedCollisionObjectMsg(attached_object);
    }

    allow_gripper_support_collision_ = pickup_goal.allow_gripper_support_collision;

    grasp_acm_ = planning_scene->getAllowedCollisionMatrix();
    lift_acm_ = grasp_acm_;
}

void PickPlacePlanner::PickGoal::computeGoalsForGrasp(unsigned int grasp_i)
{

    /* Convert everything into planning scene frame */
    const std::vector<std::string>& end_effector_names = manipulation_group_->getEndEffectorNames();
    for(unsigned int eff_i=0; eff_i < end_effector_names.size(); ++eff_i)
    {
        const std::string& eff_name = end_effector_names[eff_i];

        Eigen::Affine3d& grasp_pose = grasp_pose_[grasp_i][eff_name];
        planning_scene_->getTransforms()->transformPose(reference_frame_id_, grasp_pose, grasp_pose);

        // FIXME - pre-grasp direction should be stored in the ManipulationGroup class
        // for now, i've hardcoded it to x-axis foward, like the PR2 gripper
        Eigen::Vector3d pre_grasp_direction(1.0, 0.0, 0.0);

        /* FIXME - this only works for multi-end-effector grasps */
        Eigen::Translation3d pre_grasp_translation(pre_grasp_direction * fabs(pickup_goal_.multi_end_effector_desired_grasps[grasp_i].grasps[eff_i].desired_approach_distance));

        Eigen::Affine3d pre_grasp_transform(pre_grasp_translation * Eigen::Quaterniond::Identity());

        pre_grasp_pose_[grasp_i][eff_i] = grasp_pose * pre_grasp_transform; // assume this is in gripper frame
        lift_pose_[grasp_i][eff_i] = lift_transform_ * grasp_pose;
    }
}

bool PickPlacePlanner::checkPoses(PickGoal &pick_goal,
                                  unsigned int grasp_i,
                                  Plan &plan)
{
    //Check the grasp pose
    /*
    collision_detection::AllowedCollisionMatrix allowed_collision_matrix_for_grasp = pick_goal.planning_scene_diff_->getAllowedCollisionMatrix();
    if(!checkEndEffectorPose(pick_goal.planning_scene_diff_,
                             pick_goal.grasp_pose_[grasp_i],
                             pick_goal.grasp_posture_[grasp_i],
                             allowed_collision_matrix_for_grasp))
    {
        pick_plan.setStatus(pick_place_planner::GRASP_POSE_IN_CONTACT);
        return false;
    }

    collision_detection::AllowedCollisionMatrix allowed_collision_matrix_for_pre_grasp = pick_goal.planning_scene_diff_->getAllowedCollisionMatrix();
    if(!checkEndEffectorPose(pick_goal.planning_scene_diff_,
                             grasp_pose_[grasp_i],
                             grasp_posture_[grasp_i],
                             allowed_collision_matrix_for_pre_grasp))
    {
        pick_plan.setStatus(pick_place_planner::PRE_GRASP_POSE_IN_CONTACT);
        return false;
    }

    collision_detection::AllowedCollisionMatrix allowed_collision_matrix_for_lift = pick_goal.attached_object_diff_->getAllowedCollisionMatrix();
    if(!checkEndEffectorPose(pick_goal.attached_object_diff_,
                             grasp_pose_[grasp_i],
                             grasp_posture_[grasp_i],
                             allowed_collision_matrix_for_lift))
    {
        pick_plan.setStatus(pick_place_planner::LIFT_IN_CONTACT);
        return false;
    }
    */

    return true;
}

bool PickPlacePlanner::PickGoal::checkEndEffectorPose(planning_scene::PlanningSceneConstPtr &planning_scene,
                                                      const std::vector<Eigen::Affine3d> &poses,
                                                      const std::map<std::string, double> &end_effector_posture,
                                                      const collision_detection::AllowedCollisionMatrix &acm)
{
    /*
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

*/
    return true;
}

PickPlacePlanner::Plan::Plan()
    : success_(false)
{
    /* do nothing; the real work gets done int initializeForPick or initializeForPlace */
}

void PickPlacePlanner::Plan::initializeForPick(const moveit_manipulation_msgs::PickupGoal &pickup_goal,
                                               const planning_scene::PlanningSceneConstPtr &planning_scene)

{
    /* just in case this plan has already been initialized */
    robot_trajectories_.clear();
    trajectory_types_.clear();
    control_modes_.clear();

    robot_trajectories_.resize(PICK_NUM_PHASES);

    trajectory_types_.push_back(pick_place_planner::PickPlacePlanner::JOINT_TRAJECTORY);
    trajectory_types_.push_back(pick_place_planner::PickPlacePlanner::JOINT_TRAJECTORY);
    trajectory_types_.push_back(pick_place_planner::PickPlacePlanner::JOINT_TRAJECTORY);

    control_modes_.push_back(pick_place_planner::PickPlacePlanner::FREESPACE_TRAJECTORY);
    control_modes_.push_back(pick_place_planner::PickPlacePlanner::CONTACT_TRAJECTORY);
    control_modes_.push_back(pick_place_planner::PickPlacePlanner::CONTACT_TRAJECTORY);
}


void PickPlacePlanner::Plan::initializeForPlace(const moveit_manipulation_msgs::PlaceGoal &place_goal,
                                                const planning_scene::PlanningSceneConstPtr &planning_scene)
{
    /* just in case this plan has already been initialized */
    /*
  robot_trajectories_.clear();
  trajectory_types_.clear();
  control_modes_.clear();

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
  */
}



}
