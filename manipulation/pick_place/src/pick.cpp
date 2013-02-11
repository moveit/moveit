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

/* Author: Ioan Sucan */

#include <moveit/pick_place/pick_place.h>
#include <moveit/pick_place/reachable_valid_grasp_filter.h>
#include <moveit/pick_place/approach_and_translate_stage.h>
#include <moveit/pick_place/plan_stage.h>
#include <moveit/robot_state/conversions.h>
#include <ros/console.h>

namespace pick_place
{

namespace
{
struct OrderGraspQuality
{
  OrderGraspQuality(const std::vector<manipulation_msgs::Grasp> &grasps) : grasps_(grasps)
  {
  }
  
  bool operator()(const std::size_t a, const std::size_t b) const
  {
    return grasps_[a].grasp_quality > grasps_[b].grasp_quality;
  }
  
  const std::vector<manipulation_msgs::Grasp> &grasps_;
};
}

PickPlan::PickPlan(const PickPlaceConstPtr &pick_place) :
  pick_place_(pick_place),
  pipeline_("pick", 4),
  last_plan_time_(0.0),
  done_(false)
{
  pipeline_.setSolutionCallback(boost::bind(&PickPlan::foundSolution, this));
}

PickPlan::~PickPlan()
{
}

bool PickPlan::plan(const planning_scene::PlanningSceneConstPtr &planning_scene, const moveit_msgs::PickupGoal &goal)
{
  double timeout = goal.allowed_planning_time;
  ros::WallTime endtime = ros::WallTime::now() + ros::WallDuration(timeout);
  
  std::string planning_group = goal.group_name;
  std::string end_effector = goal.end_effector;
  if (end_effector.empty() && !planning_group.empty())
  {
    const robot_model::JointModelGroup *jmg = planning_scene->getRobotModel()->getJointModelGroup(planning_group);
    if (!jmg)
    {
      error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
      return false;
    }
    const std::vector<std::string> &eefs = jmg->getAttachedEndEffectorNames();
    if (!eefs.empty())
    {
      end_effector = eefs.front();
      if (eefs.size() > 1)
        ROS_WARN_STREAM("Choice of end-effector for group '" << planning_group << "' is ambiguous. Assuming '" << end_effector << "'");
    }
  }
  else
    if (!end_effector.empty() && planning_group.empty())
    {
      const robot_model::JointModelGroup *jmg = planning_scene->getRobotModel()->getEndEffector(end_effector);
      if (!jmg)
      {
        error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;  
        return false;
      }
      planning_group = jmg->getEndEffectorParentGroup().first;
      ROS_INFO_STREAM("Assuming the planning group for end effector '" << end_effector << "' is '" << planning_group << "'");
    }      
  const robot_model::JointModelGroup *eef = end_effector.empty() ? NULL : planning_scene->getRobotModel()->getEndEffector(end_effector);
  if (!eef)
  {
    ROS_ERROR("No end-effector specified for pick action");
    error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
    return false;
  }
  const std::string &ik_link = eef->getEndEffectorParentGroup().second;

  ros::WallTime start_time = ros::WallTime::now();
  
  // construct the planning scene as it will look after the object to be picked will actually be picked
  planning_scene::PlanningScenePtr planning_scene_after_grasp = planning_scene->diff();
  moveit_msgs::AttachedCollisionObject attach_object_msg;
  attach_object_msg.link_name = ik_link;
  attach_object_msg.object.id = goal.target_name;
  attach_object_msg.object.operation = moveit_msgs::CollisionObject::ADD;
  planning_scene_after_grasp->processAttachedCollisionObjectMsg(attach_object_msg);
  
  collision_detection::AllowedCollisionMatrixPtr approach_grasp_acm(new collision_detection::AllowedCollisionMatrix(planning_scene->getAllowedCollisionMatrix()));
  
  // we are allowed to touch the target object
  approach_grasp_acm->setEntry(goal.target_name, eef->getLinkModelNames(), true);
  // we are allowed to touch certain other objects with the gripper
  approach_grasp_acm->setEntry(eef->getLinkModelNames(), goal.allowed_touch_objects, true);
  if (!goal.collision_support_surface_name.empty())
  {
    // we are allowed to have contact between the target object and the support surface before the grasp 
    approach_grasp_acm->setEntry(goal.collision_support_surface_name, goal.target_name, true);
    
    // optionally, it may be allowed to touch the support surface with the gripper
    if (goal.allow_gripper_support_collision)
      approach_grasp_acm->setEntry(goal.collision_support_surface_name, eef->getLinkModelNames(), true);
  }
  // for now, the post_grasp_acm can be the same
  
  // configure the manipulation pipeline
  pipeline_.reset();
  ManipulationStagePtr stage1(new ReachableAndValidGraspFilter(planning_scene, approach_grasp_acm, pick_place_->getConstraintsSamplerManager()));
  ManipulationStagePtr stage2(new ApproachAndTranslateStage(planning_scene, planning_scene_after_grasp, approach_grasp_acm));
  ManipulationStagePtr stage3(new PlanStage(planning_scene, pick_place_->getPlanningPipeline())); 
  pipeline_.addStage(stage1).addStage(stage2).addStage(stage3);
  
  pipeline_.start();
  
  // order the grasps by quality
  std::vector<std::size_t> grasp_order(goal.possible_grasps.size());
  for (std::size_t i = 0 ; i < goal.possible_grasps.size() ; ++i)
    grasp_order[i] = i;
  OrderGraspQuality oq(goal.possible_grasps);
  std::sort(grasp_order.begin(), grasp_order.end(), oq);

  done_ = false;
  
  // feed the available grasps to the stages we set up
  for (std::size_t i = 0 ; i < goal.possible_grasps.size() ; ++i)
  {
    ManipulationPlanPtr p(new ManipulationPlan());
    p->grasp_ = goal.possible_grasps[grasp_order[i]];
    p->planning_group_ = planning_group;
    p->end_effector_group_ = eef->getName();
    p->ik_link_name_ = ik_link;    
    p->timeout_ = endtime;
    p->processing_stage_ = 0;
    pipeline_.push(p);
  }
    
  // wait till we're done
  {
    boost::unique_lock<boost::mutex> lock(done_mutex_);
    while (!done_ && endtime > ros::WallTime::now())
      done_condition_.timed_wait(lock, (endtime - ros::WallTime::now()).toBoost());
  }
  pipeline_.stop();
  
  last_plan_time_ = (ros::WallTime::now() - start_time).toSec();
  
  if (!getSuccessfulManipulationPlans().empty())
    error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  else
  {
    if (last_plan_time_ > timeout)
      error_code_.val = moveit_msgs::MoveItErrorCodes::TIMED_OUT;
    else
      error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
  }
  ROS_INFO("Pickup completed after %lf seconds", last_plan_time_);
  
  return error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS;
}

void PickPlan::foundSolution()
{
  boost::mutex::scoped_lock slock(done_mutex_);
  done_ = true;
  done_condition_.notify_all();
}

PickPlanPtr PickPlace::planPick(const planning_scene::PlanningSceneConstPtr &planning_scene, const moveit_msgs::PickupGoal &goal) const
{
  PickPlanPtr p(new PickPlan(shared_from_this()));

  if (planning_scene::PlanningScene::isEmpty(goal.planning_options.planning_scene_diff))
    p->plan(planning_scene, goal);
  else
    p->plan(planning_scene->diff(goal.planning_options.planning_scene_diff), goal);

  if (display_computed_motion_plans_)
  {
    const std::vector<pick_place::ManipulationPlanPtr> &success = p->getSuccessfulManipulationPlans();
    if (!success.empty())
      visualizePlan(success.back());
  }
    
  if (display_grasps_)
  {
    const std::vector<pick_place::ManipulationPlanPtr> &success = p->getSuccessfulManipulationPlans();
    visualizeGrasps(success);
    const std::vector<pick_place::ManipulationPlanPtr> &failed = p->getFailedManipulationPlans();
    visualizeGrasps(failed);
  }
  
  return p;
}

}

