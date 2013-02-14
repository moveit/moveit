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
#include <moveit/pick_place/reachable_valid_pose_filter.h>
#include <moveit/pick_place/approach_and_translate_stage.h>
#include <moveit/pick_place/plan_stage.h>
#include <moveit/robot_state/conversions.h>
#include <ros/console.h>

namespace pick_place
{

PlacePlan::PlacePlan(const PickPlaceConstPtr &pick_place) :
  pick_place_(pick_place),
  pipeline_("place", 4),
  last_plan_time_(0.0),
  done_(false)
{
  pipeline_.setSolutionCallback(boost::bind(&PlacePlan::foundSolution, this));
}

PlacePlan::~PlacePlan()
{
}

bool PlacePlan::plan(const planning_scene::PlanningSceneConstPtr &planning_scene, const moveit_msgs::PlaceGoal &goal)
{
  double timeout = goal.allowed_planning_time;
  ros::WallTime endtime = ros::WallTime::now() + ros::WallDuration(timeout);
  std::string attached_object_name = goal.attached_object_name;
  
  const robot_model::JointModelGroup *jmg = planning_scene->getRobotModel()->getJointModelGroup(goal.group_name);
  if (!jmg)
  {
    error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
    return false;
  }
  // \todo: allow group & eef names;
  
  // try to infer attached body name if possible
  if (attached_object_name.empty())
  {
    
    const std::vector<std::string> &links = jmg->getLinkModelNames();
    for (std::size_t i = 0 ; i < links.size() ; ++i)
    {
      const robot_state::LinkState *ls = planning_scene->getCurrentState().getLinkState(links[i]);
      if (ls)
      {  
        std::vector<const robot_state::AttachedBody*> attached_bodies;
        ls->getAttachedBodies(attached_bodies);
        if (attached_bodies.empty())
          continue;
        if (attached_bodies.size() > 1 || !attached_object_name.empty())
        {
          ROS_ERROR("Multiple attached bodies for group '%s' but no explicit attached object to place was specified", goal.group_name.c_str());
          error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
          return false;
        }
        else
          attached_object_name = attached_bodies[0]->getName();
      }
    }
  }
  
  const robot_state::AttachedBody *attached_body = planning_scene->getCurrentState().getAttachedBody(attached_object_name);
  if (!attached_body)
  {
    ROS_ERROR("There is no object to detach");
    error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    
    return false;
  }
  
  ros::WallTime start_time = ros::WallTime::now();
  
  // construct the planning scene as it will look after the object to be picked will actually be picked
  planning_scene::PlanningScenePtr planning_scene_after_place = planning_scene->diff();
  moveit_msgs::AttachedCollisionObject detach_object_msg;
  detach_object_msg.link_name = attached_body->getAttachedLinkName();
  detach_object_msg.object.id = attached_object_name;
  detach_object_msg.object.operation = moveit_msgs::CollisionObject::REMOVE;
  planning_scene_after_place->processAttachedCollisionObjectMsg(detach_object_msg);
  
  collision_detection::AllowedCollisionMatrixPtr approach_place_acm(new collision_detection::AllowedCollisionMatrix(planning_scene->getAllowedCollisionMatrix()));
  
  // configure the manipulation pipeline
  pipeline_.reset();
  
  ManipulationStagePtr stage1(new ReachableAndValidPoseFilter(planning_scene, approach_place_acm, pick_place_->getConstraintsSamplerManager()));
  ManipulationStagePtr stage2(new ApproachAndTranslateStage(planning_scene, planning_scene_after_place, approach_place_acm));
  ManipulationStagePtr stage3(new PlanStage(planning_scene, pick_place_->getPlanningPipeline())); 
  pipeline_.addStage(stage1).addStage(stage2).addStage(stage3);


  ManipulationPlanSharedDataPtr plan_data(new ManipulationPlanSharedData());
  ManipulationPlanSharedDataConstPtr const_plan_data = plan_data;
  /*
  plan_data->planning_group_ = planning_group;
  plan_data->end_effector_group_ = eef->getName();
  plan_data->ik_link_name_ = ik_link;
  plan_data->timeout_ = endtime;
  plan_data->max_goal_sampling_attempts_ = std::max(1u, jmg->getDefaultIKAttempts());
  */

  for (std::size_t i = 0 ; i < goal.place_locations.size() ; ++i)
  {
    ManipulationPlanPtr p(new ManipulationPlan(const_plan_data));
    const manipulation_msgs::PlaceLocation &pl = goal.place_locations[i];
    p->goal_pose_ = pl.place_pose;
    p->approach_ = pl.approach;
    p->retreat_ = pl.retreat;
    p->retreat_posture_ = pl.post_place_posture;
    pipeline_.push(p);
  }
  pipeline_.start();
  
  pipeline_.stop();


  return error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS;
}

void PlacePlan::foundSolution()
{
  boost::mutex::scoped_lock slock(done_mutex_);
  done_ = true;
  done_condition_.notify_all();
}

PlacePlanPtr PickPlace::planPlace(const planning_scene::PlanningSceneConstPtr &planning_scene, const moveit_msgs::PlaceGoal &goal) const
{
  PlacePlanPtr p(new PlacePlan(shared_from_this()));
  if (planning_scene::PlanningScene::isEmpty(goal.planning_options.planning_scene_diff))
    p->plan(planning_scene, goal);
  else
    p->plan(planning_scene->diff(goal.planning_options.planning_scene_diff), goal);
  return p;
}

}

