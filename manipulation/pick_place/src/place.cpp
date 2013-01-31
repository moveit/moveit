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

PlacePlan::~PlacePlan(void)
{
}

bool PlacePlan::plan(const planning_scene::PlanningSceneConstPtr &planning_scene, const moveit_msgs::PlaceGoal &goal)
{
  double timeout = goal.allowed_planning_time;
  ros::WallTime endtime = ros::WallTime::now() + ros::WallDuration(timeout);
  std::string planning_group = goal.group_name;
  
  ros::WallTime start_time = ros::WallTime::now();
  
  // construct the planning scene as it will look after the object to be picked will actually be picked
  planning_scene::PlanningScenePtr planning_scene_after_grasp = planning_scene->diff();
  moveit_msgs::AttachedCollisionObject detach_object_msg;
  //  detach_object_msg.link_name = ik_link;
  //  detach_object_msg.object.id = goal.target_name;
  detach_object_msg.object.operation = moveit_msgs::CollisionObject::REMOVE;
  planning_scene_after_grasp->processAttachedCollisionObjectMsg(detach_object_msg);
  

  // configure the manipulation pipeline
  pipeline_.reset();


  
  
  pipeline_.start();
  
  pipeline_.stop();


  return error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS;
}

void PlacePlan::foundSolution(void)
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

