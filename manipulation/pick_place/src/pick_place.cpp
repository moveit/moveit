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
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen_conversions/eigen_msg.h>
#include <ros/console.h>

namespace pick_place
{

const std::string PickPlace::DISPLAY_PATH_TOPIC = planning_pipeline::PlanningPipeline::DISPLAY_PATH_TOPIC;
const std::string PickPlace::DISPLAY_GRASP_TOPIC = "display_grasp_markers";
const double PickPlace::DEFAULT_GRASP_POSTURE_COMPLETION_DURATION = 7.0; // seconds

// functionality specific to pick-only is in pick.cpp;
// functionality specific to place-only is in place.cpp;

PickPlace::PickPlace(const planning_pipeline::PlanningPipelinePtr &planning_pipeline) :
  nh_("~"),
  planning_pipeline_(planning_pipeline),
  display_computed_motion_plans_(false),
  display_grasps_(false)
{
  constraint_sampler_manager_loader_.reset(new constraint_sampler_manager_loader::ConstraintSamplerManagerLoader());
}

void PickPlace::displayProcessedGrasps(bool flag)
{
  if (display_grasps_ && !flag)
    grasps_publisher_.shutdown();
  else
    if (!display_grasps_ && flag)
      grasps_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(DISPLAY_GRASP_TOPIC, 10, true);
  display_grasps_ = flag;
}

void PickPlace::displayComputedMotionPlans(bool flag)
{
  if (display_computed_motion_plans_ && !flag)
    display_path_publisher_.shutdown();
  else
    if (!display_computed_motion_plans_ && flag)
      display_path_publisher_ = nh_.advertise<moveit_msgs::DisplayTrajectory>(DISPLAY_PATH_TOPIC, 10, true);
  display_computed_motion_plans_ = flag;
}

void PickPlace::visualizePlan(const ManipulationPlanPtr &plan) const
{ 
  moveit_msgs::DisplayTrajectory dtraj;
  dtraj.model_id = getRobotModel()->getName();
  if (!plan->trajectories_.empty())
  {
    robot_state::robotStateToRobotStateMsg(plan->trajectories_.front()->getFirstWayPoint(), dtraj.trajectory_start);
    dtraj.trajectory.resize(plan->trajectories_.size());
    for (std::size_t i = 0 ; i < plan->trajectories_.size() ; ++i)
      plan->trajectories_[i]->getRobotTrajectoryMsg(dtraj.trajectory[i]);
  }
  display_path_publisher_.publish(dtraj);
}

void PickPlace::visualizeGrasp(const ManipulationPlanPtr &plan) const
{
  visualizeGrasps(std::vector<ManipulationPlanPtr>(1, plan));
}

namespace
{

std::vector<std_msgs::ColorRGBA> setupDefaultGraspColors()
{
  std::vector<std_msgs::ColorRGBA> result;
  result.resize(6);
  result[0].r = 0.5f; result[0].g = 0.5f; result[0].b = 0.5f; result[0].a = 1.0f;
  result[1].r = 1.0f; result[1].g = 0.0f; result[1].b = 0.0f; result[1].a = 1.0f;
  result[2].r = 1.0f; result[2].g = 0.5f; result[2].b = 0.0f; result[2].a = 1.0f;
  result[3].r = 0.0f; result[3].g = 1.0f; result[3].b = 1.0f; result[3].a = 1.0f;
  result[4].r = 0.0f; result[4].g = 1.0f; result[4].b = 0.0f; result[4].a = 1.0f;
  result[5].r = 1.0f; result[5].g = 0.0f; result[5].b = 1.0f; result[5].a = 0.75f;
  return result;
}

}
  
void PickPlace::visualizeGrasps(const std::vector<ManipulationPlanPtr>& plans) const
{
  if (plans.empty())
    return;
  
  robot_state::RobotState state(getRobotModel());
  state.setToDefaultValues();
  
  static std::vector<std_msgs::ColorRGBA> colors(setupDefaultGraspColors());
  visualization_msgs::MarkerArray ma;
  for (std::size_t i = 0 ; i < plans.size() ; ++i)
    {
    const robot_model::JointModelGroup *jmg = getRobotModel()->getJointModelGroup(plans[i]->end_effector_group_);
    if (jmg)
    {
      unsigned int type = std::min(plans[i]->processing_stage_, colors.size() - 1);
      Eigen::Affine3d eigen_pose;
      tf::poseMsgToEigen(plans[i]->grasp_.grasp_pose, eigen_pose);
      state.updateStateWithLinkAt(plans[i]->ik_link_name_, eigen_pose);
      state.getRobotMarkers(ma, jmg->getLinkModelNames(), colors[type], "moveit_grasps:stage_" + boost::lexical_cast<std::string>(plans[i]->processing_stage_), ros::Duration(60));
    }
  }
  
  grasps_publisher_.publish(ma);
}
  
}
