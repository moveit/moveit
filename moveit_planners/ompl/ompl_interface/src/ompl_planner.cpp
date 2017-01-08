/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage, Inc.
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
*   * Neither the name of Willow Garage nor the names of its
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

#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/ompl_interface/model_based_planning_context.h>
#include <tf/transform_listener.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/profiler/profiler.h>
#include <moveit_msgs/GetMotionPlan.h>

static const std::string PLANNER_NODE_NAME = "ompl_planning";  // name of node
static const std::string PLANNER_SERVICE_NAME =
    "plan_kinematic_path";  // name of the advertised service (within the ~ namespace)
static const std::string ROBOT_DESCRIPTION =
    "robot_description";  // name of the robot description (a param name, so it can be changed externally)

class OMPLPlannerService
{
public:
  OMPLPlannerService(planning_scene_monitor::PlanningSceneMonitor& psm, bool debug = false)
    : nh_("~"), psm_(psm), ompl_interface_(psm.getPlanningScene()->getRobotModel()), debug_(debug)
  {
    plan_service_ = nh_.advertiseService(PLANNER_SERVICE_NAME, &OMPLPlannerService::computePlan, this);
    if (debug_)
    {
      pub_plan_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("display_motion_plan", 100);
      pub_request_ = nh_.advertise<moveit_msgs::MotionPlanRequest>("motion_plan_request", 100);
    }
  }

  bool computePlan(moveit_msgs::GetMotionPlan::Request& req, moveit_msgs::GetMotionPlan::Response& res)
  {
    ROS_INFO("Received new planning request...");
    if (debug_)
      pub_request_.publish(req.motion_plan_request);
    planning_interface::MotionPlanResponse response;

    ompl_interface::ModelBasedPlanningContextPtr context =
        ompl_interface_.getPlanningContext(psm_.getPlanningScene(), req.motion_plan_request);
    if (!context)
    {
      ROS_ERROR_STREAM_NAMED("computePlan", "No planning context found");
      return false;
    }
    context->clear();

    bool result = context->solve(response);

    if (debug_)
    {
      if (result)
        displaySolution(res.motion_plan_response);
      std::stringstream ss;
      ROS_INFO("%s", ss.str().c_str());
    }
    return result;
  }

  void displaySolution(const moveit_msgs::MotionPlanResponse& mplan_res)
  {
    moveit_msgs::DisplayTrajectory d;
    d.model_id = psm_.getPlanningScene()->getRobotModel()->getName();
    d.trajectory_start = mplan_res.trajectory_start;
    d.trajectory.resize(1, mplan_res.trajectory);
    pub_plan_.publish(d);
  }

  void status()
  {
    ompl_interface_.printStatus();
    ROS_INFO("Responding to planning and bechmark requests");
    if (debug_)
      ROS_INFO("Publishing debug information");
  }

private:
  ros::NodeHandle nh_;
  planning_scene_monitor::PlanningSceneMonitor& psm_;
  ompl_interface::OMPLInterface ompl_interface_;
  ros::ServiceServer plan_service_;
  ros::ServiceServer display_states_service_;
  ros::Publisher pub_plan_;
  ros::Publisher pub_request_;
  bool debug_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, PLANNER_NODE_NAME);

  bool debug = false;
  for (int i = 1; i < argc; ++i)
    if (strncmp(argv[i], "--debug", 7) == 0)
      debug = true;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  boost::shared_ptr<tf::TransformListener> tf(new tf::TransformListener());
  planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION, tf);
  if (psm.getPlanningScene())
  {
    psm.startWorldGeometryMonitor();
    psm.startSceneMonitor();
    psm.startStateMonitor();

    OMPLPlannerService pservice(psm, debug);
    pservice.status();
    ros::waitForShutdown();
  }
  else
    ROS_ERROR("Planning scene not configured");

  return 0;
}
