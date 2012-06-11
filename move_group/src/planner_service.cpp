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

#include <planning_pipeline/planning_pipeline.h>
#include <planning_scene_monitor/planning_scene_monitor.h>
#include <tf/transform_listener.h>
#include <moveit_msgs/GetMotionPlan.h>

static const std::string NODE_NAME="planner";          // name of node
static const std::string ROBOT_DESCRIPTION="robot_description";      // name of the robot description (a param name, so it can be changed externally)
static const std::string PLANNER_SERVICE_NAME="plan_kinematic_path"; // name of the advertised service (within the ~ namespace)

class PlannerService
{
public:
  
  PlannerService(planning_scene_monitor::PlanningSceneMonitor &psm) :
    nh_("~"), psm_(psm), planning_pipeline_(psm.getPlanningScene()->getKinematicModel())
  {
    plan_service_ = nh_.advertiseService(PLANNER_SERVICE_NAME, &PlannerService::computePlan, this);
  }
  
  bool computePlan(moveit_msgs::GetMotionPlan::Request &req, moveit_msgs::GetMotionPlan::Response &res)
  {
    ROS_INFO("Received new planning request...");
    return planning_pipeline_.generatePlan(psm_.getPlanningScene(), req, res);
  }
  
  void status(void)
  {
    ROS_INFO("Responding to planning and bechmark requests");
  }
  
private:
  
  ros::NodeHandle                               nh_;
  planning_scene_monitor::PlanningSceneMonitor &psm_;  
  planning_pipeline::PlanningPipeline           planning_pipeline_;
  ros::ServiceServer                            plan_service_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, NODE_NAME, ros::init_options::AnonymousName);
  
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  boost::shared_ptr<tf::TransformListener> tf(new tf::TransformListener());
  planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION, tf);
  if (psm.getPlanningScene() && psm.getPlanningScene()->isConfigured())
  {
    psm.startWorldGeometryMonitor();
    psm.startSceneMonitor();
    psm.startStateMonitor();
    
    PlannerService pservice(psm);
    pservice.status();
    ros::waitForShutdown();
  }
  else
    ROS_ERROR("Planning scene not configured");  

  return 0;
}
