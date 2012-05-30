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

/* Author: Ioan Sucan, Sachin Chitta */

#include "ompl_interface_ros/ompl_interface_ros.h"
#include "planning_scene_monitor/planning_scene_monitor.h"
#include <planning_models/conversions.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <ompl/tools/debug/Profiler.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/ComputePlanningBenchmark.h>
#include <moveit_msgs/ConstructConstraintApproximation.h>

static const std::string PLANNER_NODE_NAME="ompl_planning";          // name of node
static const std::string PLANNER_SERVICE_NAME="plan_kinematic_path"; // name of the advertised service (within the ~ namespace)
static const std::string BENCHMARK_SERVICE_NAME="benchmark_planning_problem"; // name of the advertised service (within the ~ namespace)
static const std::string CONSTRUCT_CONSTRAINT_APPROXIMATION_SERVICE_NAME="construct_constraint_approximation"; // name of the advertised service (within the ~ namespace)
static const std::string ROBOT_DESCRIPTION="robot_description";      // name of the robot description (a param name, so it can be changed externally)

class OMPLPlannerService
{
public:
  
  OMPLPlannerService(planning_scene_monitor::PlanningSceneMonitor &psm, bool debug = false) :
    nh_("~"), psm_(psm), ompl_interface_(psm.getPlanningScene()->getKinematicModel()), debug_(debug)
  {
    plan_service_ = nh_.advertiseService(PLANNER_SERVICE_NAME, &OMPLPlannerService::computePlan, this);
    benchmark_service_ = nh_.advertiseService(BENCHMARK_SERVICE_NAME, &OMPLPlannerService::computeBenchmark, this);
    construct_ca_service_ = nh_.advertiseService(CONSTRUCT_CONSTRAINT_APPROXIMATION_SERVICE_NAME, &OMPLPlannerService::constructConstraintApproximation, this);
    if (debug_)
    {
      pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 5);
      pub_plan_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("display_motion_plan", 100);
      pub_request_ = nh_.advertise<moveit_msgs::MotionPlanRequest>("motion_plan_request", 100);
    }
  }
  
  bool computePlan(moveit_msgs::GetMotionPlan::Request &req, moveit_msgs::GetMotionPlan::Response &res)
  {
    ROS_INFO("Received new planning request...");
    if (debug_)
      pub_request_.publish(req.motion_plan_request);
    bool result = ompl_interface_.solve(psm_.getPlanningScene(), req, res);
    if (debug_)
    {
      if (result)
        displaySolution(res);
      std::stringstream ss;
      ompl::tools::Profiler::Status(ss);
      ROS_INFO("%s", ss.str().c_str());
    }
    return result;
  }

  void displaySolution(const moveit_msgs::GetMotionPlan::Response &mplan_res)
  {
    moveit_msgs::DisplayTrajectory d;
    d.model_id = psm_.getPlanningScene()->getKinematicModel()->getName();
    d.trajectory_start = mplan_res.trajectory_start;
    d.trajectory = mplan_res.trajectory;
    pub_plan_.publish(d);
  }
  
  void displayPlannerData(const std::string &link_name)
  {    
    const ompl_interface::ModelBasedPlanningContextPtr &pc = ompl_interface_.getLastPlanningContext();
    if (pc)
    {
      ompl::base::PlannerData pd(pc->getOMPLSimpleSetup().getSpaceInformation());
      pc->getOMPLSimpleSetup().getPlannerData(pd);
      planning_models::KinematicState kstate = psm_.getPlanningScene()->getCurrentState();  
      visualization_msgs::MarkerArray arr; 
      std_msgs::ColorRGBA color;
      color.r = 1.0f;
      color.g = 0.0f;
      color.b = 0.0f;
      color.a = 1.0f;
      unsigned int nv = pd.numVertices();
      for (unsigned int i = 0 ; i < nv ; ++i)
      {
	pc->getOMPLStateSpace()->copyToKinematicState(kstate, pd.getVertex(i).getState());
        kstate.getJointStateGroup(pc->getJointModelGroupName())->updateLinkTransforms();
        const Eigen::Vector3d &pos = kstate.getLinkState(link_name)->getGlobalLinkTransform().translation();
        
        visualization_msgs::Marker mk;
        mk.header.stamp = ros::Time::now();
        mk.header.frame_id = psm_.getPlanningScene()->getPlanningFrame();
        mk.ns = "planner_data";
        mk.id = i;
        mk.type = visualization_msgs::Marker::SPHERE;
        mk.action = visualization_msgs::Marker::ADD;
        mk.pose.position.x = pos.x();
        mk.pose.position.y = pos.y();
        mk.pose.position.z = pos.z();
        mk.pose.orientation.w = 1.0;
        mk.scale.x = mk.scale.y = mk.scale.z = 0.035;
        mk.color = color;
        mk.lifetime = ros::Duration(30.0);
        arr.markers.push_back(mk);
      }
      pub_markers_.publish(arr); 
    }
  }
  
  bool computeBenchmark(moveit_msgs::ComputePlanningBenchmark::Request &req, moveit_msgs::ComputePlanningBenchmark::Response &res)
  {
      ROS_INFO("Received new benchmark request...");
      return ompl_interface_.benchmark(psm_.getPlanningScene(), req, res);
  }

  bool constructConstraintApproximation(moveit_msgs::ConstructConstraintApproximation::Request &req, moveit_msgs::ConstructConstraintApproximation::Response &res)
  {
    planning_models::KinematicState kstate(psm_.getPlanningScene()->getCurrentState());
    planning_models::robotStateToKinematicState(*psm_.getPlanningScene()->getTransforms(), req.start_state, kstate);
    ompl_interface::ConstraintApproximationConstructionResults ca_res = 
      ompl_interface_.getConstraintsLibrary().addConstraintApproximation(req.constraint, req.group, req.state_space_parameterization,
                                                                         kstate, req.samples, req.edges_per_sample);
    if (ca_res.approx)
    {
      res.sampling_success_rate = ca_res.sampling_success_rate;
      res.state_sampling_time = ca_res.state_sampling_time;
      res.state_connection_time = ca_res.state_connection_time;
      res.filename = ca_res.approx->getFilename();
      return ompl_interface_.saveConstraintApproximations();
    }
    else
      return false;
  }
  
  void status(void)
  {
    ompl_interface_.printStatus();
    ROS_INFO("Responding to planning and bechmark requests");
    if (debug_)
      ROS_INFO("Publishing debug information");
  }
  
private:
  
  ros::NodeHandle                               nh_;
  planning_scene_monitor::PlanningSceneMonitor &psm_;  
  ompl_interface_ros::OMPLInterfaceROS          ompl_interface_;
  ros::ServiceServer                            plan_service_;
  ros::ServiceServer                            benchmark_service_;  
  ros::ServiceServer                            construct_ca_service_;  
  ros::ServiceServer                            display_states_service_;
  ros::Publisher                                pub_markers_;
  ros::Publisher                                pub_plan_;
  ros::Publisher                                pub_request_;
  bool                                          debug_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, PLANNER_NODE_NAME);
  
  bool debug = false;
  for (int i = 1 ; i < argc ; ++i)
    if (strncmp(argv[i], "--debug", 7) == 0)
      debug = true;
  
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  boost::shared_ptr<tf::TransformListener> tf(new tf::TransformListener());
  planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION, tf);
  if (psm.getPlanningScene() && psm.getPlanningScene()->isConfigured())
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
