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

static const std::string PLANNER_NODE_NAME="ompl_planning";          // name of node
static const std::string PLANNER_SERVICE_NAME="plan_kinematic_path"; // name of the advertised service (within the ~ namespace)
static const std::string BENCHMARK_SERVICE_NAME="benchmark_planning_problem"; // name of the advertised service (within the ~ namespace)
static const std::string ROBOT_DESCRIPTION="robot_description";      // name of the robot description (a param name, so it can be changed externally)

class OMPLPlannerService
{
public:
  
  OMPLPlannerService(planning_scene_monitor::PlanningSceneMonitor &psm) : nh_("~"), psm_(psm), ompl_interface_(psm.getPlanningScene()->getKinematicModel())
  {
    plan_service_ = nh_.advertiseService(PLANNER_SERVICE_NAME, &OMPLPlannerService::computePlan, this);
    benchmark_service_ = nh_.advertiseService(BENCHMARK_SERVICE_NAME, &OMPLPlannerService::computeBenchmark, this);  
    pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 5);
    pub_plan_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("display_motion_plan", 100);
  }
  
  bool computePlan(moveit_msgs::GetMotionPlan::Request &req, moveit_msgs::GetMotionPlan::Response &res)
  {
    ROS_INFO("Received new planning request...");
    bool result = ompl_interface_.solve(psm_.getPlanningScene(), req, res);
    if (result)
      displaySolution(res);
    displayPlannerData("r_wrist_roll_link");
    std::stringstream ss;
    ompl::tools::Profiler::Status(ss);
    ROS_INFO("%s", ss.str().c_str());
    return result;
  }

  void displaySolution(const moveit_msgs::GetMotionPlan::Response &mplan_res)
  {
    moveit_msgs::DisplayTrajectory d;
    d.model_id = psm_.getPlanningScene()->getKinematicModel()->getName();
    d.robot_state = mplan_res.robot_state;
    d.trajectory = mplan_res.trajectory;
    pub_plan_.publish(d);
  }
  
  void displayRandomPaths(void)
  {
    
    const ompl_interface::ModelBasedPlanningContextPtr &pc = ompl_interface_.getLastPlanningContext();

    //    std::cout << pc->getOMPLSimpleSetup().getSpaceInformation()->probabilityOfValidState(1000) << std::endl;
    
    
    planning_models::KinematicState ks(pc->getCompleteInitialRobotState());
    ompl::base::ScopedState<> s(pc->getOMPLStateSpace());
    ompl::base::ScopedState<> s0(s);
    bool first = true;
    for (int i = 0 ; i < 120 ; ++i)
    {
      ROS_INFO("i = %d", i);
      s.random();
      if (!pc->getOMPLSimpleSetup().getSpaceInformation()->isValid(s.get()))
        continue;
      if (!first)
      {
        ompl::geometric::PathGeometric pg(pc->getOMPLSimpleSetup().getSpaceInformation(), s0.get(), s.get());
        pg.interpolate(3);
        
        if (pc->getOMPLSimpleSetup().getSpaceInformation()->checkMotion(s0.get(), s.get()))
          ROS_INFO("Valid Path");
        else
          ROS_INFO("Invalid Path");

        pc->setVerboseStateValidityChecks(true);
        for (std::size_t i = 0 ; i < pg.getStateCount() ; ++i)
        {
          const ompl_interface::ModelBasedStateSpace::StateType *s = pg.getState(i)->as<ompl_interface::ModelBasedStateSpace::StateType>();
          pc->getOMPLSimpleSetup().getSpaceInformation()->isValid(s);
        }
        pc->setVerboseStateValidityChecks(false);
        ROS_INFO("\n");
        pg.interpolate(30);
        moveit_msgs::DisplayTrajectory d;
        d.model_id = psm_.getPlanningScene()->getKinematicModel()->getName();
        planning_models::kinematicStateToRobotState(pc->getCompleteInitialRobotState(), d.robot_state);
        pc->convertPath(pg, d.trajectory);
        pub_plan_.publish(d);
        ros::Duration(5.0).sleep();
      }
      first = false;
      s0 = s;
      pc->getOMPLStateSpace()->copyToKinematicState(ks, s.get());
      moveit_msgs::DisplayTrajectory d;
      d.model_id = psm_.getPlanningScene()->getKinematicModel()->getName();
      planning_models::kinematicStateToRobotState(ks, d.robot_state);
      pub_plan_.publish(d);  
      ros::Duration(0.5).sleep();
    }

    /*
    boost::shared_ptr<kinematics_plugin_loader::KinematicsPluginLoader> kpl = ompl_interface_.getKinematicsPluginLoader();
    kinematics_plugin_loader::KinematicsLoaderFn kinematics_allocator = kpl->getLoaderFunction();
    kinematic_constraints::KinematicsSubgroupAllocator sa;
    sa[psm_.getPlanningScene()->getKinematicModel()->getJointModelGroup("left_arm")] = kinematics_allocator;
    sa[psm_.getPlanningScene()->getKinematicModel()->getJointModelGroup("right_arm")] = kinematics_allocator;
    
    kinematic_constraints::ConstraintSamplerPtr smp = kinematic_constraints::constructConstraintsSampler
      (psm_.getPlanningScene()->getKinematicModel()->getJointModelGroup("arms"),  pc->getPathConstraints()->getAllConstraints(),
       psm_.getPlanningScene()->getKinematicModel(), psm_.getPlanningScene()->getTransforms(), kinematic_constraints::KinematicsAllocator(), sa);

    for (int i = 0 ; i < 1000 ; ++i)
    {
      ROS_INFO("j = %d", i);
      std::vector<double> values;
      if (smp->sample(values, ks, 10))
      {
        ks.getJointStateGroup("arms")->setStateValues(values);
        moveit_msgs::DisplayTrajectory d;
        d.model_id = psm_.getPlanningScene()->getKinematicModel()->getName();
        planning_models::kinematicStateToRobotState(ks, d.robot_state);
        pub_plan_.publish(d);  
        ros::Duration(0.5).sleep();
      }
    }
    */

    /*    
    
    for (int i = 0 ; i < 10 ; ++i)
    {
      ROS_INFO("i = %d", i);
      
      ompl::geometric::PathGeometric pg(pc->getOMPLSimpleSetup().getSpaceInformation());
      pg.random(); 
      //      pg.interpolate(20);
      moveit_msgs::DisplayTrajectory d;
      d.model_id = psm_.getPlanningScene()->getKinematicModel()->getName();
      planning_models::kinematicStateToRobotState(pc->getCompleteInitialRobotState(), d.robot_state);
      pc->convertPath(pg, d.trajectory);
      pub_plan_.publish(d);
      //      std::cout << d << std::endl;
      
      ros::Duration(5.0).sleep();
    }
    */
  }
  
  void displayPlannerData(const std::string &link_name)
  {    
    const ompl_interface::ModelBasedPlanningContextPtr &pc = ompl_interface_.getLastPlanningContext();
    if (pc)
    {
      const ompl::base::PlannerData &pd = pc->getOMPLSimpleSetup().getPlannerData();
      planning_models::KinematicState kstate = psm_.getPlanningScene()->getCurrentState();  
      visualization_msgs::MarkerArray arr; 
      std_msgs::ColorRGBA color;
      color.r = 1.0f;
      color.g = 0.0f;
      color.b = 0.0f;
      color.a = 1.0f;
      for (std::size_t i = 0 ; i < pd.states.size() ; ++i)
      {
        pc->getOMPLStateSpace()->copyToKinematicState(kstate, pd.states[i]);
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
      
      planning_scene::PlanningScenePtr scene(new planning_scene::PlanningScene());
      scene->configure(psm_.getPlanningScene()->getUrdfModel(), psm_.getPlanningScene()->getSrdfModel());
      scene->setPlanningSceneMsg(req.scene);
      return ompl_interface_.benchmark(scene, req, res);
  }
  
  void status(void)
  {
    ompl_interface_.printStatus();
    ROS_INFO("Responding to planning and bechmark requests");
  }
  
private:
  
  ros::NodeHandle                               nh_;
  planning_scene_monitor::PlanningSceneMonitor &psm_;  
  ompl_interface_ros::OMPLInterfaceROS          ompl_interface_;
  ros::ServiceServer                            plan_service_;
  ros::ServiceServer                            benchmark_service_;  
  ros::ServiceServer                            display_states_service_;
  ros::Publisher                                pub_markers_;
  ros::Publisher                                pub_plan_;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, PLANNER_NODE_NAME);
  
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  tf::TransformListener tf;
  planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION, &tf);
  if (psm.getPlanningScene()->isConfigured())
  {
    psm.startWorldGeometryMonitor();
    psm.startSceneMonitor();
    psm.startStateMonitor();
    
    OMPLPlannerService pservice(psm);
    pservice.status();
    ros::waitForShutdown();
  }
  else
    ROS_ERROR("Planning scene not configured");
  
  return 0;
}
