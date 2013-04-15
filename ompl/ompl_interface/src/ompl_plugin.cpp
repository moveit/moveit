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

/* Author: Ioan Sucan */

#include <moveit/ompl_interface/ompl_interface_ros.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <boost/shared_ptr.hpp>
#include <class_loader/class_loader.h>

#include <dynamic_reconfigure/server.h>
#include "moveit_ompl_planners/OMPLDynamicReconfigureConfig.h"

namespace ompl_interface
{
using namespace moveit_ompl_planners;

class OMPLPlanner : public planning_interface::Planner
{
public:
  
  OMPLPlanner() : planning_interface::Planner(),
                  nh_("~")
  {
  }
  
  virtual bool initialize(const robot_model::RobotModelConstPtr& model, const std::string &ns) 
  {
    if (!ns.empty())
      nh_ = ros::NodeHandle(ns);
    ompl_interface_.reset(new OMPLInterfaceROS(model, nh_));
    pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("ompl_planner_data_marker_array", 5);
    
    dynamic_reconfigure_server_.reset(new dynamic_reconfigure::Server<OMPLDynamicReconfigureConfig>(ros::NodeHandle(nh_, "ompl")));
    dynamic_reconfigure_server_->setCallback(boost::bind(&OMPLPlanner::dynamicReconfigureCallback, this, _1, _2));
    return true;
  }
  
  bool canServiceRequest(const moveit_msgs::MotionPlanRequest &req) const
  {
    return true;
  }
  
  virtual bool solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                     const planning_interface::MotionPlanRequest &req, 
                     planning_interface::MotionPlanResponse &res) const
  {
    bool r = ompl_interface_->solve(planning_scene, req, res);
    if (!planner_data_link_name_.empty())
      displayPlannerData(planning_scene, planner_data_link_name_);
    return r;
  }
  
  virtual bool solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                     const planning_interface::MotionPlanRequest &req, 
                     planning_interface::MotionPlanDetailedResponse &res) const
  {
    bool r = ompl_interface_->solve(planning_scene, req, res);
    if (!planner_data_link_name_.empty())
      displayPlannerData(planning_scene, planner_data_link_name_);
    return r;
  }
  
  std::string getDescription() const { return "OMPL"; }
  
  void getPlanningAlgorithms(std::vector<std::string> &algs) const
  {
    const std::map<std::string, ompl_interface::PlanningConfigurationSettings> &pconfig = 
      ompl_interface_->getPlanningContextManager().getPlanningConfigurations();
    algs.clear();
    for (std::map<std::string, ompl_interface::PlanningConfigurationSettings>::const_iterator it = pconfig.begin() ; 
         it != pconfig.end() ; ++it)
      algs.push_back(it->first);
  }
  
  void terminate() const
  {
    ompl_interface_->terminateSolve();
  }
  
private:
  
  void displayPlannerData(const planning_scene::PlanningSceneConstPtr& planning_scene,
                          const std::string &link_name) const
  {    
    const ompl_interface::ModelBasedPlanningContextPtr &pc = ompl_interface_->getLastPlanningContext();
    if (pc)
    {
      ompl::base::PlannerData pd(pc->getOMPLSimpleSetup().getSpaceInformation());
      pc->getOMPLSimpleSetup().getPlannerData(pd);
      robot_state::RobotState kstate = planning_scene->getCurrentState();  
      visualization_msgs::MarkerArray arr; 
      std_msgs::ColorRGBA color;
      color.r = 1.0f;
      color.g = 0.25f;
      color.b = 1.0f;
      color.a = 1.0f;
      unsigned int nv = pd.numVertices();
      for (unsigned int i = 0 ; i < nv ; ++i)
      {
        pc->getOMPLStateSpace()->copyToRobotState(kstate, pd.getVertex(i).getState());
        kstate.getJointStateGroup(pc->getJointModelGroupName())->updateLinkTransforms();
        const Eigen::Vector3d &pos = kstate.getLinkState(link_name)->getGlobalLinkTransform().translation();
	
        visualization_msgs::Marker mk;
        mk.header.stamp = ros::Time::now();
        mk.header.frame_id = planning_scene->getPlanningFrame();
        mk.ns = "planner_data";
        mk.id = i;
        mk.type = visualization_msgs::Marker::SPHERE;
        mk.action = visualization_msgs::Marker::ADD;
        mk.pose.position.x = pos.x();
        mk.pose.position.y = pos.y();
        mk.pose.position.z = pos.z();
        mk.pose.orientation.w = 1.0;
        mk.scale.x = mk.scale.y = mk.scale.z = 0.025;
        mk.color = color;
        mk.lifetime = ros::Duration(30.0);
        arr.markers.push_back(mk);
      }
      pub_markers_.publish(arr); 
    }
  }
  
  void dynamicReconfigureCallback(OMPLDynamicReconfigureConfig &config, uint32_t level)
  {
    planner_data_link_name_ = config.link_for_exploration_tree;
    if (planner_data_link_name_.empty())
      ROS_INFO("Not displaying OMPL exploration data structures.");
    else
      ROS_INFO("Displaying OMPL exploration data structures for %s", planner_data_link_name_.c_str());
  }
  
  ros::NodeHandle nh_;
  boost::scoped_ptr<dynamic_reconfigure::Server<OMPLDynamicReconfigureConfig> > dynamic_reconfigure_server_;
  boost::scoped_ptr<OMPLInterfaceROS> ompl_interface_;
  ros::Publisher pub_markers_;
  ros::Subscriber debug_link_sub_;
  std::string planner_data_link_name_;
};

} // ompl_interface

CLASS_LOADER_REGISTER_CLASS(ompl_interface::OMPLPlanner, planning_interface::Planner);
