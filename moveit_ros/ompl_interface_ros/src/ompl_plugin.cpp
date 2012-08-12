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

#include "ompl_interface_ros/ompl_interface_ros.h"
#include <planning_interface/planning_interface.h>
#include <planning_scene/planning_scene.h>
#include <planning_models/kinematic_model.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <boost/shared_ptr.hpp>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/String.h>

namespace ompl_interface_ros
{

class OMPLPlanner : public planning_interface::Planner
{
public:

    OMPLPlanner(void) : planning_interface::Planner(),
			nh_("~")
    {
    }

    void init(const planning_models::KinematicModelConstPtr& model) 
    {
      ompl_interface_.reset(new OMPLInterfaceROS(model));
      pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("ompl_planner_data_marker_array", 5);
      debug_link_sub_ = nh_.subscribe("ompl_planner_data_link", 1, &OMPLPlanner::newLinkDisplayCallback, this);
    }

    bool canServiceRequest(const moveit_msgs::GetMotionPlan::Request &req,
                           planning_interface::PlannerCapability &capabilities) const
    {
      // TODO: this is a dummy implementation
	//      capabilities.dummy = false;
      return true;
    }

    bool solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
               const moveit_msgs::GetMotionPlan::Request &req, 
               moveit_msgs::GetMotionPlan::Response &res) const
    {
      bool r = ompl_interface_->solve(planning_scene, req, res);
      if (!planner_data_link_name_.empty())
	displayPlannerData(planning_scene, planner_data_link_name_);
      return r;
    }

    bool solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
	       const moveit_msgs::GetMotionPlan::Request &req, 
	       moveit_msgs::MotionPlanDetailedResponse &res) const
    {
      bool r = ompl_interface_->solve(planning_scene, req, res);
      if (!planner_data_link_name_.empty())
	displayPlannerData(planning_scene, planner_data_link_name_);
      return r;
    }

    std::string getDescription(void) const { return "OMPL"; }

    void getPlanningAlgorithms(std::vector<std::string> &algs) const
    {
      const std::map<std::string, ompl_interface::PlanningConfigurationSettings> &pconfig = 
        ompl_interface_->getPlanningContextManager().getPlanningConfigurations();
      algs.clear();
      for (std::map<std::string, ompl_interface::PlanningConfigurationSettings>::const_iterator it = pconfig.begin() ; 
           it != pconfig.end() ; ++it)
        algs.push_back(it->first);
    }

    void terminate(void) const
    {
      ompl_interface_->terminateSolve();
    }

private:

    void newLinkDisplayCallback(const std_msgs::StringConstPtr &msg)
    {
      planner_data_link_name_ = msg->data;
      if (planner_data_link_name_.empty())
	ROS_INFO("Not displaying OMPL exploration data structures.");
      else
	ROS_INFO("Displaying OMPL exploration data structures for %s", planner_data_link_name_.c_str());
    }

    void displayPlannerData(const planning_scene::PlanningSceneConstPtr& planning_scene,
			    const std::string &link_name) const
    {    
      const ompl_interface::ModelBasedPlanningContextPtr &pc = ompl_interface_->getLastPlanningContext();
      if (pc)
      {
	ompl::base::PlannerData pd(pc->getOMPLSimpleSetup().getSpaceInformation());
	pc->getOMPLSimpleSetup().getPlannerData(pd);
	planning_models::KinematicState kstate = planning_scene->getCurrentState();  
	visualization_msgs::MarkerArray arr; 
	std_msgs::ColorRGBA color;
	color.r = 1.0f;
	color.g = 0.25f;
	color.b = 1.0f;
	color.a = 1.0f;
	unsigned int nv = pd.numVertices();
	for (unsigned int i = 0 ; i < nv ; ++i)
	{
	  pc->getOMPLStateSpace()->copyToKinematicState(kstate, pd.getVertex(i).getState());
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

  ros::NodeHandle nh_;
  boost::shared_ptr<OMPLInterfaceROS> ompl_interface_;
  ros::Publisher pub_markers_;
  ros::Subscriber debug_link_sub_;
  std::string planner_data_link_name_;
};

} // ompl_interface_ros

PLUGINLIB_DECLARE_CLASS(ompl_interface_ros, OMPLPlanner,
                        ompl_interface_ros::OMPLPlanner, 
                        planning_interface::Planner);
