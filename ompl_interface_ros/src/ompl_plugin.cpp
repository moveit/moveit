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

namespace ompl_interface_ros
{

class OMPLPlanner : public planning_interface::Planner
{
  public:
    void init(const planning_models::KinematicModelConstPtr& model)
    {
      ompl_interface_.reset(new OMPLInterfaceROS(model));
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
      bool result = ompl_interface_->solve(planning_scene, req, res);
      return result;
    }

    std::string getDescription(void) const { return "OMPL Interface"; }

    void terminate(void) const
    {
      ompl_interface_->terminateSolve();
    }
  
  private:
    boost::shared_ptr<OMPLInterfaceROS> ompl_interface_;
};

} // ompl_interface_ros

PLUGINLIB_DECLARE_CLASS(ompl_interface_ros, OMPLPlanner,
                        ompl_interface_ros::OMPLPlanner, 
                        planning_interface::Planner);
