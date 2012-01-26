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

#include "ompl_interface_ros/ompl_interface_ros.h"
#include "planning_scene_monitor/planning_scene_monitor.h"
#include <tf/transform_listener.h>
#include <moveit_msgs/ComputeConstraintSpaceApproximation.h>

static const std::string DBCONSTR_SERVICE_NAME="/compute_space_approximation";
static const std::string ROBOT_DESCRIPTION="robot_description";

class ComputeConstraintSpaceApproximation
{
public:
    ComputeConstraintSpaceApproximation(void) :
	nh_("~"), psm_(ROBOT_DESCRIPTION), ompl_interface_(psm_.getPlanningScene())
    {
	ompl_interface_.printStatus();
	service_ = nh_.advertiseService(DBCONSTR_SERVICE_NAME, &ComputeConstraintSpaceApproximation::computeDB, this);
    }

    bool computeDB(moveit_msgs::ComputeConstraintSpaceApproximation::Request &req, moveit_msgs::ComputeConstraintSpaceApproximation::Response &res)
    {   
	ompl_interface::PlanningGroupPtr pg = ompl_interface_.getPlanningConfiguration(req.group);
	if (!pg)
	    return false;
	if (pg->constructValidStateDatabase(req.constraints, req.size, req.filename.c_str()))
	    res.success = true;
	return true;
    }
    
private:
    
    ros::NodeHandle nh_;
    planning_scene_monitor::PlanningSceneMonitor psm_;  
    ompl_interface_ros::OMPLInterfaceROS ompl_interface_;
    ros::ServiceServer service_;
    
};

    
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ompl_db");
    
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    ComputeConstraintSpaceApproximation ccsa;    
    
    return 0;
}
