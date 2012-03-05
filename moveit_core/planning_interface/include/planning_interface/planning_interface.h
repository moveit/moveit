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

#ifndef PLANNING_INTERFACE_PLANNING_INTERFACE_H
#define PLANNING_INTERFACE_PLANNING_INTERFACE_H

#include <planning_scene/planning_scene.h>
#include <planning_models/kinematic_model.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/MotionPlanDetailedResponse.h>
#include <string>

namespace planning_interface
{

// This struct will change a lot over time as we figure out what's
// needed.
struct PlannerCapability
{
  PlannerCapability(void)
  {
  }

  /*
  bool can_plan_from_collision;
  bool can_plan_to_collision;
  bool is_anytime;
  bool can_handle_goal_constraints;
  bool can_handle_path_constraints;
  bool needs_distance_field;
  */
};

class Planner
{
  public:
    Planner() {}
    virtual ~Planner() {};

    /// Subclass may implement methods below
    virtual void init(const planning_models::KinematicModelConstPtr& model) {}
    
    /// 
    virtual std::string getDescription(void) const { return ""; }
    
    /// Subclass must implement methods below
    virtual bool solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                       const moveit_msgs::GetMotionPlan::Request &req, 
                       moveit_msgs::GetMotionPlan::Response &res) const = 0;

    virtual bool solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
		       const moveit_msgs::GetMotionPlan::Request &req, 
		       moveit_msgs::MotionPlanDetailedResponse &res) const = 0;

    /// Determine whether this plugin instance is able to represent this planning request
    virtual bool canServiceRequest(const moveit_msgs::GetMotionPlan::Request &req,
                                   PlannerCapability& capabilities) const = 0;

    /// Request termination, if a solve() function is currently computing plans
    virtual void terminate(void) const = 0;
  
};

} // planning_interface

#endif // PLANNING_INTERFACE_PLANNING_INTERFACE_H
