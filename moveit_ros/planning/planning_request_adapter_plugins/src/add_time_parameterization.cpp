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

#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <moveit/trajectory_processing/iterative_smoother.h>
#include <class_loader/class_loader.h>
#include <ros/console.h>

namespace default_planner_request_adapters
{

class AddTimeParameterization : public planning_request_adapter::PlanningRequestAdapter
{
public:
  
  AddTimeParameterization(void) : planning_request_adapter::PlanningRequestAdapter()
  {
  }
  
  virtual std::string getDescription(void) const { return "Add Time Parameterization"; }
  
  virtual bool adaptAndPlan(const planning_request_adapter::PlannerFn &planner,
                            const planning_scene::PlanningSceneConstPtr& planning_scene,
                            const moveit_msgs::GetMotionPlan::Request &req, 
                            moveit_msgs::GetMotionPlan::Response &res,
                            std::vector<std::size_t> &added_path_index) const
  { 
    bool result = planner(planning_scene, req, res);
    if (result)
    {  
      ROS_DEBUG("Running '%s'", getDescription().c_str());
      trajectory_msgs::JointTrajectory trajectory_out;
      const kinematic_model::JointModelGroup *jmg = planning_scene->getKinematicModel()->getJointModelGroup(req.motion_plan_request.group_name);
      if (jmg)
      {
        const std::vector<moveit_msgs::JointLimits> &jlim = jmg->getVariableLimits();
        smoother_.smooth(res.trajectory.joint_trajectory, trajectory_out, jlim, req.motion_plan_request.start_state);
        res.trajectory.joint_trajectory = trajectory_out;
      }
    }
    
    return result;
  }   
  
private:
  
  trajectory_processing::IterativeParabolicSmoother smoother_;
};

}

CLASS_LOADER_REGISTER_CLASS(default_planner_request_adapters::AddTimeParameterization,
                            planning_request_adapter::PlanningRequestAdapter);
