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
#include <moveit/robot_state/conversions.h>
#include <class_loader/class_loader.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <ros/ros.h>

namespace default_planner_request_adapters
{

class FixStartStatePathConstraints : public planning_request_adapter::PlanningRequestAdapter
{
public:

  FixStartStatePathConstraints() : planning_request_adapter::PlanningRequestAdapter()
  {
  }
  
  virtual std::string getDescription() const { return "Fix Start State Path Constraints"; }
  
  
  virtual bool adaptAndPlan(const PlannerFn &planner,
                            const planning_scene::PlanningSceneConstPtr& planning_scene,
                            const planning_interface::MotionPlanRequest &req, 
                            planning_interface::MotionPlanResponse &res,
                            std::vector<std::size_t> &added_path_index) const
  {
    ROS_DEBUG("Running '%s'", getDescription().c_str());
    
    // get the specified start state
    robot_state::RobotState start_state = planning_scene->getCurrentState();
    robot_state::robotStateMsgToRobotState(*planning_scene->getTransforms(), req.start_state, start_state);
    
    // if the start state is otherwise valid but does not meet path constraints
    if (planning_scene->isStateValid(start_state) && 
        !planning_scene->isStateValid(start_state, req.path_constraints))
    {
      ROS_DEBUG("Planning to path constraints...");
      
      planning_interface::MotionPlanRequest req2 = req;
      req2.goal_constraints.resize(1);
      req2.goal_constraints[0] = req.path_constraints;
      req2.path_constraints = moveit_msgs::Constraints();
      planning_interface::MotionPlanResponse res2;
      bool solved1 = planner(planning_scene, req2, res2);

      if (solved1)
      { 
        planning_interface::MotionPlanRequest req3 = req;
        ROS_DEBUG("Planned to path constraints. Resuming original planning request.");
        
        // extract the last state of the computed motion plan and set it as the new start state
        robot_state::robotStateToRobotStateMsg(res2.trajectory_->getLastWayPoint(), req3.start_state);
        bool solved2 = planner(planning_scene, req3, res);
        res.planning_time_ += res2.planning_time_;
        
        if (solved2)
        {
          // we need to append the solution paths. 
          res.trajectory_->append(*res2.trajectory_, 0.0);
          return true;
        }
        else
          return false;
      }
      else
      { 
        ROS_WARN("Unable to plan to path constraints. Running usual motion plan.");
        bool result = planner(planning_scene, req, res);
        res.planning_time_ += res2.planning_time_;
        return result;
      }
    }
    else
    {
      ROS_DEBUG("Path constraints are OK. Running usual motion plan.");
      return planner(planning_scene, req, res);
    }
  }

};

}

CLASS_LOADER_REGISTER_CLASS(default_planner_request_adapters::FixStartStatePathConstraints,
                            planning_request_adapter::PlanningRequestAdapter);
