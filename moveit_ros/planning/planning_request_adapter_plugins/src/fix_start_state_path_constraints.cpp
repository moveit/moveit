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
#include <moveit/kinematic_state/conversions.h>
#include <class_loader/class_loader.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <ros/ros.h>

namespace default_planner_request_adapters
{

class FixStartStatePathConstraints : public planning_request_adapter::PlanningRequestAdapter
{
public:

  FixStartStatePathConstraints(void) : planning_request_adapter::PlanningRequestAdapter()
  {
  }
  
  virtual std::string getDescription(void) const { return "Fix Start State Path Constraints"; }
  
  
  virtual bool adaptAndPlan(const planning_request_adapter::PlannerFn &planner,
                            const planning_scene::PlanningSceneConstPtr& planning_scene,
                            const moveit_msgs::GetMotionPlan::Request &req, 
                            moveit_msgs::GetMotionPlan::Response &res,
                            std::vector<std::size_t> &added_path_index) const
  {
    ROS_DEBUG("Running '%s'", getDescription().c_str());
    
    // get the specified start state
    kinematic_state::KinematicState start_state = planning_scene->getCurrentState();
    kinematic_state::robotStateToKinematicState(*planning_scene->getTransforms(), req.motion_plan_request.start_state, start_state);
    
    // if the start state is otherwise valid but does not meet path constraints
    if (planning_scene->isStateValid(start_state) && 
        !planning_scene->isStateValid(start_state, req.motion_plan_request.path_constraints))
    {
      ROS_DEBUG("Planning to path constraints...");
      
      moveit_msgs::GetMotionPlan::Request req2 = req;
      req2.motion_plan_request.goal_constraints.resize(1);
      req2.motion_plan_request.goal_constraints[0] = req.motion_plan_request.path_constraints;
      req2.motion_plan_request.path_constraints = moveit_msgs::Constraints();
      moveit_msgs::GetMotionPlan::Response res2;
      bool solved1 = planner(planning_scene, req2, res2);

      if (solved1)
      { 
        moveit_msgs::GetMotionPlan::Request req3 = req;
        std::size_t last_index = trajectory_processing::trajectoryPointCount(res2.trajectory);
        ROS_DEBUG("Planned to path constraints. Resuming original planning request.");
        assert(last_index > 0);
        // extract the last state of the computed motion plan and set it as the new start state
        moveit_msgs::RobotState new_start;
        trajectory_processing::robotTrajectoryPointToRobotState(res2.trajectory, last_index - 1, new_start);
        kinematic_state::robotStateToKinematicState(*planning_scene->getTransforms(), new_start, start_state);
        kinematic_state::kinematicStateToRobotState(start_state, req3.motion_plan_request.start_state);
                
        bool solved2 = planner(planning_scene, req3, res);
        if (solved2)
        {
          // we need to append the solution paths. 
          res.trajectory_start = res2.trajectory_start;
          
          if (last_index > 1)
          {
            // if the structure of the paths is the same, then things are easy
            if (res2.trajectory.joint_trajectory.joint_names == res.trajectory.joint_trajectory.joint_names &&
                res2.trajectory.multi_dof_joint_trajectory.joint_names == res.trajectory.multi_dof_joint_trajectory.joint_names &&
                res2.trajectory.multi_dof_joint_trajectory.frame_ids == res.trajectory.multi_dof_joint_trajectory.frame_ids &&
                res2.trajectory.multi_dof_joint_trajectory.child_frame_ids == res.trajectory.multi_dof_joint_trajectory.child_frame_ids)
            {
              // insert all but the last point
              if (!res2.trajectory.joint_trajectory.points.empty())
              {
                res.trajectory.joint_trajectory.points.insert(res.trajectory.joint_trajectory.points.begin(),
                                                              res2.trajectory.joint_trajectory.points.begin(),
                                                              --res2.trajectory.joint_trajectory.points.end());
                for (std::size_t i = res2.trajectory.joint_trajectory.points.size() - 1 ; i < res.trajectory.joint_trajectory.points.size() ; ++i)
                  res.trajectory.joint_trajectory.points[i].time_from_start += res2.trajectory.joint_trajectory.points.back().time_from_start;
              }     
              if (!res2.trajectory.multi_dof_joint_trajectory.points.empty())
              {
                res.trajectory.multi_dof_joint_trajectory.points.insert(res.trajectory.multi_dof_joint_trajectory.points.begin(),
                                                                        res2.trajectory.multi_dof_joint_trajectory.points.begin(),
                                                                        --res2.trajectory.multi_dof_joint_trajectory.points.end());
                for (std::size_t i = res2.trajectory.multi_dof_joint_trajectory.points.size() - 1 ; i < res.trajectory.multi_dof_joint_trajectory.points.size() ; ++i)
                  res.trajectory.multi_dof_joint_trajectory.points[i].time_from_start += res2.trajectory.multi_dof_joint_trajectory.points.back().time_from_start;
              }
            }
            else
            {
              // paths do not have the same structure. We merge them in an inefficient but hopefuly safe way
              kinematic_state::KinematicState st = planning_scene->getCurrentState();
              kinematic_state::robotStateToKinematicState(*planning_scene->getTransforms(), res2.trajectory_start, st);
              
              for (int i = last_index - 2 ; i >= 0 ; --i)
              {
                // take states from the first path one by one, in reverse, and add them as prefix
                moveit_msgs::RobotState temp;
                trajectory_processing::robotTrajectoryPointToRobotState(res2.trajectory, i, temp);
                kinematic_state::robotStateToKinematicState(*planning_scene->getTransforms(), temp, st);
                double dt = ((int)res.trajectory.joint_trajectory.points.size() > i + 1) ?
                  (res.trajectory.joint_trajectory.points[i + 1].time_from_start - res.trajectory.joint_trajectory.points[i].time_from_start).toSec() : 
                  (res.trajectory.multi_dof_joint_trajectory.points[i + 1].time_from_start - res.trajectory.multi_dof_joint_trajectory.points[i].time_from_start).toSec();
                trajectory_processing::addPrefixState(st, res.trajectory, dt, planning_scene->getTransforms());
              }
            }
          }
          return true;
        }
        else
          return false;
      }
      else
      { 
        ROS_WARN("Unable to plan to path constraints. Running usual motion plan.");
        return planner(planning_scene, req, res);
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
