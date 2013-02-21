/*********************************************************************
*
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
*
* Author: Sachin Chitta
*********************************************************************/

#ifndef KINEMATICS_SOLVER_ROS_H_
#define KINEMATICS_SOLVER_ROS_H_

// MoveIt!
#include <kinematics_planner/kinematics_solver.h>
#include <planning_scene_monitor/planning_scene_monitor.h>
#include <kinematics_msgs/GetConstraintAwarePositionIK.h>

namespace kinematics_planner_ros
{

/**
 * @class Can be used with multiple arms
 */
class KinematicsSolverROS
{
  public:

  KinematicsSolverROS(): node_handle_("~")
  {
  }
  
  bool initialize();
      
  /** @brief Solve the IK problem
   * @param ik_request A desired IK request
   * @param ik_response An IK response
   */
  bool getIK(kinematics_msgs::GetConstraintAwarePositionIK::Request &request,
             kinematics_msgs::GetConstraintAwarePositionIK::Response &response);

  bool isActive()
  {
    if(!getRobotModel())
    {
      return false;      
    }
    if(!planning_scene_monitor_->getPlanningScene())
    {
      return false;
    }
    return true;
  };
    
  const planning_models::RobotModelConstPtr& getRobotModel() const
  {
    return planning_scene_monitor_->getRobotModel();
  };

  const planning_scene_monitor::PlanningSceneMonitorPtr getPlanningSceneMonitor()
  {
    return planning_scene_monitor_;    
  };
      
  const kinematics_planner::KinematicsSolverPtr& getKinematicsSolver(const std::string &group_name)
  {
    if(kinematics_solver_group_name_map_.find(group_name) != kinematics_solver_group_name_map_.end())
    {
      return kinematics_solver_group_name_map_.find(group_name)->second;
    }
    return empty_ptr_;    
  };
      
private:

  ros::ServiceServer get_ik_service_;
  
  std::map<std::string,kinematics_planner::KinematicsSolverPtr> kinematics_solver_;

  std::map<std::string,kinematics_planner::KinematicsSolverPtr> kinematics_solver_group_name_map_;

  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  
  kinematics_planner::KinematicsSolverPtr empty_ptr_;
  
  ros::NodeHandle node_handle_;  
};

}

#endif
