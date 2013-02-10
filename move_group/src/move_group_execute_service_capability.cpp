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

#include <moveit/move_group/names.h>
#include <moveit/move_group/move_group_execute_service_capability.h>

move_group::MoveGroupExecuteService::MoveGroupExecuteService(const planning_scene_monitor::PlanningSceneMonitorPtr& psm, 
                                                             const trajectory_execution_manager::TrajectoryExecutionManagerPtr &trajectory_execution_manager, 
                                                             bool debug):
  MoveGroupCapability(psm, debug),
  trajectory_execution_manager_(trajectory_execution_manager)
{
  execute_service_ = root_node_handle_.advertiseService(EXECUTE_SERVICE_NAME, &MoveGroupExecuteService::executeTrajectoryService, this);
}

bool move_group::MoveGroupExecuteService::executeTrajectoryService(moveit_msgs::ExecuteKnownTrajectory::Request &req, moveit_msgs::ExecuteKnownTrajectory::Response &res)
{
  ROS_INFO("Received new trajectory execution service request...");
  if (!trajectory_execution_manager_)
  {
    ROS_ERROR("Cannot execute trajectory since ~allow_trajectory_execution was set to false");
    res.error_code.val = moveit_msgs::MoveItErrorCodes::CONTROL_FAILED;
    return true;
  }
  
  // \todo unwind trajectory before execution
  //    robot_trajectory::RobotTrajectory to_exec(planning_scene_monitor_->getRobotModel(), ;
  
  trajectory_execution_manager_->clear();
  if (trajectory_execution_manager_->push(req.trajectory))
  {
    trajectory_execution_manager_->execute();
    if (req.wait_for_execution)
    {
      moveit_controller_manager::ExecutionStatus es = trajectory_execution_manager_->waitForExecution();
      if (es == moveit_controller_manager::ExecutionStatus::SUCCEEDED)
        res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
      else
        if (es == moveit_controller_manager::ExecutionStatus::PREEMPTED) 
          res.error_code.val = moveit_msgs::MoveItErrorCodes::PREEMPTED;
        else
          if (es == moveit_controller_manager::ExecutionStatus::TIMED_OUT) 
            res.error_code.val = moveit_msgs::MoveItErrorCodes::TIMED_OUT;
          else
            res.error_code.val = moveit_msgs::MoveItErrorCodes::CONTROL_FAILED;
      ROS_INFO_STREAM("Execution completed: " << es.asString());
    }
    else
    {
      ROS_INFO("Trajectory was successfully forwarded to the controller");
      res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    }
  }
  else
  {    
    res.error_code.val = moveit_msgs::MoveItErrorCodes::CONTROL_FAILED;
  }
  return true;
}

