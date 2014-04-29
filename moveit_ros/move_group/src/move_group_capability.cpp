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
 *   * Neither the name of Willow Garage nor the names of its
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

#include <moveit/move_group/move_group_capability.h>
#include <moveit/robot_state/conversions.h>

void move_group::MoveGroupCapability::setContext(const MoveGroupContextPtr &context)
{
  context_ = context;
}

void move_group::MoveGroupCapability::convertToMsg(const std::vector<plan_execution::ExecutableTrajectory> &trajectory,
                                                   moveit_msgs::RobotState &first_state_msg, std::vector<moveit_msgs::RobotTrajectory> &trajectory_msg) const
{
  if (!trajectory.empty())
  {
    bool first = true;
    trajectory_msg.resize(trajectory.size());
    for (std::size_t i = 0 ; i < trajectory.size() ; ++i)
    {
      if (trajectory[i].trajectory_)
      {
        if (first && !trajectory[i].trajectory_->empty())
        {
          robot_state::robotStateToRobotStateMsg(trajectory[i].trajectory_->getFirstWayPoint(), first_state_msg);
          first = false;
        }
        trajectory[i].trajectory_->getRobotTrajectoryMsg(trajectory_msg[i]);
      }
    }
  }
}

void move_group::MoveGroupCapability::convertToMsg(const robot_trajectory::RobotTrajectoryPtr &trajectory,
                                                   moveit_msgs::RobotState &first_state_msg, moveit_msgs::RobotTrajectory &trajectory_msg) const
{
  if (trajectory && !trajectory->empty())
  {
    robot_state::robotStateToRobotStateMsg(trajectory->getFirstWayPoint(), first_state_msg);
    trajectory->getRobotTrajectoryMsg(trajectory_msg);
  }
}

void move_group::MoveGroupCapability::convertToMsg(const std::vector<plan_execution::ExecutableTrajectory> &trajectory,
                                                   moveit_msgs::RobotState &first_state_msg, moveit_msgs::RobotTrajectory &trajectory_msg) const
{
  if (trajectory.size() > 1)
    ROS_ERROR_STREAM("Internal logic error: trajectory component ignored. !!! THIS IS A SERIOUS ERROR !!!");
  if (trajectory.size() > 0)
    convertToMsg(trajectory[0].trajectory_, first_state_msg,trajectory_msg);
}

planning_interface::MotionPlanRequest move_group::MoveGroupCapability::clearRequestStartState(const planning_interface::MotionPlanRequest &request) const
{
  planning_interface::MotionPlanRequest r = request;
  r.start_state = moveit_msgs::RobotState();
  r.start_state.is_diff = true;  
  ROS_WARN("Execution of motions should always start at the robot's current state. Ignoring the state supplied as start state in the motion planning request");
  return r;
}

moveit_msgs::PlanningScene move_group::MoveGroupCapability::clearSceneRobotState(const moveit_msgs::PlanningScene &scene) const
{
  moveit_msgs::PlanningScene r = scene;
  r.robot_state = moveit_msgs::RobotState();
  r.robot_state.is_diff = true;  
  ROS_WARN("Execution of motions should always start at the robot's current state. Ignoring the state supplied as difference in the planning scene diff");
  return r;
}

std::string move_group::MoveGroupCapability::getActionResultString(const moveit_msgs::MoveItErrorCodes &error_code, bool planned_trajectory_empty, bool plan_only)
{
  if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    if (planned_trajectory_empty)
      return "Requested path and goal constraints are already met.";
    else
    {
      if (plan_only)
        return "Motion plan was computed succesfully.";
      else
        return "Solution was found and executed.";
    }
  }
  else
    if (error_code.val == moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME)
      return "Must specify group in motion plan request";
    else
      if (error_code.val == moveit_msgs::MoveItErrorCodes::PLANNING_FAILED || error_code.val == moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN)
      {
        if (planned_trajectory_empty)
          return "No motion plan found. No execution attempted.";
        else
          return "Motion plan was found but it seems to be invalid (possibly due to postprocessing). Not executing.";
      }
      else
        if (error_code.val == moveit_msgs::MoveItErrorCodes::UNABLE_TO_AQUIRE_SENSOR_DATA)
          return "Motion plan was found but it seems to be too costly and looking around did not help.";
        else
          if (error_code.val == moveit_msgs::MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE)
            return "Solution found but the environment changed during execution and the path was aborted";
          else
            if (error_code.val == moveit_msgs::MoveItErrorCodes::CONTROL_FAILED)
              return "Solution found but controller failed during execution";
            else
              if (error_code.val == moveit_msgs::MoveItErrorCodes::TIMED_OUT)
                return "Timeout reached";
              else
                if (error_code.val == moveit_msgs::MoveItErrorCodes::PREEMPTED)
                  return "Preempted";
                else
                  if (error_code.val == moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS)
                    return "Invalid goal constraints";
                  else
                    if (error_code.val == moveit_msgs::MoveItErrorCodes::INVALID_OBJECT_NAME)
                      return "Invalid object name";
                    else
                      if (error_code.val == moveit_msgs::MoveItErrorCodes::FAILURE)
                        return "Catastrophic failure";
  return "Unknown event";
}

std::string move_group::MoveGroupCapability::stateToStr(MoveGroupState state) const
{
  switch (state)
  {
  case IDLE:
    return "IDLE";
  case PLANNING:
    return "PLANNING";
  case MONITOR:
    return "MONITOR";
  case LOOK:
    return "LOOK";
  default:
    return "UNKNOWN";
  }
}

bool move_group::MoveGroupCapability::performTransform(geometry_msgs::PoseStamped &pose_msg, const std::string &target_frame) const
{
  if (!context_ || !context_->planning_scene_monitor_->getTFClient())
    return false;
  if (pose_msg.header.frame_id == target_frame)
    return true;
  if (pose_msg.header.frame_id.empty())
  {
    pose_msg.header.frame_id = target_frame;
    return true;
  }

  try
  {
    std::string error;
    ros::Time common_time;
    context_->planning_scene_monitor_->getTFClient()->getLatestCommonTime(pose_msg.header.frame_id, target_frame, common_time, &error);
    if (!error.empty())
      ROS_ERROR("TF Problem: %s", error.c_str());

    tf::Stamped<tf::Pose> pose_tf, pose_tf_out;
    tf::poseStampedMsgToTF(pose_msg, pose_tf);
    pose_tf.stamp_ = common_time;
    context_->planning_scene_monitor_->getTFClient()->transformPose(target_frame, pose_tf, pose_tf_out);
    tf::poseStampedTFToMsg(pose_tf_out, pose_msg);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("TF Problem: %s", ex.what());
    return false;
  }
  return true;
}
