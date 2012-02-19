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

#ifndef MOVEIT_MOVEIT_WAREHOUSE_WAREHOUSE_
#define MOVEIT_MOVEIT_WAREHOUSE_WAREHOUSE_

#include <mongo_ros/message_collection.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/RobotTrajectory.h>

namespace moveit_warehouse
{

typedef mongo_ros::MessageWithMetadata<moveit_msgs::PlanningScene>::ConstPtr PlanningSceneWithMetadata;
typedef mongo_ros::MessageWithMetadata<moveit_msgs::MotionPlanRequest>::ConstPtr MotionPlanRequestWithMetadata;
typedef mongo_ros::MessageWithMetadata<moveit_msgs::RobotTrajectory>::ConstPtr RobotTrajectoryWithMetadata;

typedef boost::shared_ptr<mongo_ros::MessageCollection<moveit_msgs::PlanningScene> > PlanningSceneCollection;
typedef boost::shared_ptr<mongo_ros::MessageCollection<moveit_msgs::MotionPlanRequest> > MotionPlanRequestCollection;
typedef boost::shared_ptr<mongo_ros::MessageCollection<moveit_msgs::RobotTrajectory> > RobotTrajectoryCollection;

class PlanningSceneStorage
{
public:
  PlanningSceneStorage();
  
  void addPlanningScene(const moveit_msgs::PlanningScene &scene);
  void addPlanningRequest(const moveit_msgs::MotionPlanRequest &planning_query, const std::string &scene_name, const std::string &query_name = "");
  void addPlanningResult(const moveit_msgs::MotionPlanRequest &planning_query, const moveit_msgs::RobotTrajectory &result, const std::string &scene_name);
  

  void getPlanningSceneNames(std::vector<std::string> &names) const;
  void getPlanningSceneNamesAndTimes(std::vector<std::string> &names, std::vector<ros::Time>& times) const;

  /** \brief Get the planning scene named \e scene_name and time stamp closest to \e time. If the scene is found but the time difference between its stamp and \e time is larger than \e margin, a warning is issued. */
  bool getPlanningScene(PlanningSceneWithMetadata &scene_m, const std::string &scene_name, const ros::Time& time, double margin = 1.0) const;

  /** \brief Get the latest planning scene named \e scene_name */
  bool getPlanningScene(PlanningSceneWithMetadata &scene_m, const std::string &scene_name) const;

  void getPlanningQueries(std::vector<MotionPlanRequestWithMetadata> &planning_queries, const std::string &scene_name) const;
  void getPlanningResults(std::vector<RobotTrajectoryWithMetadata> &planning_results, const moveit_msgs::MotionPlanRequest &planning_query, const std::string &scene_name) const;
  void getPlanningResults(std::vector<RobotTrajectoryWithMetadata> &planning_results, const std::string &query_name, const std::string &scene_name) const;
  
  void removePlanningScene(const std::string &scene_name);
  void removePlanningSceneQueries(const std::string &scene_name);
  
private:
  
  std::string getMotionPlanRequestName(const moveit_msgs::MotionPlanRequest &planning_query, const std::string &scene_name) const;
  std::string addNewPlanningRequest(const moveit_msgs::MotionPlanRequest &planning_query, const std::string &scene_name, const std::string &query_name);
  
  PlanningSceneCollection	    planning_scene_collection_;
  MotionPlanRequestCollection motion_plan_request_collection_;
  RobotTrajectoryCollection   robot_trajectory_collection_;
  
};
}

#endif
