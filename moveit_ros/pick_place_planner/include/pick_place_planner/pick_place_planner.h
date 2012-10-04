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

#ifndef PICK_PLACE_PLANNER_H_
#define PICK_PLACE_PLANNER_H_

// System
#include <boost/shared_ptr.hpp>

#include <pick_place_planner/manipulation_group.h>

// ROS msgs
#include <moveit_manipulation_msgs/PickupGoal.h>
#include <moveit_manipulation_msgs/PlaceGoal.h>
#include <moveit_manipulation_msgs/GraspResult.h>

namespace pick_place_planner
{

static const unsigned int PICK_NUM_PHASES = 3;
static const unsigned int PLACE_NUM_PHASES = 3;

/**
 * @class A general pick place planning class
 */
class PickPlacePlanner
{
  public:

  enum ControlMode
  {
    FREESPACE_TRAJECTORY, //Don't expect any contact during this part of the trajectory
    CONTACT_TRAJECTORY //Expect contact - actual manipulation is happening here
  };
  
  enum TrajectoryType
  {
    JOINT_TRAJECTORY,
    CARTESIAN_TRAJECTORY
  };
      
  class Plan
  {
  public:
    
    Plan(const moveit_msgs::PickupGoal &pickup_goal);
    
    Plan(const moveit_msgs::PlaceGoal &place_goal);

    std::map<std::string,std::vector<moveit_msgs::RobotTrajectory> > robot_trajectories_;

    std::map<std::string,std::vector<pick_place_planner::TrajectoryType> > trajectory_types_;
    
    std::map<std::string,std::vector<pick_place_planner::ControlMode> > control_modes_;
    
    bool success_;
    
  };

  class PickGoal
  {
  public:
    
    PickGoal(const moveit_manipulation_msgs::PickupGoal &pickup_goal,
             const planning_scene::PlanningSceneConstPtr &planning_scene);
    
    unsigned int num_grasps_;
    
    std::vector<std::vector<Eigen::Affine3d> > grasp_pose_, pre_grasp_pose_, lift_pose_;

    std::map<std::string, std::vector<double> > grasp_posture_, pre_grasp_posture_;
    
    moveit_manipulation_msgs::PickupGoal pickup_goal_;
        
  };
  
    
  PickPlacePlanner(const std::string &freespace_planner,
                   const std::string &interpolation_planner);

  virtual void planPick(const moveit_manipulation_msgs::PickupGoal &pickup_goal,
                        const planning_scene::PlanningSceneConstPtr &planning_scene,
                        bool return_on_first_solution) const;
  
  virtual void planPlace(const moveit_manipulation_msgs::PlaceGoal &place_goal,
                         const planning_scene::PlanningSceneConstPtr &planning_scene,
                         bool return_on_first_solution) const;
  
private:
  
  std::string freespace_planner_;

  std::string interpolation_planner_;
  
  std::map<std::string, pick_place_planner::ManipulationGroupPtr> manipulation_groups_;

};

}

#endif
