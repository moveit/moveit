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
#include <planning_scene/planning_scene.h>
#include <planning_pipeline/planning_pipeline.h>


// ROS msgs
#include <moveit_msgs/RobotTrajectory.h>
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

    Plan();
    
    void initializeForPick(const moveit_manipulation_msgs::PickupGoal &pickup_goal,
                     const planning_scene::PlanningSceneConstPtr &planning_scene);
    
    void initializeForPlace(const moveit_manipulation_msgs::PlaceGoal &place_goal,
                       const planning_scene::PlanningSceneConstPtr &planning_scene);

    std::map<std::string,std::vector<moveit_msgs::RobotTrajectory> > robot_trajectories_;

    std::map<std::string,std::vector<TrajectoryType> > trajectory_types_;
    
    std::map<std::string,std::vector<ControlMode> > control_modes_;
    
    bool success_;    
  };


  /** Contains pre-computed values for a moveit_manipulation_msgs::PickupGoal
    */
  class PickGoal
  {
  public:
    
    PickGoal(const moveit_manipulation_msgs::PickupGoal &pickup_goal,
             const planning_scene::PlanningSceneConstPtr &planning_scene,
             const pick_place_planner::ManipulationGroupConstPtr &manipulation_group);

    void computeGoalsForGrasp(unsigned int grasp_i);

    bool checkEndEffectorPose(planning_scene::PlanningSceneConstPtr &planning_scene,
                              const std::vector<Eigen::Affine3d> &poses,
                              const std::map<std::string, double> &end_effector_posture,
                              const collision_detection::AllowedCollisionMatrix &acm);

    /* total number of grasps. each of the following vectors should have this size */
    unsigned int num_grasps_;
    
    /* grasp pose for each grasp */
    std::vector<std::vector<Eigen::Affine3d> > grasp_pose_;

    /* pre-grasp pose for each grasp */
    std::vector<std::vector<Eigen::Affine3d> > pre_grasp_pose_;

    /* lift pose for each grasp */
    std::vector<std::vector<Eigen::Affine3d> > lift_pose_;

    /* grasp posture for each grasp */
    std::vector<std::map<std::string, std::vector<double> > > grasp_posture_;

    /* pre-grasp posture for each grasp */
    std::vector<std::map<std::string, std::vector<double> > > pre_grasp_posture_;
    
    /* original pickup goal */
    moveit_manipulation_msgs::PickupGoal pickup_goal_;
        
  };
  
    
  PickPlacePlanner(const planning_models::KinematicModelConstPtr &kinematic_model_,
                   const std::string &freespace_planning_plugin_name,
                   const std::string &contact_planning_plugin_name);

  virtual bool planPick(const moveit_manipulation_msgs::PickupGoal &pickup_goal,
                        const planning_scene::PlanningSceneConstPtr &planning_scene,
                        std::vector<PickPlacePlanner::Plan> &pick_plans,
                        bool return_on_first_solution) const;
  
  virtual bool planPlace(const moveit_manipulation_msgs::PlaceGoal &place_goal,
                         const planning_scene::PlanningSceneConstPtr &planning_scene,
                         std::vector<PickPlacePlanner::Plan> &place_plans,
                         bool return_on_first_solution) const;

  /* check whether a valid IK exists for the pre-grasp, grasp, and lift poses */
  virtual void checkPoses(int grasp_i);

  bool checkPoses(PickGoal &pick_goal,
                                    unsigned int index,
                                    Plan &plan);

private:
  planning_models::KinematicModelConstPtr kinematic_model_;
  
  std::string freespace_planner_;
  boost::shared_ptr<planning_pipeline::PlanningPipeline> freespace_planning_pipeline_;

  std::string contact_planner_;
  boost::shared_ptr<planning_pipeline::PlanningPipeline> contact_planning_pipeline_;
  
  std::map<std::string, pick_place_planner::ManipulationGroupPtr> manipulation_groups_;

};

}

#endif
