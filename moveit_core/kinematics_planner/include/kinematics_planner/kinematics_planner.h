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

#ifndef KINEMATICS_PLANNER_H_
#define KINEMATICS_PLANNER_H_


// System
#include <boost/shared_ptr.hpp>

// ROS msgs
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/RobotTrajectory.h>

// Plugin
#include <kinematics_base/kinematics_base.h>

// MoveIt!
#include <planning_models/kinematic_model.h>
#include <planning_models/kinematic_state.h>
#include <planning_scene/planning_scene.h>
#include <kinematic_constraints/kinematic_constraint.h>

namespace kinematics_planner
{

typedef std::map<std::string,const kinematics::KinematicsBaseConstPtr> KinematicsSolverMap;
typedef boost::shared_ptr<const KinematicsSolverMap> KinematicsSolverMapConstPtr;
typedef std::map<std::string,std::vector<std::vector<double> > > SolutionTrajectoryMap;
typedef std::map<std::string,std::vector<double> > SolutionStateMap;

/**
 * @class An interpolated IK planner. Can be used with multiple arms
 */
class KinematicsPlanner
{
  public:

  KinematicsPlanner(){}

  /** @brief Initialize
   * @param kinematic_model An instance of a kinematic model
   * @param kinematics_solver_map A set of solvers for all the sub groups in the desired group
   * @param group_name The group name to plan for
   * @return False if group_name is invalid or kinematics solvers are not defined for all subgroups
   */
  bool initialize(const planning_models::KinematicModelConstPtr &kinematic_model,
                  const kinematics_planner::KinematicsSolverMapConstPtr &kinematics_solver_map,
                  const std::string &group_name);

  /** @brief Solve the planning problem
   * @param start_request A map from group names to desired start poses
   * @param goal_request A map from group names to desired goal poses
   * @param planning_scene A const reference to the planning scene
   * @param path_constraints The set of path constraints to check
   * @param timeout The total amount of time to be spent in the solve step (in seconds)
   * @param robot_trajectory The desired robot trajectory
   * @param error_code An error code
   * @return False if group_name is invalid or kinematics solvers are not defined for all subgroups
   */
  bool solve(const std::map<std::string,geometry_msgs::PoseStamped> &start_request,
             const std::map<std::string,geometry_msgs::PoseStamped> &goal_request,
             const planning_scene::PlanningSceneConstPtr& planning_scene,
             const moveit_msgs::Constraints &path_constraints,
             double timeout,
             moveit_msgs::RobotTrajectory &robot_trajectory,
             moveit_msgs::MoveItErrorCodes &error_code) const;

  /** @brief Solve the planning problem
   * @param start_request A map from group names to desired start poses
   * @param goal_request A map from group names to desired goal poses
   * @param planning_scene A const reference to the planning scene
   * @param kinematic_constraint_set A pre-allocated instance of KinematicConstraintSet (for efficiency)
   * @param timeout The total amount of time to be spent in the solve step (in seconds)
   * @param robot_trajectory The desired robot trajectory
   * @param error_code An error code
   * @return False if group_name is invalid or kinematics solvers are not defined for all subgroups
   */
  bool solve(const std::map<std::string,geometry_msgs::PoseStamped> &start_request,
             const std::map<std::string,geometry_msgs::PoseStamped> &goal_request,
             const planning_scene::PlanningSceneConstPtr& planning_scene,
             const kinematic_constraints::KinematicConstraintSet& kinematic_constraint_set,
             double timeout,
             moveit_msgs::RobotTrajectory &robot_trajectory,
             moveit_msgs::MoveItErrorCodes &error_code) const;

  /** @brief Solve the planning problem
   * @param start_request A map from group names to desired poses
   * @param planning_scene A const reference to the planning scene
   * @param kinematic_constraint_set A pre-allocated instance of KinematicConstraintSet (for efficiency)
   * @param timeout The total amount of time to be spent in the solve step (in seconds)
   * @param robot_trajectory The desired robot trajectory
   * @param error_code An error code
   * @return False if group_name is invalid or kinematics solvers are not defined for all subgroups
   */
  bool solve(const std::map<std::string,geometry_msgs::PoseStamped> &poses,
             const planning_scene::PlanningSceneConstPtr &planning_scene,
             const kinematic_constraints::KinematicConstraintSet &kinematic_constraint_set,
             double timeout,
             moveit_msgs::RobotState &solution,
             moveit_msgs::MoveItErrorCodes &error_code) const;

  /** @brief Set the discretization values
   * @param discretization_translation Expected discretization (in m) in translation
   * @param discretization_rotation Expected discretization (in radians) in rotation
   */
  void setDiscretization(double discretization_translation, double discretization_rotation)
  {
    discretization_translation_ = discretization_translation;
    discretization_rotation_ = discretization_rotation;    
  }

  /** @brief Get the discretization values
   * @param discretization_translation Expected discretization (in m) in translation
   * @param discretization_rotation Expected discretization (in radians) in rotation
   */
  void getDiscretization(double &discretization_translation, double &discretization_rotation) const
  {
    discretization_translation = discretization_translation_;
    discretization_rotation = discretization_rotation_;    
  }
        
private:

  std::map<std::string,std::vector<geometry_msgs::Pose> > getInterpolatedPosesMap(const std::map<std::string,geometry_msgs::PoseStamped> &start,
                                                                                         const std::map<std::string,geometry_msgs::PoseStamped> &goal) const;
  
  std::vector<double> getFloatingJointValues(const geometry_msgs::Pose &pose) const;
  
  geometry_msgs::Pose getPose(const std::vector<double> &values) const;
  
  unsigned int getNumSegments(const geometry_msgs::Pose &start,
                              const geometry_msgs::Pose &goal) const;
  
  std::vector<geometry_msgs::Pose> getInterpolatedPoses(const geometry_msgs::Pose &start,
                                                        const geometry_msgs::Pose &goal,
                                                        const unsigned int num_segments) const;
  
  std::map<std::string,geometry_msgs::PoseStamped> transformPoses(const planning_scene::PlanningSceneConstPtr& planning_scene, 
                                                                  const planning_models::KinematicState &kinematic_state,
                                                                  const std::map<std::string,geometry_msgs::PoseStamped> &poses,
                                                                  const std::string &target_frame) const;

  std::map<std::string,geometry_msgs::PoseStamped> transformPoses(const planning_scene::PlanningSceneConstPtr& planning_scene, 
                                                                  const planning_models::KinematicState &kinematic_state,
                                                                  const std::map<std::string,geometry_msgs::PoseStamped> &poses,
                                                                  const std::vector<std::string> &target_frames) const;

  moveit_msgs::RobotState getRobotState(const kinematics_planner::SolutionStateMap &solutions) const;  
  
  moveit_msgs::RobotTrajectory getRobotTrajectory(const  kinematics_planner::SolutionTrajectoryMap &solutions,
                                                  unsigned int num_poses) const;
  
  bool checkRequest(const std::map<std::string,geometry_msgs::PoseStamped> &start) const;

  //  const kinematics_planner::KinematicsSolverMap kinematics_solver_map_;
  std::vector<kinematics::KinematicsBaseConstPtr> kinematics_solvers_;

  std::vector<std::string> kinematics_base_frames_;
    
  std::string group_name_;

  std::vector<std::string> group_names_, joint_names_;

  unsigned int num_groups_, num_poses_;

  double discretization_translation_, discretization_rotation_;
  
};

}

#endif
