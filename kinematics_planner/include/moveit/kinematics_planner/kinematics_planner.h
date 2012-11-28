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

#ifndef MOVEIT_KINEMATICS_PLANNER_KINEMATICS_PLANNER_
#define MOVEIT_KINEMATICS_PLANNER_KINEMATICS_PLANNER_

// MoveIt!
#include <moveit/kinematics_planner/kinematics_solver.h>
#include <moveit_msgs/RobotTrajectory.h>

namespace kinematics_planner
{

typedef std::map<std::string,std::vector<std::vector<double> > > SolutionTrajectoryMap;

/**
 * @class An interpolated IK planner. Can be used with multiple arms
 */
class KinematicsPlanner : public kinematics_planner::KinematicsSolver
{
  public:

  KinematicsPlanner() : KinematicsSolver() {};

  /** @brief Initiliaze data structures
    * @param group_names Groups which will be planned for
    * @param model Kinematic model of the robot
    * @return False if initilization fails
    */
  bool initialize(const std::vector<std::string> &group_names, const kinematic_model::KinematicModelConstPtr &model);

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

  bool checkRequest(const std::map<std::string,geometry_msgs::PoseStamped> &request) const;
        
private:

  std::map<std::string,std::vector<geometry_msgs::Pose> > getInterpolatedPosesMap(const std::map<std::string,geometry_msgs::PoseStamped> &start,
                                                                                         const std::map<std::string,geometry_msgs::PoseStamped> &goal) const;
  
  
  unsigned int getNumSegments(const geometry_msgs::Pose &start,
                              const geometry_msgs::Pose &goal) const;
  
  std::vector<geometry_msgs::Pose> getInterpolatedPoses(const geometry_msgs::Pose &start,
                                                        const geometry_msgs::Pose &goal,
                                                        const unsigned int num_segments) const;
    
  void getRobotTrajectory(const  kinematics_planner::SolutionTrajectoryMap &solutions,
                          unsigned int num_poses,
                          moveit_msgs::RobotTrajectory &robot_trajectory) const;
  

  double discretization_translation_, discretization_rotation_;

  KinematicsSolverConstPtr kinematics_solver_;
};

}

#endif
