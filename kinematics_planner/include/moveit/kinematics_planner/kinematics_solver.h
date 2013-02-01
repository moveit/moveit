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

#ifndef KINEMATICS_SOLVER_H_
#define KINEMATICS_SOLVER_H_

// System
#include <boost/shared_ptr.hpp>

// ROS msgs
#include <moveit_msgs/GetConstraintAwarePositionIK.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/RobotState.h>

// Plugin
#include <moveit/kinematics_base/kinematics_base.h>

// MoveIt!
#include <moveit/kinematic_model/kinematic_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>

namespace kinematics_planner
{

typedef std::map<std::string, const kinematics::KinematicsBaseConstPtr> KinematicsSolverMap;
typedef boost::shared_ptr<const KinematicsSolverMap> KinematicsSolverMapConstPtr;
typedef std::map<std::string,std::vector<double> > SolutionStateMap;

/**
 * @class An IK solver that can be used with multiple arms
 */
class KinematicsSolver
{
  public:

  KinematicsSolver(){}

  /** @brief Initialize
   * @param kinematic_model An instance of a kinematic model
   * @return False if any error occurs
   */
  bool initialize(const kinematic_model::KinematicModelConstPtr &kinematic_model);

  /** @brief Solve the planning problem
   * @param planning_scene A const reference to the planning scene
   * @param request A const reference to the kinematics request
   * @param response The solution (if it exists)
   * @return False if group_name is invalid or ik fails
   */
  bool getIK(const planning_scene::PlanningSceneConstPtr &planning_scene,
             const moveit_msgs::GetConstraintAwarePositionIK::Request &request,
             moveit_msgs::GetConstraintAwarePositionIK::Response &response) const;
    
protected:

  std::vector<double> getFloatingJointValues(const geometry_msgs::Pose &pose) const;
  
  geometry_msgs::Pose getPose(const std::vector<double> &values) const;
  
  std::map<std::string,geometry_msgs::PoseStamped> transformPoses(const planning_scene::PlanningSceneConstPtr& planning_scene, 
                                                                  const robot_state::RobotState &kinematic_state,
                                                                  const std::map<std::string,geometry_msgs::PoseStamped> &poses,
                                                                  const std::map<std::string,std::string> &target_frames) const;

  bool getGoal(const planning_scene::PlanningSceneConstPtr &planning_scene,
               const robot_state::RobotState &kinematic_state,
               const moveit_msgs::GetConstraintAwarePositionIK::Request &request,
               std::map<std::string,geometry_msgs::PoseStamped>& pose_stamped) const;

  geometry_msgs::Pose getTipFramePose(const planning_scene::PlanningSceneConstPtr& planning_scene, 
                                      const robot_state::RobotState &kinematic_state,
                                      const geometry_msgs::Pose &pose,
                                      const std::string &link_name,
                                      const std::string &group_name) const;
  
  moveit_msgs::RobotState *getRobotState(const kinematics_planner::SolutionStateMap &solutions,
                                        const std::vector<std::string> &group_names) const;  
  
  std::vector<kinematics::KinematicsBaseConstPtr> getKinematicsSolvers(const std::vector<std::string> &group_names) const;
  
  std::map<std::string,kinematics::KinematicsBaseConstPtr> kinematics_solver_map_;    

  std::map<std::string, std::vector<std::string> > group_map_;

  kinematic_model::KinematicModelConstPtr kinematic_model_;
  

};

typedef boost::shared_ptr<KinematicsSolver> KinematicsSolverPtr;
typedef boost::shared_ptr<const KinematicsSolver> KinematicsSolverConstPtr;

}

#endif
