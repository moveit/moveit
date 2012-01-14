//Software License Agreement (BSD License)

//Copyright (c) 2011, Willow Garage, Inc.
//All rights reserved.

//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions
//are met:

// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of Willow Garage, Inc. nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.

//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//POSSIBILITY OF SUCH DAMAGE.

#ifndef KINEMATICS_SOLVER_CONSTRAINT_AWARE_
#define KINEMATICS_SOLVER_CONSTRAINT_AWARE_

#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit_msgs/Constraints.h>

#include <kinematics_base/kinematics_base.h>
#include <planning_models/kinematic_model.h>
#include <planning_models/kinematic_state.h>
#include <planning_scene/planning_scene.h>

namespace kinematics_constraint_aware 
{

class KinematicsSolverConstraintAware 
{
public:
  
  KinematicsSolverConstraintAware(kinematics::KinematicsBasePtr& solver,
                                  const planning_models::KinematicModelConstPtr& kmodel,
                                  const std::string& group_name);
  
  ~KinematicsSolverConstraintAware() {}
  
  bool isActive() const {
    return active_;
  }

  bool getPositionFK(const planning_models::KinematicState* robot_state,
                     const std::vector<std::string>& link_names,
                     std::vector<geometry_msgs::Pose> &poses);

  bool getPositionIK(const geometry_msgs::Pose &ik_pose,
                     const planning_scene::PlanningSceneConstPtr& scene,
                     sensor_msgs::JointState& solution,
                     moveit_msgs::MoveItErrorCodes& error_code);
  
  bool findConstraintAwareSolution(const geometry_msgs::Pose& pose,
                                   const moveit_msgs::Constraints& constraints,

                                   const planning_scene::PlanningSceneConstPtr& scene,
                                   sensor_msgs::JointState& solution,
                                   moveit_msgs::MoveItErrorCodes& error_code,
                                   const bool& do_initial_pose_check = true);
  
  bool findConsistentConstraintAwareSolution(const geometry_msgs::Pose& pose,
                                             const moveit_msgs::Constraints& constraints,
                                             const planning_scene::PlanningSceneConstPtr& scene,
                                             sensor_msgs::JointState& solution,
                                             moveit_msgs::MoveItErrorCodes& error_code,
                                             const unsigned int& redundancy,
                                             const double& max_consistency,
                                             const bool& do_initial_pose_check = true);

  bool interpolateIKDirectional(const geometry_msgs::Pose& start_pose,
                                const Eigen::Vector3d& direction,
                                const double& distance,
                                const moveit_msgs::Constraints& constraints,
                                const planning_scene::PlanningSceneConstPtr& scene,
                                moveit_msgs::MoveItErrorCodes& error_code, 
                                trajectory_msgs::JointTrajectory& traj,
                                const unsigned int& redundancy,
                                const double& max_consistency,
                                const bool& reverse, 
                                const bool& premultiply,
                                const unsigned int& num_points,
                                const ros::Duration& total_dur,
                                const bool& do_initial_pose_check);

  //pass-throughs to solver

  double getSearchDiscretization() const {
    return kinematics_solver_->getSearchDiscretization();
  }

  void setSearchDiscretization(const double& sd) {
    kinematics_solver_->setSearchDiscretization(sd);
  }

  const std::string& getGroupName() const {
    return kinematics_solver_->getGroupName();
  }

  virtual const std::string& getBaseFrame() const {
    return kinematics_solver_->getBaseFrame();
  }

  const std::string& getTipFrame() const {
    return kinematics_solver_->getTipFrame();
  }

  const std::vector<std::string>& getJointNames() const {
    return kinematics_solver_->getJointNames();
  }

  const std::vector<std::string>& getLinkNames() const {
    return kinematics_solver_->getLinkNames();
  }

  const std::vector<std::string>& getEndEffectorLinks() const {
    return end_effector_collision_links_;
  }
  
  const collision_detection::CollisionResult& getLastInitialPoseCheckCollisionResult() const
  {
    return last_initial_pose_check_collision_result_;
  }

protected:

  planning_models::KinematicModelConstPtr kmodel_;
  planning_scene::PlanningSceneConstPtr planning_scene_;
  bool active_;

  kinematics::KinematicsBasePtr kinematics_solver_;

  std::vector<std::string> end_effector_collision_links_;

  //for caching in a particular check
  bool do_initial_pose_check_;
  planning_models::KinematicState* state_;
  moveit_msgs::Constraints constraints_;

  collision_detection::CollisionResult last_initial_pose_check_collision_result_;

  std::string getCollisionDetectedString(const collision_detection::CollisionResult& res);

  void collisionCheck(const geometry_msgs::Pose &ik_pose,
                      const std::vector<double> &ik_solution,
                      moveit_msgs::MoveItErrorCodes &error_code);

  void initialPoseCheck(const geometry_msgs::Pose &ik_pose,
                        const std::vector<double> &ik_solution,
                        moveit_msgs::MoveItErrorCodes &error_code);
};
}
#endif
