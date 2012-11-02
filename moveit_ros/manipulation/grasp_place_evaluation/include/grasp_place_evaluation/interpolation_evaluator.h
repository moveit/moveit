/*********************************************************************
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

#ifndef _INTERPOLATION_EVALUATOR_H_
#define _INTERPOLATION_EVALUATOR_H_

#include <ros/ros.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <planning_scene/planning_scene.h>
#include <planning_models/kinematic_state.h>
#include <kinematics_constraint_aware/kinematics_solver_constraint_aware.h>

namespace grasp_place_evaluation {

class InterpolationEvaluator {

public:

  static inline geometry_msgs::Vector3 doNegate(const geometry_msgs::Vector3& vec) {
    geometry_msgs::Vector3 v;
    v.x = - vec.x;
    v.y = - vec.y;
    v.z = - vec.z;
    return v;
  }

  InterpolationEvaluator(const planning_models::KinematicModelConstPtr& kmodel,
                         const std::map<std::string, kinematics::KinematicsBasePtr>& solver_map,
                         const double consistent_angle = M_PI/12.0,
                         const unsigned int num_points = 10,
                         const unsigned int redundancy = 2,
                         const double discretization = .05);

  virtual ~InterpolationEvaluator() {};

protected:

  bool getInterpolatedIK(const std::string& arm_name,
                         const planning_scene::PlanningSceneConstPtr& scene,
                         const collision_detection::AllowedCollisionMatrix& acm,
                         const moveit_msgs::Constraints& path_constraints,
                         const geometry_msgs::Pose& first_pose,
                         const Eigen::Vector3d& direction,
                         const double& distance,
                         const bool& reverse, 
                         const bool& premultiply,
                         const bool& use_unpadded_robot,
                         const planning_models::KinematicState* seed_state,
                         trajectory_msgs::JointTrajectory& traj);
  
  const planning_models::KinematicModelConstPtr kmodel_;
  std::map<std::string, boost::shared_ptr<kinematics_constraint_aware::KinematicsSolverConstraintAware> > constraint_aware_solver_map_;
  
  double consistent_angle_;
  unsigned int num_points_;
  unsigned int redundancy_;
  
};

} //namespace grasp_place_evaluation

#endif
