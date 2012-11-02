/*********************************************************************
*
*  Copyright (c) 2009, Willow Garage, Inc.
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

// Author(s): E. Gil Jones

#include <grasp_place_evaluation/interpolation_evaluator.h>

namespace grasp_place_evaluation {

InterpolationEvaluator::InterpolationEvaluator(const planning_models::KinematicModelConstPtr& kmodel,
                                               const std::map<std::string, kinematics::KinematicsBasePtr>& solver_map,
                                               const double consistent_angle,
                                               const unsigned int num_points,
                                               const unsigned int redundancy,
                                               const double discretization)
  : kmodel_(kmodel),
    consistent_angle_(consistent_angle), 
    num_points_(num_points), 
    redundancy_(redundancy)
{  
  ROS_DEBUG_STREAM("Consistent angle is " << consistent_angle_);

  for(std::map<std::string, kinematics::KinematicsBasePtr>::const_iterator it = solver_map.begin(); 
      it != solver_map.end();
      it++) {
    constraint_aware_solver_map_[it->first].reset(new kinematics_constraint_aware::KinematicsSolverConstraintAware(it->second, kmodel_, it->first));
    constraint_aware_solver_map_[it->first]->setSearchDiscretization(discretization);
  }
}

bool InterpolationEvaluator::getInterpolatedIK(const std::string& arm_name,
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
                                               trajectory_msgs::JointTrajectory& traj) 
{
  moveit_msgs::MoveItErrorCodes error_code;
  return constraint_aware_solver_map_[arm_name]->interpolateIKDirectional(first_pose,
                                                                          direction,
                                                                          distance,
                                                                          path_constraints,
                                                                          seed_state,
                                                                          scene,
                                                                          acm,
                                                                          error_code,
                                                                          traj,
                                                                          redundancy_, 
                                                                          consistent_angle_,
                                                                          reverse,
                                                                          premultiply,
                                                                          num_points_,
                                                                          ros::Duration(2.5),
                                                                          false, 
                                                                          use_unpadded_robot);
}

} //namespace grasp_place_evaluation
