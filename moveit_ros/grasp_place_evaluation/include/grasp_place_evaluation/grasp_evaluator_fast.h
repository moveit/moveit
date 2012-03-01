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

#ifndef _GRASP_EVALUATOR_FAST_
#define _GRASP_EVALUATOR_FAST_

#include <grasp_place_evaluation/grasp_evaluator.h>
#include <kinematics_constraint_aware/kinematics_solver_constraint_aware.h>

namespace grasp_place_evaluation {

//! Uses an interpolated IK approach from pregrasp to final grasp
/*! Initial check consists of IK for the pre-grasp, followed by generation
  of an interpolated IK path from pre-grasp to grasp. This is then followed
  by computation of an interpolated IK path from grasp to lift.

  Execution consists of using move arm to go to the pre-grasp, then execution of 
  the interpolated IK paths for grasp. Lift is executed separately, called from
  the higher level executor.

  Note that we do not use the pre-grasp from the database directly; rather, the
  pre-grasp is obtained by backing up the grasp by a pre-defined amount. This
  allows us more flexibility in choosing the pre-grasp. However, we can no longer
  always assume the pre-grasp is collision free, which we could if we used the 
  pre-grasp from the database directly.

  In the most recent database version, the pre-grasp was obtained by backing up 
  10 cm, so we know at least that that is not colliding with the object. 
*/

inline geometry_msgs::Vector3 doNegate(const geometry_msgs::Vector3& vec) {
  geometry_msgs::Vector3 v;
  v.x = - vec.x;
  v.y = - vec.y;
  v.z = - vec.z;
  return v;
}

class GraspEvaluatorFast : public GraspEvaluator
{
public:

  GraspEvaluatorFast(const planning_models::KinematicModelConstPtr& kmodel,
                     const std::map<std::string, kinematics::KinematicsBasePtr>& solver_map);

  ~GraspEvaluatorFast(){}

  void getGroupJoints(const std::string& group_name,
                      std::vector<std::string>& group_links);
  
  void getGroupLinks(const std::string& group_name,
                     std::vector<std::string>& group_links);

  std::string getEndEffectorName(const boost::shared_ptr<const srdf::Model>& srdf_model,
                                 const std::string& arm_name);

  std::string getTipLink(const std::string& group_name);

  std::string getAttachLink(const std::string& end_effector_name);  

  virtual void testGrasps(const planning_scene::PlanningSceneConstPtr& planning_scene,
                          const planning_models::KinematicState* seed_state,
                          const moveit_manipulation_msgs::PickupGoal &pickup_goal,
                          const std::vector<moveit_manipulation_msgs::Grasp> &grasps,
                          std::vector<GraspExecutionInfo> &execution_info,
                          bool return_on_first_hit);
    
protected:
  bool getInterpolatedIK(const std::string& arm_name,
                         const planning_scene::PlanningSceneConstPtr& scene,
                         const collision_detection::AllowedCollisionMatrix& acm,
                         const geometry_msgs::Pose& first_pose,
                         const Eigen::Vector3d& direction,
                         const double& distance,
                         const std::vector<double>& ik_solution,
                         const bool& reverse, 
                         const bool& premultiply,
                         const bool& use_unpadded_robot,
                         planning_models::KinematicState* seed_state,
                         trajectory_msgs::JointTrajectory& traj);
  
  const planning_models::KinematicModelConstPtr kmodel_;
  std::map<std::string, boost::shared_ptr<kinematics_constraint_aware::KinematicsSolverConstraintAware> > constraint_aware_solver_map_;
  
  double consistent_angle_;
  unsigned int num_points_;
  unsigned int redundancy_;
  
};

} //namespace grasp_execution

#endif
