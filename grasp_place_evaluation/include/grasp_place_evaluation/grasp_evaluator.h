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

#ifndef _GRASP_EVALUATOR_H_
#define _GRASP_EVALUATOR_H_

#include <ros/ros.h>

#include <moveit_manipulation_msgs/GraspResult.h>
#include <moveit_manipulation_msgs/PickupGoal.h>

#include <trajectory_msgs/JointTrajectory.h>

namespace grasp_place_evaluation {

//! Encapsulates the result of feasibility testing and the information needed for eexcuting
//! a grasp, assuming an approach-grasp-lift method.
struct GraspExecutionInfo {
  trajectory_msgs::JointTrajectory approach_trajectory_; 
  trajectory_msgs::JointTrajectory lift_trajectory_; 
  moveit_manipulation_msgs::GraspResult result_;
  int marker_id_;
};

// ---------------------------- Definitions ---------------------------------

//! Tests grasps for feasibility in the current environment, and generates info
//! needed for execution
class GraspEvaluator {
protected:
  //! Tests a single grasp
  virtual void testGrasp(const moveit_manipulation_msgs::PickupGoal &pickup_goal,
                         const moveit_manipulation_msgs::Grasp &grasp,
                         GraspExecutionInfo &execution_info) = 0;  

  //! Function used to provide feedback on which grasp is being tested
  //! Might find a more elegant mechanism in the future
  boost::function<void(size_t)> feedback_function_;

  //! Function used to check for interrupts
  boost::function<bool()> interrupt_function_;
public:
  GraspTester() : marker_publisher_(NULL) {}

  //! Tests a set of grasps and provides their execution info
  virtual void testGrasps(const object_manipulation_msgs::PickupGoal &pickup_goal,
                          const std::vector<object_manipulation_msgs::Grasp> &grasps,
                          std::vector<GraspExecutionInfo> &execution_info,
                          bool return_on_first_hit);

  //! Sets the feedback function
  void setFeedbackFunction(boost::function<void(size_t)> f){feedback_function_ = f;}

  //! Sets the interrupt function
  void setInterruptFunction(boost::function<bool()> f){interrupt_function_ = f;}

  //! Helper function for convenience
  object_manipulation_msgs::GraspResult Result(int result_code, bool continuation)
  {
    object_manipulation_msgs::GraspResult result;
    result.result_code = result_code;
    result.continuation_possible = continuation;
    return result;
  }
};

}

#endif
