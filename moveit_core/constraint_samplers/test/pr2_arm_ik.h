//Software License Agreement (BSD License)

//Copyright (c) 2008, Willow Garage, Inc.
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

#ifndef PR2_ARM_IK_H
#define PR2_ARM_IK_H

#include <urdf_model/model.h>
#include <Eigen/Core>
#include <Eigen/LU>// provides LU decomposition
#include <kdl/chainiksolver.hpp>

#include "pr2_arm_kinematics_utils.h"
#include "pr2_arm_kinematics_constants.h"


namespace pr2_arm_kinematics
{
class PR2ArmIK
{
public:
    
  /** @class
   *  @brief Inverse kinematics for the PR2 arm.
   *  @author Sachin Chitta <sachinc@willowgarage.com>
   *
   */
  PR2ArmIK();    
  ~PR2ArmIK(){};

  /** 
      @brief Initialize the solver by providing a urdf::Model and a root and tip name.
      @param A urdf::Model representation of the PR2 robot model
      @param The root joint name of the arm 
      @param The tip joint name of the arm
      @return true if initialization was successful, false otherwise.
  */
  bool init(const urdf::ModelInterface &robot_model, const std::string &root_name, const std::string &tip_name);

  /**
     @brief compute IK based on an initial guess for the shoulder pan angle.
     @param Input pose for end-effector
     @param Initial guess for shoulder pan angle
  */
  void computeIKShoulderPan(const Eigen::Matrix4f &g_in, const double &shoulder_pan_initial_guess,std::vector<std::vector<double> > &solution) const;

  /**
     @brief compute IK based on an initial guess for the shoulder roll angle.
     h       @param Input pose for end-effector
     @param Initial guess for shoulder roll angle
  */
  void computeIKShoulderRoll(const Eigen::Matrix4f &g_in, const double &shoulder_roll_initial_guess,std::vector<std::vector<double> > &solution) const;

    
  //  std::vector<std::vector<double> > solution_ik_;/// a vector of ik solutions

  /**
     @brief get chain information about the arm. This populates the IK query response, filling in joint level information including names and joint limits. 
     @param The response structure to be filled in.
  */
  void getSolverInfo(moveit_msgs::KinematicSolverInfo &info);

  /**
     @brief get chain information about the arm.
  */
  moveit_msgs::KinematicSolverInfo solver_info_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private:

  void addJointToChainInfo(boost::shared_ptr<const urdf::Joint> joint,moveit_msgs::KinematicSolverInfo &info);

  bool checkJointLimits(const std::vector<double> &joint_values) const;
 
  bool checkJointLimits(const double &joint_value, const int &joint_num) const;

  Eigen::Matrix4f grhs_, gf_, home_inv_, home_;

  std::vector<double> angle_multipliers_;

  std::vector<double> solution_;

  double shoulder_upperarm_offset_, upperarm_elbow_offset_, elbow_wrist_offset_, shoulder_wrist_offset_, shoulder_elbow_offset_, torso_shoulder_offset_x_, torso_shoulder_offset_y_, torso_shoulder_offset_z_;

  std::vector<double> min_angles_;

  std::vector<double> max_angles_;

  std::vector<bool> continuous_joint_;

};
}
#endif// PR2_ARM_IK_H
