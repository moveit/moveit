/*********************************************************************
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
*   * Neither the name of Willow Garage Inc. nor the names of its
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

/* Author: Sachin Chitta */

#ifndef MOVEIT_DYNAMICS_SOLVER_DYNAMICS_SOLVER_
#define MOVEIT_DYNAMICS_SOLVER_DYNAMICS_SOLVER_

// KDL
#include <kdl/chain.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

#include <moveit/kinematic_state/kinematic_state.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Wrench.h>

namespace dynamics_solver
{
/**
 * @class This solver currently computes the required torques given a 
 * joint configuration, velocities, accelerations and external wrenches 
 * acting on the links of a robot
 */
class DynamicsSolver
{
public:
  
  DynamicsSolver();
  
  /**
   * @brief Initialize the dynamics solver
   * @param urdf_model The urdf model for the robot
   * @param srdf_model The srdf model for the robot
   * @param group_name The name of the group to compute stuff for
   * @return False if initialization failed
   */
  bool initialize(const boost::shared_ptr<const urdf::ModelInterface> &urdf_model,                    
                  const boost::shared_ptr<const srdf::Model> &srdf_model,
                  const std::string &group_name);
      
  /**
   * @brief Get the torques
   * @param joint_angles The joint angles (desired joint configuration)
   * this must have size = number of joints in the group
   * @param joint_velocities The desired joint velocities
   * this must have size = number of joints in the group
   * @param joint_accelerations The desired joint accelerations
   * this must have size = number of joints in the group
   * @param wrenches External wrenches acting on the links of the robot
   * this must have size = number of links in the group
   * @param torques Computed set of torques are filled in here
   * this must have size = number of joints in the group
   * @return False if any of the input vectors are of the wrong size
   */
  bool getTorques(const std::vector<double> &joint_angles,
                  const std::vector<double> &joint_velocities,
                  const std::vector<double> &joint_accelerations,
                  const std::vector<geometry_msgs::Wrench> &wrenches,
                  std::vector<double> &torques) const;
    
  /**
   * @brief Get the maximum payload for this group (in kg). Payload is 
   * the weight that this group can hold when the weight is attached to the origin
   * of the last link of this group
   * @param joint_angles The joint angles (desired joint configuration)
   * this must have size = number of joints in the group
   * @param payload The computed maximum payload
   * @param joint_saturation The first saturated joint and the maximum payload
   * @return False if the input set of joint angles is of the wrong size
   */
  bool getMaxPayload(const std::vector<double> &joint_angles,
                     double &payload,
                     unsigned int &joint_saturated) const;

  /**
   * @brief Get torques corresponding to a particular payload value.  Payload is 
   * the weight that this group can hold when the weight is attached to the origin
   * of the last link of this group.
   * @param joint_angles The joint angles (desired joint configuration)
   * this must have size = number of joints in the group
   * @param payload The computed maximum payload
   * @param joint_torques The resulting joint torques
   * @return False if the input vectors are of the wrong size
   */
  bool getPayloadTorques(const std::vector<double> &joint_angles,
                         double &payload,
                         std::vector<double> &joint_torques) const;

  /**
   * @brief Get maximum torques for this group
   * @return False vector of max torques
   */
  const std::vector<double>& getMaxTorques() const;  
    
private:

  boost::shared_ptr<KDL::ChainIdSolver_RNE> chain_id_solver_; // KDL chain inverse dynamics
  KDL::Chain kdl_chain_; // KDL chain
  boost::shared_ptr<const urdf::ModelInterface> urdf_model_; // URDF model
  boost::shared_ptr<const srdf::Model> srdf_model_; // SRDF model

  std::string group_name_, base_name_, tip_name_; // group name, base name, tip name 
  unsigned int num_joints_, num_segments_; // number of joints in group, number of segments in group
  std::vector<double> max_torques_; // vector of max torques

  kinematic_model::KinematicModelPtr kinematic_model_; // kinematic model
  const kinematic_model::JointModelGroup* joint_model_group_; //joint model group

  geometry_msgs::Vector3 transformVector(const Eigen::Affine3d &transform, 
                                         const geometry_msgs::Vector3 &vector) const; //local transform function

  kinematic_state::KinematicStatePtr kinematic_state_; //kinematic state
  kinematic_state::JointStateGroup* joint_state_group_; //joint state for the group
  
};

typedef boost::shared_ptr<DynamicsSolver> DynamicsSolverPtr;
typedef boost::shared_ptr<const DynamicsSolver> DynamicsSolverConstPtr;

}
#endif
