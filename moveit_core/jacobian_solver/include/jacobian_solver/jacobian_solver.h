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
*
* Author: Sachin Chitta
*********************************************************************/

#ifndef JACOBIAN_SOLVER_H_
#define JACOBIAN_SOLVER_H_

// ROS
#include <ros/ros.h>

// System
#include <boost/shared_ptr.hpp>
#include <algorithm>
#include <numeric>
#include <cstring>
#include <Eigen/Geometry>

#include <planning_models/kinematic_model.h>
#include <planning_models/kinematic_state.h>

namespace jacobian_solver
{
class JacobianSolver
  {
    public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /** @class
     *  @brief A jacobian solver
     */
    JacobianSolver();

    /** 
     *  @brief Specifies if the node is active or not
     *  @return True if the node is active, false otherwise.
     */
    bool initialize(const boost::shared_ptr<const urdf::Model> &urdf_model,
                    const boost::shared_ptr<const srdf::Model> &srdf_model,
                    const std::string &group_name);

    /**
     * @brief Given a set of joint angles, compute the jacobian with reference to a particular point on a given link
     * @param link_index The index number of the link on which the reference point can be found
     * @param joint_angles The complete set of joint angles for the group
     * @param reference_point_position The reference point position (with respect to the link specified in link_index)
     * @return jacobian The resultant jacobian
     */    
    bool getJacobian(const std::string &link_name,
                     const std::vector<double> &joint_angles, 
                     const Eigen::Vector3d &reference_point_position, 
                     Eigen::MatrixXd& jacobian) const;

  private:

    unsigned int num_joints_chain_;
    std::string group_name_;
    boost::shared_ptr<const urdf::Model> urdf_model_;
    boost::shared_ptr<const srdf::Model> srdf_model_;
    planning_models::KinematicModelPtr kinematic_model_;
    planning_models::KinematicModelConstPtr kinematic_model_const_;
    planning_models::KinematicStatePtr kinematic_state_;
    const planning_models::KinematicModel::JointModel* root_joint_model_;
    const planning_models::KinematicModel::JointModelGroup* joint_model_group_;
    planning_models::KinematicState::JointStateGroup* joint_state_group_;

    std::map<std::string,const planning_models::KinematicModel::JointModel*> link_name_to_parent_joint_map_;
    std::vector<std::string> active_joints_;
    Eigen::Affine3d reference_transform_;
    const planning_models::KinematicState::LinkState *root_link_state_;
    std::map<std::string,unsigned int> joint_index_map_;

    std::vector<std::string> group_link_names_,updated_link_names_;
  };
}

#endif
