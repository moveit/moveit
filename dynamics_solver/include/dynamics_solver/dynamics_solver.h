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

/* Author: Sachin Chitta */

#include <geometry_msgs/Wrench.h>

// KDL
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

#include <urdf/model.h>
#include <srdf/model.h>
#include <planning_models/kinematic_model.h>
#include <planning_models/kinematic_state.h>
#include <geometry_msgs/Vector3.h>

namespace dynamics_solver
{
class DynamicsSolver
{
public:
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  DynamicsSolver();
  
  bool initialize(const boost::shared_ptr<const urdf::Model> &urdf_model,                    
                  const boost::shared_ptr<const srdf::Model> &srdf_model,
                  const std::string &group_name);
      
  bool getTorques(const std::vector<double> &joint_angles,
                  const std::vector<double> &joint_velocities,
                  const std::vector<double> &joint_accelerations,
                  const std::vector<geometry_msgs::Wrench> &wrenches,
                  std::vector<double> &torques) const;
    
  bool getMaxPayload(const std::vector<double> &joint_angles,
                     double &payload,
                     unsigned int &joint_saturated) const;

  bool getPayloadTorques(const std::vector<double> &joint_angles,
                         double &payload,
                         std::vector<double> &joint_torques) const;

  const std::vector<double>& getMaxTorques() const;  
    
private:

  boost::shared_ptr<KDL::ChainIdSolver_RNE> chain_id_solver_;
  KDL::Chain kdl_chain_;
  boost::shared_ptr<const urdf::Model> urdf_model_;
  boost::shared_ptr<const srdf::Model> srdf_model_;

  std::string group_name_, base_name_, tip_name_;
  unsigned int num_joints_;
  std::vector<double> max_torques_;

  planning_models::KinematicModelPtr kinematic_model_;
  const planning_models::KinematicModel::JointModelGroup* joint_model_group_;

  geometry_msgs::Vector3 transformVector(const Eigen::Affine3d &transform, 
                                         const geometry_msgs::Vector3 &vector) const;

  planning_models::KinematicStatePtr kinematic_state_;
  planning_models::KinematicState::JointStateGroup* joint_state_group_;
  
};

typedef boost::shared_ptr<DynamicsSolver> DynamicsSolverPtr;
typedef boost::shared_ptr<const DynamicsSolver> DynamicsSolverConstPtr;

}
