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

#ifndef KINEMATICS_METRICS_H_
#define KINEMATICS_METRICS_H_

#include <planning_models/kinematic_model.h>
#include <planning_models/kinematic_state.h>
#include <planning_scene/planning_scene.h>

namespace kinematics_metrics
{
/**
 * @class Compute different kinds of metrics for kinematics evaluation
 */
class KinematicsMetrics
{
public:  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
  /** @class
   *  @brief Kinematics metrics
   */
  KinematicsMetrics(const planning_models::KinematicModelConstPtr &kinematic_model):kinematic_model_(kinematic_model){};    
  
  bool getManipulabilityIndex(const planning_models::KinematicState &kinematic_state, 
                              const std::string &group_name,
                              double &manipulability_index) const;
    
  bool getManipulabilityEllipsoid(const planning_models::KinematicState &kinematic_state,
                                  const std::string &group_name,
                                  Eigen::MatrixXcd &eigen_values,
                                  Eigen::MatrixXcd &eigen_vectors) const;
  
  bool getConditionNumber(const planning_models::KinematicState &kinematic_state,
                          const std::string &group_name,
                          double &condition_number);

protected:
  
  planning_models::KinematicModelConstPtr kinematic_model_;
  
  bool checkState(const planning_models::KinematicState &kinematic_state,
                  const std::string &group_name) const;
  
  Eigen::MatrixXd getJacobian(const planning_models::KinematicState &kinematic_state,
                              const std::string &group_name) const;
  
};

typedef boost::shared_ptr<KinematicsMetrics> KinematicsMetricsPtr;
typedef boost::shared_ptr<const KinematicsMetrics> KinematicsMetricsConstPtr;

}

#endif
