/**
 * @file polynomial_smoother.h
 * @brief This defines a polynomial smoother update filter
 *
 * This smooths the noisy update.
 *
 * @author Jorge Nicho
 * @date June 3, 2016
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2016, Southwest Research Institute
 *
 * @license Software License Agreement (Apache License)\n
 * \n
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at\n
 * \n
 * http://www.apache.org/licenses/LICENSE-2.0\n
 * \n
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_UPDATE_FILTERS_POLYNOMIAL_SMOOTHER_H_
#define INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_UPDATE_FILTERS_POLYNOMIAL_SMOOTHER_H_

#include <stomp_moveit/update_filters/stomp_update_filter.h>

namespace stomp_moveit
{
namespace update_filters
{

class PolynomialSmoother : public StompUpdateFilter
{
public:
  PolynomialSmoother();
  virtual ~PolynomialSmoother();

  virtual bool initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                          const std::string& group_name,const XmlRpc::XmlRpcValue& config) override;

  virtual bool configure(const XmlRpc::XmlRpcValue& config) override;

  virtual bool setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                   const moveit_msgs::MotionPlanRequest &req,
                   const stomp_core::StompConfiguration &config,
                   moveit_msgs::MoveItErrorCodes& error_code) override;

  /**
   * @brief smoothes the updates array.  Uses the Control Cost Matrix projection.
   *
   * @param start_timestep    start index into the 'parameters' array, usually 0.
   * @param num_timesteps     number of elements to use from 'parameters' starting from 'start_timestep'
   * @param iteration_number  The current iteration count in the optimization loop
   * @param parameters        The parameters generated in the previous iteration [num_dimensions] x [num_timesteps]
   * @param updates           The updates to be applied to the parameters [num_dimensions] x [num_timesteps]
   * @param filtered          set ot 'true' if the updates were modified.
   * @return false if something failed
   */
  virtual bool filter(std::size_t start_timestep,
                      std::size_t num_timesteps,
                      int iteration_number,
                      const Eigen::MatrixXd& parameters,
                      Eigen::MatrixXd& updates,
                      bool& filtered) override;

  virtual std::string getGroupName() const
  {
    return group_name_;
  }

  virtual std::string getName() const
  {
    return name_ + "/" + group_name_;
  }

protected:

  void fillVandermondeMatrix(double poly_order,const Eigen::ArrayXd& domain_vals,Eigen::MatrixXd& X);

protected:

  std::string name_;
  std::string group_name_;

  // parameters
  unsigned int poly_order_;

  // temp
  Eigen::ArrayXd domain_vals_;
  Eigen::MatrixXd X_matrix_;
  Eigen::MatrixXd X_pseudo_inv_;
  Eigen::VectorXd smoothed_parameters_;

  // robot
  moveit::core::RobotModelConstPtr robot_model_;
};

} /* namespace update_filters */
} /* namespace stomp_moveit */

#endif /* INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_UPDATE_FILTERS_POLYNOMIAL_SMOOTHER_H_ */
