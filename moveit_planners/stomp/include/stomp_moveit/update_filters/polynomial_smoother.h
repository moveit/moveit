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
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_UPDATE_FILTERS_POLYNOMIAL_SMOOTHER_H_
#define MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_UPDATE_FILTERS_POLYNOMIAL_SMOOTHER_H_

#include <stomp_moveit/update_filters/stomp_update_filter.h>

namespace stomp_moveit
{
namespace update_filters
{
/**
 * @class stomp_moveit::update_filters::PolynomialSmoother
 * @brief This is a constrained polynomial trajectory smoother.
 *
 * It uses the constrained least-squares technique shown below.
 *
 * |p| - | 2*A*A', C |^-1 * | 2*A*b |
 * |z| - |     C', 0 |      |     d |
 *
 * Where:
 *   p - An array of the polynomial coefficients solved for.
 *   z - An array of Lagrange mulitipliers
 *   A - Is the Vandermonde matrix of all (constrained and unconstrained) domain values
 *   C - Is the Vandermonde matrix of the constrained domain values
 *   b - An array of the values to perform the fit on
 *   d - An array of the values corresponding to the constrained domain values
 *
 * @par Examples:
 * All examples are located here @ref stomp_moveit_examples
 *
 */
class PolynomialSmoother : public StompUpdateFilter
{
public:
  PolynomialSmoother();
  virtual ~PolynomialSmoother();

  /** @brief see base class for documentation*/
  virtual bool initialize(moveit::core::RobotModelConstPtr robot_model_ptr, const std::string& group_name,
                          const XmlRpc::XmlRpcValue& config) override;

  /** @brief see base class for documentation*/
  virtual bool configure(const XmlRpc::XmlRpcValue& config) override;

  /** @brief see base class for documentation*/
  virtual bool setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                    const moveit_msgs::MotionPlanRequest& req,
                                    const stomp_core::StompConfiguration& config,
                                    moveit_msgs::MoveItErrorCodes& error_code) override;

  /**
   * @brief smoothes the updates array by using a constrained polynomial fit.
   *
   * @param start_timestep    start index into the 'parameters' array, usually 0.
   * @param num_timesteps     number of elements to use from 'parameters' starting from 'start_timestep'
   * @param iteration_number  The current iteration count in the optimization loop
   * @param parameters        The parameters generated in the previous iteration [num_dimensions x num_timesteps]
   * @param updates           The updates to be applied to the parameters [num_dimensions x num_timesteps]
   * @param filtered          set ot 'true' if the updates were modified.
   * @return false if there was an irrecoverable failure, true otherwise.
   */
  virtual bool filter(std::size_t start_timestep, std::size_t num_timesteps, int iteration_number,
                      const Eigen::MatrixXd& parameters, Eigen::MatrixXd& updates, bool& filtered) override;

  virtual std::string getGroupName() const
  {
    return group_name_;
  }

  virtual std::string getName() const
  {
    return name_ + "/" + group_name_;
  }

protected:
  std::string name_;
  std::string group_name_;

  // parameters
  unsigned int poly_order_;

  // robot
  moveit::core::RobotModelConstPtr robot_model_;
};

} /* namespace update_filters */
} /* namespace stomp_moveit */

#endif /* INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_UPDATE_FILTERS_POLYNOMIAL_SMOOTHER_H_ */
