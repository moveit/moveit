/**
 * @file control_cost_projection.h
 * @brief This defines a control cost projection update filter
 *
 * This will force goal constraints into the task space.
 *
 * @author Jorge Nicho
 * @date April 12, 2016
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
#ifndef INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_UPDATE_FILTERS_CONTROL_COST_PROJECTION_H_
#define INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_UPDATE_FILTERS_CONTROL_COST_PROJECTION_H_

#include <stomp_moveit/update_filters/stomp_update_filter.h>
#include <Eigen/Core>

namespace stomp_moveit
{
namespace update_filters
{

/**
 * @class stomp_moveit::update_filters
 * @brief Uses a Control Cost Matrix projection in order to smooth out the trajectory.
 *    The matrix is built using a numerical derivative method
 *
 * @par Examples:
 * All examples are located here @ref stomp_moveit_examples
 *
 */
class ControlCostProjection : public StompUpdateFilter
{
public:
  ControlCostProjection();
  virtual ~ControlCostProjection();

  /** @brief see base class for documentation*/
  virtual bool initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                          const std::string& group_name,const XmlRpc::XmlRpcValue& config) override;

  /** @brief see base class for documentation*/
  virtual bool configure(const XmlRpc::XmlRpcValue& config) override;

  /** @brief see base class for documentation*/
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
   * @param parameters        The parameters generated in the previous iteration [num_dimensions x num_timesteps]
   * @param updates           Output argument which contains the updates to be applied to the parameters [num_dimensions x num_timesteps]
   * @param filtered          Output argument which is set to 'true' if the updates were modified.
   * @return false if there was an irrecoverable failure, true otherwise.
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

  // local
  std::string name_;

  // robot properties
  moveit::core::RobotModelConstPtr robot_model_;
  std::string group_name_;

  // smoothing matrix
  int num_timesteps_;
  Eigen::MatrixXd projection_matrix_M_;

};

} /* namespace smoothers */
} /* namespace stomp_moveit */

#endif /* INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_UPDATE_FILTERS_CONTROL_COST_PROJECTION_H_ */
