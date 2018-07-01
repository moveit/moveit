/**
 * @file update_logger.h
 * @brief This defines a update trajectory logger which writes the data to a file.
 *
 * @author Jorge Nicho
 * @date April 13, 2016
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
#ifndef MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_UPDATE_FILTERS_UPDATE_LOGGER_H_
#define MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_UPDATE_FILTERS_UPDATE_LOGGER_H_

#include <stomp_moveit/update_filters/stomp_update_filter.h>
#include <fstream>

namespace stomp_moveit
{
namespace update_filters
{

/**
 * @class stomp_moveit::update_filters::UpdateLogger
 * @brief Saves the update values into a file for post analysis.  The file is compatible with the python numpy
 *  library and can be loaded into a numpy array by running.
 *    'numpy.loadtxt(file_name)'
 *
 * @par Examples:
 * All examples are located here @ref stomp_moveit_examples
 *
 */
class UpdateLogger : public StompUpdateFilter
{
public:
  UpdateLogger();
  virtual ~UpdateLogger();

  /** @brief see base class for documentation*/
  virtual bool initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                          const std::string& group_name,const XmlRpc::XmlRpcValue& config);

  /** @brief see base class for documentation*/
  virtual bool configure(const XmlRpc::XmlRpcValue& config);

  /** @brief see base class for documentation*/
  virtual bool setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                   const moveit_msgs::MotionPlanRequest &req,
                   const stomp_core::StompConfiguration &config,
                   moveit_msgs::MoveItErrorCodes& error_code);

  /**
   * @brief Store the updates values into a file that can be loaded into a numpy array.
   *
   * @param start_timestep    start index into the 'parameters' array, usually 0.
   * @param num_timesteps     number of elements to use from 'parameters' starting from 'start_timestep'
   * @param iteration_number  The current iteration count in the optimization loop
   * @param parameters        The parameters generated in the previous iteration [num_dimensions x num_timesteps]
   * @param updates           The updates to be applied to the parameters [num_dimensions x num_timesteps]
   * @param filtered          Always false as this filter never changes the updates values.
   * @return false if there was an irrecoverable failure, true otherwise.
   */
  virtual bool filter(std::size_t start_timestep,
                      std::size_t num_timesteps,
                      int iteration_number,
                      const Eigen::MatrixXd& parameters,
                      Eigen::MatrixXd& updates,
                      bool& filtered) override;

  virtual std::string getGroupName() const override
  {
    return group_name_;
  }

  virtual std::string getName() const override
  {
    return name_ + "/" + group_name_;
  }

  virtual void done(bool success, int total_iterations,double final_cost,const Eigen::MatrixXd& parameters) override;

protected:

  std::string name_;
  std::string group_name_;

  // parameters
  std::string filename_;
  std::string package_;
  std::string directory_;

  // config
  stomp_core::StompConfiguration stomp_config_;

  // logging
  std::stringstream stream_;
  std::string full_file_name_;
  std::ofstream file_stream_;
  Eigen::IOFormat format_;

};

} /* namespace smoothers */
} /* namespace stomp_moveit */

#endif /* MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_UPDATE_FILTERS_UPDATE_LOGGER_H_ */
