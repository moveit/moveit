/**
 * @file stomp_noisy_filter.h
 * @brief This is the base class for all stomp filters.
 *
 * @author Jorge Nicho
 * @date March 23, 2016
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
#ifndef MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_NOISY_FILTER_H_
#define MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_NOISY_FILTER_H_

#include <Eigen/Core>
#include <XmlRpc.h>
#include <stomp_core/utils.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/MotionPlanRequest.h>

namespace stomp_moveit
{

namespace noisy_filters
{

class StompNoisyFilter;
typedef std::shared_ptr<StompNoisyFilter> StompNoisyFilterPtr;


/**
 * @class stomp_moveit::noisy_filters::StompNoisyFilter
 * @brief Interface class for filtering noisy trajectories
 *
 * @par Examples:
 * All examples are located here @ref stomp_moveit_examples
 */
class StompNoisyFilter
{
public:
  StompNoisyFilter(){}
  virtual ~StompNoisyFilter(){}

  /**
   * @brief Initializes and configures.
   * @param robot_model_ptr A pointer to the robot model.
   * @param group_name      The designated planning group.
   * @param config          The configuration data.  Usually loaded from the ros parameter server
   * @return true if succeeded, false otherwise.
   */
  virtual bool initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                          const std::string& group_name,const XmlRpc::XmlRpcValue& config) = 0;

  /**
   * @brief Sets internal members of the plugin from the configuration data.
   * @param config  The configuration data.  Usually loaded from the ros parameter server
   * @return  true if succeeded, false otherwise.
   */
  virtual bool configure(const XmlRpc::XmlRpcValue& config) = 0;

  /**
   * @brief Stores the planning details.
   * @param planning_scene      A smart pointer to the planning scene
   * @param req                 The motion planning request
   * @param config              The  Stomp configuration.
   * @param error_code          Moveit error code.
   * @return  true if succeeded, false otherwise.
   */
  virtual bool setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                   const moveit_msgs::MotionPlanRequest &req,
                   const stomp_core::StompConfiguration &config,
                   moveit_msgs::MoveItErrorCodes& error_code) = 0;

  /**
   * @brief Applies a filtering method to the parameters which may modify the original values
   *
   * @param start_timestep    Start index into the 'parameters' array, usually 0.
   * @param num_timesteps     Number of elements to use from 'parameters' starting from 'start_timestep'
   * @param iteration_number  The current iteration count in the optimization loop.
   * @param rollout_number    Index of the noisy trajectory whose cost is being evaluated.
   * @param parameters        Output argument containing the parameters to be filtered [num_dimensions x num_timesteps].
   * @param filtered          Output argument that's set to 'true' if the parameters were changed according to the filtering method.
   * @return false if there was an irrecoverable failure, true otherwise.
   */
  virtual bool filter(std::size_t start_timestep,
                      std::size_t num_timesteps,
                      int iteration_number,
                      int rollout_number,
                      Eigen::MatrixXd& parameters,
                      bool& filtered) = 0 ;

  /**
   * @brief Called by STOMP at the end of each iteration.
   * @param start_timestep    The start index into the 'parameters' array, usually 0.
   * @param num_timesteps     The number of elements to use from 'parameters' starting from 'start_timestep'
   * @param iteration_number  The current iteration count in the optimization loop
   * @param cost              The cost value for the current parameters.
   * @param parameters        The value of the parameters at the end of the current iteration [num_dimensions x num_timesteps].
   */
  virtual void postIteration(std::size_t start_timestep,
                                std::size_t num_timesteps,int iteration_number,double cost,const Eigen::MatrixXd& parameters){}

  /**
   * @brief Called by the Stomp at the end of the optimization process
   *
   * @param success           Whether the optimization succeeded
   * @param total_iterations  Number of iterations used
   * @param final_cost        The cost value after optimizing.
   * @param parameters        The parameters generated at the end of current iteration[num_dimensions x num_timesteps]
   */
  virtual void done(bool success,int total_iterations,double final_cost,const Eigen::MatrixXd& parameters){}


  virtual std::string getName() const
  {
    return "Not implemented";
  }


  virtual std::string getGroupName() const
  {
    return "Not implemented";
  }


};

} /* namespace filters */

} /* namespace stomp_moveit */

#endif /* MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_NOISY_FILTER_H_ */
