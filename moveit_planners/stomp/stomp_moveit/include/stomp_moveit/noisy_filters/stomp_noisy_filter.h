/**
 * @file stomp_filter.h
 * @brief This is the base class for all stomp filters.
 *
 * @author Jorge Nicho
 * @date March 23, 2016
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
#ifndef INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_NOISY_FILTER_H_
#define INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_NOISY_FILTER_H_

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
typedef boost::shared_ptr<StompNoisyFilter> StompNoisyFilterPtr;

class StompNoisyFilter
{
public:
  StompNoisyFilter(){}
  virtual ~StompNoisyFilter(){}

  virtual bool initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                          const std::string& group_name,const XmlRpc::XmlRpcValue& config) = 0;

  virtual bool configure(const XmlRpc::XmlRpcValue& config) = 0;

  virtual bool setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                   const moveit_msgs::MotionPlanRequest &req,
                   const stomp_core::StompConfiguration &config,
                   moveit_msgs::MoveItErrorCodes& error_code) = 0;

  /**
   * @brief filters the parameters and modifies the original values
   *
   * @param start_timestep    start index into the 'parameters' array, usually 0.
   * @param num_timesteps     number of elements to use from 'parameters' starting from 'start_timestep'
   * @param iteration_number  The current iteration count in the optimization loop
   * @param rollout_number    index of the noisy trajectory whose cost is being evaluated.
   * @param parameters [num_dimensions] x [num_timesteps]
   * @return false if no filtering was applied
   */
  virtual bool filter(std::size_t start_timestep,
                      std::size_t num_timesteps,
                      int iteration_number,
                      int rollout_number,
                      Eigen::MatrixXd& parameters,
                      bool& filtered) = 0 ;

  /**
   * @brief Called by the Stomp at the end of the optimization process
   *
   * @param success           Whether the optimization succeeded
   * @param total_iterations  Number of iterations used
   * @param final_cost        The cost value after optimizing.
   */
  virtual void done(bool success,int total_iterations,double final_cost){}


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

#endif /* INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_NOISY_FILTER_H_ */
