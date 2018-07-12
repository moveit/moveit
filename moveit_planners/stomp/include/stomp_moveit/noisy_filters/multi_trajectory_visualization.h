/**
 * @file multi_trajectory_visualization.h
 * @brief This defines a multi trajectory visualizer for publishing the noisy trajectories.
 *
 * @author Jorge Nicho
 * @date April 15, 2016
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
#ifndef MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_NOISY_FILTERS_MULTI_TRAJECTORY_VISUALIZATION_H_
#define MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_NOISY_FILTERS_MULTI_TRAJECTORY_VISUALIZATION_H_

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <Eigen/Core>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <stomp_moveit/noisy_filters/stomp_noisy_filter.h>

namespace stomp_moveit
{
namespace noisy_filters
{
/**
 * @class stomp_moveit::noisy_filters::MultiTrajectoryVisualization
 * @brief Publishes rviz markers to visualize the noisy trajectories
 *
 * @par Examples:
 * All examples are located here @ref stomp_moveit_examples
 *
 */
class MultiTrajectoryVisualization : public StompNoisyFilter
{
public:
  MultiTrajectoryVisualization();
  virtual ~MultiTrajectoryVisualization();

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
   * @brief Creates rviz markers for visualizing the noisy trajectories, it does not change the parameters.
   *
   * @param start_timestep    Start index into the 'parameters' array, usually 0.
   * @param num_timesteps     Number of elements to use from 'parameters' starting from 'start_timestep'
   * @param iteration_number  The current iteration count in the optimization loop.
   * @param rollout_number    Index of the noisy trajectory whose cost is being evaluated.
   * @param parameters        Output argument containing the parameters to be filtered [num_dimensions x num_timesteps].
   * @param filtered          Output argument that's set to 'true' if the parameters were changed according to the
   * filtering method.
   * @return false if there was an irrecoverable failure, true otherwise.
   */
  virtual bool filter(std::size_t start_timestep, std::size_t num_timesteps, int iteration_number, int rollout_number,
                      Eigen::MatrixXd& parameters, bool& filtered) override;

  virtual std::string getName() const override
  {
    return name_ + "/" + group_name_;
  }

  virtual std::string getGroupName() const override
  {
    return group_name_;
  }

protected:
  // identity
  std::string name_;

  // robot
  std::string group_name_;
  moveit::core::RobotModelConstPtr robot_model_;
  moveit::core::RobotStatePtr state_;

  // ros comm
  ros::NodeHandle nh_;
  ros::Publisher viz_pub_;

  // parameters
  double line_width_;
  std_msgs::ColorRGBA rgb_;
  std::string marker_topic_;
  std::string marker_namespace_;

  // tool trajectory
  std::size_t traj_total_;
  Eigen::MatrixXd tool_traj_line_;
  visualization_msgs::MarkerArray tool_traj_markers_;
  visualization_msgs::MarkerArray tool_points_markers_;
};

} /* namespace filters */
} /* namespace stomp_moveit */

#endif /* MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_NOISY_FILTERS_MULTI_TRAJECTORY_VISUALIZATION_H_ */
