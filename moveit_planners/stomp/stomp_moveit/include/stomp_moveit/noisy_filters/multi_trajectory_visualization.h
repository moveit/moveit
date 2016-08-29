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
#ifndef INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_NOISY_FILTERS_MULTI_TRAJECTORY_VISUALIZATION_H_
#define INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_NOISY_FILTERS_MULTI_TRAJECTORY_VISUALIZATION_H_

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

class MultiTrajectoryVisualization : public StompNoisyFilter
{
public:
  MultiTrajectoryVisualization();
  virtual ~MultiTrajectoryVisualization();

  virtual bool initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                          const std::string& group_name,const XmlRpc::XmlRpcValue& config) override;

  virtual bool configure(const XmlRpc::XmlRpcValue& config) override;

  virtual bool setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                   const moveit_msgs::MotionPlanRequest &req,
                   const stomp_core::StompConfiguration &config,
                   moveit_msgs::MoveItErrorCodes& error_code) override;

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
                      bool& filtered) override;


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

#endif /* INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_NOISY_FILTERS_MULTI_TRAJECTORY_VISUALIZATION_H_ */
