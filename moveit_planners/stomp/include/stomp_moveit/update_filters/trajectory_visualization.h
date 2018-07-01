/**
 * @file trajectory_visualization.h
 * @brief This defines a trajectory visualizer for publishing the smooth trajectory.
 *
 * @author Jorge Nicho
 * @date April 14, 2016
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
#ifndef MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_UPDATE_FILTERS_TRAJECTORY_VISUALIZATION_H_
#define MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_UPDATE_FILTERS_TRAJECTORY_VISUALIZATION_H_

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <Eigen/Core>
#include <geometry_msgs/Point.h>
#include <stomp_moveit/update_filters/stomp_update_filter.h>
#include <visualization_msgs/Marker.h>

namespace stomp_moveit
{
namespace update_filters
{

/**
 * @class stomp_moveit::update_filters::TrajectoryVisualization
 * @brief Publishes rviz markers to visualize the optimized trajectory
 *
 * @par Examples:
 * All examples are located here @ref stomp_moveit_examples
 *
 */
class TrajectoryVisualization : public StompUpdateFilter
{
public:
  TrajectoryVisualization();
  virtual ~TrajectoryVisualization();

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
   * @brief Publishes rviz markers to visualize the optimized trajectory at each iteration.
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

  /**
   * @brief Called by the Stomp at the end of the optimization process
   *
   * @param success           Whether the optimization succeeded
   * @param total_iterations  Number of iterations used
   * @param final_cost        The cost value after optimizing.
   * @param parameters        The parameters generated at the end of current iteration [num_dimensions x num_timesteps]
   */
  virtual void done(bool success,int total_iterations,double final_cost,const Eigen::MatrixXd& parameters) override;


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
  std_msgs::ColorRGBA error_rgb_;
  bool publish_intermediate_;
  std::string marker_topic_;
  std::string marker_namespace_;

  // tool trajectory
  Eigen::MatrixXd tool_traj_line_;
  visualization_msgs::Marker tool_traj_marker_;


};

} /* namespace filters */
} /* namespace stomp_moveit */

#endif /* MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_UPDATE_FILTERS_TRAJECTORY_VISUALIZATION_H_ */
