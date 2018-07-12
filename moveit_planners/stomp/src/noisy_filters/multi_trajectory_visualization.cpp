/**
 * @file multi_trajectory_visualization.cpp
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
#include <moveit/robot_state/conversions.h>
#include <tf/transform_datatypes.h>
#include <pluginlib/class_list_macros.h>
#include <stomp_moveit/noisy_filters/multi_trajectory_visualization.h>
#include <eigen_conversions/eigen_msg.h>

PLUGINLIB_EXPORT_CLASS(stomp_moveit::noisy_filters::MultiTrajectoryVisualization,
                       stomp_moveit::noisy_filters::StompNoisyFilter);

inline void eigenToPointsMsgs(const Eigen::MatrixXd& in, std::vector<geometry_msgs::Point>& out)
{
  // resizing
  if (out.size() != in.cols())
  {
    out.resize(in.cols());
  }

  // copying points
  for (auto t = 0u; t < in.cols(); t++)
  {
    out[t].x = in(0, t);
    out[t].y = in(1, t);
    out[t].z = in(2, t);
  }
}

inline void createToolPathMarker(const Eigen::MatrixXd& tool_line, int id, std::string frame_id,
                                 const std_msgs::ColorRGBA& rgb, double line_width, std::string ns,
                                 visualization_msgs::Marker& m)
{
  m.ns = ns;
  m.id = id;
  m.header.frame_id = frame_id;
  m.type = m.LINE_STRIP;
  m.action = m.ADD;
  m.color = rgb;
  tf::poseTFToMsg(tf::Transform::getIdentity(), m.pose);
  m.scale.x = line_width;

  if (tool_line.cols() == 0)
  {
    return;
  }

  // copying points into marker
  eigenToPointsMsgs(tool_line, m.points);
}

inline void createSphereMarker(const Eigen::Vector3d& tool_point, int id, std::string frame_id,
                               const std_msgs::ColorRGBA& rgb, double radius, std::string ns,
                               visualization_msgs::Marker& m)
{
  m.ns = ns;
  m.id = id;
  m.header.frame_id = frame_id;
  m.type = m.SPHERE;
  m.action = m.ADD;
  m.color = rgb;
  tf::poseTFToMsg(tf::Transform::getIdentity(), m.pose);
  m.scale.x = m.scale.y = m.scale.z = 2 * radius;
  tf::pointEigenToMsg(tool_point, m.pose.position);
}

namespace stomp_moveit
{
namespace noisy_filters
{
MultiTrajectoryVisualization::MultiTrajectoryVisualization()
  : name_("MultiTrajectoryVisualization"), line_width_(0.01), traj_total_(0)
{
  // TODO Auto-generated constructor stub
}

MultiTrajectoryVisualization::~MultiTrajectoryVisualization()
{
  // TODO Auto-generated destructor stub
}

bool MultiTrajectoryVisualization::initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                                              const std::string& group_name, const XmlRpc::XmlRpcValue& config)
{
  robot_model_ = robot_model_ptr;
  group_name_ = group_name;

  if (!configure(config))
  {
    return false;
  }

  // initializing publisher
  viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 1);

  return true;
}

bool MultiTrajectoryVisualization::configure(const XmlRpc::XmlRpcValue& config)
{
  auto toColorRgb = [](XmlRpc::XmlRpcValue& p) {
    std_msgs::ColorRGBA rgb;
    rgb.r = (static_cast<int>(p[0])) / 255.0;
    rgb.g = static_cast<int>(p[1]) / 255.0;
    rgb.b = static_cast<int>(p[2]) / 255.0;
    rgb.a = 1.0;
    return rgb;
  };

  // check parameter presence
  auto members = { "line_width", "marker_array_topic", "rgb", "marker_namespace" };
  for (auto& m : members)
  {
    if (!config.hasMember(m))
    {
      ROS_ERROR("%s failed to find one or more required parameters", getName().c_str());
      return false;
    }
  }

  XmlRpc::XmlRpcValue c = config;
  try
  {
    line_width_ = static_cast<double>(c["line_width"]);
    rgb_ = toColorRgb(c["rgb"]);
    marker_topic_ = static_cast<std::string>(c["marker_array_topic"]);
    marker_namespace_ = static_cast<std::string>(c["marker_namespace"]);
  }
  catch (XmlRpc::XmlRpcException& e)
  {
    ROS_ERROR("%s failed to read one or more required parameters, %s", getName().c_str(), e.getMessage().c_str());
    return false;
  }

  return true;
}

bool MultiTrajectoryVisualization::setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                        const moveit_msgs::MotionPlanRequest& req,
                                                        const stomp_core::StompConfiguration& config,
                                                        moveit_msgs::MoveItErrorCodes& error_code)
{
  using namespace moveit::core;

  error_code.val = error_code.SUCCESS;

  // initializing points array
  tool_traj_line_ = Eigen::MatrixXd::Zero(3, config.num_timesteps);
  Eigen::Vector3d tool_point;

  // initializing marker array
  traj_total_ = config.num_rollouts;
  tool_traj_markers_.markers.resize(traj_total_);
  tool_points_markers_.markers.resize(traj_total_);
  for (auto r = 0u; r < config.num_rollouts; r++)
  {
    createToolPathMarker(tool_traj_line_, r + 1, robot_model_->getRootLinkName(), rgb_, line_width_, marker_namespace_,
                         tool_traj_markers_.markers[r]);
    createSphereMarker(tool_point, r + 1, robot_model_->getRootLinkName(), rgb_, line_width_,
                       marker_namespace_ + "/goal", tool_points_markers_.markers[r]);
  }

  // updating state
  state_.reset(new RobotState(robot_model_));
  if (!robotStateMsgToRobotState(req.start_state, *state_, true))
  {
    ROS_ERROR("%s Failed to get current robot state from request", getName().c_str());
    return false;
  }

  // delete current markers
  visualization_msgs::MarkerArray m;
  viz_pub_.publish(m);

  return true;
}

bool MultiTrajectoryVisualization::filter(std::size_t start_timestep, std::size_t num_timesteps, int iteration_number,
                                          int rollout_number, Eigen::MatrixXd& parameters, bool& filtered)
{
  if (!state_)
  {
    ROS_ERROR("%s Robot State has not been updated", getName().c_str());
    return false;
  }

  if (rollout_number >= traj_total_)
  {
    // exceeded allocated rollouts
    ROS_WARN("%s rollout allocation was exceeded", getName().c_str());
    return true;
  }

  // FK on each point
  const moveit::core::JointModelGroup* joint_group = robot_model_->getJointModelGroup(group_name_);
  std::string tool_link = joint_group->getLinkModelNames().back();
  for (auto t = 0u; t < parameters.cols(); t++)
  {
    state_->setJointGroupPositions(joint_group, parameters.col(t));
    Eigen::Affine3d tool_pos = state_->getFrameTransform(tool_link);
    tool_traj_line_(0, t) = tool_pos.translation()(0);
    tool_traj_line_(1, t) = tool_pos.translation()(1);
    tool_traj_line_(2, t) = tool_pos.translation()(2);
  }

  // storing into marker
  eigenToPointsMsgs(tool_traj_line_, tool_traj_markers_.markers[rollout_number].points);
  Eigen::Vector3d goal_tool_point = tool_traj_line_.rightCols(1);
  tf::pointEigenToMsg(goal_tool_point, tool_points_markers_.markers[rollout_number].pose.position);

  // publish after collecting all rollouts
  if (rollout_number == traj_total_ - 1)
  {
    viz_pub_.publish(tool_traj_markers_);
    viz_pub_.publish(tool_points_markers_);
  }

  return true;
}

} /* namespace filters */
} /* namespace stomp_moveit */
