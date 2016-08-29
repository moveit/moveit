/**
 * @file trajectory_visualization.cpp
 * @brief This defines a trajectory visualizer for publishing the smooth trajectory.
 *
 * @author Jorge Nicho
 * @date April 14, 2016
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
#include <moveit/robot_state/conversions.h>
#include <tf/transform_datatypes.h>
#include <pluginlib/class_list_macros.h>
#include <stomp_moveit/update_filters/trajectory_visualization.h>

PLUGINLIB_EXPORT_CLASS(stomp_moveit::update_filters::TrajectoryVisualization,stomp_moveit::update_filters::StompUpdateFilter);


typedef std::vector<geometry_msgs::Point> ToolLine;
using namespace moveit::core;

static const int MARKER_ID = 1;


inline void eigenToPointsMsgs(const Eigen::MatrixXd& in,std::vector<geometry_msgs::Point>& out)
{
  // resizing
  if(out.size()!= in.cols())
  {
    out.resize(in.cols());
  }

  // copying points
  for(auto t = 0u; t < in.cols(); t++)
  {
    out[t].x = in(0,t);
    out[t].y = in(1,t);
    out[t].z = in(2,t);
  }
}

inline void createToolPathMarker(const Eigen::MatrixXd& tool_line, int id, std::string frame_id,
                          const std_msgs::ColorRGBA& rgb,double line_width,
                          std::string ns,visualization_msgs::Marker& m)
{
  m.ns = ns;
  m.id = id;
  m.header.frame_id = frame_id;
  m.type = m.LINE_STRIP;
  m.action = m.ADD;
  m.color = rgb;
  tf::poseTFToMsg(tf::Transform::getIdentity(),m.pose);
  m.scale.x = line_width;

  if(tool_line.cols() == 0)
  {
    return;
  }

  // copying points into marker
  eigenToPointsMsgs(tool_line,m.points);
}

namespace stomp_moveit
{
namespace update_filters
{

TrajectoryVisualization::TrajectoryVisualization():
    name_("TrajectoryVisualization"),
    nh_("~"),
    line_width_(0.0),
    publish_intermediate_(false)
{
  // TODO Auto-generated constructor stub

}

TrajectoryVisualization::~TrajectoryVisualization()
{
  // TODO Auto-generated destructor stub
}

bool TrajectoryVisualization::initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                        const std::string& group_name,const XmlRpc::XmlRpcValue& config)
{
  robot_model_ = robot_model_ptr;
  group_name_ = group_name;

  if(!configure(config))
  {
    return false;
  }

  // initializing publisher
  viz_pub_ = nh_.advertise<visualization_msgs::Marker>(marker_topic_,1);

  return true;
}

bool TrajectoryVisualization::configure(const XmlRpc::XmlRpcValue& config)
{

  auto toColorRgb = [](XmlRpc::XmlRpcValue& p)
  {
    std_msgs::ColorRGBA rgb;
    rgb.r = (static_cast<int>(p[0]))/255.0;
    rgb.g = static_cast<int>(p[1])/255.0;
    rgb.b = static_cast<int>(p[2])/255.0;
    rgb.a = 1.0;
    return rgb;
  };

  // check parameter presence
  auto members = {"line_width","rgb","error_rgb","publish_intermediate","marker_topic","marker_namespace"};
  for(auto& m : members)
  {
    if(!config.hasMember(m))
    {
      ROS_ERROR("%s failed to find one or more required parameters",getName().c_str());
      return false;
    }
  }

  XmlRpc::XmlRpcValue c = config;
  try
  {
    line_width_ = static_cast<double>(c["line_width"]);
    rgb_ = toColorRgb(c["rgb"]);
    error_rgb_ = toColorRgb(c["error_rgb"]);
    publish_intermediate_ = static_cast<bool>(c["publish_intermediate"]);
    marker_topic_ = static_cast<std::string>(c["marker_topic"]);
    marker_namespace_ = static_cast<std::string>(c["marker_namespace"]);
  }
  catch(XmlRpc::XmlRpcException& e)
  {
    ROS_ERROR("%s failed to load required parameters, %s",getName().c_str(),e.getMessage().c_str());
    return false;
  }

  return true;
}

bool TrajectoryVisualization::setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                 const moveit_msgs::MotionPlanRequest &req,
                 const stomp_core::StompConfiguration &config,
                 moveit_msgs::MoveItErrorCodes& error_code)
{
  using namespace moveit::core;

  error_code.val = error_code.SUCCESS;


  // initializing points array
  tool_traj_line_ = Eigen::MatrixXd::Zero(3,config.num_timesteps);

  // initializing marker
  createToolPathMarker(tool_traj_line_,
                       MARKER_ID,robot_model_->getRootLinkName(),
                       rgb_,line_width_,
                       marker_namespace_,tool_traj_marker_);

  // updating state
  state_.reset(new RobotState(robot_model_));
  if(!robotStateMsgToRobotState(req.start_state,*state_,true))
  {
    ROS_ERROR("%s Failed to get current robot state from request",getName().c_str());
    return false;
  }

  //delete current marker
  visualization_msgs::Marker m;
  createToolPathMarker(Eigen::MatrixXd(),MARKER_ID,robot_model_->getRootLinkName(),rgb_,line_width_,marker_namespace_,m);
  m.action = m.DELETE;
  viz_pub_.publish(m);

  return true;
}

bool TrajectoryVisualization::filter(std::size_t start_timestep,
                                     std::size_t num_timesteps,
                                     int iteration_number,
                                     const Eigen::MatrixXd& parameters,
                                     Eigen::MatrixXd& updates,
                                     bool& filtered)
{

  if(!state_)
  {
    ROS_ERROR("%s Robot State has not been updated",getName().c_str());
    return false;
  }

  // FK on each point
  const moveit::core::JointModelGroup* joint_group = robot_model_->getJointModelGroup(group_name_);
  std::string tool_link = joint_group->getLinkModelNames().back();
  Eigen::MatrixXd updated_parameters = parameters + updates;
  for(auto t = 0u; t < updated_parameters.cols();t++)
  {
    state_->setJointGroupPositions(joint_group,updated_parameters.col(t));
    Eigen::Affine3d tool_pos = state_->getFrameTransform(tool_link);
    tool_traj_line_(0,t) = tool_pos.translation()(0);
    tool_traj_line_(1,t) = tool_pos.translation()(1);
    tool_traj_line_(2,t) = tool_pos.translation()(2);
  }

  if(publish_intermediate_)
  {
    eigenToPointsMsgs(tool_traj_line_,tool_traj_marker_.points);
    viz_pub_.publish(tool_traj_marker_);
  }

  return true;
}


void TrajectoryVisualization::done(bool success,int total_iterations,double final_cost)
{
  eigenToPointsMsgs(tool_traj_line_,tool_traj_marker_.points);

  if(!success)
  {
    tool_traj_marker_.color = error_rgb_;
  }

  viz_pub_.publish(tool_traj_marker_);
}

} /* namespace updated_filters */
} /* namespace stomp_moveit */
