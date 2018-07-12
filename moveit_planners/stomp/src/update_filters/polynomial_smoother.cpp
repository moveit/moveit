/**
 * @file polynomial_smoother.cpp
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
#include <ros/console.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <pluginlib/class_list_macros.h>
#include <stomp_moveit/update_filters/polynomial_smoother.h>
#include <stomp_moveit/utils/polynomial.h>
#include <XmlRpcException.h>

PLUGINLIB_EXPORT_CLASS(stomp_moveit::update_filters::PolynomialSmoother,
                       stomp_moveit::update_filters::StompUpdateFilter)

namespace stomp_moveit
{
namespace update_filters
{
const double JOINT_LIMIT_MARGIN = 0.00001;

PolynomialSmoother::PolynomialSmoother() : name_("ExponentialSmoother")
{
  // TODO Auto-generated constructor stub
}

PolynomialSmoother::~PolynomialSmoother()
{
  // TODO Auto-generated destructor stub
}

bool PolynomialSmoother::initialize(moveit::core::RobotModelConstPtr robot_model_ptr, const std::string& group_name,
                                    const XmlRpc::XmlRpcValue& config)
{
  group_name_ = group_name;
  robot_model_ = robot_model_ptr;

  return configure(config);
}

bool PolynomialSmoother::configure(const XmlRpc::XmlRpcValue& config)
{
  using namespace XmlRpc;

  try
  {
    XmlRpcValue params = config;
    poly_order_ = static_cast<int>(params["poly_order"]);
  }
  catch (XmlRpc::XmlRpcException& e)
  {
    ROS_ERROR("%s failed to load parameters, %s", getName().c_str(), e.getMessage().c_str());
    return false;
  }

  return true;
}

bool PolynomialSmoother::setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                              const moveit_msgs::MotionPlanRequest& req,
                                              const stomp_core::StompConfiguration& config,
                                              moveit_msgs::MoveItErrorCodes& error_code)
{
  error_code.val = error_code.SUCCESS;
  return true;
}

bool PolynomialSmoother::filter(std::size_t start_timestep, std::size_t num_timesteps, int iteration_number,
                                const Eigen::MatrixXd& parameters, Eigen::MatrixXd& updates, bool& filtered)
{
  using namespace Eigen;
  using namespace moveit::core;
  using namespace utils::polynomial;

  filtered = false;
  Eigen::MatrixXd parameters_updates = parameters + updates;
  if (applyPolynomialSmoothing(robot_model_, group_name_, parameters_updates, poly_order_, JOINT_LIMIT_MARGIN))
  {
    updates = parameters_updates - parameters;
    filtered = true;
  }
  else
  {
    ROS_ERROR("Unable to polynomial smooth trajectory!");
    return false;
  }

  return true;
}

} /* namespace update_filters */
} /* namespace stomp_moveit */
