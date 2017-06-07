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
#include <stomp_moveit/update_filters/constrained_cartesian_goal.h>
#include <ros/console.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <pluginlib/class_list_macros.h>
#include <stomp_moveit/update_filters/polynomial_smoother.h>
#include <XmlRpcException.h>

PLUGINLIB_EXPORT_CLASS(stomp_moveit::update_filters::PolynomialSmoother,stomp_moveit::update_filters::StompUpdateFilter);

namespace stomp_moveit
{
namespace update_filters
{

PolynomialSmoother::PolynomialSmoother():
    name_("ExponentialSmoother")
{
  // TODO Auto-generated constructor stub

}

PolynomialSmoother::~PolynomialSmoother()
{
  // TODO Auto-generated destructor stub
}

bool PolynomialSmoother::initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                        const std::string& group_name,const XmlRpc::XmlRpcValue& config)
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
  catch(XmlRpc::XmlRpcException& e)
  {
    ROS_ERROR("%s failed to load parameters, %s",getName().c_str(),e.getMessage().c_str());
    return false;
  }

  return true;
}

bool PolynomialSmoother::setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                 const moveit_msgs::MotionPlanRequest &req,
                 const stomp_core::StompConfiguration &config,
                 moveit_msgs::MoveItErrorCodes& error_code)
{
  using namespace Eigen;
  using namespace moveit::core;

  const JointModelGroup* joint_group = robot_model_->getJointModelGroup(group_name_);
  int num_joints = joint_group->getActiveJointModels().size();
  smoothed_parameters_.resize(config.num_timesteps);
  domain_vals_ = ArrayXd::LinSpaced(config.num_timesteps,0,config.num_timesteps-1);
  fillVandermondeMatrix(poly_order_,domain_vals_,X_matrix_);
  X_pseudo_inv_ = ((X_matrix_.transpose() * X_matrix_).inverse()) * X_matrix_.transpose();


  error_code.val = error_code.SUCCESS;
  return true;
}

bool PolynomialSmoother::filter(std::size_t start_timestep,
                    std::size_t num_timesteps,
                    int iteration_number,
                    const Eigen::MatrixXd& parameters,
                    Eigen::MatrixXd& updates,
                    bool& filtered)
{
  using namespace Eigen;
  using namespace moveit::core;

  // updating local parameters
  double start_offset;
  double scale_factor;
  double x_range, smoothed_range, update_range;
  double angle, angle_smoothed, angle_update;
  x_range = num_timesteps - 1;

  for(auto r = 0; r < parameters.rows(); r++)
  {
    smoothed_parameters_ = X_matrix_ * (X_pseudo_inv_ * updates.row(r).transpose());

    // shifting parameters
    start_offset = updates.row(r)(0)  -   smoothed_parameters_(0);
    smoothed_parameters_ = smoothed_parameters_ + start_offset * VectorXd::Ones(smoothed_parameters_.size());

    // rotating and scaling
    update_range = updates.row(r)(updates.cols()-1) - updates.row(r)(0);
    smoothed_range = smoothed_parameters_(smoothed_parameters_.size()-1) - smoothed_parameters_(0);

    angle_update = std::atan2(update_range,x_range);
    angle_smoothed = std::atan2(smoothed_range,x_range);
    angle = angle_update - angle_smoothed;

    // scale factor using hypotenuse ratio
    scale_factor = std::sqrt(std::pow(x_range,2) + std::pow(update_range,2))/std::sqrt(std::pow(x_range,2) + std::pow(smoothed_range,2));

    // applying rotation and scale
    smoothed_parameters_ = scale_factor*(std::cos(angle)*smoothed_parameters_ + std::sin(angle)*domain_vals_.matrix());

    updates.row(r) = smoothed_parameters_.transpose();

  }

  filtered = true;

  return true;
}

void PolynomialSmoother::fillVandermondeMatrix(double poly_order,
                                                const Eigen::ArrayXd& domain_vals,
                                                Eigen::MatrixXd& X)
{

  Eigen::ArrayXd scaled_domain_vals = domain_vals/(domain_vals.maxCoeff());
  X = Eigen::MatrixXd::Ones(domain_vals.size(),poly_order+1);
  for(auto p = 1u; p <=  poly_order; p++)
  {
    X.col(p) = scaled_domain_vals * X.col(p-1).array();
  }
}

} /* namespace update_filters */
} /* namespace stomp_moveit */
