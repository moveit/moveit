/**
 * @file goal_guided_multivariate_gaussian.cpp
 * @brief This class generates noisy trajectories to an under-constrained cartesian goal pose
 *
 * @author Jorge Nicho
 * @date Jun 14, 2016
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

#include "stomp_plugins/noise_generators/goal_guided_multivariate_gaussian.h"
#include <stomp_moveit/utils/multivariate_gaussian.h>
#include <XmlRpcException.h>
#include <pluginlib/class_list_macros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <boost/filesystem.hpp>
#include <fstream>

PLUGINLIB_EXPORT_CLASS(stomp_moveit::noise_generators::GoalGuidedMultivariateGaussian,
                       stomp_moveit::noise_generators::StompNoiseGenerator);

static int const IK_ITERATIONS = 40;
static double const JOINT_UPDATE_RATE = 0.5f;
static double const CARTESIAN_POS_CONVERGENCE = 0.01;
static double const CARTESIAN_ROT_CONVERGENCE = 0.01;
static const std::vector<double> ACC_MATRIX_DIAGONAL_VALUES = { -1.0 / 12.0, 16.0 / 12.0, -30.0 / 12.0, 16.0 / 12.0,
                                                                -1.0 / 12.0 };
static const std::vector<int> ACC_MATRIX_DIAGONAL_INDICES = { -2, -1, 0, 1, 2 };
static const int CARTESIAN_DOF_SIZE = 6;

namespace stomp_moveit
{
namespace noise_generators
{
GoalGuidedMultivariateGaussian::GoalGuidedMultivariateGaussian()
  : name_("GoalGuidedMultivariateGaussian")
  , goal_rand_generator_(new RandomGenerator(RGNType(), boost::uniform_real<>(-1, 1)))
{
}

GoalGuidedMultivariateGaussian::~GoalGuidedMultivariateGaussian()
{
}

bool GoalGuidedMultivariateGaussian::initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                                                const std::string& group_name, const XmlRpc::XmlRpcValue& config)
{
  using namespace moveit::core;

  group_ = group_name;
  robot_model_ = robot_model_ptr;
  const JointModelGroup* joint_group = robot_model_ptr->getJointModelGroup(group_name);
  if (!joint_group)
  {
    ROS_ERROR("Invalid joint group %s", group_name.c_str());
    return false;
  }

  stddev_.resize(joint_group->getActiveJointModelNames().size());
  goal_stddev_.resize(CARTESIAN_DOF_SIZE);

  // goal kinematics parameters
  kc_.joint_update_rates = JOINT_UPDATE_RATE * Eigen::VectorXd::Ones(stddev_.size());
  kc_.cartesian_convergence_thresholds << CARTESIAN_POS_CONVERGENCE, CARTESIAN_POS_CONVERGENCE,
      CARTESIAN_POS_CONVERGENCE, CARTESIAN_ROT_CONVERGENCE, CARTESIAN_ROT_CONVERGENCE, CARTESIAN_ROT_CONVERGENCE;
  kc_.constrained_dofs << 1, 1, 1, 1, 1, 1;
  kc_.max_iterations = IK_ITERATIONS;

  return configure(config);
}

bool GoalGuidedMultivariateGaussian::configure(const XmlRpc::XmlRpcValue& config)
{
  using namespace XmlRpc;

  try
  {
    XmlRpcValue params = config;

    // noise generation parameters
    XmlRpcValue stddev_param = params["stddev"];
    XmlRpcValue goal_stddev_param = params["goal_stddev"];

    // check  stddev
    if (stddev_param.size() < stddev_.size())
    {
      ROS_ERROR("%s the 'stddev' parameter has fewer elements than the number of joints", getName().c_str());
      return false;
    }

    // check goal stddev
    if (goal_stddev_param.size() != CARTESIAN_DOF_SIZE)
    {
      ROS_ERROR("%s the 'goal_stddev' parameter must have 6 entries [x y z rx ry rz] for the tool cartesian goal",
                getName().c_str());
      return false;
    }

    // parsing parameters
    for (auto i = 0u; i < stddev_param.size(); i++)
    {
      stddev_[i] = static_cast<double>(stddev_param[i]);
    }

    // goal stddev
    for (auto i = 0u; i < goal_stddev_param.size(); i++)
    {
      goal_stddev_[i] = static_cast<double>(goal_stddev_param[i]);
    }

    // goal constraint parameters
    XmlRpcValue dof_nullity_param = params["constrained_dofs"];
    if ((dof_nullity_param.getType() != XmlRpcValue::TypeArray) || dof_nullity_param.size() < CARTESIAN_DOF_SIZE)
    {
      ROS_ERROR("UnderconstrainedGoal received invalid array parameters");
      return false;
    }

    for (auto i = 0u; i < dof_nullity_param.size(); i++)
    {
      kc_.constrained_dofs(i) = static_cast<int>(dof_nullity_param[i]);
    }
  }
  catch (XmlRpc::XmlRpcException& e)
  {
    ROS_ERROR("%s failed to load parameters", getName().c_str());
    return false;
  }

  return true;
}

bool GoalGuidedMultivariateGaussian::setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                          const moveit_msgs::MotionPlanRequest& req,
                                                          const stomp_core::StompConfiguration& config,
                                                          moveit_msgs::MoveItErrorCodes& error_code)
{
  bool succeed = setNoiseGeneration(planning_scene, req, config, error_code) &&
                 setGoalConstraints(planning_scene, req, config, error_code);

  return succeed;
}

bool GoalGuidedMultivariateGaussian::setNoiseGeneration(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                        const moveit_msgs::MotionPlanRequest& req,
                                                        const stomp_core::StompConfiguration& config,
                                                        moveit_msgs::MoveItErrorCodes& error_code)
{
  using namespace Eigen;

  // convenience lambda function to fill matrix
  auto fill_diagonal = [](Eigen::MatrixXd& m, double coeff, int diag_index) {
    std::size_t size = m.rows() - std::abs(diag_index);
    m.diagonal(diag_index) = VectorXd::Constant(size, coeff);
  };

  // creating finite difference acceleration matrix
  std::size_t num_timesteps = config.num_timesteps;
  Eigen::MatrixXd A = MatrixXd::Zero(num_timesteps, num_timesteps);
  int num_elements = (int((ACC_MATRIX_DIAGONAL_INDICES.size() - 1) / 2.0) + 1) * num_timesteps;
  for (auto i = 0u; i < ACC_MATRIX_DIAGONAL_INDICES.size(); i++)
  {
    fill_diagonal(A, ACC_MATRIX_DIAGONAL_VALUES[i], ACC_MATRIX_DIAGONAL_INDICES[i]);
  }

  // create and scale covariance matrix
  Eigen::MatrixXd covariance = MatrixXd::Identity(num_timesteps, num_timesteps);
  covariance = A.transpose() * A;
  covariance = covariance.fullPivLu().inverse();
  double max_val = covariance.array().abs().matrix().maxCoeff();
  covariance /= max_val;

  // create random generators
  traj_noise_generators_.resize(stddev_.size());
  for (auto& r : traj_noise_generators_)
  {
    r.reset(new utils::MultivariateGaussian(VectorXd::Zero(num_timesteps), covariance));
  }

  // preallocating noise data
  raw_noise_.resize(config.num_timesteps);
  raw_noise_.setZero();

  error_code.val = error_code.SUCCESS;

  return true;
}

bool GoalGuidedMultivariateGaussian::setGoalConstraints(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                        const moveit_msgs::MotionPlanRequest& req,
                                                        const stomp_core::StompConfiguration& config,
                                                        moveit_msgs::MoveItErrorCodes& error_code)
{
  using namespace Eigen;
  using namespace moveit::core;
  using namespace utils::kinematics;

  const JointModelGroup* joint_group = robot_model_->getJointModelGroup(group_);
  int num_joints = joint_group->getActiveJointModels().size();
  tool_link_ = joint_group->getLinkModelNames().back();
  state_.reset(new RobotState(robot_model_));
  robotStateMsgToRobotState(req.start_state, *state_);

  ROS_DEBUG("%s using '%s' tool link", getName().c_str(), tool_link_.c_str());
  error_code.val = error_code.SUCCESS;

  return true;
}

bool GoalGuidedMultivariateGaussian::generateNoise(const Eigen::MatrixXd& parameters, std::size_t start_timestep,
                                                   std::size_t num_timesteps, int iteration_number, int rollout_number,
                                                   Eigen::MatrixXd& parameters_noise, Eigen::MatrixXd& noise)
{
  using namespace Eigen;
  using namespace stomp_moveit::utils;

  VectorXd goal_joint_pose, goal_joint_noise;
  if (parameters.rows() != stddev_.size())
  {
    ROS_ERROR("Number of rows in parameters %i differs from expected number of joints", int(parameters.rows()));
    return false;
  }

  if (generateRandomGoal(parameters.rightCols(1), goal_joint_pose))
  {
    goal_joint_noise = goal_joint_pose - parameters.rightCols(1);
  }
  else
  {
    ROS_WARN("%s failed to generate random goal pose in the task space, not applying noise at goal", getName().c_str());
    goal_joint_noise = VectorXd::Zero(parameters.rows());
  }

  // generating noise
  int sign;
  for (auto d = 0u; d < parameters.rows(); d++)
  {
    traj_noise_generators_[d]->sample(raw_noise_, true);

    // shifting data towards goal
    sign = goal_joint_noise(d) > 0 ? 1 : -1;
    noise.row(d).transpose() = stddev_[d] * raw_noise_ +
                               sign * Eigen::VectorXd::LinSpaced(raw_noise_.size(), 0, std::abs(goal_joint_noise(d)));
  }

  parameters_noise = parameters + noise;

  return true;
}

bool GoalGuidedMultivariateGaussian::generateRandomGoal(const Eigen::VectorXd& seed_joint_pose,
                                                        Eigen::VectorXd& goal_joint_pose)
{
  using namespace Eigen;
  using namespace moveit::core;
  using namespace stomp_moveit::utils;

  // generate noise
  Eigen::VectorXd noise = Eigen::VectorXd::Zero(CARTESIAN_DOF_SIZE);
  for (auto d = 0u; d < noise.size(); d++)
  {
    noise(d) = goal_stddev_[d] * (*goal_rand_generator_)();
  }

  // applying noise onto tool pose
  state_->setJointGroupPositions(group_, seed_joint_pose);
  state_->updateLinkTransforms();
  Affine3d tool_pose = state_->getGlobalLinkTransform(tool_link_);
  auto& n = noise;
  kc_.tool_goal_pose = tool_pose * Translation3d(Vector3d(n(0), n(1), n(2))) * AngleAxisd(n(3), Vector3d::UnitX()) *
                       AngleAxisd(n(4), Vector3d::UnitY()) * AngleAxisd(n(5), Vector3d::UnitZ());
  kc_.init_joint_pose = seed_joint_pose;

  if (!kinematics::solveIK(state_, group_, kc_, goal_joint_pose))
  {
    ROS_DEBUG("%s 'solveIK(...)' failed, returning noiseless goal pose", getName().c_str());
    goal_joint_pose = seed_joint_pose;
    return false;
  }

  return true;
}

} /* namespace noise_generators */
} /* namespace stomp_moveit */
