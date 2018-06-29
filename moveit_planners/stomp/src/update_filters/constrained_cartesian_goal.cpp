/**
 * @file constrained_cartesian_goal.cpp
 * @brief This defines a underconstrained goal update filter.
 *
 * This will force goal constraints into the task space.
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
#include <stomp_plugins/update_filters/constrained_cartesian_goal.h>
#include <ros/console.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <pluginlib/class_list_macros.h>
#include <XmlRpcException.h>

PLUGINLIB_EXPORT_CLASS(stomp_moveit::update_filters::ConstrainedCartesianGoal,stomp_moveit::update_filters::StompUpdateFilter);

static int CARTESIAN_DOF_SIZE = 6;
static int const IK_ATTEMPTS = 10;
static int const IK_TIMEOUT = 0.05;

namespace stomp_moveit
{
namespace update_filters
{

ConstrainedCartesianGoal::ConstrainedCartesianGoal():
    name_("ConstrainedCartesianGoal")
{
  // TODO Auto-generated constructor stub

}

ConstrainedCartesianGoal::~ConstrainedCartesianGoal()
{
  // TODO Auto-generated destructor stub
}

bool ConstrainedCartesianGoal::initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                        const std::string& group_name,const XmlRpc::XmlRpcValue& config)
{
  group_name_ = group_name;
  robot_model_ = robot_model_ptr;

  return configure(config);
}

bool ConstrainedCartesianGoal::configure(const XmlRpc::XmlRpcValue& config)
{
  using namespace XmlRpc;

  try
  {
    XmlRpcValue params = config;

    XmlRpcValue dof_nullity_param = params["constrained_dofs"];
    XmlRpcValue dof_thresholds_param = params["cartesian_convergence"];
    XmlRpcValue joint_updates_param = params["joint_update_rates"];
    if((dof_nullity_param.getType() != XmlRpcValue::TypeArray) ||
        dof_nullity_param.size() < CARTESIAN_DOF_SIZE ||
        dof_thresholds_param.getType() != XmlRpcValue::TypeArray ||
        dof_thresholds_param.size() < CARTESIAN_DOF_SIZE  ||
        joint_updates_param.getType() != XmlRpcValue::TypeArray ||
        joint_updates_param.size() == 0)
    {
      ROS_ERROR("UnderconstrainedGoal received invalid array parameters");
      return false;
    }

    for(auto i = 0u; i < dof_nullity_param.size(); i++)
    {
      kc_.constrained_dofs(i) = static_cast<int>(dof_nullity_param[i]);
    }

    for(auto i = 0u; i < dof_thresholds_param.size(); i++)
    {
      kc_.cartesian_convergence_thresholds(i) = static_cast<double>(dof_thresholds_param[i]);
    }

    kc_.joint_update_rates.resize(joint_updates_param.size());
    for(auto i = 0u; i < joint_updates_param.size(); i++)
    {
      kc_.joint_update_rates(i) = static_cast<double>(joint_updates_param[i]);
    }

    kc_.max_iterations = static_cast<int>(params["max_ik_iterations"]);
  }
  catch(XmlRpc::XmlRpcException& e)
  {
    ROS_ERROR("%s failed to load parameters, %s",getName().c_str(),e.getMessage().c_str());
    return false;
  }

  return true;
}

bool ConstrainedCartesianGoal::setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                 const moveit_msgs::MotionPlanRequest &req,
                 const stomp_core::StompConfiguration &config,
                 moveit_msgs::MoveItErrorCodes& error_code)
{
  using namespace Eigen;
  using namespace moveit::core;
  using namespace utils::kinematics;

  const JointModelGroup* joint_group = robot_model_->getJointModelGroup(group_name_);
  int num_joints = joint_group->getActiveJointModels().size();
  tool_link_ = joint_group->getLinkModelNames().back();
  state_.reset(new RobotState(robot_model_));
  robotStateMsgToRobotState(req.start_state,*state_);

  const std::vector<moveit_msgs::Constraints>& goals = req.goal_constraints;
  if(goals.empty())
  {
    ROS_ERROR("A goal constraint was not provided");
    error_code.val = error_code.INVALID_GOAL_CONSTRAINTS;
    return false;
  }

  // storing tool goal pose
  bool found_goal = false;
  for(const auto& g: goals)
  {

    if(!g.position_constraints.empty() &&
        !g.orientation_constraints.empty()) // check cartesian pose constraints first
    {

      // storing cartesian goal pose using ik
      const moveit_msgs::PositionConstraint& pos_constraint = g.position_constraints.front();
      const moveit_msgs::OrientationConstraint& orient_constraint = g.orientation_constraints.front();

      KinematicConfig kc;
      Eigen::VectorXd joint_pose;
      if(createKinematicConfig(joint_group,pos_constraint,orient_constraint,req.start_state,kc))
      {
        kc_.tool_goal_pose = kc.tool_goal_pose;
        if(!solveIK(state_,group_name_,kc,joint_pose))
        {
          ROS_WARN("%s failed calculating ik for cartesian goal pose in the MotionPlanRequest",getName().c_str());
        }

        found_goal = true;
        break;
      }

    }

    if(!found_goal )
    {
      // check joint constraints
      if(g.joint_constraints.empty())
      {
        ROS_WARN_STREAM("No joint values for the goal were found");
        continue;
      }

      ROS_WARN("%s a cartesian goal pose in MotionPlanRequest was not provided,calculating it from FK",getName().c_str());

      // compute FK to obtain cartesian goal pose
      const std::vector<moveit_msgs::JointConstraint>& joint_constraints = g.joint_constraints;

      // copying goal values into state
      for(auto& jc: joint_constraints)
      {
        state_->setVariablePosition(jc.joint_name,jc.position);
      }

      // storing reference goal position tool and pose
      state_->update(true);
      kc_.tool_goal_pose = state_->getGlobalLinkTransform(tool_link_);
      found_goal = true;
      break;

    }
  }

  if(!found_goal)
  {
    ROS_ERROR("%s No valid goal pose was found",getName().c_str());
    error_code.val = error_code.INVALID_GOAL_CONSTRAINTS;
    return false;
  }

  error_code.val = error_code.SUCCESS;
  return true;
}

bool ConstrainedCartesianGoal::filter(std::size_t start_timestep,
                    std::size_t num_timesteps,
                    int iteration_number,
                    const Eigen::MatrixXd& parameters,
                    Eigen::MatrixXd& updates,
                    bool& filtered)
{
  using namespace Eigen;
  using namespace moveit::core;
  using namespace stomp_moveit::utils;


  filtered = false;
  kc_.init_joint_pose = parameters.rightCols(1);
  VectorXd joint_pose;
  MatrixXd jacb_nullspace;

  // projecting update into nullspace
  if(kinematics::computeJacobianNullSpace(state_,group_name_,tool_link_,kc_.constrained_dofs,kc_.init_joint_pose,jacb_nullspace))
  {
    kc_.init_joint_pose  += jacb_nullspace*(updates.rightCols(1));
  }
  else
  {
    ROS_WARN("%s failed to project into the nullspace of the jacobian",getName().c_str());
  }

  if(kinematics::solveIK(state_,group_name_,kc_,joint_pose))
  {
    filtered = true;
    updates.rightCols(1) = joint_pose - parameters.rightCols(1);
  }
  else
  {
    ROS_DEBUG("%s failed to update under-constrained tool goal pose due to ik error",getName().c_str());
    filtered = true;
    updates.rightCols(1) = Eigen::VectorXd::Zero(updates.rows());
  }


  return true;
}

} /* namespace update_filters */
} /* namespace stomp_moveit */
