/**
 * @file tool_goal_pose.cpp
 * @brief This defines a cost function for tool goal pose.
 *
 * @author Jorge Nicho
 * @date June 2, 2016
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
#include <stomp_plugins/cost_functions/tool_goal_pose.h>
#include <XmlRpcException.h>
#include <pluginlib/class_list_macros.h>
#include <stomp_moveit/utils/kinematics.h>
#include <ros/console.h>

PLUGINLIB_EXPORT_CLASS(stomp_moveit::cost_functions::ToolGoalPose,stomp_moveit::cost_functions::StompCostFunction);

static const int CARTESIAN_DOF_SIZE = 6;

namespace stomp_moveit
{
namespace cost_functions
{

ToolGoalPose::ToolGoalPose():
    name_("ToolGoalPose")
{
  // TODO Auto-generated constructor stub

}

ToolGoalPose::~ToolGoalPose()
{
  // TODO Auto-generated destructor stub
}

bool ToolGoalPose::initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                        const std::string& group_name,XmlRpc::XmlRpcValue& config)
{
  group_name_ = group_name;
  robot_model_ = robot_model_ptr;

  return configure(config);
}

bool ToolGoalPose::configure(const XmlRpc::XmlRpcValue& config)
{
  using namespace XmlRpc;

  try
  {
    XmlRpcValue params = config;

    XmlRpcValue dof_nullity_param = params["constrained_dofs"];
    XmlRpcValue pos_error_range_param = params["position_error_range"];
    XmlRpcValue orient_error_range_param = params["orientation_error_range"];

    if((dof_nullity_param.getType() != XmlRpcValue::TypeArray) || dof_nullity_param.size() < CARTESIAN_DOF_SIZE ||
        (pos_error_range_param.getType() != XmlRpcValue::TypeArray) || pos_error_range_param.size() != 2 ||
        (orient_error_range_param.getType() != XmlRpcValue::TypeArray) || orient_error_range_param.size() != 2 )
    {
      ROS_ERROR("%s received invalid array parameters",getName().c_str());
      return false;
    }

    dof_nullity_.resize(CARTESIAN_DOF_SIZE);
    for(auto i = 0u; i < dof_nullity_param.size(); i++)
    {
      dof_nullity_(i) = static_cast<int>(dof_nullity_param[i]);
    }

    position_error_range_.first = static_cast<double>(pos_error_range_param[0]);
    position_error_range_.second = static_cast<double>(pos_error_range_param[1]);

    orientation_error_range_.first = static_cast<double>(orient_error_range_param[0]);
    orientation_error_range_.second = static_cast<double>(orient_error_range_param[1]);

    position_cost_weight_ = static_cast<double>(params["position_cost_weight"]);

    orientation_cost_weight_ = static_cast<double>(params["orientation_cost_weight"]);

    // total weight
    cost_weight_ = position_cost_weight_ + orientation_cost_weight_;

  }
  catch(XmlRpc::XmlRpcException& e)
  {
    ROS_ERROR("%s failed to load parameters, %s",getName().c_str(),e.getMessage().c_str());
    return false;
  }

  return true;
}

bool ToolGoalPose::setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                 const moveit_msgs::MotionPlanRequest &req,
                 const stomp_core::StompConfiguration &config,
                 moveit_msgs::MoveItErrorCodes& error_code)
{
  using namespace Eigen;
  using namespace moveit::core;

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
        !g.orientation_constraints.empty())
    {
      // tool cartesian goal
      const moveit_msgs::PositionConstraint& pos_constraint = g.position_constraints.front();
      const moveit_msgs::OrientationConstraint& orient_constraint = g.orientation_constraints.front();

      geometry_msgs::Pose pose;
      pose.position = pos_constraint.constraint_region.primitive_poses[0].position;
      pose.orientation = orient_constraint.orientation;
      tf::poseMsgToEigen(pose,tool_goal_pose_);
      found_goal = true;
      break;

    }


    if(!found_goal)
    {
      ROS_WARN("%s a cartesian goal pose in MotionPlanRequest was not provided,calculating it from FK",getName().c_str());

      // check joint constraints
      if(g.joint_constraints.empty())
      {
        ROS_ERROR_STREAM("No joint values for the goal were found");
        error_code.val = error_code.INVALID_GOAL_CONSTRAINTS;
        return false;
      }

      // compute FK to obtain tool pose
      const std::vector<moveit_msgs::JointConstraint>& joint_constraints = g.joint_constraints;

      // copying goal values into state
      for(auto& jc: joint_constraints)
      {
        state_->setVariablePosition(jc.joint_name,jc.position);
      }

      // storing reference goal position tool and pose
      state_->update(true);
      tool_goal_pose_ = state_->getGlobalLinkTransform(tool_link_);
      found_goal = true;
      break;
    }
  }



  return true;
}

bool ToolGoalPose::computeCosts(const Eigen::MatrixXd& parameters,
                          std::size_t start_timestep,
                          std::size_t num_timesteps,
                          int iteration_number,
                          int rollout_number,
                          Eigen::VectorXd& costs,
                          bool& validity)
{

  using namespace Eigen;
  using namespace utils::kinematics;
  validity = true;

  auto compute_scaled_error = [](const double& raw_cost,const std::pair<double,double>& range,bool& below_min)
  {
    below_min = false;

    // error above range
    if(raw_cost > range.second)
    {
      return 1.0;
    }

    // error in range
    if(raw_cost >= range.first)
    {
      return raw_cost/(range.second - range.first);
    }

    // error below range
    below_min = true;
    return 0.0;
  };

  costs.resize(parameters.cols());
  costs.setConstant(0.0);

  last_joint_pose_ = parameters.rightCols(1);
  state_->setJointGroupPositions(group_name_,last_joint_pose_);
  last_tool_pose_ = state_->getGlobalLinkTransform(tool_link_);

  computeTwist(last_tool_pose_,tool_goal_pose_,dof_nullity_,tool_twist_error_);

  double pos_error = tool_twist_error_.segment(0,3).norm();
  double orientation_error = tool_twist_error_.segment(3,3).norm();


  // scaling errors so that max total error  = pos_weight + orient_weight
  bool valid;
  pos_error = compute_scaled_error(pos_error,position_error_range_,valid);
  validity &= valid;

  orientation_error = compute_scaled_error(orientation_error,orientation_error_range_,valid);
  validity &= valid;

  costs(costs.size()-1) = pos_error*position_cost_weight_ + orientation_error * orientation_cost_weight_;

  return true;
}

} /* namespace cost_functions */
} /* namespace stomp_moveit */
