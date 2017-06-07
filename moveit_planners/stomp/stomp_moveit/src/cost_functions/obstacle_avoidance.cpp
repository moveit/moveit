/**
 * @file obstacle_avoidance.cpp
 * @brief This defines a cost function for obstacle avoidance.
 *
 * @author Jorge Nicho
 * @date March 30, 2016
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
#include <ros/console.h>
#include <pluginlib/class_list_macros.h>
#include <moveit/robot_state/conversions.h>
#include "stomp_moveit/cost_functions/obstacle_avoidance.h"

PLUGINLIB_EXPORT_CLASS(stomp_moveit::cost_functions::ObstacleAvoidance,stomp_moveit::cost_functions::StompCostFunction)

const std::string DEFAULT_COLLISION_DETECTOR = "IndustrialFCL";

namespace stomp_moveit
{
namespace cost_functions
{

ObstacleAvoidance::ObstacleAvoidance():
    name_("ObstacleAvoidancePlugin"),
    robot_state_(),
    collision_clearance_(0.0),
    collision_penalty_(0.0)
{
  // TODO Auto-generated constructor stub

}

ObstacleAvoidance::~ObstacleAvoidance()
{
  // TODO Auto-generated destructor stub
}

bool ObstacleAvoidance::initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                        const std::string& group_name,XmlRpc::XmlRpcValue& config)
{
  robot_model_ptr_ = robot_model_ptr;
  group_name_ = group_name;
  return configure(config);
}

bool ObstacleAvoidance::setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                 const moveit_msgs::MotionPlanRequest &req,
                 const stomp_core::StompConfiguration &config,
                 moveit_msgs::MoveItErrorCodes& error_code)
{
  using namespace moveit::core;

  planning_scene_ = planning_scene;
  plan_request_ = req;
  error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

  // initialize collision request
  collision_request_.group_name = group_name_;
  collision_request_.cost = false;
  collision_request_.distance = true;
  collision_request_.max_contacts = 0;
  collision_request_.max_contacts_per_pair = 0;
  collision_request_.contacts = false;
  collision_request_.verbose = false;

  //Check and make sure the correct collision detector is loaded.
  if (planning_scene->getActiveCollisionDetectorName() != DEFAULT_COLLISION_DETECTOR)
  {
    throw std::runtime_error("STOMP Moveit Interface requires the use of collision detector \"" + DEFAULT_COLLISION_DETECTOR + "\"\n"
                             "To resolve the issue add the ros parameter collision_detector = " + DEFAULT_COLLISION_DETECTOR +
                             ".\nIt is recommend to added it where the move_group node is launched, usually in the in the "
                             "(robot_name)_moveit_config/launch/move_group.launch");
  }
  collision_robot_ = boost::dynamic_pointer_cast<const collision_detection::CollisionRobotIndustrial>(planning_scene->getCollisionRobot());
  collision_world_ = boost::dynamic_pointer_cast<const collision_detection::CollisionWorldIndustrial>(planning_scene->getCollisionWorld());

  // storing robot state
  robot_state_.reset(new RobotState(robot_model_ptr_));
  if(!robotStateMsgToRobotState(req.start_state,*robot_state_,true))
  {
    ROS_ERROR("%s Failed to get current robot state from request",getName().c_str());
    return false;
  }

  return true;
}

bool ObstacleAvoidance::computeCosts(const Eigen::MatrixXd& parameters,
                          std::size_t start_timestep,
                          std::size_t num_timesteps,
                          int iteration_number,
                          int rollout_number,
                          Eigen::VectorXd& costs,
                          bool& validity)
{

  using namespace moveit::core;

  if(!robot_state_)
  {
    ROS_ERROR("%s Robot State has not been updated",getName().c_str());
    return false;
  }

  typedef collision_detection::CollisionResult::ContactMap ContactMap;
  typedef ContactMap::iterator ContactMapIterator;
  typedef std::vector<collision_detection::Contact> ContactArray;

  // initializing result array
  costs = Eigen::VectorXd::Zero(num_timesteps);

  // collision
  collision_detection::CollisionRequest request = collision_request_;
  collision_detection::CollisionResult result_world_collision, result_robot_collision;
  std::vector<collision_detection::CollisionResult> results(2);
  validity = true;

  // robot state
  const JointModelGroup* joint_group = robot_model_ptr_->getJointModelGroup(group_name_);
  std::vector<double> joint_values(parameters.rows(),0);

  if(parameters.cols()<start_timestep + num_timesteps)
  {
    ROS_ERROR_STREAM("Size in the 'parameters' matrix is less than required");
    return false;
  }

  // iterating through collisions
  for (auto t=start_timestep; t<start_timestep + num_timesteps; ++t)
  {
    robot_state_->setJointGroupPositions(joint_group,parameters.col(t));
    robot_state_->update();

    // checking robot vs world (attached objects, octomap, not in urdf) collisions
    result_world_collision.distance = std::numeric_limits<double>::max();

    collision_world_->checkRobotCollision(request,
                                          result_world_collision,
                                          *collision_robot_,
                                          *robot_state_,
                                          planning_scene_->getAllowedCollisionMatrix());

    collision_robot_->checkSelfCollision(request,
                                         result_robot_collision,
                                         *robot_state_,
                                         planning_scene_->getAllowedCollisionMatrix());

    results[0]= result_world_collision;
    results[1] = result_robot_collision;

    double penalty = 0;
    double distance = collision_clearance_;
    bool collision = false;
    for(std::vector<collision_detection::CollisionResult>::iterator i = results.begin(); i != results.end(); i++)
    {
      collision_detection::CollisionResult& result = *i;

      // get shortest distance
      distance = distance > result.distance ? result.distance : distance;
      collision |= result.collision;
    }

    if(collision)
    {
      costs(t) = collision_penalty_;
      validity = false;
    }
    else
    {
      costs(t) = distance > collision_clearance_  ? 0 : (collision_clearance_ - distance);
    }
  }

  // scaling cost
  double max = costs.maxCoeff();
  costs /= (max > 1e-8) ? max : 1;

  return true;
}

bool ObstacleAvoidance::configure(const XmlRpc::XmlRpcValue& config)
{
  // check parameter presence
  auto members = {"collision_clearance","collision_penalty","cost_weight"};
  for(auto& m : members)
  {
    if(!config.hasMember(m))
    {
      ROS_ERROR("%s failed to find one or more required parameters",getName().c_str());
      return false;
    }
  }

  try
  {
    XmlRpc::XmlRpcValue c = config;
    collision_clearance_ = static_cast<double>(c["collision_clearance"]);
    collision_penalty_ = static_cast<double>(c["collision_penalty"]);
    cost_weight_ = static_cast<double>(c["cost_weight"]);
  }
  catch(XmlRpc::XmlRpcException& e)
  {
    ROS_ERROR("%s failed to parse configuration parameters",name_.c_str());
    return false;
  }

  return true;
}

void ObstacleAvoidance::done(bool success,int total_iterations,double final_cost)
{
  robot_state_.reset();
}

} /* namespace cost_functions */
} /* namespace stomp_moveit */
