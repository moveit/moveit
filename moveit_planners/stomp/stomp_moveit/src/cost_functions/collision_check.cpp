/**
 * @file collision_check.cpp
 * @brief This defines a cost function for collision checking.
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
#include "stomp_moveit/cost_functions/collision_check.h"

PLUGINLIB_EXPORT_CLASS(stomp_moveit::cost_functions::CollisionCheck,stomp_moveit::cost_functions::StompCostFunction)

static const std::string DEFAULT_COLLISION_DETECTOR = "IndustrialFCL";
static const int MIN_KERNEL_WINDOW_SIZE = 3;


static void applyKernelSmoothing(std::size_t window_size, const Eigen::VectorXd& data, Eigen::VectorXd& smoothed)
{
  using namespace Eigen;

  // allocation
  window_size = 2*(window_size/2) + 1;// forcing it into an odd number
  smoothed.setZero(data.size());
  VectorXd x_nneighbors = VectorXd::Zero(window_size);
  VectorXd y_nneighbors = VectorXd::Zero(window_size);
  VectorXd kernel_weights = VectorXd::Zero(window_size);


  // indexing
  int index_left, index_right;
  std::size_t window_index_middle = window_size/2;

  // kernel function
  //Epanechnikov(x,neighbors,lambda)
  auto epanechnikov_function = [](double x_m,const VectorXd& neighbors,double lambda, VectorXd& weights )
  {

    double t = 0;
    for(int i = 0; i < neighbors.size(); i++)
    {
      t = std::abs(x_m - neighbors(i))/lambda;
      if(t < 1)
      {
        weights(i) = 0.75f*(1 - std::pow(t,2));
      }
      else
      {
        weights(i) = 0;
      }
    }

  };

  for(int i = 0; i < data.size(); i++)
  {
    // middle term
    x_nneighbors(window_index_middle) = i;
    y_nneighbors(window_index_middle) = data(i);

    // grabbing neighbors
    for(int j = 1; j <= window_size/2; j++)
    {
      index_left = i - j;
      index_right = i + j;

      if(index_left < 0)
      {
        x_nneighbors(window_index_middle - j) = 0;
        y_nneighbors(window_index_middle - j) = data(0);
      }
      else
      {
        x_nneighbors(window_index_middle - j) = index_left;
        y_nneighbors(window_index_middle - j) = data(index_left);
      }

      if(index_right > data.rows()-1)
      {
        x_nneighbors(window_index_middle + j) = data.rows()-1;
        y_nneighbors(window_index_middle + j) = data(data.rows() - 1);
      }
      else
      {
        x_nneighbors(window_index_middle + j) = index_right;
        y_nneighbors(window_index_middle + j) = data(index_right);
      }

    }

    epanechnikov_function(i,x_nneighbors,window_size,kernel_weights);
    smoothed(i) = (y_nneighbors.array()*kernel_weights.array()).sum()/kernel_weights.sum();
  }


}

namespace stomp_moveit
{
namespace cost_functions
{

CollisionCheck::CollisionCheck():
    name_("CollisionCheckPlugin"),
    robot_state_(),
    collision_penalty_(0.0)
{
  // TODO Auto-generated constructor stub

}

CollisionCheck::~CollisionCheck()
{
  // TODO Auto-generated destructor stub
}

bool CollisionCheck::initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                        const std::string& group_name,XmlRpc::XmlRpcValue& config)
{
  robot_model_ptr_ = robot_model_ptr;
  group_name_ = group_name;
  return configure(config);
}

bool CollisionCheck::setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
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
  collision_request_.distance = false;
  collision_request_.max_contacts = 1;
  collision_request_.max_contacts_per_pair = 1;
  collision_request_.contacts = true;
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

  // allocating arrays
  raw_costs_ = Eigen::VectorXd::Zero(config.num_timesteps);


  return true;
}

bool CollisionCheck::computeCosts(const Eigen::MatrixXd& parameters,
                          std::size_t start_timestep,
                          std::size_t num_timesteps,
                          int iteration_number,
                          int rollout_number,
                          Eigen::VectorXd& costs,
                          bool& validity)
{

  using namespace moveit::core;
  using namespace Eigen;

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

  // resetting array
  raw_costs_.setZero();

  // collision
  collision_detection::CollisionRequest request = collision_request_;
  collision_detection::CollisionResult result_world_collision, result_robot_collision;
  std::vector<collision_detection::CollisionResult> results(2);
  validity = true;

  // robot state
  const JointModelGroup* joint_group = robot_model_ptr_->getJointModelGroup(group_name_);

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
    for(std::vector<collision_detection::CollisionResult>::iterator i = results.begin(); i != results.end(); i++)
    {
      collision_detection::CollisionResult& result = *i;
      if(result.collision)
      {
        raw_costs_(t) = collision_penalty_;
        validity = false;
        break;
      }
    }
  }

  // applying kernel smoothing
  if(!validity)
  {
    int window_size = num_timesteps*kernel_window_percentage_;
    window_size = window_size < MIN_KERNEL_WINDOW_SIZE ? MIN_KERNEL_WINDOW_SIZE : window_size;

    // adding minimum cost
    intermediate_costs_slots_ = (raw_costs_.array() < 1).cast<double>();
    raw_costs_ += (raw_costs_.sum()/raw_costs_.size())*(intermediate_costs_slots_.matrix());

    // smoothing
    applyKernelSmoothing(window_size,raw_costs_,costs);
  }

  return true;
}

bool CollisionCheck::configure(const XmlRpc::XmlRpcValue& config)
{

  // check parameter presence
  auto members = {"cost_weight","collision_penalty"};
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
    if(!config.hasMember("cost_weight") ||
        !config.hasMember("collision_penalty") ||
        !config.hasMember("kernel_window_percentage"))
    {
      ROS_ERROR("%s failed to load one or more parameters",getName().c_str());
      return false;
    }

    XmlRpc::XmlRpcValue c = config;
    cost_weight_ = static_cast<double>(c["cost_weight"]);
    collision_penalty_ = static_cast<double>(c["collision_penalty"]);
    kernel_window_percentage_ = static_cast<double>(c["kernel_window_percentage"]);
  }
  catch(XmlRpc::XmlRpcException& e)
  {
    ROS_ERROR("%s failed to parse configuration parameters",name_.c_str());
    return false;
  }

  return true;
}

void CollisionCheck::done(bool success,int total_iterations,double final_cost)
{
  robot_state_.reset();
}

} /* namespace cost_functions */
} /* namespace stomp_moveit */
