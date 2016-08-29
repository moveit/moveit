/**
 * @file stomp_optimization_task.cpp
 * @brief This defines stomp's optimization task
 *
 * @author Jorge Nicho
 * @date March 23, 2016
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
#include <stdexcept>
#include "stomp_moveit/stomp_optimization_task.h"

using PluginArray = std::vector< std::pair<std::string,XmlRpc::XmlRpcValue> >;

static const std::string COST_FUNCTIONS_FIELD = "cost_functions";
static const std::string NOISY_FILTERS_FIELD = "noisy_filters";
static const std::string UPDATE_FILTERS_FIELD = "update_filters";
static const std::string NOISE_GENERATOR_FIELD = "noise_generator";

bool parsePlugins(XmlRpc::XmlRpcValue config,
                      std::string param_name,
                      PluginArray& plugins)
{
  if(config.hasMember(param_name) && (config[param_name].getType() == XmlRpc::XmlRpcValue::TypeArray))
  {
    XmlRpc::XmlRpcValue& plugin_list = config[param_name];
    std::string class_name;

    // look for the 'class' entry
    for(auto i = 0u ; i < plugin_list.size(); i++)
    {
      XmlRpc::XmlRpcValue& plugin_config = plugin_list[i];
      if(plugin_config.hasMember("class") && (plugin_config["class"].getType() == XmlRpc::XmlRpcValue::TypeString))
      {
        class_name = static_cast<std::string>(plugin_config["class"]);
        plugins.push_back(std::make_pair(class_name,plugin_config));
      }
      else
      {
        ROS_ERROR("Element in the '%s' array parameter did not contain a 'class' entry",param_name.c_str());
        return false;
      }
    }
  }
  else
  {
    ROS_WARN("Plugin under entry '%s' was not found in ros parameter.",param_name.c_str());
    ROS_DEBUG("Failed to find plugin under entry '%s' in ros parameter %s",param_name.c_str(),config.toXml().c_str());
    return false;
  }

  return !plugins.empty();
}

namespace stomp_moveit
{

StompOptimizationTask::StompOptimizationTask(
    moveit::core::RobotModelConstPtr robot_model_ptr,
    std::string group_name,
    const XmlRpc::XmlRpcValue& config):
        robot_model_ptr_(robot_model_ptr),
        group_name_(group_name)
{
  // initializing plugin loaders
  cost_function_loader_.reset(new CostFunctionLoader("stomp_moveit", "stomp_moveit::cost_functions::StompCostFunction"));
  noise_generator_loader_.reset(new NoiseGeneratorLoader("stomp_moveit",
                                                         "stomp_moveit::noise_generators::StompNoiseGenerator"));
  noisy_filter_loader_.reset(new NoisyFilterLoader("stomp_moveit", "stomp_moveit::noisy_filters::StompNoisyFilter"));
  update_filter_loader_.reset(new UpdateFilterLoader("stomp_moveit", "stomp_moveit::update_filters::StompUpdateFilter"));


  // loading cost function plugins
  if(!initializeCostFunctionPlugins(config))
  {
    ROS_ERROR("StompOptimizationTask/%s failed to load '%s' plugins from yaml",group_name.c_str(),COST_FUNCTIONS_FIELD.c_str());
    throw std::logic_error("plugin not found");
  }

  // loading noise generators
  if(!initializeNoiseGeneratorPlugins(config))
  {
    ROS_ERROR("StompOptimizationTask/%s failed to load '%s' plugins from yaml",group_name.c_str(),
             NOISE_GENERATOR_FIELD.c_str());
    throw std::logic_error("plugin not found");
  }

  // loading noisy filter plugins
  if(!initializeNoisyFilterPlugins(config))
  {
    ROS_WARN("StompOptimizationTask/%s failed to load '%s' plugins from yaml",group_name.c_str(),NOISY_FILTERS_FIELD.c_str());
  }

  // loading filter plugins
  if(!initializeUpdateFilterPlugins(config))
  {
    ROS_WARN("StompOptimizationTask/%s failed to load '%s' plugins from yaml",group_name.c_str(),UPDATE_FILTERS_FIELD.c_str());
  }

}

StompOptimizationTask::~StompOptimizationTask()
{
  // TODO Auto-generated destructor stub
}

bool StompOptimizationTask::initializeCostFunctionPlugins(const XmlRpc::XmlRpcValue& config)
{
  PluginArray plugins;
  if(parsePlugins(config,COST_FUNCTIONS_FIELD,plugins))
  {
    for(auto& entry: plugins)
    {
      // instantiating
      cost_functions::StompCostFunctionPtr plugin;
      try
      {
        plugin = cost_function_loader_->createInstance(entry.first);
      }
      catch(pluginlib::PluginlibException& ex)
      {
        ROS_ERROR("%s CostFunction plugin could not be created",entry.first.c_str());
        return false;
      }

      // initializing
      if(plugin->initialize(robot_model_ptr_,group_name_,entry.second))
      {
        cost_functions_.push_back(plugin);
        ROS_INFO_STREAM("StompOptimizationTask loaded "<<plugin->getName()<<" CostFunction plugin");
      }
      else
      {
        ROS_ERROR("%s CostFunction plugin failed to initialize",entry.first.c_str());
        return false;
      }
    }

    std::stringstream ss;
    ss<<"[";
    auto arrayToString = [&ss](PluginArray::value_type& p)
    {
      ss<<p.first<<" ";
    };

    std::for_each(plugins.begin(),plugins.end(),arrayToString);
    ss<<"]";

    ROS_DEBUG("Loaded CostFunction plugins: %s",ss.str().c_str());
  }
  else
  {
    return false;
  }

  return true;
}

bool StompOptimizationTask::initializeNoisyFilterPlugins(const XmlRpc::XmlRpcValue& config)
{
  PluginArray plugins;
  bool success = false;
  if(parsePlugins(config,NOISY_FILTERS_FIELD,plugins))
  {
    for(auto& entry: plugins)
    {
      // instantiating
      noisy_filters::StompNoisyFilterPtr plugin;
      try
      {
        plugin = noisy_filter_loader_->createInstance(entry.first);
      }
      catch(pluginlib::PluginlibException& ex)
      {
        ROS_WARN("%s NoisyFilter plugin could not be created",entry.first.c_str());
        continue;
      }

      // initializing
      if(plugin->initialize(robot_model_ptr_,group_name_,entry.second))
      {
        noisy_filters_.push_back(plugin);
        ROS_INFO_STREAM("Stomp Optimization Task loaded "<<plugin->getName()<<" NoisyFilter plugin");
        success = true;
      }
      else
      {
        ROS_WARN("%s NoisyFilter plugin failed to initialize",entry.first.c_str());
        continue;
      }
    }
  }
  else
  {
    return false;
  }

  return success;
}

bool StompOptimizationTask::initializeUpdateFilterPlugins(const XmlRpc::XmlRpcValue& config)
{
  PluginArray plugins;
  bool success = false;
  if(parsePlugins(config,UPDATE_FILTERS_FIELD,plugins))
  {
    for(auto& entry: plugins)
    {
      // instantiating
      update_filters::StompUpdateFilterPtr plugin;
      try
      {
        plugin = update_filter_loader_->createInstance(entry.first);
      }
      catch(pluginlib::PluginlibException& ex)
      {
        ROS_WARN("%s UpdateFilter plugin could not be created",entry.first.c_str());
        continue;
      }

      // initializing
      if(plugin->initialize(robot_model_ptr_,group_name_,entry.second))
      {
        update_filters_.push_back(plugin);
        ROS_INFO_STREAM("Stomp Optimization Task loaded "<<plugin->getName()<<" UpdateFilter plugin");
        success = true;
      }
      else
      {
        ROS_WARN("%s UpdateFilter plugin failed to initialize",entry.first.c_str());
        continue;
      }
    }
  }
  else
  {
    return false;
  }

  return success;
}

bool StompOptimizationTask::initializeNoiseGeneratorPlugins(const XmlRpc::XmlRpcValue& config)
{
  PluginArray plugins;
  bool success = false;
  if(parsePlugins(config,NOISE_GENERATOR_FIELD,plugins))
  {
    for(auto& entry: plugins)
    {
      // instantiating
      noise_generators::StompNoiseGeneratorPtr plugin;
      try
      {
        plugin = noise_generator_loader_->createInstance(entry.first);
      }
      catch(pluginlib::PluginlibException& ex)
      {
        ROS_ERROR("%s NoiseGenerator plugin could not be created",entry.first.c_str());
        return false;
      }

      // initializing
      if(plugin->initialize(robot_model_ptr_,group_name_,entry.second))
      {
        noise_generators_.push_back(plugin);
        ROS_INFO_STREAM("Stomp Optimization Task loaded "<<plugin->getName()<<" NoiseGenerator plugin");
        success = true;
        break;
      }
      else
      {
        ROS_WARN("%s NoiseGenerator plugin failed to initialize",entry.first.c_str());
        continue;
      }
    }
  }
  else
  {
    return false;
  }

  return success;
}

bool StompOptimizationTask::generateNoisyParameters(const Eigen::MatrixXd& parameters,
                                     std::size_t start_timestep,
                                     std::size_t num_timesteps,
                                     int iteration_number,
                                     int rollout_number,
                                     Eigen::MatrixXd& parameters_noise,
                                     Eigen::MatrixXd& noise)
{
  return noise_generators_.back()->generateNoise(parameters,start_timestep,num_timesteps,iteration_number,rollout_number,
                                                 parameters_noise,noise);
}

bool StompOptimizationTask::computeNoisyCosts(const Eigen::MatrixXd& parameters,
                                         std::size_t start_timestep,
                                         std::size_t num_timesteps,
                                         int iteration_number,
                                         int rollout_number,
                                         Eigen::VectorXd& costs,
                                         bool& validity)
{
  Eigen::MatrixXd cost_matrix = Eigen::MatrixXd::Zero(num_timesteps,cost_functions_.size());
  Eigen::VectorXd state_costs = Eigen::VectorXd::Zero(num_timesteps);
  validity = true;
  for(auto i = 0u; i < cost_functions_.size(); i++ )
  {
    bool valid;
    auto cf = cost_functions_[i];

    if(!cf->computeCosts(parameters,start_timestep,num_timesteps,iteration_number,rollout_number,state_costs,valid))
    {
      return false;
    }

    validity &= valid;

    cost_matrix.col(i) = state_costs * cf->getWeight();
  }
  costs = cost_matrix.rowwise().sum();
  return true;
}

bool StompOptimizationTask::computeCosts(const Eigen::MatrixXd& parameters,
                                         std::size_t start_timestep,
                                         std::size_t num_timesteps,
                                         int iteration_number,
                                         Eigen::VectorXd& costs,
                                         bool& validity)
{
  Eigen::MatrixXd cost_matrix = Eigen::MatrixXd::Zero(num_timesteps,cost_functions_.size());
  Eigen::VectorXd state_costs = Eigen::VectorXd::Zero(num_timesteps);
  validity = true;
  for(auto i = 0u; i < cost_functions_.size(); i++ )
  {
    bool valid;
    auto cf = cost_functions_[i];

    if(!cf->computeCosts(parameters,start_timestep,num_timesteps,iteration_number,cf->getOptimizedIndex(),state_costs,valid))
    {
      return false;
    }

    validity &= valid;

    cost_matrix.col(i) = state_costs * cf->getWeight();
  }
  costs = cost_matrix.rowwise().sum();
  return true;
}

bool StompOptimizationTask::setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                        const moveit_msgs::MotionPlanRequest &req,
                                        const stomp_core::StompConfiguration &config,
                                        moveit_msgs::MoveItErrorCodes& error_code)
{
  for(auto p: noise_generators_)
  {
    if(!p->setMotionPlanRequest(planning_scene,req,config,error_code))
    {
      ROS_ERROR("Failed to set Plan Request on noise generator %s",p->getName().c_str());
      return false;
    }
  }

  for(auto p : cost_functions_)
  {
    if(!p->setMotionPlanRequest(planning_scene,req,config,error_code))
    {
      ROS_ERROR("Failed to set Plan Request on cost function %s",p->getName().c_str());
      return false;
    }
  }

  for(auto p: noisy_filters_)
  {
    if(!p->setMotionPlanRequest(planning_scene,req,config,error_code))
    {
      ROS_ERROR("Failed to set Plan Request on noisy filter %s",p->getName().c_str());
      return false;
    }
  }

  for(auto p: update_filters_)
  {
    if(!p->setMotionPlanRequest(planning_scene,req,config,error_code))
    {
      ROS_ERROR("Failed to set Plan Request on update filter %s",p->getName().c_str());
      return false;
    }
  }

  return true;
}

bool StompOptimizationTask::filterNoisyParameters(std::size_t start_timestep,
                                                  std::size_t num_timesteps,
                                                  int iteration_number,
                                                  int rollout_number,
                                                  Eigen::MatrixXd& parameters,bool& filtered)
{
  filtered = false;
  bool temp;
  for(auto& f: noisy_filters_)
  {
    if(f->filter(start_timestep,num_timesteps,iteration_number,rollout_number,parameters,temp))
    {
      filtered |= temp;
    }
    else
    {
      return false;
    }
  }
  return true;
}

bool StompOptimizationTask::filterParameterUpdates(std::size_t start_timestep,
                                             std::size_t num_timesteps,
                                             int iteration_number,
                                             const Eigen::MatrixXd& parameters,
                                             Eigen::MatrixXd& updates)
{
  bool filtered = false;
  bool temp;
  for(auto& f: update_filters_)
  {
    if(f->filter(start_timestep,num_timesteps,iteration_number,parameters,updates,temp))
    {
      filtered |= temp;
    }
    else
    {
      return false;
    }
  }
  return true;
}

void StompOptimizationTask::done(bool success,int total_iterations,double final_cost)
{
  for(auto p : noise_generators_)
  {
    p->done(success,total_iterations,final_cost);
  }


  for(auto p : cost_functions_)
  {
    p->done(success,total_iterations,final_cost);
  }

  for(auto p: noisy_filters_)
  {
    p->done(success,total_iterations,final_cost);
  }

  for(auto p: update_filters_)
  {
    p->done(success,total_iterations,final_cost);
  }
}

} /* namespace stomp_moveit */
