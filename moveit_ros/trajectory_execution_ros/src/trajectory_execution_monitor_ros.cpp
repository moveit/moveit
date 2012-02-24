/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/** \author E. Gil Jones */

#include <trajectory_execution_ros/trajectory_execution_monitor_ros.h>
#include <pr2_mechanism_msgs/ListControllers.h>
#include <pr2_mechanism_msgs/SwitchController.h>
#include <pr2_mechanism_msgs/LoadController.h>
#include <pr2_mechanism_msgs/UnloadController.h>

namespace trajectory_execution_ros
{

TrajectoryExecutionMonitorRos::TrajectoryExecutionMonitorRos(const planning_models::KinematicModelConstPtr& kmodel) : 
  TrajectoryExecutionMonitor(kmodel),
  controller_handler_loader_("trajectory_execution", "trajectory_execution::TrajectoryControllerHandler"),
  recorder_loader_("trajectory_execution", "trajectory_execution::TrajectoryRecorder")
{
  ros::NodeHandle gh;

  ros::NodeHandle nh("~");
  
  std::string controller_manager_ns;
  nh.param("controller_manager_ns", controller_manager_ns, std::string("pr2_controller_manager"));

  ros::service::waitForService(controller_manager_ns+"/list_controllers");

  lister_service_ = gh.serviceClient<pr2_mechanism_msgs::ListControllers>(controller_manager_ns+"/list_controllers", true);
  switcher_service_ = gh.serviceClient<pr2_mechanism_msgs::SwitchController>(controller_manager_ns+"/switch_controller", true);
  loader_service_ = gh.serviceClient<pr2_mechanism_msgs::LoadController>(controller_manager_ns+"/load_controller", true);
  unloader_service_ = gh.serviceClient<pr2_mechanism_msgs::UnloadController>(controller_manager_ns+"/unload_controller", true);

  while(1) {
    bool some_running = getRunningControllerMap(original_controller_configuration_map_);
    if(!some_running) {
      ROS_INFO_STREAM("Not enough controllers .  Waiting");
      ros::WallDuration(1.0).sleep();
    } else {
      break;
    }
  }
  const std::map<std::string, srdf::Model::Group>& group_map = kmodel_->getJointModelGroupConfigMap();

  for(std::map<std::string, srdf::Model::Group>::const_iterator it = group_map.begin();
      it != group_map.end();
      it++) 
  {
    XmlRpc::XmlRpcValue controller_names;
    if(nh.getParam(it->first+"/controllers", controller_names)) {
      if(controller_names.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_WARN_STREAM("Controller names for " << it->first << " formatted badly");
        continue;
      }
      if(controller_names.size() == 0) {
        ROS_WARN_STREAM("No controllers specified for " << it->first);
      }
      for(int i = 0; i < controller_names.size(); i++) {
        if(!controller_names[i].hasMember("name")) {
          ROS_WARN_STREAM("All controllers have a name");
          continue;
        } 
        const std::string cname = std::string(controller_names[i]["name"]);
        if(!controller_names[i].hasMember("ns") || !controller_names[i].hasMember("type")) {
          group_possible_controllers_map_[it->first][cname] = false;
          continue;
        }
        std::string ns = std::string(controller_names[i]["ns"]);
        std::string type = std::string(controller_names[i]["type"]);
        bool load = false;
        bool is_default = false;
        if(controller_names[i].hasMember("load")) {
          load = controller_names[i]["load"];
        }
        controller_possible_group_map_[cname][it->first] = true;
        group_possible_controllers_map_[it->first][cname] = true;
        if(controller_names[i].hasMember("default")) {
          is_default = controller_names[i]["default"];
        }
        if(!load) {
          if(original_controller_configuration_map_.find(cname) == original_controller_configuration_map_.end()) {
            ROS_WARN_STREAM("Controller " << cname << " not loaded and not going to load");
            continue;
          }
          ROS_INFO_STREAM("Not loading " << cname);
        }
        if(load) {
          if(original_controller_configuration_map_.find(cname) != original_controller_configuration_map_.end()) {
            ROS_INFO_STREAM("Controller " << cname << " already loaded.  won't load");
            loaded_controllers_.push_back(cname);
          } else {
            ROS_INFO_STREAM("Attempting to load " << cname);
            loadController(cname);
          }
        }
        ROS_INFO_STREAM("Loading group " << it->first << " controller " << cname << " ns " << ns 
                        << " type " << type << " load " << load << " default " << is_default);
        boost::shared_ptr<trajectory_execution::TrajectoryControllerHandler> handler;
        handler.reset(controller_handler_loader_.createClassInstance(type));
        if(!handler) {
          ROS_WARN_STREAM("Couldn't create plugin instance of type " << type);
          continue;
        } else {
          handler->initialize(it->first, 
                              cname,
                              ns);
          addTrajectoryControllerHandler(handler, is_default);
        }
      }
    }
  }
  std::vector<std::string> recorders = recorder_loader_.getDeclaredClasses();
  if(recorders.size() == 0) {
    ROS_WARN_STREAM("No recorder plugins found");
  }
  for(unsigned int i = 0; i < recorders.size(); i++) {
    boost::shared_ptr<trajectory_execution::TrajectoryRecorder> rec;
    rec.reset(recorder_loader_.createClassInstance(recorders[i]));
    addTrajectoryRecorder(rec);
  }
  
  for(std::map<std::string, std::map<std::string, bool> >::const_iterator it = group_possible_controllers_map_.begin();
      it != group_possible_controllers_map_.end();
      it++) {
    //now we need to check each other group as a potential subgroup
    for(std::map<std::string, std::map<std::string, bool> >::const_iterator it2 = group_possible_controllers_map_.begin();
        it2 != group_possible_controllers_map_.end();
        it2++) {
      const planning_models::KinematicModel::JointModelGroup* jmg = kmodel_->getJointModelGroup(it2->first);
      //if the second is a parent of the first, we need to register the first's controllers as possible controllers
      //for the second
      if(jmg->isSubgroup(it->first)) {
        ROS_INFO_STREAM("Adding for " << it2->first << " child " << it->first);
        for(std::map<std::string,bool>::const_iterator it3 = it->second.begin(); it3 != it->second.end(); it3++) {
          ROS_INFO_STREAM("Adding for " << it3->first << " child " << it2->first);
          controller_possible_group_map_[it3->first][it2->first] = true;
        }
      }
    }
  }

  for(std::map<std::string, bool>::const_iterator it = original_controller_configuration_map_.begin();
      it != original_controller_configuration_map_.end();
      it++) {
    if(it->second) {
      std::map<std::string, std::map<std::string,bool> >::const_iterator it2 = controller_possible_group_map_.find(it->first);
      if(it2 != controller_possible_group_map_.end()) {
        for(std::map<std::string, bool>::const_iterator it3 = it2->second.begin(); it3 != it2->second.end(); it3++) {
          //if this is the default for some group, set it to be the current
          if(default_trajectory_controller_handler_map_.find(it3->first) == default_trajectory_controller_handler_map_.end()) {
            ROS_DEBUG_STREAM("No default for " << it3->first);
          } else if(default_trajectory_controller_handler_map_.find(it3->first)->second->getControllerName() == it->first) {
            ROS_INFO_STREAM("Setting owner of " << it->first << " to " << it3->first);
            current_controller_group_name_map_[it->first] = it3->first;
          }
          //should mark is as current for all possible groups
          ROS_INFO_STREAM("Setting current for " << it3->first << " to " << it->first);
          current_group_controller_name_map_[it3->first] = it->first;
        }
      } 
    }
  }
}

TrajectoryExecutionMonitorRos::~TrajectoryExecutionMonitorRos() {
  ROS_INFO_STREAM("Restoring controllers");
  restoreOriginalControllers();
}

bool TrajectoryExecutionMonitorRos::getRunningControllerMap(std::map<std::string, bool>& controller_map) {
 
  bool some_running = false;

  controller_map.clear();

  pr2_mechanism_msgs::ListControllers::Request req;
  pr2_mechanism_msgs::ListControllers::Response res;

  if(!lister_service_.call(req, res)) {
    ROS_WARN_STREAM("Something wrong with lister service");
    return false;
  }
  for(unsigned int i = 0; i < res.controllers.size(); i++) {
    controller_map[res.controllers[i]] = (res.state[i] == "running");
    if(res.state[i] == "running") {
      some_running = true;
    }
  }
  return some_running;
}

void TrajectoryExecutionMonitorRos::loadController(const std::string& name) {
  pr2_mechanism_msgs::LoadController::Request req;
  pr2_mechanism_msgs::LoadController::Response res;
  req.name = name;
  if(!loader_service_.call(req, res)) {
    ROS_WARN_STREAM("Something wrong with loader service");
    return;
  }
  if(!res.ok) {
    ROS_WARN_STREAM("Loading controller " << name << " not ok");
  } else {
    loaded_controllers_.push_back(name);
  }
}

void TrajectoryExecutionMonitorRos::unloadController(const std::string& name) {
  pr2_mechanism_msgs::UnloadController::Request req;
  pr2_mechanism_msgs::UnloadController::Response res;

  req.name = name;
  if(!unloader_service_.call(req, res)) {
    ROS_WARN_STREAM("Something wrong with unloader service");
    return;
  }
  if(!res.ok) {
    ROS_WARN_STREAM("Unloading controller " << name << " not ok");
  } else {
    ROS_INFO_STREAM("Unloaded name " << name);
  }
}

void TrajectoryExecutionMonitorRos::unloadAllLoadedControllers() {
  for(unsigned int i = 0; i < loaded_controllers_.size(); i++) {
    unloadController(loaded_controllers_[i]);
  }
  loaded_controllers_.clear();
}

void TrajectoryExecutionMonitorRos::switchControllers(const std::vector<std::string>& stop_controllers,
                                                      const std::vector<std::string>& start_controllers) {
  pr2_mechanism_msgs::SwitchController::Request req;
  pr2_mechanism_msgs::SwitchController::Response res;

  req.strictness = pr2_mechanism_msgs::SwitchController::Request::BEST_EFFORT;
  req.start_controllers = start_controllers;
  req.stop_controllers = stop_controllers;
  if(!switcher_service_.call(req, res)) {
    ROS_WARN_STREAM("Something wrong with switcher service");
    return;
  }
  if(!res.ok) {
    ROS_WARN_STREAM("Switcher reports not ok");
  }
}

void TrajectoryExecutionMonitorRos::restoreOriginalControllers() 
{
  std::vector<std::string> stop_controllers, start_controllers;
  for(std::map<std::string, bool>::const_iterator it = original_controller_configuration_map_.begin();
      it != original_controller_configuration_map_.end();
      it++) {
    if(it->second) {
      start_controllers.push_back(it->first);
    } else {
      stop_controllers.push_back(it->first);
    }
  }
  //stopping anything currently running that wasn't original
  for(std::map<std::string, std::string>::const_iterator it = current_group_controller_name_map_.begin();
      it != current_group_controller_name_map_.end();
      it++) {
    if(original_controller_configuration_map_.find(it->second) == original_controller_configuration_map_.end()) {
      stop_controllers.push_back(it->second);
    }
  }
  switchControllers(stop_controllers, start_controllers);
  unloadAllLoadedControllers();
}

}
