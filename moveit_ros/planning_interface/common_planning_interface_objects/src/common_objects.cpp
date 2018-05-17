/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Ioan Sucan */

#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <tf2_ros/transform_listener.h>

namespace
{
struct SharedStorage
{
  SharedStorage()
  {
  }

  ~SharedStorage()
  {
    tf_buffer_.reset();
    tf_listener_.reset();
    state_monitors_.clear();
    model_loaders_.clear();
  }

  boost::mutex lock_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::map<std::string, robot_model_loader::RobotModelLoaderPtr> model_loaders_;
  std::map<std::string, planning_scene_monitor::CurrentStateMonitorPtr> state_monitors_;
};

SharedStorage& getSharedStorage()
{
#if 0  // destruction of static storage interferes with static destruction in class_loader
  // More specifically, class_loader's static variables might be already destroyed
  // while being accessed again in the destructor of the class_loader-based kinematics plugin.
  static SharedStorage storage;
  return storage;
#else  // thus avoid destruction at all (until class_loader is fixed)
  static SharedStorage* storage = new SharedStorage;
  return *storage;
#endif
}
}

namespace moveit
{
namespace planning_interface
{
std::shared_ptr<tf2_ros::Buffer> getSharedTF()
{
  SharedStorage& s = getSharedStorage();
  boost::mutex::scoped_lock slock(s.lock_);
  if (!s.tf_listener_)
  {
    s.tf_buffer_.reset(new tf2_ros::Buffer());
    s.tf_listener_.reset(new tf2_ros::TransformListener(*s.tf_buffer_));
  }
  return s.tf_buffer_;
}

robot_model::RobotModelConstPtr getSharedRobotModel(const std::string& robot_description)
{
  SharedStorage& s = getSharedStorage();
  boost::mutex::scoped_lock slock(s.lock_);
  if (s.model_loaders_.find(robot_description) != s.model_loaders_.end())
    return s.model_loaders_[robot_description]->getModel();
  else
  {
    robot_model_loader::RobotModelLoader::Options opt(robot_description);
    opt.load_kinematics_solvers_ = true;
    robot_model_loader::RobotModelLoaderPtr loader(new robot_model_loader::RobotModelLoader(opt));
    s.model_loaders_[robot_description] = loader;
    return loader->getModel();
  }
}

planning_scene_monitor::CurrentStateMonitorPtr getSharedStateMonitor(const robot_model::RobotModelConstPtr& kmodel,
                                                                     const std::shared_ptr<tf2_ros::Buffer>& tf_buffer)
{
  return getSharedStateMonitor(kmodel, tf_buffer, ros::NodeHandle());
}

planning_scene_monitor::CurrentStateMonitorPtr getSharedStateMonitor(const robot_model::RobotModelConstPtr& kmodel,
                                                                     const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
                                                                     ros::NodeHandle nh)
{
  SharedStorage& s = getSharedStorage();
  boost::mutex::scoped_lock slock(s.lock_);
  if (s.state_monitors_.find(kmodel->getName()) != s.state_monitors_.end())
    return s.state_monitors_[kmodel->getName()];
  else
  {
    planning_scene_monitor::CurrentStateMonitorPtr monitor(
        new planning_scene_monitor::CurrentStateMonitor(kmodel, tf_buffer, nh));
    s.state_monitors_[kmodel->getName()] = monitor;
    return monitor;
  }
}

}  // namespace planning_interface
}  // namespace moveit
