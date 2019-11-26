/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Hamburg University
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
 *   * Neither the name of Hamburg University nor the names of its
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

/* Author: Jonas Tietz */

#include "tf_publisher_capability.h"
#include <moveit/utils/message_checks.h>
#include <moveit/move_group/capability_names.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>

namespace move_group
{
TfPublisher::TfPublisher() : MoveGroupCapability("TfPublisher")
{
}

TfPublisher::~TfPublisher()
{
  keep_running_ = false;
  thread_.join();
}

void TfPublisher::publishPlanningSceneFrames()
{
  tf2_ros::TransformBroadcaster broadcaster;
  geometry_msgs::TransformStamped transform;
  ros::Rate rate(rate_);

  while (keep_running_)
  {
    {
      planning_scene_monitor::LockedPlanningSceneRO locked_planning_scene(context_->planning_scene_monitor_);
      collision_detection::WorldConstPtr world = locked_planning_scene->getWorld();
      std::string planning_frame = locked_planning_scene->getPlanningFrame();

      for (auto obj = world->begin(); obj != world->end(); ++obj)
      {
        std::string parent_frame = prefix_ + obj->second->id_;
        transform = tf2::eigenToTransform(obj->second->shape_poses_[0]);
        transform.child_frame_id = parent_frame;
        transform.header.frame_id = planning_frame;
        broadcaster.sendTransform(transform);

        moveit::core::FixedTransformsMap subframes = obj->second->subframe_poses_;
        for (auto frame = subframes.begin(); frame != subframes.end(); ++frame)
        {
          transform = tf2::eigenToTransform(frame->second);
          transform.child_frame_id = parent_frame + "/" + frame->first;
          transform.header.frame_id = parent_frame;
          broadcaster.sendTransform(transform);
        }
      }
    }

    rate.sleep();
  }
}

void TfPublisher::initialize()
{
  ros::NodeHandle nh = ros::NodeHandle("~");

  std::string prefix = nh.getNamespace() + "/";
  nh.param("planning_scene_frame_publishing_rate", rate_, 10);
  nh.param("planning_scene_tf_prefix", prefix_, prefix);
  keep_running_ = true;

  ROS_INFO("Initializing MoveGroupTfPublisher with a frame publishing rate of %d", rate_);
  thread_ = std::thread(&TfPublisher::publishPlanningSceneFrames, this);
}
}

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(move_group::TfPublisher, move_group::MoveGroupCapability)
