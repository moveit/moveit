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
#include <thread>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

namespace move_group
{
MoveGroupTfPublisher::MoveGroupTfPublisher() : MoveGroupCapability("TfPublisher")
{
}

void MoveGroupTfPublisher::publishPlanningSceneFrames()
{
  tf::TransformBroadcaster broadcaster;
  tf::Transform transform;
  ros::Rate rate(rate_);

  while (ros::ok())
  {
    {
      planning_scene_monitor::LockedPlanningSceneRO locked_planning_scene(context_->planning_scene_monitor_);
      collision_detection::WorldConstPtr world = locked_planning_scene->getWorld();
      std::string planning_frame = locked_planning_scene->getPlanningFrame();

      for (auto obj = world->begin(); obj != world->end(); ++obj)
      {
        tf::poseEigenToTF(obj->second->shape_poses_[0], transform);
        std::string parent_frame = prefix_ + obj->second->id_;
        broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), planning_frame, parent_frame));

        moveit::core::FixedTransformsMap subframes = obj->second->subframe_poses_;
        for (auto frame = subframes.begin(); frame != subframes.end(); ++frame)
        {
          tf::poseEigenToTF(frame->second, transform);
          broadcaster.sendTransform(
              tf::StampedTransform(transform, ros::Time::now(), parent_frame, parent_frame + "/" + frame->first));
        }
      }
    }

    rate.sleep();
  }
}

void MoveGroupTfPublisher::initialize()
{
  ros::NodeHandle nh = ros::NodeHandle("~");

  prefix_ = nh.getNamespace() + "/";
  nh.param("planning_scene_frame_publishing_rate", rate_, 10);

  ROS_INFO("Initializing MoveGroupTfPublisher with a frame publishing rate of %d", rate_);
  std::thread publisher_thread(&MoveGroupTfPublisher::publishPlanningSceneFrames, this);
  publisher_thread.detach();
}
}

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(move_group::MoveGroupTfPublisher, move_group::MoveGroupCapability)
