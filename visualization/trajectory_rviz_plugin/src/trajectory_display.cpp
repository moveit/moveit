/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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

/* Author: Dave Coleman */

#include <moveit/trajectory_rviz_plugin/trajectory_display.h>

namespace moveit_rviz_plugin
{

TrajectoryDisplay::TrajectoryDisplay() :
  Display()
{
  traj_visual_.reset(new TrajectoryVisualization(this, this));
}

TrajectoryDisplay::~TrajectoryDisplay()
{
}

void TrajectoryDisplay::onInitialize()
{
  Display::onInitialize();
  traj_visual_->onInitialize(scene_node_, context_, update_nh_);
}

void TrajectoryDisplay::reset()
{
  Display::reset();
  traj_visual_->reset();
}

void TrajectoryDisplay::onEnable()
{
  Display::onEnable();
  traj_visual_->onEnable();
}

void TrajectoryDisplay::onDisable()
{
  Display::onDisable();
  traj_visual_->onDisable();
}

void TrajectoryDisplay::update(float wall_dt, float ros_dt)
{
  Display::update(wall_dt, ros_dt);
  traj_visual_->update(wall_dt, ros_dt);
}

} // namespace moveit_rviz_plugin
