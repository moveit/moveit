/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
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

/* Author: Suat Gedikli */

#include <moveit/mesh_filter/transform_provider.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>

using namespace std;
using namespace boost;
using namespace Eigen;
using namespace tf;
using namespace mesh_filter;

TransformProvider::TransformProvider(unsigned long interval_us) : stop_(true), interval_us_(interval_us)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
  tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));
  psm_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description", tf_buffer_));
  psm_->startStateMonitor();
}

TransformProvider::~TransformProvider()
{
  stop();
}

void TransformProvider::start()
{
  stop_ = false;
  thread_ = thread(&TransformProvider::run, this);
}

void TransformProvider::stop()
{
  stop_ = true;
  thread_.join();
}

void TransformProvider::addHandle(MeshHandle handle, const string& name)
{
  if (!stop_)
    throw runtime_error("Can not add handles if TransformProvider is running");

  handle2context_[handle].reset(new TransformContext(name));
}

void TransformProvider::setFrame(const string& frame)
{
  if (frame_id_ != frame)
  {
    frame_id_ = frame;
    for (map<MeshHandle, shared_ptr<TransformContext> >::iterator contextIt = handle2context_.begin();
         contextIt != handle2context_.end(); ++contextIt)
    {
      // invalidate transformations
      contextIt->second->mutex_.lock();
      contextIt->second->transformation_.matrix().setZero();
      contextIt->second->mutex_.unlock();
    }
  }
}

bool TransformProvider::getTransform(MeshHandle handle, Affine3d& transform) const
{
  map<MeshHandle, shared_ptr<TransformContext> >::const_iterator contextIt = handle2context_.find(handle);

  if (contextIt == handle2context_.end())
  {
    ROS_ERROR("Unable to find mesh with handle %d", handle);
    return false;
  }
  contextIt->second->mutex_.lock();
  transform = contextIt->second->transformation_;
  contextIt->second->mutex_.unlock();
  return !(transform.matrix().isZero(0));
}

void TransformProvider::run()
{
  if (handle2context_.empty())
    throw runtime_error("TransformProvider is listening to empty list of frames!");

  while (!stop_)
  {
    updateTransforms();
    usleep(interval_us_);
  }
}

void TransformProvider::setUpdateInterval(unsigned long usecs)
{
  interval_us_ = usecs;
}

void TransformProvider::updateTransforms()
{
  static tf2::Stamped<Affine3d> input_transform, output_transform;
  static robot_state::RobotStatePtr robot_state;
  robot_state = psm_->getStateMonitor()->getCurrentState();
  try
  {
    geometry_msgs::TransformStamped common_tf =
        tf_buffer_->lookupTransform(frame_id_, psm_->getPlanningScene()->getPlanningFrame(), ros::Time(0.0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_ERROR("TF Problem: %s", ex.what());
    return;
  }
  input_transform.stamp_ = common_tf.header.stamp;
  input_transform.frame_id_ = psm_->getPlanningScene()->getPlanningFrame();

  for (map<MeshHandle, shared_ptr<TransformContext> >::const_iterator contextIt = handle2context_.begin();
       contextIt != handle2context_.end(); ++contextIt)
  {
    try
    {
      // TODO: check logic here - which global collision body's transform should be used?
      input_transform.setData(
          robot_state->getAttachedBody(contextIt->second->frame_id_)->getGlobalCollisionBodyTransforms()[0]);
      tf_buffer_->transform(input_transform, output_transform, frame_id_);
    }
    catch (const tf2::TransformException& ex)
    {
      handle2context_[contextIt->first]->mutex_.lock();
      handle2context_[contextIt->first]->transformation_.matrix().setZero();
      handle2context_[contextIt->first]->mutex_.unlock();
      continue;
    }
    catch (std::exception& ex)
    {
      ROS_ERROR("Caught %s while updating transforms", ex.what());
    }
    handle2context_[contextIt->first]->mutex_.lock();
    handle2context_[contextIt->first]->transformation_ = static_cast<Affine3d>(output_transform);
    handle2context_[contextIt->first]->mutex_.unlock();
  }
}
