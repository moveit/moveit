/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Ioan Sucan */

#include "moveit_rviz_plugin/background_processing.h"
#include <ros/console.h>

moveit_rviz_plugin::BackgroundProcessing::BackgroundProcessing(void)
{
  // spin a thread that will process user events
  run_processing_thread_ = true;
  processing_thread_.reset(new boost::thread(boost::bind(&BackgroundProcessing::processingThread, this)));
}

moveit_rviz_plugin::BackgroundProcessing::~BackgroundProcessing(void)
{
  run_processing_thread_ = false; 
  new_action_condition_.notify_all();
  processing_thread_->join();
}

void moveit_rviz_plugin::BackgroundProcessing::processingThread(void)
{
  boost::unique_lock<boost::mutex> ulock(action_lock_);

  while (run_processing_thread_)
  {
    while (actions_.empty() && run_processing_thread_)
      new_action_condition_.wait(ulock);
    
    while (!actions_.empty())
    {
      boost::function<void(void)> fn = actions_.front();
      actions_.pop_front();
      
      // make sure we are unlocked while we process the event
      action_lock_.unlock();
      try
      {
        fn();
      } 
      catch(std::runtime_error &ex)
      {
        ROS_ERROR("Exception caught while processing event: %s", ex.what());
      }
      catch(...)
      {
        ROS_ERROR("Exception caught while processing event");
      }
      action_lock_.lock();
    }
  }
}

void moveit_rviz_plugin::BackgroundProcessing::addJob(const boost::function<void(void)> &job)
{
  boost::mutex::scoped_lock slock(action_lock_);
  actions_.push_back(job);
  new_action_condition_.notify_all();
}

void moveit_rviz_plugin::BackgroundProcessing::clear(void)
{
  boost::mutex::scoped_lock slock(action_lock_);
  actions_.clear();
}
