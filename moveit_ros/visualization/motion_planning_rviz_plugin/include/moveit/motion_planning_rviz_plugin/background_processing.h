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

#ifndef MOVEIT_MOTION_PLANNING_RVIZ_PLUGIN_BACKGROUND_PROCESSING_
#define MOVEIT_MOTION_PLANNING_RVIZ_PLUGIN_BACKGROUND_PROCESSING_

#include <deque>
#include <boost/thread.hpp>
#include <boost/function.hpp>
#include <boost/scoped_ptr.hpp>

namespace moveit_rviz_plugin
{

class BackgroundProcessing
{
public:
  BackgroundProcessing();
  ~BackgroundProcessing();

  void addJob(const boost::function<void()> &job);
  std::size_t getJobCount() const;
  void clear();
  
  void setCompletionEvent(const boost::function<void()> &completion_event);
  
private:
  
  boost::scoped_ptr<boost::thread> processing_thread_;
  bool run_processing_thread_;
  
  mutable boost::mutex action_lock_;
  boost::condition_variable new_action_condition_;
  std::deque<boost::function<void()> > actions_;
  boost::function<void()> completion_event_;
  bool processing_;
  
  void processingThread();
};

}

#endif

