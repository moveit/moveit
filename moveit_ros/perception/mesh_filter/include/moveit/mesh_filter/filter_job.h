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

#ifndef MOVEIT_MESH_FILTER_FILTER_JOB_
#define MOVEIT_MESH_FILTER_FILTER_JOB_

#include <moveit/macros/class_forward.h>
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

namespace mesh_filter
{
MOVEIT_CLASS_FORWARD(Job);

/**
 * \brief This class is used to execute functions within the thread that holds the OpenGL context.
 */

class Job
{
public:
  Job() : done_(false)
  {
  }
  inline void wait() const;
  virtual void execute() = 0;
  inline void cancel();
  inline bool isDone() const;

protected:
  bool done_;
  mutable boost::condition_variable condition_;
  mutable boost::mutex mutex_;
};

void Job::wait() const
{
  boost::unique_lock<boost::mutex> lock(mutex_);
  while (!done_)
    condition_.wait(lock);
}

void Job::cancel()
{
  boost::unique_lock<boost::mutex> lock(mutex_);
  done_ = true;
  condition_.notify_all();
}

bool Job::isDone() const
{
  return done_;
}

template <typename ReturnType>
class FilterJob : public Job
{
public:
  FilterJob(const boost::function<ReturnType()>& exec) : Job(), exec_(exec)
  {
  }
  virtual void execute();
  const ReturnType& getResult() const;

private:
  boost::function<ReturnType()> exec_;
  ReturnType result_;
};

template <typename ReturnType>
void FilterJob<ReturnType>::execute()
{
  boost::unique_lock<boost::mutex> lock(mutex_);
  if (!done_)  // not canceled !
    result_ = exec_();

  done_ = true;
  condition_.notify_all();
}

template <typename ReturnType>
const ReturnType& FilterJob<ReturnType>::getResult() const
{
  wait();
  return result_;
}

template <>
class FilterJob<void> : public Job
{
public:
  FilterJob(const boost::function<void()>& exec) : Job(), exec_(exec)
  {
  }
  virtual void execute()
  {
    boost::unique_lock<boost::mutex> lock(mutex_);
    if (!done_)  // not canceled !
      exec_();

    done_ = true;
    condition_.notify_all();
  }

private:
  boost::function<void()> exec_;
};
}
#endif
