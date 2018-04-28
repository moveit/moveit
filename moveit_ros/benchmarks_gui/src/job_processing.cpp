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

/* Author: Mario Prats, Ioan Sucan */

#include <ros/ros.h>
#include <job_processing.h>

namespace benchmark_tool
{
std::deque<boost::function<void(void)> > JobProcessing::main_loop_jobs_;
boost::mutex JobProcessing::main_loop_jobs_lock_;
moveit::tools::BackgroundProcessing JobProcessing::background_process_;

void JobProcessing::addBackgroundJob(const boost::function<void(void)>& job)
{
  background_process_.addJob(job, "noname");
}

void JobProcessing::addMainLoopJob(const boost::function<void(void)>& job)
{
  boost::mutex::scoped_lock slock(main_loop_jobs_lock_);
  main_loop_jobs_.push_back(job);
}

void JobProcessing::executeMainLoopJobs()
{
  main_loop_jobs_lock_.lock();
  while (!main_loop_jobs_.empty())
  {
    boost::function<void(void)> fn = main_loop_jobs_.front();
    main_loop_jobs_.pop_front();
    main_loop_jobs_lock_.unlock();
    try
    {
      fn();
    }
    catch (std::runtime_error& ex)
    {
      ROS_ERROR("Exception caught executing main loop job: %s", ex.what());
    }
    catch (...)
    {
      ROS_ERROR("Exception caught executing main loop job");
    }
    main_loop_jobs_lock_.lock();
  }
  main_loop_jobs_lock_.unlock();
}

}  // namespace
