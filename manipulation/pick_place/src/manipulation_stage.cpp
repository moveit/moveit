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

/* Author: Ioan Sucan, Sachin Chitta */

#include <moveit/pick_place/manipulation_stage.h>
#include <ros/console.h>

namespace pick_place
{

ManipulationStage::ManipulationStage(unsigned int nthreads) :
  nthreads_(nthreads),
  stop_processing_(true)
{
  processing_queues_.resize(nthreads_);
  processing_threads_.resize(processing_queues_.size());
  failed_.resize(processing_queues_.size());
  for (unsigned int i = 0 ; i < nthreads ; ++i)
    processing_threads_[i] = new ProcessingThread();
}

ManipulationStage::~ManipulationStage(void)
{  
  for (unsigned int i = 0 ; i < nthreads_ ; ++i)
    delete processing_threads_[i];
}

void ManipulationStage::start(void)
{
  if (!stop_processing_)
    return;
  stop_processing_ = false;
  for (std::size_t i = 0; i < processing_threads_.size() ; ++i)
    if (!processing_threads_[i]->thread_)
      processing_threads_[i]->thread_.reset(new boost::thread(boost::bind(&ManipulationStage::processingThread, this, i)));
  if (next_)
    next_->start();
}

void ManipulationStage::startAll(void)
{
  start();
  if (next_)
    next_->startAll();
}

void ManipulationStage::stop(void)
{  
  if (stop_processing_)
    return;
  stop_processing_ = true;
  
  for (std::size_t i = 0; i < processing_threads_.size() ; ++i)
    if (processing_threads_[i]->thread_)
      processing_threads_[i]->cond_.notify_all();
  
  for (std::size_t i = 0; i < processing_threads_.size() ; ++i)
    if (processing_threads_[i]->thread_)
    {
      processing_threads_[i]->thread_->join();
      processing_threads_[i]->thread_.reset();
    }
}

void ManipulationStage::stopAll(void)
{  
  if (next_)
    next_->stopAll();
  stop();
}

void ManipulationStage::processingThread(unsigned int index)
{
  std::deque<ManipulationPlanPtr> &q = processing_queues_[index];
  ProcessingThread &p = *processing_threads_[index];
  ROS_DEBUG_STREAM("Start thread " << index << " on '" << name_ << "'");
  
  while (!stop_processing_)
  {
    boost::unique_lock<boost::mutex> ulock(p.mutex_);
    
    while (q.empty() && !stop_processing_)
      p.cond_.wait(ulock); 

    while (!stop_processing_ && !q.empty())
    {
      ManipulationPlanPtr g = q.front();
      q.pop_front();
      
      p.mutex_.unlock();
      try
      {
        ROS_INFO_STREAM("Calling evaluate for stage '" << name_ << "' with thread index " << index << " (queue is of size " << q.size() << ")");
        if (evaluate(index, g))
        {       
          if (next_ && !stop_processing_)
          {
            ROS_INFO_STREAM("Evaluation of stage '" << name_ << "' with thread index " << index << " was succesful. Forwarding.");
            next_->push(g);
          }
        }
        else
        {
          if (!stop_processing_)
          {
            ROS_INFO_STREAM("Manipulation plan failed at stage '" << name_ << "' on thread " << index);
            failed_[index].push_back(g);
          }
        }
      }
      catch (std::runtime_error &ex)
      {
        ROS_ERROR("[%s:%u] %s", name_.c_str(), index, ex.what());
      }
      catch (...)
      {
        ROS_ERROR("[%s:%u] Caught unknown exception while processing manipulation stage", name_.c_str(), index);
      }  
      p.mutex_.lock();
    }
  }
}

void ManipulationStage::push(const ManipulationPlanPtr &plan)
{
  if (stop_processing_)
    return;

  // the detection of the smallest queue is not perfect, i.e., it can find a
  // queue that is not the smallest, since we do not lock, but it should be good enough
  // for what we need
  std::size_t index = 0;
  std::size_t min_size = processing_queues_[0].size();
  for (std::size_t i = 1 ; i < processing_queues_.size() ; ++i)
    if (processing_queues_[i].size() < min_size)
    {
      index = i;
      min_size = processing_queues_[i].size();
    }
  
  boost::mutex::scoped_lock slock(processing_threads_[index]->mutex_);
  processing_queues_[index].push_back(plan);
  ROS_INFO_STREAM("Added plan for stage '" << name_ << "' at thread index " << index << " (queue is now of size " << processing_queues_[index].size() << ")");
  processing_threads_[index]->cond_.notify_all();
}

}
