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

#include <moveit/pick_place/manipulation_pipeline.h>
#include <ros/console.h>

namespace pick_place
{

ManipulationPipeline::ManipulationPipeline(const std::string &name, unsigned int nthreads) :
  name_(name),
  nthreads_(nthreads),
  stop_processing_(true)
{
  processing_threads_.resize(nthreads, NULL);
}

ManipulationPipeline::~ManipulationPipeline(void)
{  
  reset();
}

ManipulationPipeline& ManipulationPipeline::addStage(const ManipulationStagePtr &next)
{
  stages_.push_back(next);
  return *this;
} 

const ManipulationStagePtr& ManipulationPipeline::getFirstStage(void) const
{
  if (stages_.empty())
  {
    static const ManipulationStagePtr empty;
    return empty;
  }
  else
    return stages_.front();    
}

const ManipulationStagePtr& ManipulationPipeline::getLastStage(void) const
{
  if (stages_.empty())
  {
    static const ManipulationStagePtr empty;
    return empty;
  }
  else
    return stages_.back();
}

const std::vector<ManipulationPlanPtr>& ManipulationPipeline::getSuccessfulManipulationPlans(void) const
{    
  return success_;
}

const std::vector<ManipulationPlanPtr>& ManipulationPipeline::getFailedPlans(void) const
{    
  return failed_;
}

void ManipulationPipeline::reset(void)
{
  clear();
  stages_.clear();
}

void ManipulationPipeline::clear(void)
{
  stop();
  {
    boost::mutex::scoped_lock slock(queue_access_lock_);
    queue_.clear();
  }
  {
    boost::mutex::scoped_lock slock(result_lock_);
    success_.clear();
    failed_.clear();
  }
}

void ManipulationPipeline::start(void)
{
  stop_processing_ = false;
  for (std::size_t i = 0; i < processing_threads_.size() ; ++i)
    if (!processing_threads_[i])
      processing_threads_[i] = new boost::thread(boost::bind(&ManipulationPipeline::processingThread, this, i));
}

void ManipulationPipeline::signalStop(void)
{
  stop_processing_ = true;
  queue_access_cond_.notify_all();  
}

void ManipulationPipeline::stop(void)
{ 
  signalStop(); 
  for (std::size_t i = 0; i < processing_threads_.size() ; ++i)
    if (processing_threads_[i])
    {
      processing_threads_[i]->join();
      delete processing_threads_[i];
      processing_threads_[i] = NULL;
    }
}

void ManipulationPipeline::processingThread(unsigned int index)
{
  ROS_DEBUG_STREAM("Start thread " << index << " for '" << name_ << "'");
  
  while (!stop_processing_)
  {
    boost::unique_lock<boost::mutex> ulock(queue_access_lock_);
    while (queue_.empty() && !stop_processing_)
      queue_access_cond_.wait(ulock); 
    while (!stop_processing_ && !queue_.empty())
    {
      ManipulationPlanPtr g = queue_.front();
      queue_.pop_front();

      queue_access_lock_.unlock();
      try
      {
        g->error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
        for (std::size_t i = 0 ; !stop_processing_ && i < stages_.size() ; ++i)
        {  
          g->processing_stage_ = i;
          if (!stages_[i]->evaluate(g))
          {
            boost::mutex::scoped_lock slock(result_lock_);
            failed_.push_back(g);
            ROS_INFO_STREAM("Manipulation plan failed at stage '" << stages_[i]->getName() << "' on thread " << index);
            break;
          }
        }
        if (g->error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
          {
            boost::mutex::scoped_lock slock(result_lock_);
            success_.push_back(g);
          }
          signalStop();
          ROS_INFO_STREAM("Found successful manipulation plan!");
          if (solution_callback_)
            solution_callback_();
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
      queue_access_lock_.lock();
    }
  }
}

void ManipulationPipeline::push(const ManipulationPlanPtr &plan)
{
  boost::mutex::scoped_lock slock(queue_access_lock_);
  queue_.push_back(plan);
  ROS_INFO_STREAM("Added plan for pipeline '" << name_ << "'. Queue is now of size " << queue_.size());
  queue_access_cond_.notify_all();
}

}
