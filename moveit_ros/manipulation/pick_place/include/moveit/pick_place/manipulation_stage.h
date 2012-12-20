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

#ifndef MOVEIT_PICK_PLACE_MANIPULATION_STAGE_
#define MOVEIT_PICK_PLACE_MANIPULATION_STAGE_

#include <moveit/pick_place/manipulation_plan.h>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <vector>
#include <deque>

namespace pick_place
{

class ManipulationStage;
typedef boost::shared_ptr<ManipulationStage> ManipulationStagePtr;
typedef boost::shared_ptr<const ManipulationStage> ManipulationStageConstPtr;

class ManipulationStage
{
public:
  
  ManipulationStage(unsigned int nthreads);
  virtual ~ManipulationStage(void);
    
  const ManipulationStagePtr& follow(const ManipulationStagePtr &next)
  {
    next_ = next;
    return next;
  }
  
  void start(void);
  
  void stop(void);
  
  virtual void push(const ManipulationPlanPtr &grasp);
  
  virtual bool evaluate(unsigned int thread_id, const ManipulationPlanPtr &grasp) const = 0;
  
  virtual bool done(void) const
  {
    return next_ ? next_->done() : false;
  }
  
protected:
  
  void processingThread(unsigned int index);
  
  struct ProcessingThread
  {
    ProcessingThread(void)
    {
    }
    
    ~ProcessingThread(void)
    {
      if (thread_)
        thread_->join();
    }
    
    boost::mutex mutex_;
    boost::condition_variable cond_;
    boost::scoped_ptr<boost::thread> thread_;
  };
  
  unsigned int nthreads_;
  std::vector< std::deque<ManipulationPlanPtr> > processing_queues_;
  std::vector< ProcessingThread* > processing_threads_;
  bool stop_processing_;
  ManipulationStagePtr next_;
};

}

#endif

