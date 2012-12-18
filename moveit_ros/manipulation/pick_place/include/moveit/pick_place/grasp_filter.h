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

/* Author: Ioan Sucan */

#ifndef MOVEIT_PICK_PLACE_GRASP_FILTER_
#define MOVEIT_PICK_PLACE_GRASP_FILTER_

#include <moveit/planning_scene/planning_scene.h>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <vector>
#include <deque>

namespace pick_place
{

struct Grasp
{
};

class GraspFilter;
typedef boost::shared_ptr<GraspFilter> GraspFilterPtr;
typedef boost::shared_ptr<const GraspFilter> GraspFilterConstPtr;

class GraspFilter
{
public:
  typedef boost::function<void(void)> SuccessCallback;
  
  GraspFilter(const planning_scene::PlanningSceneConstPtr &scene, unsigned int nthreads = 4);
  virtual ~GraspFilter(void);
    
  const GraspFilterPtr& follow(const GraspFilterPtr &next)
  {
    next_ = next;
    return next;
  }
  
  void setSuccessCallback(const SuccessCallback &callback)
  {
    callback_ = callback;
  }
  
  void start(void);
  
  void stop(void);
  
  void push(const Grasp &grasp);
  
  virtual bool evaluate(unsigned int thread_id, const Grasp &grasp) const = 0;
  
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
  
  planning_scene::PlanningSceneConstPtr planning_scene_;
  unsigned int nthreads_;
  std::vector< std::deque<Grasp> > processing_queues_;
  std::vector< ProcessingThread* > processing_threads_;
  SuccessCallback callback_;
  bool stop_processing_;
  GraspFilterPtr next_;
};

}

#endif

