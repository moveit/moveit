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

#include <moveit/pick_place/pick_place.h>
#include <moveit/pick_place/reachable_valid_grasp_filter.h>
#include <moveit/pick_place/output_grasp_filter.h>
#include <ros/console.h>

namespace pick_place
{

PickPlace::PickPlace(void)
{
  constraint_sampler_manager_loader_.reset(new constraint_sampler_manager_loader::ConstraintSamplerManagerLoader());
}

class PickPlan
{
public:
  
  PickPlan(const PickPlace *pick_place) :
    pick_place_(pick_place),
    done_(false)
  {
  }
  
  Grasp plan(const planning_scene::PlanningScenePtr &planning_scene, const moveit_msgs::PickupGoal &goal, double timeout)
  {   
    ros::WallTime endtime = ros::WallTime::now() + ros::WallDuration(timeout);
    
    done_ = false;
    ReachableAndValidGraspFilter::Options opt("", "");
    root_.reset(new ReachableAndValidGraspFilter(opt, planning_scene, pick_place_->getConstraintsSamplerManager(), 4));
    GraspFilterPtr f1 = root_->follow(GraspFilterPtr(new OutputGraspFilter(boost::bind(&PickPlan::foundSolution, this, _1))));

    root_->start();

    // feed the available grasps to the filter we set up


    // .......
    

    // wait till we're done
    boost::unique_lock<boost::mutex> lock(mut_);
    while (!done_ && endtime > ros::WallTime::now())
      cond_.timed_wait(lock, (endtime - ros::WallTime::now()).toBoost());

    // make sure we stopped
    root_->stop();
    
    // read the output of the last filter
    const std::vector<Grasp> &out = static_cast<OutputGraspFilter*>(f1.get())->getOutput();
    
    return out.empty() ? Grasp() : out.front();
  } 

  
private:

  void foundSolution(const Grasp &grasp)
  {
    root_->stop();
    boost::mutex::scoped_lock slock(mut_);
    done_ = true;
    cond_.notify_all();
  }
  
  const PickPlace *pick_place_;  
  bool done_;
  boost::condition_variable cond_;
  boost::mutex mut_;
  GraspFilterPtr root_;
};

void PickPlace::planPick(const planning_scene::PlanningScenePtr &planning_scene, const moveit_msgs::PickupGoal &goal, double timeout) const
{    
  ros::WallTime start = ros::WallTime::now();
  PickPlan p(this);
  const Grasp &g = p.plan(planning_scene, goal, timeout);
  double dt = (ros::WallTime::now() - start).toSec();
  ROS_INFO("Pick plan took %lf seconds", dt);
}

}
