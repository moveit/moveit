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
#include <moveit/pick_place/reachable_valid_pre_grasp_filter.h>
#include <moveit/pick_place/output_stage.h>
#include <ros/console.h>

namespace pick_place
{

namespace
{
  
class PickPlan
{
public:
  
  PickPlan(const PickPlace *pick_place) :
    pick_place_(pick_place),
    done_(false)
  {
  }
  
  const std::vector<ManipulationPlanPtr>& plan(const planning_scene::PlanningScenePtr &planning_scene, const moveit_msgs::PickupGoal &goal, double timeout)
  {
    static const std::vector<ManipulationPlanPtr> empty_result;
    
    ros::WallTime endtime = ros::WallTime::now() + ros::WallDuration(timeout);
    
    std::string planning_group = goal.group_name;
    std::string end_effector = goal.end_effector;
    if (end_effector.empty() && !planning_group.empty())
    {
      const kinematic_model::JointModelGroup *jmg = planning_scene->getKinematicModel()->getJointModelGroup(planning_group);
      if (!jmg)
        return empty_result;
      const std::vector<std::string> &eefs = jmg->getAttachedEndEffectorNames();
      if (!eefs.empty())
      {
        end_effector = eefs.front();
        if (eefs.size() > 1)
          ROS_WARN_STREAM("Choice of end-effector for group '" << planning_group << "' is ambiguous. Assuming '" << end_effector << "'");
      }
    }
    else
      if (!end_effector.empty() && planning_group.empty())
      {
        const kinematic_model::JointModelGroup *jmg = planning_scene->getKinematicModel()->getEndEffector(end_effector);
        if (!jmg)
          return empty_result;
        planning_group = jmg->getEndEffectorParentGroup().first;
        ROS_INFO_STREAM("Assuming the planning group for end effector '" << end_effector << "' is '" << planning_group << "'");
      }      
    const kinematic_model::JointModelGroup *eef = end_effector.empty() ? NULL : planning_scene->getKinematicModel()->getEndEffector(end_effector);
    if (!eef)
    {
      ROS_ERROR("No end-effector specified for pick action");
      return empty_result;
    }
    
    done_ = false;
    ReachableAndValidGraspFilter::Options opt(planning_group, eef->getEndEffectorParentGroup().second);
    root_.reset(new ReachableAndValidGraspFilter(opt, planning_scene, pick_place_->getConstraintsSamplerManager(), 4));
    ManipulationStagePtr f0 = root_->follow(ManipulationStagePtr(new ReachableAndValidPreGraspFilter(opt, planning_scene, pick_place_->getConstraintsSamplerManager(), 4)));
    ManipulationStagePtr f1 = f0->follow(ManipulationStagePtr(new OutputStage(boost::bind(&PickPlan::foundSolution, this, _1))));

    root_->startAll();

    // feed the available grasps to the filter we set up
    for (std::size_t i = 0 ; i < goal.possible_grasps.size() ; ++i)
    {
      ManipulationPlanPtr p(new ManipulationPlan());
      p->grasp_ = goal.possible_grasps[i];
      root_->push(p);
    }
    
    // wait till we're done
    {
      boost::unique_lock<boost::mutex> lock(mut_);
      while (!done_ && endtime > ros::WallTime::now())
        cond_.timed_wait(lock, (endtime - ros::WallTime::now()).toBoost());
    }
    
    // make sure we stopped
    root_->stopAll();

    // read the output of the last filter
    return static_cast<OutputStage*>(f1.get())->getOutput();
  }
  
private:

  void foundSolution(const ManipulationPlanPtr &plan)
  {
    boost::mutex::scoped_lock slock(mut_);
    done_ = true;
    cond_.notify_all();
  }
  
  const PickPlace *pick_place_;  
  bool done_;
  boost::condition_variable cond_;
  boost::mutex mut_;
  ManipulationStagePtr root_;
};

}

ManipulationPlanPtr PickPlace::planPick(const planning_scene::PlanningScenePtr &planning_scene, const moveit_msgs::PickupGoal &goal) const
{
  ros::WallTime start = ros::WallTime::now();
  PickPlan p(this);
  const std::vector<ManipulationPlanPtr> &g = p.plan(planning_scene, goal, 1.0);
  double dt = (ros::WallTime::now() - start).toSec();
  ROS_INFO("Pick plan took %lf seconds", dt);
  if (g.empty())
  {
    static const ManipulationPlanPtr empty;
    return empty;
  }
  else
    return g.back();
}

}

