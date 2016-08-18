/*
 * chomp_planning_context.cpp
 *
 *  Created on: 27-Jul-2016
 *      Author: ace
 */

#include "chomp_interface/chomp_planning_context.h"

namespace chomp_interface {

ChompPlanningContext::ChompPlanningContext(const std::string &name, const std::string &group, const robot_model::RobotModelConstPtr& model):
  planning_interface::PlanningContext(name, group),
  kmodel_ (model) {
  chomp_interface_ = boost::shared_ptr <CHOMPInterface> (new CHOMPInterface(model));

//  tf_ = boost::shared_ptr<tf::TransformListener>(new tf::TransformListener());
//  planning_scene_monitor::PlanningSceneMonitorPtr psm(new planning_scene_monitor::PlanningSceneMonitor("robot_description", tf_));
//  ROS_INFO_STREAM("PlanningContext " + this->name_ + " is congifured for group: " + this->group_); //, this->group_);
//
//  planning_scene_monitor::LockedPlanningSceneRW psm_rw(psm);

  // Get the planning scene and set it for the context.
//  this->setPlanningScene(psm_rw.operator ->());

  boost::shared_ptr<collision_detection::CollisionDetectorAllocator> hybrid_cd (collision_detection::CollisionDetectorAllocatorHybrid::create());

  if (!this->getPlanningScene())
  {
    ROS_INFO_STREAM("Configuring New Planning Scene.");
    planning_scene::PlanningScenePtr planning_scene_ptr(new planning_scene::PlanningScene(model));
    //planning_scene_ptr->addCollisionDetector(hybrid_cd);
    //planning_scene_ptr->setActiveCollisionDetector(hybrid_cd->getName());
    planning_scene_ptr->setActiveCollisionDetector(hybrid_cd, true);
    setPlanningScene(planning_scene_ptr);
  }
}

ChompPlanningContext::~ChompPlanningContext() {
  // TODO Auto-generated destructor stub
}

bool ChompPlanningContext::solve(planning_interface::MotionPlanDetailedResponse &res)
{

  moveit_msgs::MotionPlanDetailedResponse res2;
  if (chomp_interface_->solve(planning_scene_, request_,
      chomp_interface_->getParams(),res2))
  {
    res.trajectory_.resize(1);
    //res.trajectory_[0].trajectory_start = res2.motion_plan_response.trajectory_start;
    res.trajectory_[0] = robot_trajectory::RobotTrajectoryPtr(new robot_trajectory::RobotTrajectory(kmodel_, getGroupName()));

    moveit::core::RobotState start_state(kmodel_);
    robot_state::robotStateMsgToRobotState(res2.trajectory_start, start_state);
    res.trajectory_[0]->setRobotTrajectoryMsg(start_state, res2.trajectory[0]);

    res.description_.push_back("plan");
    res.processing_time_ = res2.processing_time;
    return true;
  }
  else
    return false;
}

bool ChompPlanningContext::solve(planning_interface::MotionPlanResponse &res) {
  planning_interface::MotionPlanDetailedResponse res_detailed;
  bool result = solve(res_detailed);

  res.error_code_ = res_detailed.error_code_;
  res.trajectory_ = res_detailed.trajectory_[0];
  res.planning_time_ = res_detailed.processing_time_[0];

  return result;

}

bool ChompPlanningContext::terminate()
{
  //TODO - make interruptible
  return true;
}

void ChompPlanningContext::clear() {

}

} /* namespace chomp_interface */
