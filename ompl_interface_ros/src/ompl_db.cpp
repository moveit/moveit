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

#include "ompl_interface_ros/ompl_interface_ros.h"
#include "planning_scene_monitor/planning_scene_monitor.h"
#include <moveit_msgs/DisplayTrajectory.h>
#include <planning_models/conversions.h>

static const std::string ROBOT_DESCRIPTION="robot_description";


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ompl_planning");
  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh("~");
  planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION);
  ompl_interface_ros::OMPLInterfaceROS ompl_interface(psm.getPlanningScene()->getKinematicModel());

  /*
  ros::Publisher pub_state = nh.advertise<moveit_msgs::DisplayTrajectory>("/display_motion_plan", 20);
  
  sleep(1);
  const ompl_interface::ConstraintApproximation &ca = ompl_interface.getConstraintApproximations()->at(0);
  const ompl::base::StateStorageWithMetadata< std::vector<std::size_t> > *sswm =
    static_cast<const ompl::base::StateStorageWithMetadata< std::vector<std::size_t> > *>(ca.state_storage_.get());  
  for (unsigned int i = 0 ; i < 100 ; ++i)
  {
    planning_models::KinematicState ks = ca.getState(ompl_interface.getPlanningConfiguration(ca.group_), i);
    moveit_msgs::DisplayTrajectory d;
    d.model_id = psm.getPlanningScene()->getKinematicModel()->getName();
    planning_models::kinematicStateToRobotState(ks, d.robot_state);

    const ompl::base::State *a = ca.state_storage_->getStates()[i];
    if (sswm->getMetadata(i).empty())
      continue;
    const ompl::base::State *b = ca.state_storage_->getStates()[sswm->getMetadata(i).front()];
  
  //    const ompl::base::State *b = ca.state_storage_->getStates()[i + 100];
    ompl::geometric::PathGeometric pg(ompl_interface.getPlanningConfiguration("right_arm")->getOMPLSimpleSetup().getSpaceInformation(), a, b);
    pg.interpolate(10);
    ompl_interface.getPlanningConfiguration("right_arm")->convertPath(pg, d.trajectory);
    

    pub_state.publish(d);
    ros::Duration(1.0).sleep();
  }
  */

  sleep(1);

  moveit_msgs::Constraints constr1;
  constr1.orientation_constraints.resize(1);
  moveit_msgs::OrientationConstraint &ocm1 = constr1.orientation_constraints[0];
  ocm1.link_name = "r_wrist_roll_link";
  ocm1.orientation.header.frame_id = psm.getPlanningScene()->getPlanningFrame();
  ocm1.orientation.quaternion.x = 0.0;
  ocm1.orientation.quaternion.y = 0.0;
  ocm1.orientation.quaternion.z = 0.0;
  ocm1.orientation.quaternion.w = 1.0;
  ocm1.absolute_x_axis_tolerance = 0.15;
  ocm1.absolute_y_axis_tolerance = 0.15;
  ocm1.absolute_z_axis_tolerance = M_PI;
  ocm1.weight = 1.0;
  
  moveit_msgs::Constraints constr1S = constr1;
  constr1S.orientation_constraints[0].absolute_x_axis_tolerance = 1e-6;
  constr1S.orientation_constraints[0].absolute_y_axis_tolerance = 1e-6;

  ompl_interface.addConstraintApproximation(constr1S, constr1, "right_arm", "PoseModel", 1000);



    /*
  moveit_msgs::Constraints constr2;
  constr2.orientation_constraints.resize(1);
  moveit_msgs::OrientationConstraint &ocm2 = constr2.orientation_constraints[0];
  ocm2.link_name = "l_wrist_roll_link";
  ocm2.orientation.header.frame_id = psm.getPlanningScene()->getPlanningFrame();
  ocm2.orientation.quaternion.x = 0.0;
  ocm2.orientation.quaternion.y = 0.0;
  ocm2.orientation.quaternion.z = 0.0;
  ocm2.orientation.quaternion.w = 1.0;
  ocm2.absolute_x_axis_tolerance = 0.01;
  ocm2.absolute_y_axis_tolerance = 0.01;
  ocm2.absolute_z_axis_tolerance = M_PI;
  ocm2.weight = 1.0;

  ompl_interface.addConstraintApproximation(constr2, "left_arm", 100000);
    */
  ompl_interface.saveConstraintApproximations("/home/isucan/c/");

  return 0;
}

