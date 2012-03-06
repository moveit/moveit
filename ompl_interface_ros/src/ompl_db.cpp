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
  ros::Publisher pub_markers = nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 5);
  ros::Publisher pub_state = nh.advertise<moveit_msgs::DisplayTrajectory>("/display_motion_plan", 20);

  

  sleep(1);
  


  
  moveit_msgs::Constraints c;


  moveit_msgs::PositionConstraint pcm2;
  pcm2.link_name = "r_wrist_roll_link";
  pcm2.target_point_offset.x = 0.7;
  pcm2.target_point_offset.y = 0;
  pcm2.target_point_offset.z = 0;
  pcm2.constraint_region_shape.type = moveit_msgs::Shape::BOX;
  pcm2.constraint_region_shape.dimensions.push_back(0.01);
  pcm2.constraint_region_shape.dimensions.push_back(0.01);
  pcm2.constraint_region_shape.dimensions.push_back(0.01);
  
  pcm2.constraint_region_pose.header.frame_id = "l_wrist_roll_link";
  pcm2.constraint_region_pose.pose.position.x = 0.0;
  pcm2.constraint_region_pose.pose.position.y = 0.0;
  pcm2.constraint_region_pose.pose.position.z = 0.0;
  pcm2.constraint_region_pose.pose.orientation.x = 0.0;
  pcm2.constraint_region_pose.pose.orientation.y = 0.0;
  pcm2.constraint_region_pose.pose.orientation.z = 0.0;
  pcm2.constraint_region_pose.pose.orientation.w = 1.0;
  pcm2.weight = 1.0;
  c.position_constraints.push_back(pcm2);


  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "l_wrist_roll_link";
  ocm.orientation.header.frame_id = psm.getPlanningScene()->getPlanningFrame();
  ocm.orientation.quaternion.x = 0.5;
  ocm.orientation.quaternion.y = 0.5;
  ocm.orientation.quaternion.z = 0.5;
  ocm.orientation.quaternion.w = 0.5;
  ocm.absolute_x_axis_tolerance = 0.01;
  ocm.absolute_y_axis_tolerance = M_PI;
  ocm.absolute_z_axis_tolerance = 0.01;
  ocm.weight = 1.0;
  c.orientation_constraints.push_back(ocm);

  ocm.link_name = "r_wrist_roll_link";
  ocm.orientation.header.frame_id = "l_wrist_roll_link";
  ocm.orientation.quaternion.x = 0.0;
  ocm.orientation.quaternion.y = 0.0;
  ocm.orientation.quaternion.z = 1.0;
  ocm.orientation.quaternion.w = 0.0;
  ocm.absolute_x_axis_tolerance = 0.01;
  ocm.absolute_y_axis_tolerance = 0.01;
  ocm.absolute_z_axis_tolerance = 0.01;
  ocm.weight = 1.0;
  c.orientation_constraints.push_back(ocm);
  
  moveit_msgs::Constraints cS = c;

  //  ompl_interface.addConstraintApproximation(cS, c, "arms", "PoseModel", psm.getPlanningScene()->getCurrentState(), 1000);



  /*

  moveit_msgs::Constraints constr1;
  constr1.orientation_constraints.resize(1);
  moveit_msgs::OrientationConstraint &ocm1 = constr1.orientation_constraints[0];
  ocm1.link_name = "r_wrist_roll_link";
  ocm1.orientation.header.frame_id = psm.getPlanningScene()->getPlanningFrame();
  ocm1.orientation.quaternion.x = 0.0;
  ocm1.orientation.quaternion.y = 0.0;
  ocm1.orientation.quaternion.z = 0.0;
  ocm1.orientation.quaternion.w = 1.0;
  ocm1.absolute_x_axis_tolerance = 0.1;
  ocm1.absolute_y_axis_tolerance = 0.1;
  ocm1.absolute_z_axis_tolerance = M_PI;
  ocm1.weight = 1.0;
  
  moveit_msgs::Constraints constr1S = constr1;
  constr1S.orientation_constraints[0].absolute_x_axis_tolerance = 1e-6;
  constr1S.orientation_constraints[0].absolute_y_axis_tolerance = 1e-6;

  ompl_interface.addConstraintApproximation(constr1S, constr1, "right_arm", "PoseModel", 10000);
  */


  

  const ompl_interface::ConstraintApproximation &ca = ompl_interface.getConstraintApproximations()->at(0);
  visualization_msgs::MarkerArray arr;  
  ca.visualizeDistribution("r_wrist_roll_link", 1000, arr);
  pub_markers.publish(arr); 

    

  ompl_interface::ModelBasedPlanningContextPtr pc = ompl_interface.getPlanningContext("arms", "PoseModel");

  kinematic_constraints::KinematicConstraintSet kset(pc->getKinematicModel(),
						     planning_models::TransformsPtr(new planning_models::Transforms(pc->getKinematicModel()->getModelFrame())));
  kset.add(c);


  planning_models::KinematicState ks(pc->getKinematicModel());
  ks.setToDefaultValues();
  sleep(1);
  //  const ompl_interface::ConstraintApproximation &ca = ompl_interface.getConstraintApproximations()->at(0);
  for (unsigned int i = 0 ; i < 5 ; ++i)
  {
    const ompl::base::State *a = ca.state_storage_->getStates()[i];
    pc->getOMPLStateSpace()->copyToKinematicState(ks, a);
    ks.updateLinkTransforms();
    
    double dd;
    if (!kset.decide(ks, dd))
    {
	kset.decide(ks, dd, true);
	std::cout << std::endl;
    }
    
    
    moveit_msgs::DisplayTrajectory d;
    d.model_id = psm.getPlanningScene()->getKinematicModel()->getName();
    planning_models::kinematicStateToRobotState(ks, d.trajectory_start);
    pub_state.publish(d);
    ros::Duration(1.0).sleep();
  }  
  

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
  ocm2.absolute_x_axis_tolerance = 0.1;
  ocm2.absolute_y_axis_tolerance = 0.1;
  ocm2.absolute_z_axis_tolerance = M_PI;
  ocm2.weight = 1.0;
  moveit_msgs::Constraints constr2S = constr2;
  constr2S.orientation_constraints[0].absolute_x_axis_tolerance = 1e-6;
  constr2S.orientation_constraints[0].absolute_y_axis_tolerance = 1e-6;

  ompl_interface.addConstraintApproximation(constr2S, constr2, "left_arm", "PoseModel", 10000);
  */    
  //  ompl_interface.saveConstraintApproximations("/home/isucan/c/");
  ROS_INFO("Done");
  
  ros::waitForShutdown();
  
  return 0;
}

