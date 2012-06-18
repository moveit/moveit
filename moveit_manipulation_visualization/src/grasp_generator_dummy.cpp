/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Author: E. Gil Jones

#include <moveit_manipulation_visualization/grasp_generator_dummy.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_tools/shape_extents.h>

namespace moveit_manipulation_visualization {

bool GraspGeneratorDummy::generateGrasps(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                         const std::string& obj,
                                         const std::string& arm_name,
                                         std::vector<moveit_manipulation_msgs::Grasp>& grasps,
                                         std::string& frame_name)
{
  grasps.clear();
  moveit_msgs::CollisionObject co;
  if(!planning_scene->getCollisionObjectMsg(obj, 
                                           co)) {
    ROS_WARN_STREAM("Don't appear to have object " << obj << " in planning scene");
    return false;
  }

  frame_name = obj;

  //Eigen::Affine3d obj_pose;
  //planning_models::poseFromMsg(co.poses[0], obj_pose);

  double xex, yex, zex;   
  if (!co.primitives.empty())
    shape_tools::getShapeExtents(co.primitives[0], xex, yex, zex);
  else
    if (!co.meshes.empty())
      shape_tools::getShapeExtents(co.meshes[0], xex, yex, zex);
  
  ROS_INFO_STREAM("Extents are " << xex << " " << yex << " " << zex);
  if(obj.find("drive") != std::string::npos) {
    double xtrans = -(xex/2.0+.15);
    Eigen::Affine3d rot(Eigen::Translation3d(xtrans, 0, 0)*(Eigen::AngleAxisd(90.f * (M_PI/180.f), Eigen::Vector3d::UnitX())));
    moveit_manipulation_msgs::Grasp grasp;
    planning_models::msgFromPose(rot,grasp.grasp_pose);
    grasp.desired_approach_distance = .12;
    grasp.min_approach_distance = .12;
    grasps.push_back(grasp);
  } else if(xex > .3 && yex > .3 && zex > .3) {

    double spacing = .1;
    unsigned int znum = floor(zex/spacing);    
    double xtrans = -(xex/2.0+.12+.10);
    for(unsigned int i = 0; i < znum; i++) { 
      double ztrans = -(zex/2.0)+((i*1.0)*spacing);
      ROS_INFO_STREAM("Xtrans " << xtrans << " z trans " << ztrans);
      Eigen::Affine3d rot(Eigen::Translation3d(xtrans, 0, ztrans)*(Eigen::AngleAxisd(90.f * (M_PI/180.f), Eigen::Vector3d::UnitX())));
      moveit_manipulation_msgs::Grasp grasp;
      planning_models::msgFromPose(rot,grasp.grasp_pose);
      grasp.desired_approach_distance = .12;
      grasp.min_approach_distance = .12;
      grasps.push_back(grasp);
    }
  } else {

    Eigen::Affine3d ident(Eigen::Affine3d::Identity());
    grasps.resize(4);

    //FRONT
    planning_models::msgFromPose(ident, grasps[0].grasp_pose);
    grasps[0].grasp_pose.position.x -= (xex/2.0+.15); 
    grasps[0].desired_approach_distance = .12;
    grasps[0].min_approach_distance = .12;
    
    {
      //TOP
      Eigen::Affine3d rot(Eigen::AngleAxisd(90.f * (M_PI/180.f), Eigen::Vector3d::UnitY()));
      Eigen::Affine3d np = ident*rot;
      planning_models::msgFromPose(np, grasps[1].grasp_pose);
      grasps[1].grasp_pose.position.z += (zex/2.0+.15); 
      grasps[1].desired_approach_distance = .12;
      grasps[1].min_approach_distance = .12;
    }
    {
      //LEFT
      Eigen::Affine3d rot(Eigen::AngleAxisd(90.f * (M_PI/180.f), Eigen::Vector3d::UnitZ()));
      Eigen::Affine3d np = ident*rot;
      planning_models::msgFromPose(np, grasps[2].grasp_pose);
      grasps[2].grasp_pose.position.y -= (yex/2.0+.15); 
      grasps[2].desired_approach_distance = .12;
      grasps[2].min_approach_distance = .12;
    }
    {
      //RIGHT
      Eigen::Affine3d rot(Eigen::AngleAxisd(-90.f * (M_PI/180.f), Eigen::Vector3d::UnitZ()));
      Eigen::Affine3d np = ident*rot;
      planning_models::msgFromPose(np, grasps[3].grasp_pose);
      grasps[3].grasp_pose.position.y += (yex/2.0+.15); 
      grasps[3].desired_approach_distance = .12;
      grasps[3].min_approach_distance = .12;
    }
  }

  for(unsigned int i = 0; i < grasps.size(); i++) {
    if(arm_name == "right_arm") {
      grasps[i].pre_grasp_posture.name.push_back("r_gripper_r_finger_joint");
      grasps[i].pre_grasp_posture.name.push_back("r_gripper_l_finger_joint");
      grasps[i].pre_grasp_posture.name.push_back("r_gripper_r_finger_tip_joint");
      grasps[i].pre_grasp_posture.name.push_back("r_gripper_l_finger_tip_joint");
    } else {
      grasps[i].pre_grasp_posture.name.push_back("l_gripper_r_finger_joint");
      grasps[i].pre_grasp_posture.name.push_back("l_gripper_l_finger_joint");
      grasps[i].pre_grasp_posture.name.push_back("l_gripper_r_finger_tip_joint");
      grasps[i].pre_grasp_posture.name.push_back("l_gripper_l_finger_tip_joint");
    }
    grasps[i].pre_grasp_posture.position.push_back(.4);
    grasps[i].pre_grasp_posture.position.push_back(.4);
    grasps[i].pre_grasp_posture.position.push_back(.4);
    grasps[i].pre_grasp_posture.position.push_back(.4);
    grasps[i].grasp_posture.name = grasps[i].pre_grasp_posture.name;
    grasps[i].grasp_posture.position.push_back(.15);
    grasps[i].grasp_posture.position.push_back(.15);
    grasps[i].grasp_posture.position.push_back(.15);
    grasps[i].grasp_posture.position.push_back(.15);
  }
  return true;
}

}

