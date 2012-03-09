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

  frame_name = planning_scene->getPlanningFrame();

  grasps.resize(1);
  grasps[0].grasp_pose = co.poses[0];
  grasps[0].grasp_pose.position.x -= ((co.shapes[0].dimensions[0]/2.0)+.15); 
  grasps[0].desired_approach_distance = .12;
  grasps[0].min_approach_distance = .12;
  if(arm_name == "right_arm") {
    grasps[0].pre_grasp_posture.name.push_back("r_gripper_r_finger_joint");
    grasps[0].pre_grasp_posture.name.push_back("r_gripper_l_finger_joint");
    grasps[0].pre_grasp_posture.name.push_back("r_gripper_r_finger_tip_joint");
    grasps[0].pre_grasp_posture.name.push_back("r_gripper_l_finger_tip_joint");
  } else {
    grasps[0].pre_grasp_posture.name.push_back("l_gripper_r_finger_joint");
    grasps[0].pre_grasp_posture.name.push_back("l_gripper_l_finger_joint");
    grasps[0].pre_grasp_posture.name.push_back("l_gripper_r_finger_tip_joint");
    grasps[0].pre_grasp_posture.name.push_back("l_gripper_l_finger_tip_joint");
  }
  grasps[0].pre_grasp_posture.position.push_back(.35);
  grasps[0].pre_grasp_posture.position.push_back(.35);
  grasps[0].pre_grasp_posture.position.push_back(.35);
  grasps[0].pre_grasp_posture.position.push_back(.35);
  grasps[0].grasp_posture.name = grasps[0].pre_grasp_posture.name;
  grasps[0].grasp_posture.position.push_back(.25);
  grasps[0].grasp_posture.position.push_back(.25);
  grasps[0].grasp_posture.position.push_back(.25);
  grasps[0].grasp_posture.position.push_back(.25);
  return true;
}

}

