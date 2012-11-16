//Software License Agreement (BSD License)

//Copyright (c) 2011, Willow Garage, Inc.
//All rights reserved.

//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions
//are met:

// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of Willow Garage, Inc. nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.

//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//POSSIBILITY OF SUCH DAMAGE.

#include <kinematics_constraint_aware/inverse_kinematics_sanity_checker.h>
#include <Eigen/Geometry>
#include <planning_models/kinematic_state.h>
#include <float.h>
#include <planning_models/transforms.h>
#include <ros/console.h>

static const double JOINT_STATE_EPSILON = 1e-2;

InverseKinematicsSanityChecker::InverseKinematicsSanityChecker(const std::map<std::string, kinematics::KinematicsBasePtr>& solver_map,
                                                               const planning_scene::PlanningSceneConstPtr& scene):
  solver_map_(solver_map),
  planning_scene_(scene)
{
} 


void InverseKinematicsSanityChecker::runTest(const std::string& group,
                                             std::vector<std::pair<std::vector<double>, std::vector<double> > >& wrong_solutions,
                                             unsigned int samples,
                                             bool normalize) const 
{
  const planning_models::KinematicModel::JointModelGroup* joint_model_group = planning_scene_->getKinematicModel()->getJointModelGroup(group);
  if(!joint_model_group) {
    ROS_ERROR_STREAM("Group " << group << "not found");
    return;
  }

  std::map<std::pair<std::string, std::string>, unsigned int> collision_data;

  //Eigen::Translation3d max_trans_diff;
  //Eigen::Quaternion3d max_rot_diff = Eigen::Quaterniond::Idenity();
  const kinematics::KinematicsBasePtr& solver = solver_map_.at(group);
  planning_models::KinematicState state(planning_scene_->getKinematicModel());
  state.setToDefaultValues();
  planning_models::KinematicState::JointStateGroup* jsg = state.getJointStateGroup(group);
  std::vector<moveit_msgs::JointLimits> limits = joint_model_group->getVariableLimits();
  unsigned int num_success = 0;
  unsigned int num_wrong_solution = 0;
  double xmax = -DBL_MAX;
  double ymax = -DBL_MAX;
  double zmax = -DBL_MAX;
  double cartesian_x_min = DBL_MAX;
  double cartesian_y_min = DBL_MAX;
  double cartesian_z_min = DBL_MAX;
  double cartesian_x_max = -DBL_MAX;
  double cartesian_y_max = -DBL_MAX;
  double cartesian_z_max = -DBL_MAX;
  double tot_x_diff = 0.0;
  double tot_y_diff = 0.0;
  double tot_z_diff = 0.0;
  for(unsigned int i = 0; i < samples; i++) {
    std::vector<double> joint_sample = sampleJointValues(limits);
    if(normalize) {
      for(unsigned int j = 0; j < joint_sample.size(); j++) {
        while(joint_sample[j] > M_PI) 
          joint_sample[j] -= 2*M_PI;
        while(joint_sample[j] < -M_PI) 
          joint_sample[j] += 2*M_PI;
      }
    }
    jsg->setStateValues(joint_sample);
    Eigen::Affine3d base_pose_in_world = state.getLinkState(solver->getBaseFrame())->getGlobalLinkTransform();
    Eigen::Affine3d tip_pose_in_world = state.getLinkState(solver->getTipFrame())->getGlobalLinkTransform();
    Eigen::Affine3d tip_pose_in_base = base_pose_in_world.inverse()*tip_pose_in_world;
    if(tip_pose_in_base.translation().x() > cartesian_x_max) {
      cartesian_x_max = tip_pose_in_base.translation().x();
    }
    if(tip_pose_in_base.translation().y() > cartesian_y_max) {
      cartesian_y_max = tip_pose_in_base.translation().y();
    }
    if(tip_pose_in_base.translation().z() > cartesian_z_max) {
      cartesian_z_max = tip_pose_in_base.translation().z();
    }
    if(tip_pose_in_base.translation().x() < cartesian_x_min) {
      cartesian_x_min = tip_pose_in_base.translation().x();
    }
    if(tip_pose_in_base.translation().y() < cartesian_y_min) {
      cartesian_y_min = tip_pose_in_base.translation().y();
    }
    if(tip_pose_in_base.translation().z() < cartesian_z_min) {
      cartesian_z_min = tip_pose_in_base.translation().z();
    }
    geometry_msgs::Pose tip_pose_in_base_msg;
    planning_models::msgFromPose(tip_pose_in_base, tip_pose_in_base_msg);
    std::vector<double> solution;
    moveit_msgs::MoveItErrorCodes err;
    if(!solver->getPositionIK(tip_pose_in_base_msg,
                              joint_sample,
                              solution,
                              err)) {
      continue;
    }
    double diff = 0.0;
    for(unsigned int j = 0; j < solution.size(); j++) {
      diff += fabs(joint_sample[j]-solution[j]);
    }
    if(diff > JOINT_STATE_EPSILON) {
      //ROS_INFO_STREAM("Diff " << diff);
      num_wrong_solution++;
      wrong_solutions.push_back(std::pair<std::vector<double>, std::vector<double> >(joint_sample, solution));
    } else {
      num_success++;
    }
    jsg->setStateValues(solution);
 
    collision_detection::CollisionRequest req;
    req.contacts = true;
    req.max_contacts = 10000;
    req.max_contacts_per_pair = 1;

    collision_detection::CollisionResult res;
    planning_scene_->checkSelfCollision(req, res, state);

    if(res.collision) {
      for(collision_detection::CollisionResult::ContactMap::iterator it = res.contacts.begin();
          it != res.contacts.end();
          it++) {
        collision_data[it->first]++;
      }
    }
    
    Eigen::Affine3d sol_tip_pose_in_world = state.getLinkState(solver->getTipFrame())->getGlobalLinkTransform();
    double xdiff = fabs(tip_pose_in_world.translation().x() - sol_tip_pose_in_world.translation().x());
    double ydiff = fabs(tip_pose_in_world.translation().y() - sol_tip_pose_in_world.translation().y());
    double zdiff = fabs(tip_pose_in_world.translation().z() - sol_tip_pose_in_world.translation().z());
    tot_x_diff += xdiff;
    tot_y_diff += ydiff;
    tot_z_diff += zdiff;
    
    if(xdiff > xmax) {
      xmax = xdiff;
    }
    if(ydiff > ymax) {
      ymax = ydiff;
    }
    if(zdiff > zmax) {
      zmax = zdiff;
    }
  }
  for(std::map<std::pair<std::string, std::string>, unsigned int>::iterator it = collision_data.begin();
      it != collision_data.end();
      it++) {
    ROS_INFO_STREAM("Pair " << it->first.first << "," << it->first.second 
                    << " in contact percentage " << (it->second*1.0)/(samples*1.0));
  }

  ROS_INFO_STREAM("Success " << (num_success*1.0)/(samples*1.0) << " diff max " << xmax << " " << ymax << " " << zmax);
  ROS_INFO_STREAM("Av diff " << tot_x_diff/(samples*1.0) 
                  << " " << tot_y_diff/(samples*1.0)
                  << " " << tot_z_diff/(samples*1.0));
  ROS_INFO_STREAM("Wrong solution " << (num_wrong_solution*1.0)/(samples*1.0));
  ROS_INFO_STREAM("Cartesian limits in base frame " 
                  << cartesian_x_min << " "
                  << cartesian_y_min << " "
                  << cartesian_z_min << " "
                  << cartesian_x_max << " "
                  << cartesian_y_max << " "
                  << cartesian_z_max);
}

std::vector<double> 
InverseKinematicsSanityChecker::sampleJointValues(const std::vector<moveit_msgs::JointLimits>& limits) const
{
  std::vector<double> ret;
  for(unsigned int i = 0; i < limits.size(); i++) {
    ret.push_back(rng_.uniformReal(limits[i].min_position, limits[i].max_position));
    ROS_DEBUG_STREAM("Pushing back " << limits[i].joint_name << " limits " << limits[i].min_position << " "
                     << limits[i].max_position << " val " << ret.back());
  }
  return ret;
}
