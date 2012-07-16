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

/** \author E. Gil Jones */

#ifndef _COLLISION_ROBOT_HYBRID_ROS_H_
#define _COLLISION_ROBOT_HYBRID_ROS_H_

#include <ros/ros.h>
#include <collision_distance_field_ros/collision_distance_field_ros_helpers.h>
#include <collision_distance_field/hybrid_collision_robot.h>

namespace collision_detection {

class CollisionRobotHybridROS : public CollisionRobotHybrid
{
public:
  
  CollisionRobotHybridROS(const planning_models::KinematicModelConstPtr& kmodel, 
                          double size_x = 3.0, 
                          double size_y = 3.0,
                          double size_z = 4.0,
                          bool use_signed_distance_field = false,
                          double resolution = .02,
                          double collision_tolerance = 0.0,
                          double max_propogation_distance = .25,
                          double padding = 0.0, 
                          double scale = 1.0) :
    CollisionRobotHybrid(kmodel)
  {
    ros::NodeHandle nh;
    std::map<std::string, std::vector<CollisionSphere> > coll_spheres;
    collision_detection::loadLinkBodySphereDecompositions(nh,
                                                          getKinematicModel(),
                                                          coll_spheres);
    initializeRobotDistanceField(coll_spheres, size_x, size_y, size_z, use_signed_distance_field, resolution, collision_tolerance, max_propogation_distance);
  }
};

}

#endif
