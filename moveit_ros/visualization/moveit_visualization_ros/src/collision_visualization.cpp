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

// Author: Adam E Leeper, E. Gil Jones (adapted from joint_trajectory_visualization)

#include <moveit_visualization_ros/collision_visualization.h>
#include <moveit_visualization_ros/interactive_marker_helper_functions.h>
#include <planning_models/transforms.h>

static const ros::WallDuration sleep_time = ros::WallDuration(0.01);

namespace moveit_visualization_ros
{

CollisionVisualization::CollisionVisualization(ros::Publisher& marker_publisher)
  : marker_publisher_(marker_publisher)
{
  
}; 


void CollisionVisualization::drawCollisions(const collision_detection::CollisionResult& data,
                                            const std::string &frame)
{
  ROS_INFO("Contact count: %zd", data.contact_count);
  if(data.collision)
  {
    visualization_msgs::MarkerArray array;
    ros::Time now = ros::Time::now();
    int arrow_count = 0;
    for( collision_detection::CollisionResult::ContactMap::const_iterator it = data.contacts.begin(); it != data.contacts.end(); ++it)
    {
      std::string contact1 = it->first.first;
      std::string contact2 = it->first.second;
      const std::vector<collision_detection::Contact>& vec = it->second;

      for(size_t contact_index = 0; contact_index < vec.size(); contact_index++)
      {
        Eigen::Vector3d pos =     vec[contact_index].pos;
        Eigen::Vector3d normal =  vec[contact_index].normal;
        double depth = vec[contact_index].depth;
        ROS_INFO("Contact between [%s] and [%s] point: %.2f %.2f %.2f normal: %.2f %.2f %.2f depth: %.3f",
                 contact1.c_str(), contact2.c_str(),
                 pos(0), pos(1), pos(2),
                 normal(0), normal(1), normal(2),
                 depth);
        geometry_msgs::Point p1, p2;
        planning_models::msgFromPoint(pos, p1);
        planning_models::msgFromPoint(pos+0.2*normal, p2);
        visualization_msgs::Marker marker = makeArrow(p1, p2);
        marker.header.frame_id= frame;
        marker.header.stamp = now;
        marker.id = arrow_count++;
        marker.ns = "collision_markers";
        array.markers.push_back(marker);
      }
    }
    marker_publisher_.publish(array);
  }
}

}
