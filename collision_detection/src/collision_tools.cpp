/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

#include "collision_detection/collision_tools.h"

void collision_detection::getCostMarkers(visualization_msgs::MarkerArray& arr, const std::string& frame_id, std::set<CostSource> &cost_sources)
{
  std_msgs::ColorRGBA color;
  color.r = 1.0f; color.g = 0.5f; color.b = 0.0f; color.a = 0.4f;
  getCostMarkers(arr, frame_id, cost_sources, color, ros::Duration(60.0));
}

void collision_detection::getCollisionMarkersFromContacts(visualization_msgs::MarkerArray& arr,
                                                          const std::string& frame_id,
                                                          const CollisionResult::ContactMap &con)
{
  std_msgs::ColorRGBA color;
  color.r = 1.0f; color.g = 0.0f; color.b = 0.0f; color.a = 0.8f;
  getCollisionMarkersFromContacts(arr, frame_id, con, color, ros::Duration(60.0));
}

void collision_detection::getCostMarkers(visualization_msgs::MarkerArray& arr,
                                         const std::string& frame_id,
                                         std::set<CostSource> &cost_sources,
                                         const std_msgs::ColorRGBA& color,
                                         const ros::Duration& lifetime)
{
  int id = 0;
  for (std::set<CostSource>::iterator it = cost_sources.begin() ; it != cost_sources.end() ; ++it)
  {
    visualization_msgs::Marker mk;
    mk.header.stamp = ros::Time::now();
    mk.header.frame_id = frame_id;
    mk.ns = "cost_source";
    mk.id = id++;
    mk.type = visualization_msgs::Marker::CUBE;
    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.position.x = (it->aabb_max[0] + it->aabb_min[0]) / 2.0;
    mk.pose.position.y = (it->aabb_max[1] + it->aabb_min[1]) / 2.0;
    mk.pose.position.z = (it->aabb_max[2] + it->aabb_min[2]) / 2.0;
    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;    
    mk.pose.orientation.w = 1.0;
    mk.scale.x = it->aabb_max[0] - it->aabb_min[0];
    mk.scale.y = it->aabb_max[1] - it->aabb_min[1];
    mk.scale.z = it->aabb_max[2] - it->aabb_min[2];
    mk.color = color;
    if (mk.color.a == 0.0)
      mk.color.a = 1.0;
    mk.lifetime = lifetime;
    arr.markers.push_back(mk);
  }
}

void collision_detection::getCollisionMarkersFromContacts(visualization_msgs::MarkerArray& arr,
                                                          const std::string& frame_id,
                                                          const CollisionResult::ContactMap& con,
                                                          const std_msgs::ColorRGBA& color,
                                                          const ros::Duration& lifetime,
                                                          double radius)
  
{
  std::map<std::string, unsigned> ns_counts;
  for(CollisionResult::ContactMap::const_iterator it = con.begin(); it != con.end(); ++it)
  {
    for(unsigned int i = 0; i < it->second.size(); ++i)
    {
      std::string ns_name = it->second[i].body_name_1 + "=" + it->second[i].body_name_2;
      if (ns_counts.find(ns_name) == ns_counts.end())
        ns_counts[ns_name] = 0;
      else
        ns_counts[ns_name]++;
      visualization_msgs::Marker mk;
      mk.header.stamp = ros::Time::now();
      mk.header.frame_id = frame_id;
      mk.ns = ns_name;
      mk.id = ns_counts[ns_name];
      mk.type = visualization_msgs::Marker::SPHERE;
      mk.action = visualization_msgs::Marker::ADD;
      mk.pose.position.x = it->second[i].pos.x();
      mk.pose.position.y = it->second[i].pos.y();
      mk.pose.position.z = it->second[i].pos.z();
      mk.pose.orientation.x = 0.0;
      mk.pose.orientation.y = 0.0;
      mk.pose.orientation.z = 0.0;    
      mk.pose.orientation.w = 1.0;
      mk.scale.x = mk.scale.y = mk.scale.z = radius * 2.0;
      mk.color = color;
      if(mk.color.a == 0.0)
        mk.color.a = 1.0;
      mk.lifetime = lifetime;
      arr.markers.push_back(mk);
    }
  }
}
