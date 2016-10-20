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
 *   * Neither the name of Willow Garage nor the names of its
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

#include <moveit/collision_detection/collision_tools.h>
#include <eigen_conversions/eigen_msg.h>

void collision_detection::getCostMarkers(visualization_msgs::MarkerArray &arr, const std::string &frame_id,
                                         std::set<CostSource> &cost_sources)
{
  std_msgs::ColorRGBA color;
  color.r = 1.0f;
  color.g = 0.5f;
  color.b = 0.0f;
  color.a = 0.4f;
  getCostMarkers(arr, frame_id, cost_sources, color, ros::Duration(60.0));
}

void collision_detection::getCollisionMarkersFromContacts(visualization_msgs::MarkerArray &arr,
                                                          const std::string &frame_id,
                                                          const CollisionResult::ContactMap &con)
{
  std_msgs::ColorRGBA color;
  color.r = 1.0f;
  color.g = 0.0f;
  color.b = 0.0f;
  color.a = 0.8f;
  getCollisionMarkersFromContacts(arr, frame_id, con, color, ros::Duration(60.0));
}

void collision_detection::getCostMarkers(visualization_msgs::MarkerArray &arr, const std::string &frame_id,
                                         std::set<CostSource> &cost_sources, const std_msgs::ColorRGBA &color,
                                         const ros::Duration &lifetime)
{
  int id = 0;
  for (std::set<CostSource>::iterator it = cost_sources.begin(); it != cost_sources.end(); ++it)
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

void collision_detection::getCollisionMarkersFromContacts(visualization_msgs::MarkerArray &arr,
                                                          const std::string &frame_id,
                                                          const CollisionResult::ContactMap &con,
                                                          const std_msgs::ColorRGBA &color,
                                                          const ros::Duration &lifetime, double radius)

{
  std::map<std::string, unsigned> ns_counts;
  for (CollisionResult::ContactMap::const_iterator it = con.begin(); it != con.end(); ++it)
  {
    for (unsigned int i = 0; i < it->second.size(); ++i)
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
      if (mk.color.a == 0.0)
        mk.color.a = 1.0;
      mk.lifetime = lifetime;
      arr.markers.push_back(mk);
    }
  }
}

bool collision_detection::getSensorPositioning(geometry_msgs::Point &point, const std::set<CostSource> &cost_sources)
{
  if (cost_sources.empty())
    return false;
  std::set<CostSource>::const_iterator it = cost_sources.begin();
  for (std::size_t i = 0; i < 4 * cost_sources.size() / 5; ++i)
    ++it;
  point.x = (it->aabb_max[0] + it->aabb_min[0]) / 2.0;
  point.y = (it->aabb_max[1] + it->aabb_min[1]) / 2.0;
  point.z = (it->aabb_max[2] + it->aabb_min[2]) / 2.0;
  return true;
}

double collision_detection::getTotalCost(const std::set<CostSource> &cost_sources)
{
  double cost = 0.0;
  for (std::set<collision_detection::CostSource>::const_iterator it = cost_sources.begin(); it != cost_sources.end();
       ++it)
    cost += it->getVolume() * it->cost;
  return cost;
}

void collision_detection::intersectCostSources(std::set<CostSource> &cost_sources, const std::set<CostSource> &a,
                                               const std::set<CostSource> &b)
{
  cost_sources.clear();
  CostSource tmp;
  for (std::set<CostSource>::const_iterator it = a.begin(); it != a.end(); ++it)
    for (std::set<CostSource>::const_iterator jt = b.begin(); jt != b.end(); ++jt)
    {
      tmp.aabb_min[0] = std::max(it->aabb_min[0], jt->aabb_min[0]);
      tmp.aabb_min[1] = std::max(it->aabb_min[1], jt->aabb_min[1]);
      tmp.aabb_min[2] = std::max(it->aabb_min[2], jt->aabb_min[2]);

      tmp.aabb_max[0] = std::min(it->aabb_max[0], jt->aabb_max[0]);
      tmp.aabb_max[1] = std::min(it->aabb_max[1], jt->aabb_max[1]);
      tmp.aabb_max[2] = std::min(it->aabb_max[2], jt->aabb_max[2]);

      if (tmp.aabb_min[0] >= tmp.aabb_max[0] || tmp.aabb_min[1] >= tmp.aabb_max[1] ||
          tmp.aabb_min[2] >= tmp.aabb_max[2])
        continue;
      tmp.cost = std::max(it->cost, jt->cost);
      cost_sources.insert(tmp);
    }
}

void collision_detection::removeOverlapping(std::set<CostSource> &cost_sources, double overlap_fraction)
{
  double p[3], q[3];
  for (std::set<CostSource>::iterator it = cost_sources.begin(); it != cost_sources.end(); ++it)
  {
    double vol = it->getVolume() * overlap_fraction;
    std::vector<std::set<CostSource>::iterator> remove;
    std::set<CostSource>::iterator it1 = it;
    for (std::set<CostSource>::iterator jt = ++it1; jt != cost_sources.end(); ++jt)
    {
      p[0] = std::max(it->aabb_min[0], jt->aabb_min[0]);
      p[1] = std::max(it->aabb_min[1], jt->aabb_min[1]);
      p[2] = std::max(it->aabb_min[2], jt->aabb_min[2]);

      q[0] = std::min(it->aabb_max[0], jt->aabb_max[0]);
      q[1] = std::min(it->aabb_max[1], jt->aabb_max[1]);
      q[2] = std::min(it->aabb_max[2], jt->aabb_max[2]);

      if (p[0] >= q[0] || p[1] >= q[1] || p[2] >= q[2])
        continue;

      double intersect_volume = (q[0] - p[0]) * (q[1] - p[1]) * (q[2] - p[2]);
      if (intersect_volume >= vol)
        remove.push_back(jt);
    }
    for (std::size_t i = 0; i < remove.size(); ++i)
      cost_sources.erase(remove[i]);
  }
}

void collision_detection::removeCostSources(std::set<CostSource> &cost_sources,
                                            const std::set<CostSource> &cost_sources_to_remove, double overlap_fraction)
{
  // remove all the boxes that overlap with the intersection previously computed in \e rem
  double p[3], q[3];
  for (std::set<CostSource>::const_iterator jt = cost_sources_to_remove.begin(); jt != cost_sources_to_remove.end();
       ++jt)
  {
    std::vector<std::set<CostSource>::iterator> remove;
    std::set<CostSource> add;
    for (std::set<CostSource>::iterator it = cost_sources.begin(); it != cost_sources.end(); ++it)
    {
      p[0] = std::max(it->aabb_min[0], jt->aabb_min[0]);
      p[1] = std::max(it->aabb_min[1], jt->aabb_min[1]);
      p[2] = std::max(it->aabb_min[2], jt->aabb_min[2]);

      q[0] = std::min(it->aabb_max[0], jt->aabb_max[0]);
      q[1] = std::min(it->aabb_max[1], jt->aabb_max[1]);
      q[2] = std::min(it->aabb_max[2], jt->aabb_max[2]);

      if (p[0] >= q[0] || p[1] >= q[1] || p[2] >= q[2])
        continue;

      double intersect_volume = (q[0] - p[0]) * (q[1] - p[1]) * (q[2] - p[2]);
      if (intersect_volume >= it->getVolume() * overlap_fraction)
        remove.push_back(it);
      else
      {
        // there is some overlap, but not too large, so we split the cost source into multiple ones
        for (int i = 0; i < 3; ++i)
        {
          // is there a box above axis i in the intersection?
          if (it->aabb_max[i] > q[i])
          {
            CostSource cs = *it;
            cs.aabb_min[i] = q[i];
            add.insert(cs);
          }
          // is there a box below axis i in the intersection?
          if (it->aabb_min[i] < p[i])
          {
            CostSource cs = *it;
            cs.aabb_max[i] = p[i];
            add.insert(cs);
          }
        }
      }
    }
    for (std::size_t i = 0; i < remove.size(); ++i)
      cost_sources.erase(remove[i]);
    cost_sources.insert(add.begin(), add.end());
  }
}

void collision_detection::costSourceToMsg(const CostSource &cost_source, moveit_msgs::CostSource &msg)
{
  msg.cost_density = cost_source.cost;
  msg.aabb_min.x = cost_source.aabb_min[0];
  msg.aabb_min.y = cost_source.aabb_min[1];
  msg.aabb_min.z = cost_source.aabb_min[2];
  msg.aabb_max.x = cost_source.aabb_max[0];
  msg.aabb_max.y = cost_source.aabb_max[1];
  msg.aabb_max.z = cost_source.aabb_max[2];
}

void collision_detection::contactToMsg(const Contact &contact, moveit_msgs::ContactInformation &msg)
{
  tf::pointEigenToMsg(contact.pos, msg.position);
  tf::vectorEigenToMsg(contact.normal, msg.normal);
  msg.depth = contact.depth;
  msg.contact_body_1 = contact.body_name_1;
  msg.contact_body_2 = contact.body_name_2;
  if (contact.body_type_1 == BodyTypes::ROBOT_LINK)
    msg.body_type_1 = moveit_msgs::ContactInformation::ROBOT_LINK;
  else if (contact.body_type_1 == BodyTypes::ROBOT_ATTACHED)
    msg.body_type_1 = moveit_msgs::ContactInformation::ROBOT_ATTACHED;
  else
    msg.body_type_1 = moveit_msgs::ContactInformation::WORLD_OBJECT;
  if (contact.body_type_2 == BodyTypes::ROBOT_LINK)
    msg.body_type_2 = moveit_msgs::ContactInformation::ROBOT_LINK;
  else if (contact.body_type_2 == BodyTypes::ROBOT_ATTACHED)
    msg.body_type_2 = moveit_msgs::ContactInformation::ROBOT_ATTACHED;
  else
    msg.body_type_2 = moveit_msgs::ContactInformation::WORLD_OBJECT;
}
