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

/* Author: Jon Binney, Ioan Sucan */

#pragma once

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <moveit/occupancy_map_monitor/occupancy_map_updater.h>
#include <moveit/point_containment_filter/shape_mask.h>
#include <moveit_msgs/UpdatePointcloudOctomap.h>

#include <memory>
#include <mutex>

namespace occupancy_map_monitor
{
/* Maintaining the octomap is supported in two ways:
 * INCREMENTAL: Maintains a an octomap from multiple updates over time.
                Cells that are marked occupied will remain so even if obscured from the camera view in subsequent
 updates,
                and marked free only after they are seen to be free in multiple pointclouds.

   SNAPSHOT:    Create an entirely new octomap from each pointcloud, forgetting previous data with each update.
 */
enum class UpdateMethod
{
  INCREMENTAL,
  SNAPSHOT
};

class PointCloudOctomapUpdater : public OccupancyMapUpdater
{
public:
  PointCloudOctomapUpdater();
  ~PointCloudOctomapUpdater() override;

  bool setParams(XmlRpc::XmlRpcValue& params) override;

  bool initialize() override;
  void start() override;
  void stop() override;
  ShapeHandle excludeShape(const shapes::ShapeConstPtr& shape) override;

  /** @brief Exclude a shape from the octomap when processing.
             Allows specifying an initial transform to put in the transform cache.
             For use outside of the perception pipeline.
      @param shape The shape to exclude
      @param pose Pose of the shape in the sensor frame */
  ShapeHandle excludeShape(const shapes::ShapeConstPtr& shape, const Eigen::Isometry3d& pose);

  void forgetShape(ShapeHandle handle) override;

  /** @brief Process a pointcloud message and update the octomap.
             Does not update the transform cache.

             This function is intended for users maintaining their own Octree outside of the perception pipeline.
             The user can directly add shapes to filter by passing a shape together with a pose to `excludeShape`, and
             then can use Eigen for all poses, skipping any tf lookups.
      @param cloud_msg The pointcloud to process
      @param sensor_pose Eigen pose of the frame in which the pointcloud is given
      @param update_method Whether to update the current octomap probabilities or to forget all previous octomap data */
  bool processCloud(const sensor_msgs::PointCloud2& cloud_msg, const Eigen::Isometry3d& sensor_pose,
                    UpdateMethod update_method);

  /** @brief Process a pointcloud message and update the octomap.

             This version of the function is used by PlanningSceneMonitor in the perception pipeline.
             Uses tf to get the pose of the camera relative to the Octree, and a callback is used to get the poses of
             all filtered shapes.
             Thus, this requires setTransformCacheCallback to have been used to set a callback which is used to get the
     poses of filtered shapes
      @param cloud_msg The pointcloud to process */
  bool processCloud(const sensor_msgs::PointCloud2& cloud_msg);

protected:
  virtual void updateMask(const sensor_msgs::PointCloud2& cloud, const Eigen::Vector3d& sensor_origin,
                          std::vector<int>& mask);

private:
  bool getShapeTransform(ShapeHandle h, Eigen::Isometry3d& transform) const;
  void cloudMsgCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
  void stopHelper();
  bool updatePointcloudOctomapService(moveit_msgs::UpdatePointcloudOctomap::Request& req,
                                      moveit_msgs::UpdatePointcloudOctomap::Response& res);
  ros::NodeHandle root_nh_;
  ros::NodeHandle private_nh_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  ros::Time last_update_time_;

  /* params */
  std::string point_cloud_topic_;
  double scale_;
  double padding_;
  double max_range_;
  unsigned int point_subsample_;
  double max_update_rate_;
  UpdateMethod update_method_;
  std::string filtered_cloud_topic_;
  std::string service_name_;
  ros::Publisher filtered_cloud_publisher_;
  ros::ServiceServer update_service_;

  message_filters::Subscriber<sensor_msgs::PointCloud2>* point_cloud_subscriber_;
  tf2_ros::MessageFilter<sensor_msgs::PointCloud2>* point_cloud_filter_;

  /* used to store all cells in the map which a given ray passes through during raycasting.
     we cache this here because it dynamically pre-allocates a lot of memory in its contsructor */
  octomap::KeyRay key_ray_;

  std::unique_ptr<point_containment_filter::ShapeMask> shape_mask_;
  std::vector<int> mask_;
};
}  // namespace occupancy_map_monitor
