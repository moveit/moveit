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

#include <XmlRpcException.h>
#include <cmath>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>
#include <moveit/pointcloud_octomap_updater/pointcloud_octomap_updater.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_eigen/tf2_eigen.h>

#include <memory>

namespace occupancy_map_monitor
{
static const std::string LOGNAME = "occupancy_map_monitor";
PointCloudOctomapUpdater::PointCloudOctomapUpdater()
  : OccupancyMapUpdater("PointCloudUpdater")
  , private_nh_("~")
  , scale_(1.0)
  , padding_(0.0)
  , max_range_(std::numeric_limits<double>::infinity())
  , point_subsample_(1)
  , max_update_rate_(0)
  , update_method_(UpdateMethod::INCREMENTAL)
  , point_cloud_subscriber_(nullptr)
  , point_cloud_filter_(nullptr)
{
}

PointCloudOctomapUpdater::~PointCloudOctomapUpdater()
{
  stopHelper();
}

bool PointCloudOctomapUpdater::setParams(XmlRpc::XmlRpcValue& params)
{
  try
  {
    if (params.hasMember("point_cloud_topic"))
      point_cloud_topic_ = static_cast<const std::string&>(params["point_cloud_topic"]);

    readXmlParam(params, "max_range", &max_range_);
    readXmlParam(params, "padding_offset", &padding_);
    readXmlParam(params, "padding_scale", &scale_);
    readXmlParam(params, "point_subsample", &point_subsample_);
    if (params.hasMember("max_update_rate"))
      readXmlParam(params, "max_update_rate", &max_update_rate_);
    if (params.hasMember("incremental"))
      update_method_ = (static_cast<bool>(params["incremental"]) ? UpdateMethod::INCREMENTAL : UpdateMethod::SNAPSHOT);
    if (params.hasMember("filtered_cloud_topic"))
      filtered_cloud_topic_ = static_cast<const std::string&>(params["filtered_cloud_topic"]);
    if (params.hasMember("service_name"))
      service_name_ = static_cast<const std::string&>(params["service_name"]);

    if (point_cloud_topic_.empty() && service_name_.empty())
    {
      ROS_WARN_NAMED(LOGNAME,
                     "Neither point_cloud_topic nor service_name was specified.  Updates may not be processed.");
    }
  }
  catch (XmlRpc::XmlRpcException& ex)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "XmlRpc Exception: " << ex.getMessage());
    return false;
  }

  return true;
}

bool PointCloudOctomapUpdater::initialize()
{
  tf_buffer_.reset(new tf2_ros::Buffer());
  tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_, root_nh_));
  shape_mask_.reset(new point_containment_filter::ShapeMask());
  shape_mask_->setTransformCallback(boost::bind(&PointCloudOctomapUpdater::getShapeTransform, this, _1, _2));
  if (!filtered_cloud_topic_.empty())
    filtered_cloud_publisher_ = private_nh_.advertise<sensor_msgs::PointCloud2>(filtered_cloud_topic_, 10, false);
  return true;
}

void PointCloudOctomapUpdater::start()
{
  if (point_cloud_subscriber_)
    return;
  /* subscribe to point cloud topic using tf filter*/
  if (!point_cloud_topic_.empty())
    point_cloud_subscriber_ =
        new message_filters::Subscriber<sensor_msgs::PointCloud2>(root_nh_, point_cloud_topic_, 5);
  if (!service_name_.empty())
    update_service_ =
        private_nh_.advertiseService(service_name_, &PointCloudOctomapUpdater::updatePointcloudOctomapService, this);
  if (tf_listener_ && tf_buffer_ && !monitor_->getMapFrame().empty())
  {
    point_cloud_filter_ = new tf2_ros::MessageFilter<sensor_msgs::PointCloud2>(*point_cloud_subscriber_, *tf_buffer_,
                                                                               monitor_->getMapFrame(), 5, root_nh_);
    point_cloud_filter_->registerCallback(boost::bind(&PointCloudOctomapUpdater::cloudMsgCallback, this, _1));
    ROS_INFO_NAMED(LOGNAME, "Listening to '%s' using message filter with target frame '%s'", point_cloud_topic_.c_str(),
                   point_cloud_filter_->getTargetFramesString().c_str());
  }
  else
  {
    point_cloud_subscriber_->registerCallback(boost::bind(&PointCloudOctomapUpdater::cloudMsgCallback, this, _1));
    ROS_INFO_NAMED(LOGNAME, "Listening to '%s'", point_cloud_topic_.c_str());
  }
}

void PointCloudOctomapUpdater::stopHelper()
{
  delete point_cloud_filter_;
  delete point_cloud_subscriber_;
}

void PointCloudOctomapUpdater::stop()
{
  stopHelper();
  point_cloud_filter_ = nullptr;
  point_cloud_subscriber_ = nullptr;
}

ShapeHandle PointCloudOctomapUpdater::excludeShape(const shapes::ShapeConstPtr& shape)
{
  std::lock_guard<std::recursive_mutex> lock(update_mutex_);
  ShapeHandle h = 0;
  if (shape_mask_)
    h = shape_mask_->addShape(shape, scale_, padding_);
  else
    ROS_ERROR_NAMED(LOGNAME, "Shape filter not yet initialized!");
  return h;
}

ShapeHandle PointCloudOctomapUpdater::excludeShape(const shapes::ShapeConstPtr& shape, const Eigen::Isometry3d& pose)
{
  std::lock_guard<std::recursive_mutex> lock(update_mutex_);
  ShapeHandle h = excludeShape(shape);
  // Add to transform cache, or update if it is already in it
  transform_cache_[h] = pose;
  ROS_DEBUG_NAMED(LOGNAME, "Updated shape pose in transform cache");

  return h;
}

void PointCloudOctomapUpdater::forgetShape(ShapeHandle handle)
{
  std::lock_guard<std::recursive_mutex> lock(update_mutex_);
  if (shape_mask_)
    shape_mask_->removeShape(handle);
}

bool PointCloudOctomapUpdater::getShapeTransform(ShapeHandle h, Eigen::Isometry3d& transform) const
{
  ShapeTransformCache::const_iterator it = transform_cache_.find(h);
  if (it != transform_cache_.end())
  {
    transform = it->second;
  }
  return it != transform_cache_.end();
}

void PointCloudOctomapUpdater::updateMask(const sensor_msgs::PointCloud2& /*cloud*/,
                                          const Eigen::Vector3d& /*sensor_origin*/, std::vector<int>& /*mask*/)
{
}

bool PointCloudOctomapUpdater::processCloud(const sensor_msgs::PointCloud2& cloud_msg,
                                            const Eigen::Isometry3d& sensor_pose, UpdateMethod update_method)
{
  std::lock_guard<std::recursive_mutex> lock(update_mutex_);

  ros::WallTime start = ros::WallTime::now();

  /* Mask out points contained in filtered shapes
   * by default this includes the robot and all collision objects
   */
  octomap::point3d sensor_origin(sensor_pose.translation().x(), sensor_pose.translation().y(),
                                 sensor_pose.translation().z());
  if (shape_mask_)
    shape_mask_->maskContainment(cloud_msg, sensor_pose.translation(), 0.0, max_range_, mask_);
  else
  {
    ROS_ERROR_NAMED(LOGNAME, "Shape filter not yet initialized!");
    return false;
  }
  updateMask(cloud_msg, sensor_pose.translation(), mask_);

  octomap::KeySet free_cells, occupied_cells, model_cells, clip_cells;
  std::unique_ptr<sensor_msgs::PointCloud2> filtered_cloud;

  // We only use these iterators if we are creating a filtered_cloud for
  // publishing. We cannot default construct these, so we use unique_ptr's
  // to defer construction
  std::unique_ptr<sensor_msgs::PointCloud2Iterator<float>> iter_filtered_x;
  std::unique_ptr<sensor_msgs::PointCloud2Iterator<float>> iter_filtered_y;
  std::unique_ptr<sensor_msgs::PointCloud2Iterator<float>> iter_filtered_z;

  if (!filtered_cloud_topic_.empty())
  {
    filtered_cloud.reset(new sensor_msgs::PointCloud2());
    filtered_cloud->header = cloud_msg.header;
    sensor_msgs::PointCloud2Modifier pcd_modifier(*filtered_cloud);
    pcd_modifier.setPointCloud2FieldsByString(1, "xyz");
    pcd_modifier.resize(cloud_msg.width * cloud_msg.height);

    // we have created a filtered_out, so we can create the iterators now
    iter_filtered_x.reset(new sensor_msgs::PointCloud2Iterator<float>(*filtered_cloud, "x"));
    iter_filtered_y.reset(new sensor_msgs::PointCloud2Iterator<float>(*filtered_cloud, "y"));
    iter_filtered_z.reset(new sensor_msgs::PointCloud2Iterator<float>(*filtered_cloud, "z"));
  }
  size_t filtered_cloud_size = 0;

  tree_->lockRead();

  try
  {
    for (unsigned int row = 0; row < cloud_msg.height; row += point_subsample_)
    {
      unsigned int row_c = row * cloud_msg.width;
      sensor_msgs::PointCloud2ConstIterator<float> pt_iter(cloud_msg, "x");
      // set iterator to point at start of the current row
      pt_iter += row_c;

      for (unsigned int col = 0; col < cloud_msg.width; col += point_subsample_, pt_iter += point_subsample_)
      {
        // if (mask_[row_c + col] == point_containment_filter::ShapeMask::CLIP)
        //  continue;

        /* check for NaN */
        if (!std::isnan(pt_iter[0]) && !std::isnan(pt_iter[1]) && !std::isnan(pt_iter[2]))
        {
          /* transform to map frame */
          Eigen::Vector3d point_pose = sensor_pose * Eigen::Vector3d(pt_iter[0], pt_iter[1], pt_iter[2]);

          /* occupied cell at ray endpoint if ray is shorter than max range and
             this point
             isn't on a part of the robot*/
          if (mask_[row_c + col] == point_containment_filter::ShapeMask::INSIDE)
            model_cells.insert(tree_->coordToKey(point_pose.x(), point_pose.y(), point_pose.z()));
          else if (mask_[row_c + col] == point_containment_filter::ShapeMask::CLIP)
            clip_cells.insert(tree_->coordToKey(point_pose.x(), point_pose.y(), point_pose.z()));
          else
          {
            occupied_cells.insert(tree_->coordToKey(point_pose.x(), point_pose.y(), point_pose.z()));
            // build list of valid points if we want to publish them
            if (filtered_cloud)
            {
              **iter_filtered_x = pt_iter[0];
              **iter_filtered_y = pt_iter[1];
              **iter_filtered_z = pt_iter[2];
              ++filtered_cloud_size;
              ++*iter_filtered_x;
              ++*iter_filtered_y;
              ++*iter_filtered_z;
            }
          }
        }
      }
    }

    if (update_method == UpdateMethod::INCREMENTAL)
    {
      /* If we are performing an incremental update, do ray tracing to find which cells this point cloud indicates
       * should be free */

      /* compute the free cells along each ray that ends at an occupied cell */
      for (const octomap::OcTreeKey& occupied_cell : occupied_cells)
        if (tree_->computeRayKeys(sensor_origin, tree_->keyToCoord(occupied_cell), key_ray_))
          free_cells.insert(key_ray_.begin(), key_ray_.end());

      /* compute the free cells along each ray that ends at a model cell */
      for (const octomap::OcTreeKey& model_cell : model_cells)
        if (tree_->computeRayKeys(sensor_origin, tree_->keyToCoord(model_cell), key_ray_))
          free_cells.insert(key_ray_.begin(), key_ray_.end());

      /* compute the free cells along each ray that ends at a clipped cell */
      for (const octomap::OcTreeKey& clip_cell : clip_cells)
        if (tree_->computeRayKeys(sensor_origin, tree_->keyToCoord(clip_cell), key_ray_))
          free_cells.insert(key_ray_.begin(), key_ray_.end());
    }
  }
  catch (...)
  {
    tree_->unlockRead();
    ROS_ERROR_NAMED(LOGNAME, "Internal error while updating octree");
    return false;
  }

  tree_->unlockRead();

  if (update_method == UpdateMethod::INCREMENTAL)
  {
    /* occupied cells are not free */
    for (const octomap::OcTreeKey& occupied_cell : occupied_cells)
      free_cells.erase(occupied_cell);
  }

  tree_->lockWrite();

  if (update_method == UpdateMethod::SNAPSHOT)
    tree_->clear();

  bool success = true;
  try
  {
    /* mark free cells only if not seen occupied in this cloud */
    for (const octomap::OcTreeKey& free_cell : free_cells)
      tree_->updateNode(free_cell, false);

    /* now mark all occupied cells */
    for (const octomap::OcTreeKey& occupied_cell : occupied_cells)
      tree_->updateNode(occupied_cell, true);

    // set the logodds to the minimum for the cells that are part of the model
    const float lg = tree_->getClampingThresMinLog() - tree_->getClampingThresMaxLog();
    for (const octomap::OcTreeKey& model_cell : model_cells)
      tree_->updateNode(model_cell, lg);
  }
  catch (...)
  {
    ROS_ERROR_NAMED(LOGNAME, "Internal error while updating octree");
    success = false;
  }
  tree_->unlockWrite();

  if (filtered_cloud)
  {
    sensor_msgs::PointCloud2Modifier pcd_modifier(*filtered_cloud);
    pcd_modifier.resize(filtered_cloud_size);
    filtered_cloud_publisher_.publish(*filtered_cloud);
  }

  ROS_DEBUG_NAMED(LOGNAME, "Processed point cloud in %lf ms", (ros::WallTime::now() - start).toSec() * 1000.0);
  tree_->triggerUpdateCallback();

  if (success)
  {
    last_update_time_ = ros::Time::now();
  }

  return success;
}

bool PointCloudOctomapUpdater::processCloud(const sensor_msgs::PointCloud2& cloud_msg)
{
  std::lock_guard<std::recursive_mutex> lock(update_mutex_);

  if (monitor_->getMapFrame().empty())
    monitor_->setMapFrame(cloud_msg.header.frame_id);

  /* get transform for cloud into map frame */
  Eigen::Isometry3d sensor_origin_eigen;

  if (monitor_->getMapFrame() == cloud_msg.header.frame_id)
    sensor_origin_eigen.setIdentity();
  else
  {
    if (tf_buffer_)
    {
      try
      {
        geometry_msgs::TransformStamped sensor_origin =
            tf_buffer_->lookupTransform(monitor_->getMapFrame(), cloud_msg.header.frame_id, cloud_msg.header.stamp);
        sensor_origin_eigen = tf2::transformToEigen(sensor_origin);
      }
      catch (tf2::TransformException& ex)
      {
        ROS_ERROR_STREAM_NAMED(LOGNAME, "Transform error of sensor data: " << ex.what() << "; quitting callback");
        return false;
      }
    }
    else
      return false;
  }

  if (!updateTransformCache(cloud_msg.header.frame_id, cloud_msg.header.stamp))
  {
    ROS_ERROR_THROTTLE_NAMED(1, LOGNAME, "Transform cache was not updated. Self-filtering may fail.");
    return false;
  }

  return processCloud(cloud_msg, sensor_origin_eigen, update_method_);
}

void PointCloudOctomapUpdater::cloudMsgCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  std::lock_guard<std::recursive_mutex> lock(update_mutex_);
  ROS_DEBUG_NAMED(LOGNAME, "Received a new point cloud message");
  if (max_update_rate_ > 0)
  {
    // ensure we are not updating the octomap representation too often
    if (ros::Time::now() - last_update_time_ <= ros::Duration(1.0 / max_update_rate_))
      return;
  }

  processCloud(*cloud_msg);
}

bool PointCloudOctomapUpdater::updatePointcloudOctomapService(moveit_msgs::UpdatePointcloudOctomap::Request& req,
                                                              moveit_msgs::UpdatePointcloudOctomap::Response& res)
{
  std::lock_guard<std::recursive_mutex> lock(update_mutex_);
  res.success = processCloud(req.cloud);
  return res.success;
}

}  // namespace occupancy_map_monitor
