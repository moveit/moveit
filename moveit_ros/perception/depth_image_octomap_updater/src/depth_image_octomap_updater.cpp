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

/* Author: Ioan Sucan, Suat Gedikli */

#include <moveit/depth_image_octomap_updater/depth_image_octomap_updater.h>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>
#include <geometric_shapes/shape_operations.h>
#include <sensor_msgs/image_encodings.h>
#include <XmlRpcException.h>
#include <stdint.h>

#include <memory>

namespace occupancy_map_monitor
{
DepthImageOctomapUpdater::DepthImageOctomapUpdater()
  : OccupancyMapUpdater("DepthImageUpdater")
  , nh_("~")
  , input_depth_transport_(nh_)
  , model_depth_transport_(nh_)
  , filtered_depth_transport_(nh_)
  , filtered_label_transport_(nh_)
  , image_topic_("depth")
  , queue_size_(5)
  , near_clipping_plane_distance_(0.3)
  , far_clipping_plane_distance_(5.0)
  , shadow_threshold_(0.04)
  , padding_scale_(0.0)
  , padding_offset_(0.02)
  , max_update_rate_(0)
  , skip_vertical_pixels_(4)
  , skip_horizontal_pixels_(6)
  , image_callback_count_(0)
  , average_callback_dt_(0.0)
  , good_tf_(5)
  ,  // start optimistically, so we do not output warnings right from the beginning
  failed_tf_(0)
  , K0_(0.0)
  , K2_(0.0)
  , K4_(0.0)
  , K5_(0.0)
{
}

DepthImageOctomapUpdater::~DepthImageOctomapUpdater()
{
  stopHelper();
}

bool DepthImageOctomapUpdater::setParams(XmlRpc::XmlRpcValue& params)
{
  try
  {
    sensor_type_ = (std::string)params["sensor_type"];
    if (params.hasMember("image_topic"))
      image_topic_ = (std::string)params["image_topic"];
    if (params.hasMember("queue_size"))
      queue_size_ = (int)params["queue_size"];

    readXmlParam(params, "near_clipping_plane_distance", &near_clipping_plane_distance_);
    readXmlParam(params, "far_clipping_plane_distance", &far_clipping_plane_distance_);
    readXmlParam(params, "shadow_threshold", &shadow_threshold_);
    readXmlParam(params, "padding_scale", &padding_scale_);
    readXmlParam(params, "padding_offset", &padding_offset_);
    if (params.hasMember("max_update_rate"))
      readXmlParam(params, "max_update_rate", &max_update_rate_);
    readXmlParam(params, "skip_vertical_pixels", &skip_vertical_pixels_);
    readXmlParam(params, "skip_horizontal_pixels", &skip_horizontal_pixels_);
    if (params.hasMember("filtered_cloud_topic"))
      filtered_cloud_topic_ = static_cast<const std::string&>(params["filtered_cloud_topic"]);
  }
  catch (XmlRpc::XmlRpcException& ex)
  {
    ROS_ERROR("XmlRpc Exception: %s", ex.getMessage().c_str());
    return false;
  }

  return true;
}

bool DepthImageOctomapUpdater::initialize()
{
  tf_ = monitor_->getTFClient();
  free_space_updater_.reset(new LazyFreeSpaceUpdater(tree_));

  // create our mesh filter
  mesh_filter_.reset(new mesh_filter::MeshFilter<mesh_filter::StereoCameraModel>(
      mesh_filter::MeshFilterBase::TransformCallback(), mesh_filter::StereoCameraModel::RegisteredPSDKParams));
  mesh_filter_->parameters().setDepthRange(near_clipping_plane_distance_, far_clipping_plane_distance_);
  mesh_filter_->setShadowThreshold(shadow_threshold_);
  mesh_filter_->setPaddingOffset(padding_offset_);
  mesh_filter_->setPaddingScale(padding_scale_);
  mesh_filter_->setTransformCallback(boost::bind(&DepthImageOctomapUpdater::getShapeTransform, this, _1, _2));

  return true;
}

void DepthImageOctomapUpdater::start()
{
  image_transport::TransportHints hints("raw", ros::TransportHints(), nh_);
  pub_model_depth_image_ = model_depth_transport_.advertiseCamera("model_depth", 1);

  if (!filtered_cloud_topic_.empty())
    pub_filtered_depth_image_ = filtered_depth_transport_.advertiseCamera(filtered_cloud_topic_, 1);
  else
    pub_filtered_depth_image_ = filtered_depth_transport_.advertiseCamera("filtered_depth", 1);

  pub_filtered_label_image_ = filtered_label_transport_.advertiseCamera("filtered_label", 1);

  sub_depth_image_ = input_depth_transport_.subscribeCamera(image_topic_, queue_size_,
                                                            &DepthImageOctomapUpdater::depthImageCallback, this, hints);
}

void DepthImageOctomapUpdater::stop()
{
  stopHelper();
}

void DepthImageOctomapUpdater::stopHelper()
{
  sub_depth_image_.shutdown();
}

mesh_filter::MeshHandle DepthImageOctomapUpdater::excludeShape(const shapes::ShapeConstPtr& shape)
{
  mesh_filter::MeshHandle h = 0;
  if (mesh_filter_)
  {
    if (shape->type == shapes::MESH)
      h = mesh_filter_->addMesh(static_cast<const shapes::Mesh&>(*shape));
    else
    {
      std::unique_ptr<shapes::Mesh> m(shapes::createMeshFromShape(shape.get()));
      if (m)
        h = mesh_filter_->addMesh(*m);
    }
  }
  else
    ROS_ERROR("Mesh filter not yet initialized!");
  return h;
}

void DepthImageOctomapUpdater::forgetShape(mesh_filter::MeshHandle handle)
{
  if (mesh_filter_)
    mesh_filter_->removeMesh(handle);
}

bool DepthImageOctomapUpdater::getShapeTransform(mesh_filter::MeshHandle h, Eigen::Affine3d& transform) const
{
  ShapeTransformCache::const_iterator it = transform_cache_.find(h);
  if (it == transform_cache_.end())
  {
    ROS_ERROR("Internal error. Mesh filter handle %u not found", h);
    return false;
  }
  transform = it->second;
  return true;
}

namespace
{
bool host_is_big_endian(void)
{
  union
  {
    uint32_t i;
    char c[sizeof(uint32_t)];
  } bint = { 0x01020304 };
  return bint.c[0] == 1;
}
}

static const bool HOST_IS_BIG_ENDIAN = host_is_big_endian();

void DepthImageOctomapUpdater::depthImageCallback(const sensor_msgs::ImageConstPtr& depth_msg,
                                                  const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  ROS_DEBUG("Received a new depth image message (frame = '%s', encoding='%s')", depth_msg->header.frame_id.c_str(),
            depth_msg->encoding.c_str());
  ros::WallTime start = ros::WallTime::now();

  if (max_update_rate_ > 0)
  {
    // ensure we are not updating the octomap representation too often
    if (ros::Time::now() - last_update_time_ <= ros::Duration(1.0 / max_update_rate_))
      return;
    last_update_time_ = ros::Time::now();
  }

  // measure the frequency at which we receive updates
  if (image_callback_count_ < 1000)
  {
    if (image_callback_count_ > 0)
    {
      const double dt_start = (start - last_depth_callback_start_).toSec();
      if (image_callback_count_ < 2)
        average_callback_dt_ = dt_start;
      else
        average_callback_dt_ =
            ((image_callback_count_ - 1) * average_callback_dt_ + dt_start) / (double)image_callback_count_;
    }
  }
  else
    // every 1000 updates we reset the counter almost to the beginning (use 2 so we don't have so much of a ripple in
    // the measured average)
    image_callback_count_ = 2;
  last_depth_callback_start_ = start;
  ++image_callback_count_;

  if (monitor_->getMapFrame().empty())
    monitor_->setMapFrame(depth_msg->header.frame_id);

  /* get transform for cloud into map frame */
  tf::StampedTransform map_H_sensor;
  if (monitor_->getMapFrame() == depth_msg->header.frame_id)
    map_H_sensor.setIdentity();
  else
  {
    if (tf_)
    {
      // wait at most 50ms
      static const double TEST_DT = 0.005;
      const int nt = (int)(0.5 + average_callback_dt_ / TEST_DT) * std::max(1, ((int)queue_size_ / 2));
      bool found = false;
      std::string err;
      for (int t = 0; t < nt; ++t)
        try
        {
          tf_->lookupTransform(monitor_->getMapFrame(), depth_msg->header.frame_id, depth_msg->header.stamp,
                               map_H_sensor);
          found = true;
          break;
        }
        catch (tf::TransformException& ex)
        {
          static const ros::Duration d(TEST_DT);
          err = ex.what();
          d.sleep();
        }
      static const unsigned int MAX_TF_COUNTER = 1000;  // so we avoid int overflow
      if (found)
      {
        good_tf_++;
        if (good_tf_ > MAX_TF_COUNTER)
        {
          const unsigned int div = MAX_TF_COUNTER / 10;
          good_tf_ /= div;
          failed_tf_ /= div;
        }
      }
      else
      {
        failed_tf_++;
        if (failed_tf_ > good_tf_)
          ROS_WARN_THROTTLE(1, "More than half of the image messages discared due to TF being unavailable (%u%%). "
                               "Transform error of sensor data: %s; quitting callback.",
                            (100 * failed_tf_) / (good_tf_ + failed_tf_), err.c_str());
        else
          ROS_DEBUG_THROTTLE(1, "Transform error of sensor data: %s; quitting callback", err.c_str());
        if (failed_tf_ > MAX_TF_COUNTER)
        {
          const unsigned int div = MAX_TF_COUNTER / 10;
          good_tf_ /= div;
          failed_tf_ /= div;
        }
        return;
      }
    }
    else
      return;
  }

  if (!updateTransformCache(depth_msg->header.frame_id, depth_msg->header.stamp))
  {
    ROS_ERROR_THROTTLE(1, "Transform cache was not updated. Self-filtering may fail.");
    return;
  }

  if (depth_msg->is_bigendian && !HOST_IS_BIG_ENDIAN)
    ROS_ERROR_THROTTLE(1, "endian problem: received image data does not match host");

  const int w = depth_msg->width;
  const int h = depth_msg->height;

  // call the mesh filter
  mesh_filter::StereoCameraModel::Parameters& params = mesh_filter_->parameters();
  params.setCameraParameters(info_msg->K[0], info_msg->K[4], info_msg->K[2], info_msg->K[5]);
  params.setImageSize(w, h);

  const bool is_u_short = depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1;
  if (is_u_short)
    mesh_filter_->filter(&depth_msg->data[0], GL_UNSIGNED_SHORT);
  else
  {
    if (depth_msg->encoding != sensor_msgs::image_encodings::TYPE_32FC1)
    {
      ROS_ERROR_THROTTLE(1, "Unexpected encoding type: '%s'. Ignoring input.", depth_msg->encoding.c_str());
      return;
    }
    mesh_filter_->filter(&depth_msg->data[0], GL_FLOAT);
  }

  // the mesh filter runs in background; compute extra things in the meantime

  // Use correct principal point from calibration
  const double px = info_msg->K[2];
  const double py = info_msg->K[5];

  // if the camera parameters have changed at all, recompute the cache we had
  if (w >= static_cast<int>(x_cache_.size()) || h >= static_cast<int>(y_cache_.size()) || K2_ != px || K5_ != py ||
      K0_ != info_msg->K[0] || K4_ != info_msg->K[4])
  {
    K2_ = px;
    K5_ = py;
    K0_ = info_msg->K[0];
    K4_ = info_msg->K[4];

    inv_fx_ = 1.0 / K0_;
    inv_fy_ = 1.0 / K4_;

    // if there are any NaNs, discard data
    if (!(px == px && py == py && inv_fx_ == inv_fx_ && inv_fy_ == inv_fy_))
      return;

    // Pre-compute some constants
    if (static_cast<int>(x_cache_.size()) < w)
      x_cache_.resize(w);
    if (static_cast<int>(y_cache_.size()) < h)
      y_cache_.resize(h);

    for (int x = 0; x < w; ++x)
      x_cache_[x] = (x - px) * inv_fx_;

    for (int y = 0; y < h; ++y)
      y_cache_[y] = (y - py) * inv_fy_;
  }

  const octomap::point3d sensor_origin(map_H_sensor.getOrigin().getX(), map_H_sensor.getOrigin().getY(),
                                       map_H_sensor.getOrigin().getZ());

  octomap::KeySet* occupied_cells_ptr = new octomap::KeySet();
  octomap::KeySet* model_cells_ptr = new octomap::KeySet();
  octomap::KeySet& occupied_cells = *occupied_cells_ptr;
  octomap::KeySet& model_cells = *model_cells_ptr;

  // allocate memory if needed
  std::size_t img_size = h * w;
  if (filtered_labels_.size() < img_size)
    filtered_labels_.resize(img_size);

  // get the labels of the filtered data
  const unsigned int* labels_row = &filtered_labels_[0];
  mesh_filter_->getFilteredLabels(&filtered_labels_[0]);

  // publish debug information if needed
  if (debug_info_)
  {
    sensor_msgs::Image debug_msg;
    debug_msg.header = depth_msg->header;
    debug_msg.height = depth_msg->height;
    debug_msg.width = depth_msg->width;
    debug_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    debug_msg.is_bigendian = depth_msg->is_bigendian;
    debug_msg.step = depth_msg->step;
    debug_msg.data.resize(img_size * sizeof(float));
    mesh_filter_->getModelDepth(reinterpret_cast<float*>(&debug_msg.data[0]));
    pub_model_depth_image_.publish(debug_msg, *info_msg);

    sensor_msgs::Image filtered_depth_msg;
    filtered_depth_msg.header = depth_msg->header;
    filtered_depth_msg.height = depth_msg->height;
    filtered_depth_msg.width = depth_msg->width;
    filtered_depth_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    filtered_depth_msg.is_bigendian = depth_msg->is_bigendian;
    filtered_depth_msg.step = depth_msg->step;
    filtered_depth_msg.data.resize(img_size * sizeof(float));

    mesh_filter_->getFilteredDepth(reinterpret_cast<float*>(&filtered_depth_msg.data[0]));
    pub_filtered_depth_image_.publish(filtered_depth_msg, *info_msg);

    sensor_msgs::Image label_msg;
    label_msg.header = depth_msg->header;
    label_msg.height = depth_msg->height;
    label_msg.width = depth_msg->width;
    label_msg.encoding = sensor_msgs::image_encodings::RGBA8;
    label_msg.is_bigendian = depth_msg->is_bigendian;
    label_msg.step = w * sizeof(unsigned int);
    label_msg.data.resize(img_size * sizeof(unsigned int));
    mesh_filter_->getFilteredLabels(reinterpret_cast<unsigned int*>(&label_msg.data[0]));

    pub_filtered_label_image_.publish(label_msg, *info_msg);
  }

  if (!filtered_cloud_topic_.empty())
  {
    static std::vector<float> filtered_data;
    sensor_msgs::Image filtered_msg;
    filtered_msg.header = depth_msg->header;
    filtered_msg.height = depth_msg->height;
    filtered_msg.width = depth_msg->width;
    filtered_msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    filtered_msg.is_bigendian = depth_msg->is_bigendian;
    filtered_msg.step = depth_msg->step;
    filtered_msg.data.resize(img_size * sizeof(unsigned short));
    if (filtered_data.size() < img_size)
      filtered_data.resize(img_size);
    mesh_filter_->getFilteredDepth(reinterpret_cast<float*>(&filtered_data[0]));
    unsigned short* tmp_ptr = (unsigned short*)&filtered_msg.data[0];
    for (std::size_t i = 0; i < img_size; ++i)
    {
      tmp_ptr[i] = (unsigned short)(filtered_data[i] * 1000 + 0.5);
    }
    pub_filtered_depth_image_.publish(filtered_msg, *info_msg);
  }

  // figure out occupied cells and model cells
  tree_->lockRead();

  try
  {
    const int h_bound = h - skip_vertical_pixels_;
    const int w_bound = w - skip_horizontal_pixels_;

    if (is_u_short)
    {
      const uint16_t* input_row = reinterpret_cast<const uint16_t*>(&depth_msg->data[0]);

      for (int y = skip_vertical_pixels_; y < h_bound; ++y, labels_row += w, input_row += w)
        for (int x = skip_horizontal_pixels_; x < w_bound; ++x)
        {
          // not filtered
          if (labels_row[x] == mesh_filter::MeshFilterBase::Background)
          {
            float zz = (float)input_row[x] * 1e-3;  // scale from mm to m
            float yy = y_cache_[y] * zz;
            float xx = x_cache_[x] * zz;
            /* transform to map frame */
            tf::Vector3 point_tf = map_H_sensor * tf::Vector3(xx, yy, zz);
            occupied_cells.insert(tree_->coordToKey(point_tf.getX(), point_tf.getY(), point_tf.getZ()));
          }
          // on far plane or a model point -> remove
          else if (labels_row[x] >= mesh_filter::MeshFilterBase::FarClip)
          {
            float zz = input_row[x] * 1e-3;
            float yy = y_cache_[y] * zz;
            float xx = x_cache_[x] * zz;
            /* transform to map frame */
            tf::Vector3 point_tf = map_H_sensor * tf::Vector3(xx, yy, zz);
            // add to the list of model cells
            model_cells.insert(tree_->coordToKey(point_tf.getX(), point_tf.getY(), point_tf.getZ()));
          }
        }
    }
    else
    {
      const float* input_row = reinterpret_cast<const float*>(&depth_msg->data[0]);

      for (int y = skip_vertical_pixels_; y < h_bound; ++y, labels_row += w, input_row += w)
        for (int x = skip_horizontal_pixels_; x < w_bound; ++x)
        {
          if (labels_row[x] == mesh_filter::MeshFilterBase::Background)
          {
            float zz = input_row[x];
            float yy = y_cache_[y] * zz;
            float xx = x_cache_[x] * zz;
            /* transform to map frame */
            tf::Vector3 point_tf = map_H_sensor * tf::Vector3(xx, yy, zz);
            occupied_cells.insert(tree_->coordToKey(point_tf.getX(), point_tf.getY(), point_tf.getZ()));
          }
          else if (labels_row[x] >= mesh_filter::MeshFilterBase::FarClip)
          {
            float zz = input_row[x];
            float yy = y_cache_[y] * zz;
            float xx = x_cache_[x] * zz;
            /* transform to map frame */
            tf::Vector3 point_tf = map_H_sensor * tf::Vector3(xx, yy, zz);
            // add to the list of model cells
            model_cells.insert(tree_->coordToKey(point_tf.getX(), point_tf.getY(), point_tf.getZ()));
          }
        }
    }
  }
  catch (...)
  {
    tree_->unlockRead();
    ROS_ERROR("Internal error while parsing depth data");
    return;
  }
  tree_->unlockRead();

  /* cells that overlap with the model are not occupied */
  for (octomap::KeySet::iterator it = model_cells.begin(), end = model_cells.end(); it != end; ++it)
    occupied_cells.erase(*it);

  // mark occupied cells
  tree_->lockWrite();
  try
  {
    /* now mark all occupied cells */
    for (octomap::KeySet::iterator it = occupied_cells.begin(), end = occupied_cells.end(); it != end; ++it)
      tree_->updateNode(*it, true);
  }
  catch (...)
  {
    ROS_ERROR("Internal error while updating octree");
  }
  tree_->unlockWrite();
  tree_->triggerUpdateCallback();

  // at this point we still have not freed the space
  free_space_updater_->pushLazyUpdate(occupied_cells_ptr, model_cells_ptr, sensor_origin);

  ROS_DEBUG("Processed depth image in %lf ms", (ros::WallTime::now() - start).toSec() * 1000.0);
}
}
