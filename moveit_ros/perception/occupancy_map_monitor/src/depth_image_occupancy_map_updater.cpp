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

/* Author: Ioan Sucan, Suat Gedikli, Jon Binney */

#include <moveit/occupancy_map_monitor/depth_image_occupancy_map_updater.h>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>
#include <sensor_msgs/image_encodings.h>
#include <XmlRpcException.h>
#include <stdint.h>

namespace occupancy_map_monitor
{

DepthImageOccupancyMapUpdater::DepthImageOccupancyMapUpdater(OccupancyMapMonitor *monitor) :
  OccupancyMapUpdater(monitor, "DepthImageUpdater"),
  nh_("~"),
  tf_(monitor->getTFClient()),
  input_depth_transport_(nh_),
  model_depth_transport_(nh_),
  filtered_depth_transport_(nh_),
  image_topic_("depth"),
  queue_size_(5),
  near_clipping_plane_distance_(0.3),
  far_clipping_plane_distance_(5.0),
  shadow_threshold_(0.5),
  padding_scale_(3.0),
  padding_offset_(0.02),
  skip_vertical_pixels_(4),
  skip_horizontal_pixels_(6)
{
}

DepthImageOccupancyMapUpdater::~DepthImageOccupancyMapUpdater()
{
  stopHelper();
}

static void readXmlParam(XmlRpc::XmlRpcValue &params, const std::string &param_name, double *value)
{
  if (params.hasMember(param_name))
  {
    if (params[param_name].getType() == XmlRpc::XmlRpcValue::TypeInt)
      *value = (int) params[param_name];
    else
      *value = (double) params[param_name];
  }
}
static void readXmlParam(XmlRpc::XmlRpcValue &params, const std::string &param_name, unsigned int *value)
{
  if (params.hasMember(param_name))
    *value = (int) params[param_name];
}

bool DepthImageOccupancyMapUpdater::setParams(XmlRpc::XmlRpcValue &params)
{
  try
  {
    sensor_type_ = (std::string) params["sensor_type"];
    if (params.hasMember("image_topic"))
      image_topic_ = (std::string) params["image_topic"];
    if (params.hasMember("queue_size"))
      queue_size_ = (int)params["queue_size"];
    
    readXmlParam(params, "near_clipping_plane_distance", &near_clipping_plane_distance_);
    readXmlParam(params, "far_clipping_plane_distance", &far_clipping_plane_distance_);
    readXmlParam(params, "shadow_threshold", &shadow_threshold_);
    readXmlParam(params, "padding_scale", &padding_scale_);
    readXmlParam(params, "padding_offset", &padding_offset_);
    readXmlParam(params, "skip_vertical_pixels", &skip_vertical_pixels_);
    readXmlParam(params, "skip_horizontal_pixels", &skip_horizontal_pixels_);
  }
  catch (XmlRpc::XmlRpcException &ex)
  {
    ROS_ERROR("XmlRpc Exception: %s", ex.getMessage().c_str());
    return false;
  }
  
  return true;
}

bool DepthImageOccupancyMapUpdater::initialize()
{
  // create our mesh filter
  mesh_filter_.reset(new mesh_filter::MeshFilter<mesh_filter::StereoCameraModel>(mesh_filter::MeshFilterBase::TransformCallback(),
                                                                                 mesh_filter::StereoCameraModel::RegisteredPSDKParams));
  mesh_filter_->parameters().setDepthRange(near_clipping_plane_distance_, far_clipping_plane_distance_);
  mesh_filter_->setShadowThreshold(shadow_threshold_);
  mesh_filter_->setPaddingOffset(padding_offset_);
  mesh_filter_->setPaddingScale(padding_scale_);
  mesh_filter_->setTransformCallback(boost::bind(&DepthImageOccupancyMapUpdater::getShapeTransform, this, _1, _2));
  
  return true;
}

void DepthImageOccupancyMapUpdater::start()
{
  image_transport::TransportHints hints("raw", ros::TransportHints(), nh_);
  sub_depth_image_ = input_depth_transport_.subscribeCamera(image_topic_, queue_size_, &DepthImageOccupancyMapUpdater::depthImageCallback, this, hints);
  pub_model_depth_image_ = model_depth_transport_.advertiseCamera("model_depth", 10);
  pub_filtered_depth_image_ = filtered_depth_transport_.advertiseCamera("filtered_depth", 10);
}

void DepthImageOccupancyMapUpdater::stop()
{ 
  stopHelper();
}

void DepthImageOccupancyMapUpdater::stopHelper()
{   
  sub_depth_image_.shutdown();
}

mesh_filter::MeshHandle DepthImageOccupancyMapUpdater::excludeShape(const shapes::ShapeConstPtr &shape)
{
  mesh_filter::MeshHandle h = 0;
  if (mesh_filter_)
  {
    if (shape->type == shapes::MESH)
      h = mesh_filter_->addMesh(static_cast<const shapes::Mesh&>(*shape));
  }
  else
    ROS_ERROR("Mesh filter not yet initialized!");  
  return h;
}

void DepthImageOccupancyMapUpdater::forgetShape(mesh_filter::MeshHandle handle)
{
  mesh_filter_->removeMesh(handle);
}

bool DepthImageOccupancyMapUpdater::getShapeTransform(mesh_filter::MeshHandle h, Eigen::Affine3d &transform) const
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
  union {
    uint32_t i;
    char c[sizeof(uint32_t)];
  } bint = {0x01020304};
  return bint.c[0] == 1; 
}
}

static const bool HOST_IS_BIG_ENDIAN = host_is_big_endian();

void DepthImageOccupancyMapUpdater::depthImageCallback(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  ROS_DEBUG("Received a new depth image message");
  ros::WallTime start = ros::WallTime::now();
 
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
      try
      {
        tf_->lookupTransform(monitor_->getMapFrame(), depth_msg->header.frame_id, depth_msg->header.stamp, map_H_sensor);
      }
      catch (tf::TransformException& ex)
      {
        ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << "; quitting callback");
        return;
      }
    }
    else
      return;
  } 
  
  if (!updateTransformCache(depth_msg->header.frame_id, depth_msg->header.stamp))
    ROS_ERROR_THROTTLE(1, "Transform cache was not updated. Self-filtering may fail.");
  
  if (depth_msg->is_bigendian && !HOST_IS_BIG_ENDIAN)
    ROS_ERROR_THROTTLE(1, "endian problem: received image data does not match host");
  
  const int w = depth_msg->width;
  const int h = depth_msg->height;
  
  // call the mesh filter
  mesh_filter::StereoCameraModel::Parameters& params = mesh_filter_->parameters();
  params.setCameraParameters (info_msg->K[0], info_msg->K[4], info_msg->K[2], info_msg->K[5]);
  params.setImageSize(w, h);
  
  const bool is_u_short = depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1;
  if (is_u_short)
    mesh_filter_->filter(&depth_msg->data[0], GL_UNSIGNED_SHORT);
  else
    mesh_filter_->filter(&depth_msg->data[0], GL_FLOAT);
  
  // Use correct principal point from calibration
  const double px = info_msg->K[2];
  const double py = info_msg->K[5];
  
  const double inv_fx = 1.0 / info_msg->K[0];
  const double inv_fy = 1.0 / info_msg->K[4];
  
  // Pre-compute some constants
  if (x_cache_.size() < w)
    x_cache_.resize(w);
  if (y_cache_.size() < h)
    y_cache_.resize(h);
  
  for (int x = 0; x < w; ++x)
    x_cache_[x] = (x - px) * inv_fx;
  
  for (int y = 0; y < h; ++y)
    y_cache_[y] = (y - py) * inv_fy;
  
  octomap::point3d sensor_origin(map_H_sensor.getOrigin().getX(), map_H_sensor.getOrigin().getY(), map_H_sensor.getOrigin().getZ());
  
  OccMapTreePtr tree = monitor_->getOcTreePtr();
  octomap::KeySet free_cells, occupied_cells, model_cells;
  
  // allocate memory if needed
  std::size_t img_size = h * w;
  if (filtered_data_.size() < img_size)
    filtered_data_.resize(img_size);
  const float* filtered_row = reinterpret_cast<const float*>(&filtered_data_[0]);
  
  mesh_filter_->getFilteredDepth(&filtered_data_[0]);
  
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
    memcpy(&debug_msg.data[0], &filtered_data_[0], sizeof(float) * img_size);
    pub_filtered_depth_image_.publish(debug_msg, *info_msg);
  }
  
  tree->lockRead();
  
  try
  {
    const int h_bound = h - skip_vertical_pixels_;
    const int w_bound = w - skip_horizontal_pixels_;
    
    if (is_u_short)
    {
      const uint16_t *input_row = reinterpret_cast<const uint16_t*>(&depth_msg->data[0]);
      
      for (int y = skip_vertical_pixels_ ; y < h_bound ; ++y, filtered_row += w, input_row += w)
	if (y_cache_[y] == y_cache_[y]) // if not NaN
	  for (int x = skip_horizontal_pixels_ ; x < w_bound ; ++x)
          {
            float zz = filtered_row[x];
            uint16_t zz0 = input_row[x];
            
            if (zz0 != 0)
            {
              if (zz == 0.0)
              {
                float zz1 = (float)zz0 * 1e-3; // scale from mm to m
                float yy = y_cache_[y] * zz1;
                float xx = x_cache_[x] * zz1;
                /* transform to map frame */
                tf::Vector3 point_tf = map_H_sensor * tf::Vector3(xx, yy, zz1);
                // add to the list of model cells
                model_cells.insert(tree->coordToKey(point_tf.getX(), point_tf.getY(), point_tf.getZ()));
              }
              else
              {
                if (zz == zz)
                {
                  float yy = y_cache_[y] * zz;
                  float xx = x_cache_[x] * zz;
                  /* transform to map frame */
                  tf::Vector3 point_tf = map_H_sensor * tf::Vector3(xx, yy, zz);
                  
                  // add to the list of occupied cells
                  occupied_cells.insert(tree->coordToKey(point_tf.getX(), point_tf.getY(), point_tf.getZ()));
                }
              }
            }
          }
    }
    else
    {
      const float *input_row = reinterpret_cast<const float*>(&depth_msg->data[0]);
      
      for (int y = skip_vertical_pixels_ ; y < h_bound ; ++y, filtered_row += w, input_row += w)
	if (y_cache_[y] == y_cache_[y]) // if not NaN
	  for (int x = skip_horizontal_pixels_ ; x < w_bound ; ++x)
          {
            float zz = filtered_row[x];
            float zz0 = input_row[x];
            if (zz0 == zz0 && zz == zz) // check for NaN
            {
              float yy = y_cache_[y] * zz0;
              float xx = x_cache_[x] * zz0;
              if (xx == xx)
              {
                /* transform to map frame */
                tf::Vector3 point_tf = map_H_sensor * tf::Vector3(xx, yy, zz0);
                if (zz == 0.0)
                  // add to the list of model cells
                  model_cells.insert(tree->coordToKey(point_tf.getX(), point_tf.getY(), point_tf.getZ()));
                else
                  // add to the list of occupied cells
                  occupied_cells.insert(tree->coordToKey(point_tf.getX(), point_tf.getY(), point_tf.getZ()));
              }
            }
          }
    }
    
    /* compute the free cells along each ray that ends at an occupied cell */
    for (octomap::KeySet::iterator it = occupied_cells.begin(), end = occupied_cells.end(); it != end; ++it)
      if (tree->computeRayKeys(sensor_origin, tree->keyToCoord(*it), key_ray_))
        free_cells.insert(key_ray_.begin(), key_ray_.end());
    
    /* compute the free cells along each ray that ends at a model cell */
    for (octomap::KeySet::iterator it = model_cells.begin(), end = model_cells.end(); it != end; ++it)
      if (tree->computeRayKeys(sensor_origin, tree->keyToCoord(*it), key_ray_))
        free_cells.insert(key_ray_.begin(), key_ray_.end());
  }
  catch (...)
  { 
    tree->unlockRead();
    return;
  }
  
  tree->unlockRead();
  
  /* cells that overlap with the model are not occupied */
  for (octomap::KeySet::iterator it = model_cells.begin(), end = model_cells.end(); it != end; ++it)
    occupied_cells.erase(*it);
  
  /* occupied cells are not free */
  for (octomap::KeySet::iterator it = occupied_cells.begin(), end = occupied_cells.end(); it != end; ++it)
    free_cells.erase(*it);
  
  tree->lockWrite();
  
  try
  {    
    /* mark free cells only if not seen occupied in this cloud */
    for (octomap::KeySet::iterator it = free_cells.begin(), end = free_cells.end(); it != end; ++it)
      tree->updateNode(*it, false);
    
    /* now mark all occupied cells */
    for (octomap::KeySet::iterator it = occupied_cells.begin(), end = occupied_cells.end(); it != end; ++it)
      tree->updateNode(*it, true);
    
    // set the logodds to the minimum for the cells that are part of the model
    const float lg = tree->getClampingThresMinLog() - tree->getClampingThresMaxLog();
    for (octomap::KeySet::iterator it = model_cells.begin(), end = model_cells.end(); it != end; ++it)
      tree->updateNode(*it, lg);
  }
  catch (...)
  {
    ROS_ERROR("Internal error while updating octree");
  }
  tree->unlockWrite();
  ROS_DEBUG("Processed depth image in %lf ms", (ros::WallTime::now() - start).toSec() * 1000.0);
  triggerUpdateCallback();
}

}
