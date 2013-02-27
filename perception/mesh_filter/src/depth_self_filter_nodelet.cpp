/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Willow Garage, Inc.
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

/* Author: Suat Gedikli */

#include <moveit/mesh_filter/depth_self_filter_nodelet.h>
#include <moveit/mesh_filter/stereo_camera_model.h>
#include <moveit/mesh_filter/mesh_filter.h>
#include <ros/ros.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/image_encodings.h>
#include <moveit/planning_models_loader/kinematic_model_loader.h>
#include <eigen3/Eigen/Eigen>
#include <cv_bridge/cv_bridge.h>


namespace enc = sensor_msgs::image_encodings;
using namespace std;
using namespace boost;

mesh_filter::DepthSelfFiltering::~DepthSelfFiltering ()
{
  stop_ = true;
  // make sure the thread doesn't miss the notification!
  filter_mutex_.lock ();
  filter_condition_.notify_one();
  filter_mutex_.unlock ();
  filter_thread_.join ();
}

void mesh_filter::DepthSelfFiltering::onInit()
{
  ros::NodeHandle& nh         = getNodeHandle();
  ros::NodeHandle& private_nh = getPrivateNodeHandle();
  input_depth_transport_.reset(new image_transport::ImageTransport(nh));
  filtered_depth_transport_.reset(new image_transport::ImageTransport(nh));
  filtered_label_transport_.reset(new image_transport::ImageTransport(nh));
  model_depth_transport_.reset(new image_transport::ImageTransport(nh));
  model_label_transport_.reset(new image_transport::ImageTransport(nh));

  // Read parameters
  private_nh.param("queue_size", queue_size_, 5);
  private_nh.param("near_clipping_plane_distance", near_clipping_plane_distance_, 0.4);
  private_nh.param("far_clipping_plane_distance", far_clipping_plane_distance_, 5.0);;
  private_nh.param("shadow_threshold", shadow_threshold_, 0.3);
  private_nh.param("padding_scale", padding_scale_, 0.0035);
  private_nh.param("padding_offset", padding_offset_, 0.0);
  double tf_update_rate = 30;
  private_nh.param("tf_update_rate", tf_update_rate, 30.0);
  transform_provider_.setUpdateInterval (long(1000000.0 / tf_update_rate));

  image_transport::SubscriberStatusCallback itssc = bind(&DepthSelfFiltering::connectCb, this);
  ros::SubscriberStatusCallback rssc = bind(&DepthSelfFiltering::connectCb, this);

  lock_guard<mutex> lock(connect_mutex_);
  pub_filtered_depth_image_ = filtered_depth_transport_->advertiseCamera("/filtered/depth", 10, itssc, itssc, rssc, rssc);
  pub_filtered_label_image_ = filtered_label_transport_->advertiseCamera("/filtered/labels", 10, itssc, itssc, rssc, rssc);
  pub_model_depth_image_ = model_depth_transport_->advertiseCamera("/model/depth", 10, itssc, itssc, rssc, rssc);
  pub_model_label_image_ = model_depth_transport_->advertiseCamera("/model/label", 10, itssc, itssc, rssc, rssc);
  
  stop_ = false;
  filter_thread_ = thread (&DepthSelfFiltering::filter, this);
}


void mesh_filter::DepthSelfFiltering::filter ()
{
  // create our mesh filter
  MeshFilter<mesh_filter::StereoCameraModel> mesh_filter (bind(&TransformProvider::getTransform, &transform_provider_, _1, _2),
                                                          mesh_filter::StereoCameraModel::RegisteredKinectParams);
  mesh_filter.parameters ().setDepthRange (near_clipping_plane_distance_, far_clipping_plane_distance_);
  mesh_filter.setShadowThreshold(shadow_threshold_);
  mesh_filter.setPaddingOffset (padding_offset_);
  mesh_filter.setPaddingScale (padding_scale_);
  // add meshesfla
  addMeshes (mesh_filter);
  transform_provider_.start ();
  cv_bridge::CvImagePtr filtered_label_ptr (new cv_bridge::CvImage);
  filtered_label_ptr->encoding = sensor_msgs::image_encodings::RGBA8;

  cv_bridge::CvImagePtr filtered_depth_ptr (new cv_bridge::CvImage);
  filtered_depth_ptr->encoding = sensor_msgs::image_encodings::TYPE_32FC1;

  cv_bridge::CvImagePtr model_depth_ptr (new cv_bridge::CvImage);
  model_depth_ptr->encoding = sensor_msgs::image_encodings::TYPE_32FC1;

  cv_bridge::CvImagePtr model_label_ptr (new cv_bridge::CvImage);
  model_label_ptr->encoding = sensor_msgs::image_encodings::RGBA8;

  while (!stop_)
  {
    unique_lock<mutex> lock (filter_mutex_);
    
    if (!next_depth_msg_)
      filter_condition_.wait(lock);
    
    if (next_depth_msg_)
    {
      current_depth_msg_.swap (next_depth_msg_);
      current_info_msg_.swap (next_info_msg_);
      next_depth_msg_.reset ();
      next_info_msg_.reset ();
    }
    lock.unlock ();
    
    if (current_depth_msg_)
    {
      transform_provider_.setFrame (current_depth_msg_->header.frame_id);
      // do filtering here
      mesh_filter::StereoCameraModel::Parameters& params = mesh_filter.parameters ();
      params.setCameraParameters (current_info_msg_->K[0], current_info_msg_->K[4], current_info_msg_->K[2], current_info_msg_->K[5]);
      params.setImageSize (current_depth_msg_->width, current_depth_msg_->height);
      mesh_filter.filter ((float*) &current_depth_msg_->data [0]);

      if (pub_filtered_depth_image_.getNumSubscribers() > 0)
      {
        filtered_depth_ptr->header = current_depth_msg_->header;

        if (filtered_depth_ptr->image.cols != current_depth_msg_->width || filtered_depth_ptr->image.rows != current_depth_msg_->height)
          filtered_depth_ptr->image = cv::Mat (current_depth_msg_->height, current_depth_msg_->width, CV_32FC1);
        mesh_filter.getFilteredDepth ((float*)filtered_depth_ptr->image.data);
        pub_filtered_depth_image_.publish (filtered_depth_ptr->toImageMsg(), current_info_msg_);
      }
      
      // this is from rendering of the model
      if (pub_model_depth_image_.getNumSubscribers() > 0)
      {
        model_depth_ptr->header = current_depth_msg_->header;

        if (model_depth_ptr->image.cols != current_depth_msg_->width || model_depth_ptr->image.rows != current_depth_msg_->height)
          model_depth_ptr->image = cv::Mat (current_depth_msg_->height, current_depth_msg_->width, CV_32FC1);
        
        mesh_filter.getModelDepth ((float*)model_depth_ptr->image.data);
        pub_model_depth_image_.publish (model_depth_ptr->toImageMsg(), current_info_msg_);
      }
      
      if (pub_filtered_label_image_.getNumSubscribers() > 0)
      {
        filtered_label_ptr->header = current_depth_msg_->header;

        if (filtered_label_ptr->image.cols != current_depth_msg_->width || filtered_label_ptr->image.rows != current_depth_msg_->height)
          filtered_label_ptr->image = cv::Mat (current_depth_msg_->height, current_depth_msg_->width, CV_8UC4);

        mesh_filter.getFilteredLabels ((unsigned int*)filtered_label_ptr->image.data);
        pub_filtered_label_image_.publish (filtered_label_ptr->toImageMsg(), current_info_msg_);
      }

      if (pub_model_label_image_.getNumSubscribers() > 0)
      {
        model_label_ptr->header = current_depth_msg_->header;

        if (model_label_ptr->image.cols != current_depth_msg_->width || model_label_ptr->image.rows != current_depth_msg_->height)
          model_label_ptr->image = cv::Mat (current_depth_msg_->height, current_depth_msg_->width, CV_8UC4);

        mesh_filter.getModelLabels ((unsigned int*)model_label_ptr->image.data);
        pub_model_label_image_.publish (model_label_ptr->toImageMsg(), current_info_msg_);
      }
    }
  }
}

void mesh_filter::DepthSelfFiltering::addMeshes (MeshFilter<StereoCameraModel>& mesh_filter)
{
  planning_models_loader::KinematicModelLoader kml("robot_description");
  kinematic_model::KinematicModelConstPtr kmodel = kml.getModel();
  const vector<kinematic_model::LinkModel*> &links = kmodel->getLinkModelsWithCollisionGeometry();
  for (size_t i = 0 ; i < links.size() ; ++i)
  {
    shapes::ShapeConstPtr shape = links[i]->getShape();
    if (shape->type == shapes::MESH)
    {
      const shapes::Mesh &m = static_cast<const shapes::Mesh&>(*shape);
      MeshHandle mesh_handle = mesh_filter.addMesh (m);
      transform_provider_.addHandle (mesh_handle, links[i]->getName ());
    }
  }
}

// Handles (un)subscribing when clients (un)subscribe
void mesh_filter::DepthSelfFiltering::connectCb()
{
  lock_guard<mutex> lock(connect_mutex_);
  if (pub_filtered_depth_image_.getNumSubscribers() == 0 && 
      pub_filtered_label_image_.getNumSubscribers() == 0 && 
      pub_model_depth_image_.getNumSubscribers() == 0 &&
      pub_model_label_image_.getNumSubscribers() == 0 )
  {
    sub_depth_image_.shutdown();
  }
  else if (!sub_depth_image_)
  {
    image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
    sub_depth_image_ = input_depth_transport_->subscribeCamera("depth", queue_size_, &DepthSelfFiltering::depthCb, this, hints);
  }
}

void mesh_filter::DepthSelfFiltering::depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
                                 const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  filter_mutex_.lock();
  if (!next_depth_msg_.get ())
  {
    next_depth_msg_ = depth_msg;
    next_info_msg_ = info_msg;
    filter_condition_.notify_one();
  }
  filter_mutex_.unlock ();
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS (mesh_filter, DepthSelfFiltering, mesh_filter::DepthSelfFiltering, nodelet::Nodelet)
//PLUGINLIB_EXPORT_CLASS (mesh_filter::DepthSelfFiltering, nodelet::Nodelet);
