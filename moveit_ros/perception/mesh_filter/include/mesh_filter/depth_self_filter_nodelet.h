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
#pragma once
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/condition_variable.hpp>
#include <mesh_filter/transform_provider.h>
#include <mesh_filter/mesh_filter.h>
#include <mesh_filter/stereo_camera_model.h>


namespace mesh_filter
{
  
/**
 * \brief Nodelet for filtering meshes from depth images. e.g. meshes of the robot or any attached object where 
 * a transformation can be provided for.
 * \author Suat Gedikli (gedikli@willowgarage.com)
 */
class DepthSelfFiltering : public nodelet::Nodelet
{
  public:
    /** \brief Nodelet init callback*/
    virtual void onInit ();
    
  private:
    ~DepthSelfFiltering ();
    
    /**
     * \brief adding the meshes to a given mesh filter object.
     * \param[in,out] mesh_filter mesh filter object that gets meshes from the robot description added to
     * \author Suat Gedikli (gedikli@willowgarage.com)
     */
    void addMeshes (mesh_filter::MeshFilter<mesh_filter::StereoCameraModel>& mesh_filter);
    
    /**
     * \brie entry point of filtering thread
     * \author Suat Gedikli (gedikli@willowgarage.com)
     */
    void filter ();
    
    /**
     * \brief Callback for connection/deconnection of listener
     * \author Suat Gedikli (gedikli@willowgarage.com)
     */
    void connectCb();
    
    /**
     * \brief Callback for subscribed depth images
     * \author Suat Gedikli (gedikli@willowgarage.com)
     * @param depth_msg depth image
     * @param info_msg camera information containing parameters frame, etc.
     */
    void depthCb(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
    
  private:
    // member variables to handle ros messages
    boost::shared_ptr<image_transport::ImageTransport> input_depth_transport_;
    boost::shared_ptr<image_transport::ImageTransport> filtered_label_transport_;
    boost::shared_ptr<image_transport::ImageTransport> filtered_depth_transport_;
    boost::shared_ptr<image_transport::ImageTransport> model_depth_transport_;
    boost::shared_ptr<image_transport::ImageTransport> model_label_transport_;
    image_transport::CameraSubscriber sub_depth_image_;
    image_transport::CameraPublisher pub_filtered_depth_image_;
    image_transport::CameraPublisher pub_filtered_label_image_;
    image_transport::CameraPublisher pub_model_depth_image_;
    image_transport::CameraPublisher pub_model_label_image_;
    
    /** \brief required to avoid listener registration before we are all set*/
    boost::mutex connect_mutex_;
    int queue_size_;
    TransformProvider transform_provider_;
    // filtering members
    mutable sensor_msgs::ImageConstPtr next_depth_msg_;
    mutable sensor_msgs::ImageConstPtr current_depth_msg_;
    mutable sensor_msgs::CameraInfoConstPtr next_info_msg_;
    mutable sensor_msgs::CameraInfoConstPtr current_info_msg_;
    
    /** \brief distance of near clipping plane*/
    double near_clipping_plane_distance_;
    
    /** \brief distance of far clipping plane*/
    double far_clipping_plane_distance_;
    
    /** \brief threshold that indicates a pixel to be in shadow, rather than being filtered out */
    double shadow_threshold_;
    
    /** \brief the coefficient for the square component of padding function in  1/m*/
    double padding_scale_;
    
    /** \brief the coefficient for the linear component of the padding function*/
    double padding_offset_;
    
    /** \brief thread object for dedicated filtering thread*/
    boost::thread filter_thread_;
    
    /** \brief condition variable to notify the filtering thread if a new image arrived*/
    boost::condition_variable filter_condition_;
    
    /** \brief mutex required for synchronization of condition states*/
    boost::mutex filter_mutex_;
    
    /** \brief indicates whether the filtering loop should stop*/
    bool stop_;
};

} //namespace mesh_filter
