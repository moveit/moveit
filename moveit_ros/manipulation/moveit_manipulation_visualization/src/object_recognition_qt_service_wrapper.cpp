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

// Author: E. Gil Jones

#include <QMetaType>

#include <moveit_manipulation_visualization/object_recognition_qt_service_wrapper.h>
#include <moveit_manipulation_msgs/TabletopSegmentation.h>
#include <moveit_manipulation_msgs/TabletopObjectRecognition.h>
#include <sensor_msgs/point_cloud_conversion.h>

namespace moveit_manipulation_visualization
{

ObjectRecognitionQtServiceWrapper::ObjectRecognitionQtServiceWrapper() {
  qRegisterMetaType<sensor_msgs::PointCloud>("PointCloud");
  qRegisterMetaType<moveit_manipulation_msgs::Table>("moveit_manipulation_msgs::Table");
  qRegisterMetaType<moveit_manipulation_msgs::DatabaseModelPoseList>("moveit_manipulation_msgs::DatabaseModelPoseList");
}

bool ObjectRecognitionQtServiceWrapper::connectToSegmentationService() 
{
  if(seg_srv_) return true;
  if(!ros::service::waitForService("/tabletop_segmentation", ros::Duration(1.0))) {
    ROS_INFO_STREAM("No service for tabletop segmentation available");
    return false;
  }
  ros::NodeHandle nh;
  seg_srv_ = nh.serviceClient<moveit_manipulation_msgs::TabletopSegmentation>("/tabletop_segmentation", true);
  return true;
}

bool ObjectRecognitionQtServiceWrapper::connectToRecognitionService() 
{
  if(rec_srv_) return true;
  if(!ros::service::waitForService("/tabletop_object_recognition", ros::Duration(1.0))) {
    ROS_INFO_STREAM("No service for tabletop recognition available");
    return false;
  }
  ros::NodeHandle nh;
  rec_srv_ = nh.serviceClient<moveit_manipulation_msgs::TabletopObjectRecognition>("/tabletop_object_recognition", true);
  return true;
}


void ObjectRecognitionQtServiceWrapper::segmentAndCluster() {
  if(!connectToSegmentationService()) {
    return;
  }
  moveit_manipulation_msgs::TabletopSegmentation segmentation_srv;
  if (!seg_srv_.call(segmentation_srv))
  {
    ROS_ERROR("Call to segmentation service failed");
    return;
  }
  // std::vector<sensor_msgs::PointCloud2> conv_clouds(segmentation_srv.response.clusters.size());
  // for(unsigned int i = 0; i < conv_clouds.size(); i++) {
  //   sensor_msgs::convertPointCloudToPointCloud2(segmentation_srv.response.clusters[i], conv_clouds[i]);
  // }
  Q_EMIT tableAndClustersGenerated(segmentation_srv.response.table, segmentation_srv.response.clusters);
}

void ObjectRecognitionQtServiceWrapper::recognize(moveit_manipulation_msgs::Table table,
                                                  std::vector<sensor_msgs::PointCloud> clusters)
{
  if(!connectToRecognitionService()) {
    return;
  }
  moveit_manipulation_msgs::TabletopObjectRecognition recognition_srv;
  recognition_srv.request.table = table;
  recognition_srv.request.clusters = clusters;
  recognition_srv.request.num_models = 1;
  recognition_srv.request.perform_fit_merge = false;
  if (!rec_srv_.call(recognition_srv))
  {
    ROS_ERROR("Call to recognition service failed");
    //response.detection.result = response.detection.OTHER_ERROR;
  }
  ROS_INFO_STREAM("Got " << recognition_srv.response.models.size() << " models");

  Q_EMIT objectsRecognized(recognition_srv.response.models,
                           recognition_srv.response.cluster_model_indices);

}

}
