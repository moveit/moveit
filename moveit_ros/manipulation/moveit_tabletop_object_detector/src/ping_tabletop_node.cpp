
/*********************************************************************
*
*  Copyright (c) 2009, Willow Garage, Inc.
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

// Author(s): Matei Ciocarlie

#include <ros/ros.h>

#include <string>
#include <vector>

#include <moveit_tabletop_object_detector/TabletopSegmentation.h>
#include <moveit_tabletop_object_detector/TabletopObjectRecognition.h>

/*! Simply pings the tabletop segmentation and recognition services and prints out the result.*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "ping_tabletop_node");
  ros::NodeHandle nh;

  std::string service_name("/tabletop_segmentation");
  while ( !ros::service::waitForService(service_name, ros::Duration().fromSec(3.0)) && nh.ok() )
  {
    ROS_INFO("Waiting for service %s...", service_name.c_str());
  }
  if (!nh.ok()) exit(0);

  tabletop_object_detector::TabletopSegmentation segmentation_srv;
  if (!ros::service::call(service_name, segmentation_srv))
  {
    ROS_ERROR("Call to segmentation service failed");
    exit(0);
  }
  if (segmentation_srv.response.result != segmentation_srv.response.SUCCESS)
  {
    ROS_ERROR("Segmentation service returned error %d", segmentation_srv.response.result);
    exit(0);
  }
  ROS_INFO("Segmentation service succeeded. Detected %d clusters", (int)segmentation_srv.response.clusters.size());
  if (segmentation_srv.response.clusters.empty()) exit(0);

  service_name = "/tabletop_object_recognition";
  if ( !ros::service::waitForService(service_name, ros::Duration().fromSec(3.0)) )
  {
    ROS_INFO("Recognition service %s is not available", service_name.c_str());
    exit(0);
  }

  tabletop_object_detector::TabletopObjectRecognition recognition_srv;
  recognition_srv.request.table = segmentation_srv.response.table;
  recognition_srv.request.clusters = segmentation_srv.response.clusters;
  recognition_srv.request.num_models = 5;
  if (!ros::service::call(service_name, recognition_srv))
  {
    ROS_ERROR("Call to recognition service failed");
    exit(0);
  }

  ROS_INFO("Recognition results:");
  for (size_t i=0; i<recognition_srv.response.models.size(); i++)
  {
    if (recognition_srv.response.models[i].model_list.empty())
    {
      ROS_INFO("  Unidentifiable cluster");
    }
    else
    {
      ROS_INFO("  Model id %d",recognition_srv.response.models[i].model_list[0].model_id);
    }
  }

  ROS_INFO("Clusters correspond to following recognition results:");
  for (size_t i=0; i<recognition_srv.response.cluster_model_indices.size(); i++)
  {
    ROS_INFO("  Cluster %u: recognition result %d", (unsigned int)i, 
             recognition_srv.response.cluster_model_indices.at(i));
  }
  return true;
}
