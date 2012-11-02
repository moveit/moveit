
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

#include <moveit_manipulation_msgs/TabletopDetection.h>
#include <moveit_manipulation_msgs/TabletopSegmentation.h>
#include <moveit_manipulation_msgs/TabletopObjectRecognition.h>

namespace moveit_tabletop_object_detector {

class TabletopCompleteNode
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;

  ros::ServiceClient seg_srv_;
  ros::ServiceClient rec_srv_;
  ros::ServiceServer complete_srv_;
  
  //! Whether to perform a merge step based on model fit results
  bool perform_fit_merge_;

  bool serviceCallback(moveit_manipulation_msgs::TabletopDetection::Request &request, 
                       moveit_manipulation_msgs::TabletopDetection::Response &response);

public:
  TabletopCompleteNode();
};

TabletopCompleteNode::TabletopCompleteNode() : nh_(""), priv_nh_("~")
{
  std::string service_name;

  priv_nh_.param<std::string>("segmentation_srv", service_name, "/tabletop_segmentation");
  while ( !ros::service::waitForService(service_name, ros::Duration(2.0)) && nh_.ok() ) 
  {
    ROS_INFO("Waiting for %s service to come up", service_name.c_str());
  }
  if (!nh_.ok()) exit(0);
  seg_srv_ = nh_.serviceClient<moveit_manipulation_msgs::TabletopSegmentation>(service_name, true);

  priv_nh_.param<std::string>("recognition_srv", service_name, "/tabletop_object_recognition");
  while ( !ros::service::waitForService(service_name, ros::Duration(2.0)) && nh_.ok() ) 
  {
    ROS_INFO("Waiting for %s service to come up", service_name.c_str());
  }
  if (!nh_.ok()) exit(0);
  rec_srv_ = nh_.serviceClient<moveit_manipulation_msgs::TabletopObjectRecognition>(service_name, true);
  
  complete_srv_ = nh_.advertiseService("object_detection", &TabletopCompleteNode::serviceCallback, this);
  ROS_INFO("Tabletop complete node ready");

  priv_nh_.param<bool>("perform_fit_merge", perform_fit_merge_, true);
}

bool TabletopCompleteNode::serviceCallback(moveit_manipulation_msgs::TabletopDetection::Request &request, 
                                           moveit_manipulation_msgs::TabletopDetection::Response &response)
{
  moveit_manipulation_msgs::TabletopSegmentation segmentation_srv;
  if (!seg_srv_.call(segmentation_srv))
  {
    ROS_ERROR("Call to segmentation service failed");
    response.detection.result = response.detection.OTHER_ERROR;
    return true;
  }
  response.detection.result = segmentation_srv.response.result;
  if (segmentation_srv.response.result != segmentation_srv.response.SUCCESS)
  {
    ROS_ERROR("Segmentation service returned error %d", segmentation_srv.response.result);
    return true;
  }
  ROS_INFO("Segmentation service succeeded. Detected %d clusters", (int)segmentation_srv.response.clusters.size());
  response.detection.table = segmentation_srv.response.table;
  response.detection.clusters = segmentation_srv.response.clusters;
  if (segmentation_srv.response.clusters.empty() || !request.return_models) return true;

  moveit_manipulation_msgs::TabletopObjectRecognition recognition_srv;
  recognition_srv.request.table = segmentation_srv.response.table;
  recognition_srv.request.clusters = segmentation_srv.response.clusters;
  recognition_srv.request.num_models = request.num_models;
  recognition_srv.request.perform_fit_merge = perform_fit_merge_;
  if (!rec_srv_.call(recognition_srv))
  {
    ROS_ERROR("Call to recognition service failed");
    response.detection.result = response.detection.OTHER_ERROR;
    return true;
  }
  response.detection.models = recognition_srv.response.models;
  response.detection.cluster_model_indices = recognition_srv.response.cluster_model_indices;
  return true;
} 

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tabletop_node");
  ros::NodeHandle nh;
  moveit_tabletop_object_detector::TabletopCompleteNode node;
  ros::spin();
  return true;
}
