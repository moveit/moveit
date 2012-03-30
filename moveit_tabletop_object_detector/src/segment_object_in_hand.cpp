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
  
// Author(s): Kaijen Hsiao

#include <string>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/io.h>
#include "pcl/io/pcd_io.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl_ros/transforms.h>

#include "tabletop_object_detector/SegmentObjectInHand.h"

namespace tabletop_object_detector {

class ObjectInHandSegmenter
{
  typedef pcl::PointXYZRGB    Point;
  typedef pcl::search::KdTree<Point>::Ptr KdTreePtr;

public:
  ObjectInHandSegmenter(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
  {
    // Point cloud passthrough filter params (wrist frame)
    double box_w = 0.3;
    priv_nh_.param<double>("x_filter_min", x_filter_min_, -(box_w - 0.185));
    priv_nh_.param<double>("x_filter_max", x_filter_max_, box_w + 0.185);
    priv_nh_.param<double>("y_filter_min", y_filter_min_, -box_w);
    priv_nh_.param<double>("y_filter_max", y_filter_max_, box_w);
    priv_nh_.param<double>("z_filter_min", z_filter_min_, -box_w);
    priv_nh_.param<double>("z_filter_max", z_filter_max_, box_w);

    // Params for where we want the clusters from (containing points within dist from (x,y,z) in wrist_frame)
    priv_nh_.param<double>("object_center_x", object_center_x_, 0.185);
    priv_nh_.param<double>("object_center_y", object_center_y_, 0);
    priv_nh_.param<double>("object_center_z", object_center_z_, 0);
    priv_nh_.param<double>("max_dist_from_center", max_dist_from_center_, 0.10);

    // Euclidean clustering params
    priv_nh_.param<double>("cluster_distance", cluster_distance_, 0.025);
    priv_nh_.param<double>("min_cluster_size", min_cluster_size_, 0.025);
    priv_nh_.param<std::string>("self_filtered_cloud_name", self_filtered_cloud_name_, "/narrow_stereo_textured/object_modeling_points_filtered");

    // Publisher for resulting cluster
    priv_nh_.param<std::string>("visualization_topic", visualization_topic_, "segment_object_in_hand_output_cluster");
    pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>(visualization_topic_, 1);

    // Service for performing segmentation
    segmentation_srv_ = nh_.advertiseService(nh_.resolveName("segment_object_in_hand_srv"), 
                                             &ObjectInHandSegmenter::serviceCallback, this);

  }

private:

  ros::NodeHandle nh_, priv_nh_;
  tf::TransformListener listener_;

  // For filtering of point cloud in the gripper frame
  double x_filter_min_, x_filter_max_;
  double y_filter_min_, y_filter_max_;
  double z_filter_min_, z_filter_max_;

  // Point cloud subscriber and publisher
  ros::Subscriber sub_cloud_;
  ros::Publisher pub_cloud_;
  std::string visualization_topic_;

  // Euclidean clustering params
  double cluster_distance_;
  double min_cluster_size_;
  std::string self_filtered_cloud_name_;
  
  // Cluster location params
  double object_center_x_, object_center_y_, object_center_z_;
  double max_dist_from_center_;

  // Service server for segmenting object in hand
  ros::ServiceServer segmentation_srv_;

  /*! 
   * Service callback for segment object in hand service
   */
  bool serviceCallback(SegmentObjectInHand::Request &request, 
					   SegmentObjectInHand::Response &response)
  {
    ROS_DEBUG("Got service request for segment_object_in_hand");
    int result = segmentObject(request.wrist_frame, response.cluster);
    if(result == 0) response.result = response.SUCCESS;
    else if(result == 1) response.result = response.NO_CLOUD_RECEIVED;
    else if(result == 2) response.result = response.TF_ERROR;
    else response.result = response.OTHER_ERROR;
    return true;
  }


  /*!
   * find the connected points for an object in the robot's hand based on the next point cloud received
   */
  int segmentObject(std::string wrist_frame, sensor_msgs::PointCloud2 &output_cluster)
  {
    ROS_DEBUG("waiting for self-filtered point cloud");
    pcl::PointCloud<Point>::Ptr input_cloud(new pcl::PointCloud<Point>());
	
    // wait for the next PointCloud2 message 
    sensor_msgs::PointCloud2ConstPtr cloud_msg = 
      ros::topic::waitForMessage<sensor_msgs::PointCloud2>(self_filtered_cloud_name_, 
							   nh_, ros::Duration(5.0));
    if(!cloud_msg)
    {
      ROS_ERROR("No point cloud received!");
      return 1;
    }

    ROS_DEBUG("Point cloud received; processing");
	pcl::PointCloud<Point> wrist_frame_cloud;

    if (!wrist_frame.empty())
    {
      // convert to a pcl::PointCloud
      pcl::fromROSMsg(*cloud_msg, *input_cloud);

      // downsample the cloud
      pcl::VoxelGrid<Point> vox;
      vox.setInputCloud(input_cloud);
      vox.setLeafSize(0.005, 0.005, 0.005);
      vox.filter(*input_cloud);

      // Convert cloud to wrist frame
      try
      {
	pcl_ros::transformPointCloud(wrist_frame, *input_cloud, wrist_frame_cloud, listener_);
      }
      catch (tf::TransformException ex)
      {
	ROS_ERROR("Failed to transform cloud from frame %s into frame %s",
		  input_cloud->header.frame_id.c_str(), wrist_frame.c_str());
	return 2;
      }
      ROS_DEBUG("Input cloud converted to %s frame", wrist_frame.c_str());
    }
    else
    {
      ROS_ERROR("no wrist_frame specified!");
      return 3;
    }
    
    // Passthrough to get rid of points not in range
    pcl::PassThrough<Point> pass_;
    pcl::PointCloud<Point> cloud_filtered;
    pcl::PointCloud<Point>::ConstPtr cloud_ptr = boost::make_shared<const pcl::PointCloud<Point> > (wrist_frame_cloud);
    pass_.setInputCloud (cloud_ptr);
    pass_.setFilterFieldName ("z");
    pass_.setFilterLimits (z_filter_min_, z_filter_max_);
    pass_.filter (wrist_frame_cloud);
    cloud_ptr = boost::make_shared<const pcl::PointCloud<Point> > (wrist_frame_cloud);
    pass_.setInputCloud (cloud_ptr);
    pass_.setFilterFieldName ("y");
    pass_.setFilterLimits (y_filter_min_, y_filter_max_);
    pass_.filter (wrist_frame_cloud);
    cloud_ptr = boost::make_shared<const pcl::PointCloud<Point> > (wrist_frame_cloud);
    pass_.setInputCloud (cloud_ptr);
    pass_.setFilterFieldName ("x");
    pass_.setFilterLimits (x_filter_min_, x_filter_max_);
    pass_.filter (wrist_frame_cloud);
    cloud_ptr = boost::make_shared<const pcl::PointCloud<Point> > (wrist_frame_cloud);

    if(wrist_frame_cloud.points.size() == 0){
      ROS_INFO("no points left after passthrough!");
      return 3;
    }

    // sensor_msgs::PointCloud2 debug_cloud;
    // pcl::toROSMsg(wrist_frame_cloud, debug_cloud);
    // pub_cloud_.publish(debug_cloud);

    // Find cluster indices
    pcl::EuclideanClusterExtraction<Point> pcl_cluster_;
    KdTreePtr clusters_tree_;
    std::vector<pcl::PointIndices> clusters;
    clusters_tree_ = boost::make_shared<pcl::search::KdTree<Point> > ();
    pcl_cluster_.setClusterTolerance (cluster_distance_);
    pcl_cluster_.setMinClusterSize (min_cluster_size_);
    pcl_cluster_.setSearchMethod (clusters_tree_); 
    pcl_cluster_.setInputCloud (cloud_ptr);
    pcl_cluster_.extract (clusters);

    // Combine all clusters that are close to the gripper and add points for the indices to the output_cluster_pcl
    pcl::PointCloud<Point> output_cluster_pcl;
    if(clusters.size() != 0)
    {
      combineClustersNearPoint(clusters, wrist_frame_cloud, wrist_frame, object_center_x_, object_center_y_, object_center_z_, max_dist_from_center_, output_cluster_pcl); 
      if(output_cluster_pcl.points.size() == 0)
      {
	ROS_WARN("No points in resulting object cloud!");
      }

      // convert back to PointCloud2
      pcl::toROSMsg(output_cluster_pcl, output_cluster);
      output_cluster.header.frame_id = wrist_frame;
      output_cluster.header.stamp = ros::Time::now();
      
      // Publish the resulting PointCloud2 for visualization
      pub_cloud_.publish(output_cluster);
      
    }
    else
    {
      ROS_WARN("No clusters detected!");
    }
    return 0;
  }

  /*!
   * Combine all clusters with points within dist of (x,y,z) in wrist_frame
   */
  void combineClustersNearPoint(std::vector<pcl::PointIndices> clusters, pcl::PointCloud<Point> input_cloud, std::string wrist_frame, double x, double y, double z, double dist, pcl::PointCloud<Point> &output_cluster)
  {
    output_cluster.header.stamp = ros::Time(0);
    for(size_t cluster_ind=0; cluster_ind < clusters.size(); cluster_ind++)
    {
      ROS_DEBUG("cluster_ind: %d, size: %d\n", (int)cluster_ind, (int)clusters[cluster_ind].indices.size());

      // Check to see if the cluster has points within range
      double point_dist;
      double xdiff, ydiff, zdiff;
      bool cluster_good = 0;
      for (size_t j = 0; j < clusters[cluster_ind].indices.size(); ++j)
      {
	xdiff = x - input_cloud.points[clusters[cluster_ind].indices[j]].x;
	ydiff = y - input_cloud.points[clusters[cluster_ind].indices[j]].y;
	zdiff = z - input_cloud.points[clusters[cluster_ind].indices[j]].z;
	point_dist = sqrt(xdiff*xdiff + ydiff*ydiff + zdiff*zdiff);
	if(point_dist < dist)
	{
	  ROS_DEBUG("keeping cluster\n");
	  cluster_good = 1;
	  break;
	}
      }

      // If so, add this cluster to the resulting point cloud
      if(cluster_good)
      {
	double current_size = output_cluster.points.size();
	output_cluster.points.resize(current_size + clusters[cluster_ind].indices.size());
	for (size_t k= 0; k < clusters[cluster_ind].indices.size(); ++k)
	{
	  output_cluster.points[current_size + k].x = input_cloud.points[clusters[cluster_ind].indices[k]].x;
	  output_cluster.points[current_size + k].y = input_cloud.points[clusters[cluster_ind].indices[k]].y;
	  output_cluster.points[current_size + k].z = input_cloud.points[clusters[cluster_ind].indices[k]].z;
	}
      }
    }
    ROS_DEBUG("output_cluster size: %d\n", (int)output_cluster.points.size()); 
  }

};

} //namespace tabletop_object_detector

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "segment_object_in_hand_node");
  ros::NodeHandle nh;

  tabletop_object_detector::ObjectInHandSegmenter node(nh);
  ROS_INFO("Segment object in hand ready for service requests");
  ros::spin();
  return 0;
}
