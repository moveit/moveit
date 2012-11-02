
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
  
// Author(s): Marius Muja and Matei Ciocarlie

#include <string>

#include <ros/ros.h>

#include <sensor_msgs/PointCloud.h>

#include <visualization_msgs/Marker.h>

#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>

#include <moveit_manipulation_msgs/DatabaseModelPose.h>
#include <moveit_manipulation_msgs/DatabaseModelPoseList.h>
#include <moveit_manipulation_msgs/GetModelMesh.h>

#include <moveit_tabletop_object_detector/exhaustive_fit_detector.h>
#include <moveit_tabletop_object_detector/marker_generator.h>
#include <moveit_tabletop_object_detector/iterative_distance_fitter.h>
#include <moveit_manipulation_msgs/Table.h>
#include <moveit_manipulation_msgs/TabletopObjectRecognition.h>
#include <moveit_manipulation_msgs/ClearExclusionsList.h>
#include <moveit_manipulation_msgs/AddModelExclusion.h>
#include <moveit_manipulation_msgs/NegateExclusions.h>

namespace moveit_tabletop_object_detector {

class TabletopObjectRecognizer 
{
private:
  //! The node handle
  ros::NodeHandle nh_;
  //! Node handle in the private namespace
  ros::NodeHandle priv_nh_;
  //! Listener for incoming new-style point clouds
  ros::Subscriber cloud_new_sub_;
  //! Publisher for markers
  ros::Publisher marker_pub_;
  //! Service server for object detection
  ros::ServiceServer object_recognition_srv_;

  //! Service server for adding to the model exclusion set
  ros::ServiceServer add_model_exclusion_srv_;

  //! Service server for clearing the exclusion set
  ros::ServiceServer clear_model_exclusions_srv_;
  //! Service server for negating the exclusion set
  ros::ServiceServer negate_exclusions_srv_;
  //! Service client for getting a mesh from the database
  ros::ServiceClient get_model_mesh_srv_;

  //! Fit results below this quality are not published as markers
  double min_marker_quality_;
  //! Used to remember the number of markers we publish so we can delete them later
  int num_markers_published_;
  //! The current marker being published
  int current_marker_id_;

  //! The instance of the detector used for all detecting tasks
  ExhaustiveFitDetector<IterativeTranslationFitter> detector_;

  //! Whether to use a reduced model set from the database
  std::string model_set_;

  //! The threshold for merging two models that were fit very close to each other
  double fit_merge_threshold_;

  //! A tf transform listener
  tf::TransformListener listener_;

  //------------------ Callbacks -------------------

  //! Callback for service calls
  bool serviceCallback(moveit_manipulation_msgs::TabletopObjectRecognition::Request &request, 
                       moveit_manipulation_msgs::TabletopObjectRecognition::Response &response);
  bool addExclusionCB(moveit_manipulation_msgs::AddModelExclusion::Request &request, 
                      moveit_manipulation_msgs::AddModelExclusion::Response &response);
  bool clearExclusionsCB(moveit_manipulation_msgs::ClearExclusionsList::Request &request, 
                         moveit_manipulation_msgs::ClearExclusionsList::Response &response);
  bool negateExclusionsCB(moveit_manipulation_msgs::NegateExclusions::Request &request, 
                          moveit_manipulation_msgs::NegateExclusions::Response &response);

  //! Performs object detection on the given clusters, can also merge clusters based on detection result
  template <class PointCloudType>
  void objectDetection(std::vector<PointCloudType> &clusters, int num_models,
                       const moveit_manipulation_msgs::Table &table, bool perform_fit_merge,
                       moveit_manipulation_msgs::TabletopObjectRecognition::Response &response);

  //-------------------- Misc -------------------

  //! Helper function that returns the distance along the plane between two fit models
  double fitDistance(const ModelFitInfo &m1, const ModelFitInfo &m2);

  //! Helper function that returns the distance along the plane between a fit model and a cluster
  template <class PointCloudType>
  double fitClusterDistance(const ModelFitInfo &m, const PointCloudType &cluster);

  //! Publishes markers for all fits
  void publishFitMarkers(
                 const std::vector<moveit_manipulation_msgs::DatabaseModelPoseList> &potential_models,
                 const moveit_manipulation_msgs::Table &table);

  //! Clears old published markers and remembers the current number of published markers
  void clearOldMarkers(std::string frame_id);

public:
  //! Subscribes to and advertises topics; initializes fitter
  TabletopObjectRecognizer(ros::NodeHandle nh);

  //! Empty stub
  ~TabletopObjectRecognizer() {}
};

TabletopObjectRecognizer::TabletopObjectRecognizer(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
{
  num_markers_published_ = 1;
  current_marker_id_ = 1;
  
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>(nh_.resolveName("markers_out"), 10);
  
  std::string get_model_mesh_srv_name;
  priv_nh_.param<std::string>("get_model_mesh_srv", get_model_mesh_srv_name, "get_model_mesh_srv");
  while ( !ros::service::waitForService(get_model_mesh_srv_name, ros::Duration(2.0)) && nh_.ok() ) 
  {
    ROS_INFO("Waiting for %s service to come up", get_model_mesh_srv_name.c_str());
  }
  if (!nh_.ok()) exit(0);
  get_model_mesh_srv_ = nh_.serviceClient<moveit_manipulation_msgs::GetModelMesh>
    (get_model_mesh_srv_name, true);
  //ask fitter to load models from database
  priv_nh_.param<std::string>("model_set", model_set_, "");
  detector_.loadDatabaseModels(model_set_);
  
  object_recognition_srv_ = nh_.advertiseService(nh_.resolveName("object_recognition_srv"), 
                                                 &TabletopObjectRecognizer::serviceCallback, this);
  clear_model_exclusions_srv_ = priv_nh_.advertiseService("clear_exclusions_srv",
                                                          &TabletopObjectRecognizer::clearExclusionsCB, this);
  add_model_exclusion_srv_ = priv_nh_.advertiseService("add_exclusion_srv",
                                                       &TabletopObjectRecognizer::addExclusionCB, this);    
  negate_exclusions_srv_ = priv_nh_.advertiseService("negate_exclusions_srv",
                                                     &TabletopObjectRecognizer::negateExclusionsCB, this);
  
  //initialize operational flags
  priv_nh_.param<double>("fit_merge_threshold", fit_merge_threshold_, 0.05);
  priv_nh_.param<double>("min_marker_quality", min_marker_quality_, 0.003);
}

bool TabletopObjectRecognizer::clearExclusionsCB(moveit_manipulation_msgs::ClearExclusionsList::Request &request, 
                                                 moveit_manipulation_msgs::ClearExclusionsList::Response &response)
{
  ROS_INFO("Clearing exclusions list");
  detector_.clearExclusionList();
  return true;
}

bool TabletopObjectRecognizer::addExclusionCB(moveit_manipulation_msgs::AddModelExclusion::Request &request, 
                                              moveit_manipulation_msgs::AddModelExclusion::Response &response)
{
  ROS_INFO("Adding %d to exclusions list", request.model_id);
  detector_.addModelToExclusionList(request.model_id);
  return true;
}

bool TabletopObjectRecognizer::negateExclusionsCB(moveit_manipulation_msgs::NegateExclusions::Request &request, 
                                                  moveit_manipulation_msgs::NegateExclusions::Response &response)
{
  ROS_INFO("Setting exclusions negation to %s", request.negate ? "true" : "false");
  detector_.setNegateExclusions(request.negate);
  return true;
}

/*! Processes the latest point cloud and gives back the resulting array of models.
 */
bool TabletopObjectRecognizer::serviceCallback(moveit_manipulation_msgs::TabletopObjectRecognition::Request &request, 
                                               moveit_manipulation_msgs::TabletopObjectRecognition::Response &response)
{
  //convert point clouds to table frame
  tf::Transform table_trans;
  tf::poseMsgToTF(request.table.pose.pose, table_trans);
  tf::StampedTransform table_trans_stamped(table_trans, request.table.pose.header.stamp, 
                                           request.table.pose.header.frame_id, "table_frame");
  tf::TransformListener listener;
  listener.setTransform(table_trans_stamped);
  for (size_t i = 0; i < request.clusters.size (); ++i)
  {
    if (request.clusters[i].header.frame_id != request.table.pose.header.frame_id)
    {
      ROS_ERROR("Recognition node requires all clusters to be in the same frame as the table");
      return false;
    }
    request.clusters[i].header.stamp = request.table.pose.header.stamp;
    try
    {
      listener.transformPointCloud("table_frame", request.clusters[i], request.clusters[i]); 
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("Recognition node failed to transform cluster from frame %s into table frame; Exception: %s", 
                request.clusters[i].header.frame_id.c_str(), ex.what());
      return false;
    }
  }
  ROS_INFO("Clusters converted to table frame");

  //run detection
  objectDetection<sensor_msgs::PointCloud>(request.clusters, request.num_models, request.table, request.perform_fit_merge, response);

  publishFitMarkers(response.models, request.table);
  clearOldMarkers(request.table.pose.header.frame_id);

  return true;
}

/*! Performs the detection on each of the clusters, and populates the returned message.
*/
template <class PointCloudType>
void TabletopObjectRecognizer::objectDetection(std::vector<PointCloudType> &clusters, 
                                               int num_models,
                                               const moveit_manipulation_msgs::Table &table,
                                               bool perform_fit_merge,
                                               moveit_manipulation_msgs::TabletopObjectRecognition::Response &response)
{
  //do the model fitting part
  ROS_INFO("Fitting models to clusters");
  std::vector< std::vector<ModelFitInfo> > raw_fit_results;
  response.cluster_model_indices.resize( clusters.size(), -1 );
  for (size_t i=0; i<clusters.size(); i++) 
  {
    raw_fit_results.push_back( detector_.fitBestModels<PointCloudType>(clusters[i], std::max(1,num_models)) );
    response.cluster_model_indices[i] = i;
  }
  ROS_INFO("Raw fit results computed");

  //merge models that were fit very close to each other
  if (perform_fit_merge)
  {
    size_t i=0;
    while (i < clusters.size())
    {
      //if cluster i has already been merged continue
      if (response.cluster_model_indices[i] != (int)i || raw_fit_results.at(i).empty()) 
      {
	i++;
	continue;
      }

      size_t j;
      for (j=i+1; j<clusters.size(); j++)
      {
	//if cluster j has already been merged continue
	if (response.cluster_model_indices[j] != (int)j) continue;
	//if there are no fits, merge based on cluster vs. fit
	if (raw_fit_results.at(j).empty()) 
	{
	  if ( fitClusterDistance<PointCloudType>( raw_fit_results.at(i).at(0), 
						   clusters.at(j) ) < fit_merge_threshold_ ) break;
	  else continue;
	}
	//else merge based on fits
	if ( fitDistance(raw_fit_results.at(i).at(0), raw_fit_results.at(j).at(0)) < fit_merge_threshold_) break;
      }
      if (j<clusters.size())
      {
	ROS_INFO("Post-fit merging of clusters %u and %u", (unsigned int) i, (unsigned int) j);
	//merge cluster j into i
	clusters[i].points.insert( clusters[i].points.end(), clusters[j].points.begin(), clusters[j].points.end() );
	//delete fits for cluster j so we ignore it from now on
	raw_fit_results.at(j).clear();
	//fits for cluster j now point at fit for cluster i
	response.cluster_model_indices[j] = i;
	//refit cluster i
	raw_fit_results.at(i) = detector_.fitBestModels(clusters[i], std::max(1,num_models));
      }
      else
      {
	i++;
      }
    }
  }
  ROS_INFO("Post-fit merge completed");

  //make sure raw clusters point at the right index in fit_models
  for (size_t i=0; i<raw_fit_results.size(); i++)
  {
    if (response.cluster_model_indices[i] != (int)i)
    {
      int ind = response.cluster_model_indices[i];
      ROS_ASSERT( ind < (int)i);
      response.cluster_model_indices[i] = response.cluster_model_indices[ind];
      ROS_INFO("  - has been merged with fit for cluster %d", ind);
    }
  }
  ROS_INFO("Clustered indices arranges");

  tf::Transform table_trans;
  tf::poseMsgToTF(table.pose.pose, table_trans);
  for (size_t i=0; i<raw_fit_results.size(); i++)
  {
    moveit_manipulation_msgs::DatabaseModelPoseList model_potential_fit_list;
    //prepare the actual result for good fits, only these are returned
    for (size_t j=0; j < raw_fit_results[i].size(); j++)
    {
      //get the model pose in the cloud frame by multiplying with table transform
      tf::Transform model_trans;
      tf::poseMsgToTF(raw_fit_results[i][j].getPose(), model_trans);
      model_trans = table_trans * model_trans;
      geometry_msgs::Pose model_pose;
      tf::poseTFToMsg(model_trans, model_pose);
      //create the model fit result
      moveit_manipulation_msgs::DatabaseModelPose pose_msg;
      pose_msg.model_id = raw_fit_results[i][j].getModelId();
      pose_msg.pose.header = table.pose.header;
      pose_msg.pose.pose = model_pose;
      pose_msg.confidence = raw_fit_results[i][j].getScore();
      //and push it in the list for this cluster
      model_potential_fit_list.model_list.push_back(pose_msg);
    }
    response.models.push_back(model_potential_fit_list);
  }
  ROS_INFO("Results ready");

}

void TabletopObjectRecognizer::publishFitMarkers(const std::vector<moveit_manipulation_msgs::DatabaseModelPoseList> &potential_models,
                                                 const moveit_manipulation_msgs::Table &table)
{
  for (size_t i=0; i<potential_models.size(); i++)
  {
    const std::vector<moveit_manipulation_msgs::DatabaseModelPose> models = potential_models[i].model_list;
    for (size_t j=0; j<models.size(); j++)
    {
      if (models[j].confidence > min_marker_quality_) break;
      moveit_manipulation_msgs::GetModelMesh get_mesh;
      get_mesh.request.model_id = models[j].model_id;
      if ( !get_model_mesh_srv_.call(get_mesh) ||
           get_mesh.response.return_code.code != get_mesh.response.return_code.SUCCESS )
      {
        ROS_ERROR("tabletop_object_detector: failed to call database get mesh service for marker display");
      }
      else
      {
        double rank = ((double)j) / std::max( (int)(models.size())-1, 1 );
        visualization_msgs::Marker fitMarker =  MarkerGenerator::getFitMarker(get_mesh.response.mesh, rank);
        fitMarker.header = table.pose.header;
        fitMarker.pose = models[j].pose.pose;
        fitMarker.ns = "tabletop_node_model_" + boost::lexical_cast<std::string>(j);
        fitMarker.id = current_marker_id_++;
        marker_pub_.publish(fitMarker);
      }  
    }
  }
}

void TabletopObjectRecognizer::clearOldMarkers(std::string frame_id)
{
  for (int id=current_marker_id_; id < num_markers_published_; id++)
  {
    visualization_msgs::Marker delete_marker;
    delete_marker.header.stamp = ros::Time::now();
    delete_marker.header.frame_id = frame_id;
    delete_marker.id = id;
    delete_marker.action = visualization_msgs::Marker::DELETE;
    //a hack, but we don't know what namespace the marker was in
    for (size_t j=0; j<10; j++)
    {
      delete_marker.ns = "tabletop_node_model_" + boost::lexical_cast<std::string>(j);
      marker_pub_.publish(delete_marker);
    }
  }
  num_markers_published_ = current_marker_id_;
  current_marker_id_ = 0;
}

double TabletopObjectRecognizer::fitDistance(const ModelFitInfo &m1, const ModelFitInfo &m2)
{
  double dx = m1.getPose().position.x - m2.getPose().position.x;
  double dy = m1.getPose().position.y - m2.getPose().position.y;
  double d = dx*dx + dy*dy;
  return sqrt(d);
}

template <class PointCloudType>
double TabletopObjectRecognizer::fitClusterDistance(const ModelFitInfo &m, const PointCloudType &cluster)
{
  double dist = 100.0 * 100.0;
  double mx = m.getPose().position.x;
  double my = m.getPose().position.x;
  for (size_t i=0; i<cluster.points.size(); i++)
  {
    double dx = cluster.points[i].x - mx;
    double dy = cluster.points[i].y - my;
    double d = dx*dx + dy*dy;
    dist = std::min(d, dist);
  }
  return sqrt(dist);
}

} //namespace tabletop_object_detector

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "tabletop_object_recognition");
  ros::NodeHandle nh;

  moveit_tabletop_object_detector::TabletopObjectRecognizer node(nh);
  ros::spin();
  return 0;
}
