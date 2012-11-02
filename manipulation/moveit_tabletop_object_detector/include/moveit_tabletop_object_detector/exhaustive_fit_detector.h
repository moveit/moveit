/*********************************************************************
 * Software License Agreement (BSD License)
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

#ifndef _EXHAUSTIVE_FIT_DETECTOR_
#define _EXHAUSTIVE_FIT_DETECTOR_

#include <vector>
#include <set>
#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>

#include <sensor_msgs/PointCloud2.h>

#include <moveit_manipulation_msgs/GetModelList.h>
#include <moveit_manipulation_msgs/GetModelMesh.h>

#include <moveit_tabletop_object_detector/model_fitter.h>

namespace moveit_tabletop_object_detector {

//! Given a point cloud, computes the fit against multiple meshes and chooses the best ones
/*! Is templated on the type of individual fitter to use; the individual fitter must
  be able to store a mesh or point cloud inside of it, then later compute the fit to
  an input cloud. The individual fitter is designed to inherit from ModelToCloudFitter,
  defined inside of model_fitter.h
  
  This class just initializes a whole bunch of individual fitters, then, when given a 
  new cloud, tries to fit all of them. It will then return the fits with the best scores.  
*/
template <class Fitter>
class ExhaustiveFitDetector
{
 private:
  //! Node handle in the private namespace
  ros::NodeHandle priv_nh_;
  //! Node handle in the roote namespace
  ros::NodeHandle nh_;  
  //! Stores the individual model to cloud fitters, each initialized with a model
  std::vector<Fitter*> templates;

  //! Stores a list of model ids which may be in the list of templates, but which we should not look at
  std::set<int> model_exclusion_set_;
  bool negate_exclusions_;

 public:
  //! Just a stub; does not load models
 ExhaustiveFitDetector() : priv_nh_("~"), nh_(""), negate_exclusions_(false) {}
  //! Deletes any loaded models
  ~ExhaustiveFitDetector();

  void addModelToExclusionList(int model_id)
  {
    model_exclusion_set_.insert(model_id);
  }

  void clearExclusionList()
  {
    model_exclusion_set_.clear();
  }

  void setNegateExclusions(bool value)
  {
    negate_exclusions_ = value;
  }

  //! Loads all the models that are in the model database
  void loadDatabaseModels(std::string model_set);

  //! Main fitting function; fits all meshes against \a cloud and sorts the fits
  /*! Fits the point cloud \a cloud against all the models in the internal list.
    It always stores the list with at most \a numResults best fits, sorted by 
    their score. At the end, it returns this list.
    \param rotate true if we search for the optimal rotation as well
  */
  template <class PointCloudType>
  std::vector<ModelFitInfo> fitBestModels(const PointCloudType& cloud, int numResults)
  {
    std::vector<ModelFitInfo> fit_results;
    if (numResults <= 0) return fit_results;
    
    for (size_t i=0; i<templates.size(); ++i) 
    {
      ModelFitInfo current = templates[i]->template fitPointCloud<PointCloudType>(cloud);
      // If the model that was matched is not in the exclusion list
      bool found = (model_exclusion_set_.find(current.getModelId()) != model_exclusion_set_.end());
      if (negate_exclusions_ == found)
      {
        if ((int)fit_results.size() < numResults)
        {
          fit_results.push_back(current);
          std::sort(fit_results.begin(), fit_results.end(), ModelFitInfo::compareScores);
        }
        else
        {
          if (fit_results.back().getScore() > current.getScore())
          {
            fit_results.back() = current;
            std::sort(fit_results.begin(), fit_results.end(), ModelFitInfo::compareScores);
          }
        }
      }
    } 
    return fit_results;
  }
};

template <class Fitter>
ExhaustiveFitDetector<Fitter>::~ExhaustiveFitDetector()
{
  for (size_t i=0;i<templates.size();++i) {
    delete templates[i];
  }
}

/*! Loads all the models in the Model Database. In order to do that, it asks the
  database for a list of models, then asks for the path to the geometry file for
  each model. Then it initializes a IterativeDistanceFitter for each of them, and also sets
  the database model id correctly for each model so that we later know what model
  each instance of IterativeDistanceFitter refers to.

  WARNING: for the moment, it only uses the database models with the "orthographic"
  acquisition method. Those models are rotationally symmetric (which is what most fitters
  operating under this class are capable of handling) plus they do not have "filled insides"
  which makes them easier to grasp.
*/
template <class Fitter>
void ExhaustiveFitDetector<Fitter>::loadDatabaseModels(std::string model_set)
{
  std::string get_model_list_srv_name;
  priv_nh_.param<std::string>("get_model_list_srv", get_model_list_srv_name, "get_model_list_srv");
  while ( !ros::service::waitForService(get_model_list_srv_name, ros::Duration(2.0)) && nh_.ok() ) 
  {
    ROS_INFO("Waiting for %s service to come up", get_model_list_srv_name.c_str());
  }
  if (!nh_.ok()) exit(0);
  ros::ServiceClient get_model_list_srv = nh_.serviceClient<moveit_manipulation_msgs::GetModelList>
    (get_model_list_srv_name, true);


  std::string get_model_mesh_srv_name;
  priv_nh_.param<std::string>("get_model_mesh_srv", get_model_mesh_srv_name, "get_model_mesh_srv");
  while ( !ros::service::waitForService(get_model_mesh_srv_name, ros::Duration(2.0)) && nh_.ok() ) 
  {
    ROS_INFO("Waiting for %s service to come up", get_model_mesh_srv_name.c_str());
  }
  if (!nh_.ok()) exit(0);
  ros::ServiceClient get_model_mesh_srv = nh_.serviceClient<moveit_manipulation_msgs::GetModelMesh>
    (get_model_mesh_srv_name, true);


  moveit_manipulation_msgs::GetModelList model_list;
  model_list.request.model_set = model_set;
  if (!get_model_list_srv.call(model_list) || 
      model_list.response.return_code.code != model_list.response.return_code.SUCCESS )
  {
    ROS_ERROR("Could not retrieve list of models from database");
    return;
  }
  if (model_list.response.model_ids.empty()) 
  {
    ROS_ERROR("Empty model list retrieved from database");
    return;
  }

  ROS_INFO("Object detector: loading object models");
  for(size_t i=0; i<model_list.response.model_ids.size(); i++) 
  {
    int model_id = model_list.response.model_ids[i];

    moveit_manipulation_msgs::GetModelMesh get_mesh;
    get_mesh.request.model_id = model_id;
    if ( !get_model_mesh_srv.call(get_mesh) || 
	 get_mesh.response.return_code.code != get_mesh.response.return_code.SUCCESS )
    {
      ROS_ERROR("Failed to retrieve mesh for model %d", model_id);
      continue;
    }
    if (get_mesh.response.mesh.vertices.empty() || get_mesh.response.mesh.triangles.empty() )
    {
      ROS_ERROR("Empty mesh for model %d", model_id);
      continue;
    }

    Fitter* fitter = new Fitter();
    fitter->initializeFromMesh(get_mesh.response.mesh);
    templates.push_back(fitter);  
    //set the model ID in the template so that we can use it later
    templates.back()->setModelId( model_id );
    ROS_INFO("  Loaded database model with id %d", model_id);
  }
  ROS_INFO("Object detector: loading complete");
}

} //namespace

#endif
