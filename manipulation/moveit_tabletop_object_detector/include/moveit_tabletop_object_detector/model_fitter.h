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

#ifndef _MODEL_FITTER_H_
#define _MODEL_FITTER_H_

#include <string>
#include <vector>
#include <algorithm>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometric_shapes/eigen_types.h>

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>

#include <sensor_msgs/PointCloud.h>
#include <geometric_shapes/shape_messages.h>

namespace distance_field {
  class PropagationDistanceField;
}

namespace moveit_tabletop_object_detector {

//! Holds information about the fit betwen a mesh and a point cloud
/*! A helper class for storing information about a fit between a model and a 
  cloud. The cloud is not explicitly stored.

  Relies on the default copy operator, so be careful about what you put in 
  here.
*/
class ModelFitInfo {
 private:
  //! The database id of the model that was fit, if loaded from a database
  int model_id_;
  //! The tranform of the fit itself
  geometry_msgs::Pose fit_pose_;
  //! The score of the fit
  float score_;

 public:
  //! Populates this instance with all the relevant information
  ModelFitInfo(int model_id, const geometry_msgs::Pose &pose, float score) : model_id_(model_id), 
    fit_pose_(pose), score_(score) {}

  //! Return the model if of this fit
  int getModelId() const {return model_id_;}
  //! Gets the score of the fit
  float getScore() const {return score_;} 
  //! Returns the transform of the fit
  geometry_msgs::Pose getPose() const {return fit_pose_;}

  //! Helper function for sorting based on scores
  static bool compareScores(const ModelFitInfo &mf1, const ModelFitInfo &mf2)
  {
    return mf1.score_ < mf2.score_;
  }  
};

//! The interface for a class that can fit an individual mesh to a point cloud
/*! Must be able to:
  - initialize itself from a mesh or point cloud
  - fit a new point cloud to the internal model
  - return a fit result as a ModelFitInfo

  Inherit from this class if you have a new awesome fitting method.
 */
class ModelToCloudFitter
{
 protected:
  //! The id of this model if loaded from the model database
  int model_id_;

  void sampleMesh(const shape_msgs::Mesh &mesh, 
                  EigenSTL::vector_Vector3d& points,
                  double resolution);

 public:

  //! Initializes model_id_ to -1
  ModelToCloudFitter() : model_id_(-1) {}
  //! Stub destructor
  virtual ~ModelToCloudFitter(){}

  //! Set the database model id of this model
  void setModelId(int id){model_id_=id;}
  //! Get the database model id (if any) of this model
  int getModelId() const {return model_id_;}

  //----------------------------------------------------------------
  // Be sure to define these functions if you define your new awesome fitter:
  //----------------------------------------------------------------

  //! Initialize this template from a mesh
  // void initializeFromMesh(const shape_msgs::Shape &mesh) {}

  //! The main fitting function
  // template <class PointCloudType>
  //  ModelFitInfo fitPointCloud(const PointCloudType &cloud);
};

//! An individual fitter equipped with a distance field stored as a voxel grid
/*! Does not do any actual fitting, just initializes its internal model as a distance field.
  Used as a base class for other fitters that need this type of structure.
 */
class DistanceFieldFitter : public ModelToCloudFitter
{
 protected:
  //! Used for computing distances to point cloud and fitting errors
  distance_field::PropagationDistanceField* distance_voxel_grid_;
  //! The resolution of the distance field
  double distance_field_resolution_;
  //! Distances above this reported by the distance field will be truncated to this
  /*! Also used to decide how far beyond the object the distance field should extend */
  float truncate_value_;

  //! Initialize the distance field from a set of vertices
  void initializeFromEigenVectors(const EigenSTL::vector_Vector3d& points);

 public:
  //! Initialize distance voxel grid to NULL and set default values for grid parameters
  DistanceFieldFitter();
  //! Cleans up the distance voxel grid
  ~DistanceFieldFitter();
  
  //! Calls initialize from points on the vertices of the mesh
  void initializeFromMesh(const shape_msgs::Mesh &mesh);
};

} //namespace tabletop_object_detector

#endif
