/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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

/* Author: E. Gil Jones */

#ifndef MOVEIT_COLLISION_DISTANCE_FIELD_COLLISION_DISTANCE_FIELD_TYPES_
#define MOVEIT_COLLISION_DISTANCE_FIELD_COLLISION_DISTANCE_FIELD_TYPES_

#include <vector>
#include <string>
#include <algorithm>
#include <sstream>
#include <float.h>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/bodies.h>

#include <moveit/distance_field/distance_field.h>
#include <visualization_msgs/MarkerArray.h>
#include <console_bridge/console.h>

namespace collision_detection
{

enum CollisionType {
  NONE = 0,
  SELF = 1,
  INTRA = 2,
  ENVIRONMENT = 3,
};

struct CollisionSphere {

  CollisionSphere(const Eigen::Vector3d &rel, double radius)
  {
    relative_vec_ = rel;
    radius_ = radius;
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d relative_vec_;
  double radius_;
};

struct GradientInfo
{
  GradientInfo() :
    closest_distance(DBL_MAX),
    collision(false)
  {
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double closest_distance;
  bool collision;
  EigenSTL::vector_Vector3d sphere_locations;
  std::vector<double> distances;
  EigenSTL::vector_Vector3d gradients;
  std::vector<CollisionType> types;
  std::vector<double> sphere_radii;
  std::string joint_name;

  void clear() {
    closest_distance = DBL_MAX;
    collision = false;
    sphere_locations.clear();
    distances.clear();
    gradients.clear();
    sphere_radii.clear();
    joint_name.clear();
  }
};

//determines set of collision spheres given a posed body; this is BAD! Allocation erorrs will happen; change this function so it does not return that vector by value
std::vector<CollisionSphere> determineCollisionSpheres(const bodies::Body* body, Eigen::Affine3d& relativeTransform);

//determines a set of gradients of the given collision spheres in the distance field
bool getCollisionSphereGradients(const distance_field::DistanceField* distance_field,
                                 const std::vector<CollisionSphere>& sphere_list, 
                                 const EigenSTL::vector_Vector3d& sphere_centers,
                                 GradientInfo& gradient, 
                                 const CollisionType& type,
                                 double tolerance, 
                                 bool subtract_radii, 
                                 double maximum_value, 
                                 bool stop_at_first_collision);

bool getCollisionSphereCollision(const distance_field::DistanceField* distance_field,
                                 const std::vector<CollisionSphere>& sphere_list,
                                 const EigenSTL::vector_Vector3d& sphere_centers,
                                 double maximum_value,                          
                                 double tolerance);

bool getCollisionSphereCollision(const distance_field::DistanceField* distance_field,
                                 const std::vector<CollisionSphere>& sphere_list,
                                 const EigenSTL::vector_Vector3d& sphere_centers,
                                 double maximum_value,
                                 double tolerance,
                                 unsigned int num_coll,
                                 std::vector<unsigned int>& colls);

//forward declaration required for friending apparently
class BodyDecompositionVector;

class BodyDecomposition {

  friend class BodyDecompositionVector;

public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    
  BodyDecomposition(const shapes::ShapeConstPtr& shape, 
                    double resolution, 
                    double padding = 0.01);

  ~BodyDecomposition();

  Eigen::Affine3d relative_cylinder_pose_;

  void replaceCollisionSpheres(const std::vector<CollisionSphere>& new_collision_spheres,
                               const Eigen::Affine3d& new_relative_cylinder_pose) 
  {
    //std::cerr << "Replacing " << collision_spheres_.size() << " with " << new_collision_spheres.size() << std::endl;
    collision_spheres_ = new_collision_spheres;
    relative_cylinder_pose_ = new_relative_cylinder_pose;
  }

  const std::vector<CollisionSphere>& getCollisionSpheres() const 
  {
    return collision_spheres_;
  }

  const std::vector<double>& getSphereRadii() const {
    return sphere_radii_;
  }

  const EigenSTL::vector_Vector3d& getCollisionPoints() const
  {
    return relative_collision_points_;
  }

  const bodies::Body* getBody() const
  {
    return body_;
  }

  Eigen::Affine3d getRelativeCylinderPose() const {
    return relative_cylinder_pose_;
  }

  const bodies::BoundingSphere& getRelativeBoundingSphere() const {
    return relative_bounding_sphere_;
  }

private:
  bodies::Body* body_;

  bodies::BoundingSphere relative_bounding_sphere_;
  std::vector<double> sphere_radii_;
  std::vector<CollisionSphere> collision_spheres_;
  EigenSTL::vector_Vector3d relative_collision_points_;
};

typedef boost::shared_ptr<BodyDecomposition> BodyDecompositionPtr;
typedef boost::shared_ptr<const BodyDecomposition> BodyDecompositionConstPtr;

class PosedBodySphereDecomposition {

public:
  
  PosedBodySphereDecomposition(const BodyDecompositionConstPtr& body_decomposition);
  
  const std::vector<CollisionSphere>& getCollisionSpheres() const 
  {
    return body_decomposition_->getCollisionSpheres();
  }

  const EigenSTL::vector_Vector3d& getSphereCenters() const 
  {
    return sphere_centers_;
  }

  const std::vector<double>& getSphereRadii() const {
    return body_decomposition_->getSphereRadii();
  }  
  const Eigen::Vector3d& getBoundingSphereCenter() const {
    return posed_bounding_sphere_center_;
  }

  double getBoundingSphereRadius() const {
    return body_decomposition_->getRelativeBoundingSphere().radius;
  }

  //assumed to be in reference frame, updates the pose of the body,
  //the collision spheres, and the posed collision points
  void updatePose(const Eigen::Affine3d& linkTransform);

  void updateSpheresPose(const Eigen::Affine3d& linkTransform);

protected:

  BodyDecompositionConstPtr body_decomposition_;
  Eigen::Vector3d posed_bounding_sphere_center_;
  EigenSTL::vector_Vector3d sphere_centers_;
};

class PosedBodyPointDecomposition {

public:
  
  PosedBodyPointDecomposition(const BodyDecompositionConstPtr& body_decomposition);

  PosedBodyPointDecomposition(const BodyDecompositionConstPtr& body_decomposition,
                              const Eigen::Affine3d& pose);

  const EigenSTL::vector_Vector3d& getCollisionPoints() const
  {
    return posed_collision_points_;
  }

  //the collision spheres, and the posed collision points
  void updatePose(const Eigen::Affine3d& linkTransform);

protected:

  BodyDecompositionConstPtr body_decomposition_;
  EigenSTL::vector_Vector3d posed_collision_points_;
};

typedef boost::shared_ptr<PosedBodyPointDecomposition> PosedBodyPointDecompositionPtr;
typedef boost::shared_ptr<const PosedBodyPointDecomposition> PosedBodyPointDecompositionConstPtr;
typedef boost::shared_ptr<PosedBodySphereDecomposition> PosedBodySphereDecompositionPtr;
typedef boost::shared_ptr<const PosedBodySphereDecomposition> PosedBodySphereDecompositionConstPtr;

class PosedBodySphereDecompositionVector
{
public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PosedBodySphereDecompositionVector()
  {}

  const std::vector<CollisionSphere>& getCollisionSpheres() const
  {
    return collision_spheres_;
  }

  const EigenSTL::vector_Vector3d& getSphereCenters() const {
    return posed_collision_spheres_;
  }

  const std::vector<double>& getSphereRadii() const {
    return sphere_radii_;
  }  
  
  void addToVector(PosedBodySphereDecompositionPtr& bd)
  {
    sphere_index_map_[decomp_vector_.size()] = collision_spheres_.size();
    decomp_vector_.push_back(bd);
    collision_spheres_.insert(collision_spheres_.end(), 
                              bd->getCollisionSpheres().begin(),
                              bd->getCollisionSpheres().end());
    posed_collision_spheres_.insert(posed_collision_spheres_.end(),
                                    bd->getSphereCenters().begin(),
                                    bd->getSphereCenters().end());
    sphere_radii_.insert(sphere_radii_.end(),
                         bd->getSphereRadii().begin(),
                         bd->getSphereRadii().end());
  }

  unsigned int getSize() const {
    return decomp_vector_.size();
  }

  PosedBodySphereDecompositionConstPtr getPosedBodySphereDecomposition(unsigned int i) const {
    if(i >= decomp_vector_.size()) {
      logInform("No body decomposition");
      return empty_ptr_;
    }
    return decomp_vector_[i];
  }

  void updatePose(unsigned int ind, const Eigen::Affine3d& pose) {
    if(ind >= decomp_vector_.size()) {
      logWarn("Can't update pose");
      return;
    }
    decomp_vector_[ind]->updatePose(pose);
    for(unsigned int i = 0; i < decomp_vector_[ind]->getSphereCenters().size(); i++) {
      posed_collision_spheres_[sphere_index_map_[ind]+i] = decomp_vector_[ind]->getSphereCenters()[i];
    }
  }

private:
  PosedBodySphereDecompositionConstPtr empty_ptr_;
  std::vector<PosedBodySphereDecompositionPtr> decomp_vector_;
  std::vector<CollisionSphere> collision_spheres_;
  EigenSTL::vector_Vector3d posed_collision_spheres_;
  std::vector<double> sphere_radii_;
  std::map<unsigned int, unsigned int> sphere_index_map_;
};

class PosedBodyPointDecompositionVector
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PosedBodyPointDecompositionVector()
  {}
  
  EigenSTL::vector_Vector3d getCollisionPoints() const
  {
    EigenSTL::vector_Vector3d ret_points;
    for(unsigned int i = 0; i < decomp_vector_.size(); i++) {
      ret_points.insert(ret_points.end(),
                        decomp_vector_[i]->getCollisionPoints().begin(),
                        decomp_vector_[i]->getCollisionPoints().end());
    }
    return ret_points;
  }

  void addToVector(PosedBodyPointDecompositionPtr& bd)
  {
    decomp_vector_.push_back(bd);
  }

  unsigned int getSize() const {
    return decomp_vector_.size();
  }
    
  PosedBodyPointDecompositionConstPtr getPosedBodyDecomposition(unsigned int i) const {
    if(i >= decomp_vector_.size()) {
      logInform("No body decomposition");
      return empty_ptr_;
    }
    return decomp_vector_[i];
  }

  void updatePose(unsigned int ind, const Eigen::Affine3d& pose) {
    if(ind < decomp_vector_.size()) {
      decomp_vector_[ind]->updatePose(pose);
    } else {
      logWarn("Can't update pose");
      return;
    }
  }

private:
  PosedBodyPointDecompositionPtr empty_ptr_;
  std::vector<PosedBodyPointDecompositionPtr> decomp_vector_;
};

typedef boost::shared_ptr<PosedBodySphereDecompositionVector> PosedBodySphereDecompositionVectorPtr;
typedef boost::shared_ptr<const PosedBodySphereDecompositionVector> PosedBodySphereDecompositionVectorConstPtr;
typedef boost::shared_ptr<PosedBodyPointDecompositionVector> PosedBodyPointDecompositionVectorPtr;
typedef boost::shared_ptr<const PosedBodyPointDecompositionVector> PosedBodyPointDecompositionVectorConstPtr;

struct ProximityInfo 
{ 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::string link_name;
  std::string attached_object_name;
  double proximity;
  unsigned int sphere_index;
  unsigned int att_index;
  Eigen::Vector3d closest_point;
  Eigen::Vector3d closest_gradient;
};

bool doBoundingSpheresIntersect(const PosedBodySphereDecompositionConstPtr& p1,
                                const PosedBodySphereDecompositionConstPtr& p2);


void getCollisionSphereMarkers(const std_msgs::ColorRGBA& color,
                               const std::string& frame_id,
                               const std::string& ns,
                               const ros::Duration& dur,
                               const std::vector<PosedBodySphereDecompositionPtr>& posed_decompositions,
                               visualization_msgs::MarkerArray& arr);

void getProximityGradientMarkers(const std::string& frame_id,
                                 const std::string& ns,
                                 const ros::Duration& dur,
                                 const std::vector<PosedBodySphereDecompositionPtr>& posed_decompositions,
                                 const std::vector<PosedBodySphereDecompositionVectorPtr>& posed_vector_decompositions,
                                 const std::vector<GradientInfo>& gradients,
                                 visualization_msgs::MarkerArray& arr);

void getCollisionMarkers(const std::string& frame_id,
                         const std::string& ns,
                         const ros::Duration& dur,
                         const std::vector<PosedBodySphereDecompositionPtr>& posed_decompositions,
                         const std::vector<PosedBodySphereDecompositionVectorPtr>& posed_vector_decompositions,
                         const std::vector<GradientInfo>& gradients,
                         visualization_msgs::MarkerArray& arr);
}

#endif
