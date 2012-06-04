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

/** \author E. Gil Jones */

#include <collision_distance_field/collision_distance_field_types.h>
#include <geometric_shapes/body_operations.h>

std::vector<collision_distance_field::CollisionSphere> collision_distance_field::determineCollisionSpheres(const bodies::Body* body, 
                                                                                                           Eigen::Affine3d& relative_transform)
{
  std::vector<collision_distance_field::CollisionSphere> css;
  
  bodies::BoundingCylinder cyl;
  body->computeBoundingCylinder(cyl);
  unsigned int num_points = ceil(cyl.length/(cyl.radius/2.0));
  double spacing = cyl.length/((num_points*1.0)-1.0);
  Eigen::Vector3d vec(0.0,0.0,0.0);
  for(unsigned int i = 1; i < num_points-1; i++) {
    vec.z() = (-cyl.length/2.0)+i*spacing;
    Eigen::Vector3d p = cyl.pose*vec;
    collision_distance_field::CollisionSphere cs(vec,cyl.radius);
    css.push_back(cs);
  }
  relative_transform = body->getPose().inverse() * cyl.pose;
  return css; 
}

std::vector<Eigen::Vector3d> collision_distance_field::determineCollisionPoints(const bodies::Body* body, double resolution)
{
  std::vector<Eigen::Vector3d> ret_vec;
  bodies::BoundingSphere sphere;
  body->computeBoundingSphere(sphere);
  //ROS_INFO_STREAM("Radius is " << sphere.radius);
  //ROS_INFO_STREAM("Center is " << sphere.center.z() << " " << sphere.center.y() << " " << sphere.center.z());
  for(double xval = sphere.center.x()-sphere.radius-resolution; xval < sphere.center.x()+sphere.radius+resolution; xval += resolution) {
    for(double yval = sphere.center.y()-sphere.radius-resolution; yval < sphere.center.y()+sphere.radius+resolution; yval += resolution) {
      for(double zval = sphere.center.z()-sphere.radius-resolution; zval < sphere.center.z()+sphere.radius+resolution; zval += resolution) {
        Eigen::Vector3d rel_vec(xval, yval, zval);
        if(body->containsPoint(body->getPose()*rel_vec)) {
          ret_vec.push_back(rel_vec);
        }
      }
    }
  }
  return ret_vec;
}

bool collision_distance_field::getCollisionSphereGradients(const distance_field::DistanceField* distance_field,
                                                           const std::vector<CollisionSphere>& sphere_list,
                                                           const std::vector<Eigen::Vector3d>& sphere_centers,
                                                           GradientInfo& gradient, 
                                                           double tolerance, 
                                                           bool subtract_radii, 
                                                           double maximum_value,
                                                           bool stop_at_first_collision) {
  //assumes gradient is properly initialized
  bool in_collision = false;
  for(unsigned int i = 0; i < sphere_list.size(); i++) {
    Eigen::Vector3d p = sphere_centers[i];
    double gx, gy, gz;
    double dist = distance_field->getDistanceGradient(p.x(), p.y(), p.z(), gx, gy, gz);
    if(dist < maximum_value && subtract_radii) {
      dist -= sphere_list[i].radius_;
      if(dist <= tolerance) {
        if(stop_at_first_collision) {
          return true;
        } 
        in_collision = true;
      } 
    }
    if(dist < gradient.closest_distance) {
      gradient.closest_distance = dist;
    }
    gradient.distances[i] = dist;
    gradient.gradients[i] = Eigen::Vector3d(gx,gy,gz);
  }
  return in_collision;
}

bool collision_distance_field::getCollisionSphereCollision(const distance_field::DistanceField* distance_field,
                                                           const std::vector<CollisionSphere>& sphere_list,
                                                           const std::vector<Eigen::Vector3d>& sphere_centers,
                                                           double tolerance)
{
  for(unsigned int i = 0; i < sphere_list.size(); i++) {
    Eigen::Vector3d p = sphere_centers[i];
    double gx, gy, gz;
    double dist = distance_field->getDistanceGradient(p.x(), p.y(), p.z(), gx, gy, gz);
    if(dist - sphere_list[i].radius_ < tolerance) {
      return true;
    }
  }
  return false;

}

///
/// BodyDecomposition
///

collision_distance_field::BodyDecomposition::BodyDecomposition(const shapes::ShapeConstPtr& shape, double resolution, double padding)
{
  body_ = bodies::createBodyFromShape(shape.get()); //unpadded
  Eigen::Affine3d ident;
  ident.setIdentity();
  body_->setPose(ident);
  body_->setPadding(padding);
  collision_spheres_ = determineCollisionSpheres(body_, relative_cylinder_pose_);
  relative_collision_points_ = determineCollisionPoints(body_, resolution);
}

collision_distance_field::BodyDecomposition::~BodyDecomposition()
{
  delete body_;
}

collision_distance_field::PosedBodyDecomposition::PosedBodyDecomposition(const BodyDecompositionConstPtr& body_decomposition) 
  : body_decomposition_(body_decomposition)
{
  sphere_centers_.resize(body_decomposition_->getCollisionSpheres().size());
  posed_collision_points_ = body_decomposition_->getCollisionPoints();
}

void collision_distance_field::PosedBodyDecomposition::updateSpheresPose(const Eigen::Affine3d& trans) 
{
  Eigen::Affine3d cyl_transform = trans * body_decomposition_->getRelativeCylinderPose();
  for(unsigned int i = 0; i < body_decomposition_->getCollisionSpheres().size(); i++) {
    sphere_centers_[i] = cyl_transform*body_decomposition_->getCollisionSpheres()[i].relative_vec_;
  }
}

void collision_distance_field::PosedBodyDecomposition::updatePointsPose(const Eigen::Affine3d& trans) {
  posed_collision_points_.resize(body_decomposition_->getCollisionPoints().size());
  for(unsigned int i = 0; i < body_decomposition_->getCollisionPoints().size(); i++) {
    posed_collision_points_[i] = trans*body_decomposition_->getCollisionPoints()[i];
  }
}

void collision_distance_field::PosedBodyDecomposition::updatePose(const Eigen::Affine3d& trans)
{
  updateSpheresPose(trans);
  updatePointsPose(trans);
}

