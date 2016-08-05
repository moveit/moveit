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

#include <moveit/collision_distance_field/collision_distance_field_types.h>
#include <geometric_shapes/body_operations.h>
#include <moveit/distance_field/distance_field_common.h>

std::vector<collision_detection::CollisionSphere> collision_detection::determineCollisionSpheres(const bodies::Body* body, 
                                                                                                           Eigen::Affine3d& relative_transform)
{
  std::vector<collision_detection::CollisionSphere> css;
  
  bodies::BoundingCylinder cyl;
  body->computeBoundingCylinder(cyl);
  unsigned int num_points = ceil(cyl.length/(cyl.radius/2.0));
  double spacing = cyl.length/((num_points*1.0)-1.0);
  Eigen::Vector3d vec(0.0,0.0,0.0);
  for(unsigned int i = 1; i < num_points-1; i++) {
    vec.z() = (-cyl.length/2.0)+i*spacing;
    //Eigen::Vector3d p = cyl.pose*vec;
    collision_detection::CollisionSphere cs(vec,cyl.radius);
    css.push_back(cs);
  }
  relative_transform = body->getPose().inverse() * cyl.pose;
  return css; 
}

bool collision_detection::getCollisionSphereGradients(const distance_field::DistanceField* distance_field,
                                                           const std::vector<CollisionSphere>& sphere_list,
                                                           const EigenSTL::vector_Vector3d& sphere_centers,
                                                           GradientInfo& gradient, 
                                                           const collision_detection::CollisionType& type,
                                                           double tolerance, 
                                                           bool subtract_radii, 
                                                           double maximum_value,
                                                           bool stop_at_first_collision) {
  //assumes gradient is properly initialized
  bool in_collision = false;
  for(unsigned int i = 0; i < sphere_list.size(); i++) {
    Eigen::Vector3d p = sphere_centers[i];
    double gx, gy, gz;
    bool in_bounds;
    double dist = distance_field->getDistanceGradient(p.x(), p.y(), p.z(), gx, gy, gz, in_bounds);
    if(!in_bounds) {
      logError("Collision sphere point is out of bounds %lf, %lf, %lf", p.x(), p.y(), p.z());
      return true;
    }
    if(dist < maximum_value) {
      if(subtract_radii) {
        dist -= sphere_list[i].radius_;
      }
      if(dist <= tolerance) {
        if(stop_at_first_collision) {
          return true;
        } 
        in_collision = true;
      }
      if(dist < gradient.closest_distance) {
        gradient.closest_distance = dist;
      }
      if(dist < gradient.distances[i]) {
        gradient.types[i] = type;
        gradient.distances[i] = dist;
        gradient.gradients[i] = Eigen::Vector3d(gx,gy,gz);
      }
    }
  }
  return in_collision;
}

bool collision_detection::getCollisionSphereCollision(const distance_field::DistanceField* distance_field,
                                                           const std::vector<CollisionSphere>& sphere_list,
                                                           const EigenSTL::vector_Vector3d& sphere_centers,
                                                           double maximum_value,
                                                           double tolerance)
{
  for(unsigned int i = 0; i < sphere_list.size(); i++) {
    Eigen::Vector3d p = sphere_centers[i];
    double gx, gy, gz;
    bool in_bounds;
    double dist = distance_field->getDistanceGradient(p.x(), p.y(), p.z(), gx, gy, gz, in_bounds);
    if(!in_bounds) {
      logError("Collision sphere point is out of bounds");
      return true;
    }
    if(maximum_value > dist && dist - sphere_list[i].radius_ < tolerance) {
      //ROS_INFO_STREAM("Point " << p.x() << " " << p.y() << " " << p.z() << " " << dist << " " << sphere_list[i].radius_);
      return true;
    }
  }
  return false;

}

bool collision_detection::getCollisionSphereCollision(const distance_field::DistanceField* distance_field,
                                                           const std::vector<CollisionSphere>& sphere_list,
                                                           const EigenSTL::vector_Vector3d& sphere_centers,
                                                           double maximum_value,
                                                           double tolerance,
                                                           unsigned int num_coll,
                                                           std::vector<unsigned int>& colls)
{
  colls.clear();
  for(unsigned int i = 0; i < sphere_list.size(); i++) {
    Eigen::Vector3d p = sphere_centers[i];
    double gx, gy, gz;
    bool in_bounds;
    double dist = distance_field->getDistanceGradient(p.x(), p.y(), p.z(), gx, gy, gz, in_bounds);
    if(!in_bounds) {
      logError("Collision sphere point is out of bounds");
      return true;
    }
    if(maximum_value > dist && dist - sphere_list[i].radius_ < tolerance) {
      if(num_coll == 0) {
        return true;
      }
      colls.push_back(i);
      if(colls.size() >= num_coll) {
        return true;
      }
    }
  }
  return colls.size() > 0;
}

///
/// BodyDecomposition
///

collision_detection::BodyDecomposition::BodyDecomposition(const shapes::ShapeConstPtr& shape, double resolution, double padding)
{
  body_ = bodies::createBodyFromShape(shape.get()); //unpadded
  body_->setPose(Eigen::Affine3d::Identity());
  body_->setPadding(padding);
  collision_spheres_ = determineCollisionSpheres(body_, relative_cylinder_pose_);
  relative_collision_points_ = distance_field::determineCollisionPoints(body_, resolution);
  sphere_radii_.resize(collision_spheres_.size());
  for(unsigned int i = 0; i < collision_spheres_.size(); i++) {
    sphere_radii_[i] = collision_spheres_[i].radius_;
  }
  body_->computeBoundingSphere(relative_bounding_sphere_);
}

collision_detection::BodyDecomposition::~BodyDecomposition()
{
  delete body_;
}

collision_detection::PosedBodyPointDecomposition::PosedBodyPointDecomposition(const BodyDecompositionConstPtr& body_decomposition) 
  : body_decomposition_(body_decomposition)
{
  posed_collision_points_ = body_decomposition_->getCollisionPoints();
}

collision_detection::PosedBodyPointDecomposition::PosedBodyPointDecomposition(const BodyDecompositionConstPtr& body_decomposition,
                                                                                   const Eigen::Affine3d& trans)
  : body_decomposition_(body_decomposition)
{
  updatePose(trans);
}

void collision_detection::PosedBodyPointDecomposition::updatePose(const Eigen::Affine3d& trans) {
  posed_collision_points_.resize(body_decomposition_->getCollisionPoints().size());
  for(unsigned int i = 0; i < body_decomposition_->getCollisionPoints().size(); i++) {
    posed_collision_points_[i] = trans*body_decomposition_->getCollisionPoints()[i];
  }
}

collision_detection::PosedBodySphereDecomposition::PosedBodySphereDecomposition(const BodyDecompositionConstPtr& body_decomposition) 
  : body_decomposition_(body_decomposition)
{
  posed_bounding_sphere_center_ = body_decomposition_->getRelativeBoundingSphere().center;
  sphere_centers_.resize(body_decomposition_->getCollisionSpheres().size());
}

void collision_detection::PosedBodySphereDecomposition::updatePose(const Eigen::Affine3d& trans) 
{
  posed_bounding_sphere_center_ = trans * body_decomposition_->getRelativeBoundingSphere().center;
  Eigen::Affine3d cyl_transform = trans * body_decomposition_->getRelativeCylinderPose();
  for(unsigned int i = 0; i < body_decomposition_->getCollisionSpheres().size(); i++) {
    sphere_centers_[i] = cyl_transform*body_decomposition_->getCollisionSpheres()[i].relative_vec_;
  }
}

bool collision_detection::doBoundingSpheresIntersect(const PosedBodySphereDecompositionConstPtr& p1,
                                                          const PosedBodySphereDecompositionConstPtr& p2)
{
  Eigen::Vector3d p1_sphere_center = p1->getBoundingSphereCenter();
  Eigen::Vector3d p2_sphere_center = p2->getBoundingSphereCenter();
  double p1_radius = p1->getBoundingSphereRadius();
  double p2_radius = p2->getBoundingSphereRadius();

  double dist = (p1_sphere_center-p2_sphere_center).squaredNorm();
  if(dist < (p1_radius+p2_radius)) {
    return true;
  }
  return false;
}

void collision_detection::getCollisionSphereMarkers(const std_msgs::ColorRGBA& color,
                                                         const std::string& frame_id,
                                                         const std::string& ns,
                                                         const ros::Duration& dur,
                                                         const std::vector<PosedBodySphereDecompositionPtr>& posed_decompositions,
                                                         visualization_msgs::MarkerArray& arr) 
{
  unsigned int count = 0;
  for(unsigned int i = 0; i < posed_decompositions.size(); i++) {
    if(posed_decompositions[i]) {
      for(unsigned int j = 0; j < posed_decompositions[i]->getCollisionSpheres().size(); j++) {
        visualization_msgs::Marker sphere;
        sphere.type = visualization_msgs::Marker::SPHERE;
        sphere.header.stamp = ros::Time::now();
        sphere.header.frame_id = frame_id;
        sphere.ns = ns;
        sphere.id = count++;
        sphere.lifetime = dur;
        sphere.color = color;
        sphere.scale.x = sphere.scale.y = sphere.scale.z = posed_decompositions[i]->getCollisionSpheres()[j].radius_*2.0;
        sphere.pose.position.x = posed_decompositions[i]->getSphereCenters()[j].x();
        sphere.pose.position.y = posed_decompositions[i]->getSphereCenters()[j].y();
        sphere.pose.position.z = posed_decompositions[i]->getSphereCenters()[j].z();
        arr.markers.push_back(sphere);
      }
    }
  }
}

void collision_detection::getProximityGradientMarkers(const std::string& frame_id,
                                                           const std::string& ns,
                                                           const ros::Duration& dur,
                                                           const std::vector<PosedBodySphereDecompositionPtr>& posed_decompositions,
                                                           const std::vector<PosedBodySphereDecompositionVectorPtr>& posed_vector_decompositions,
                                                           const std::vector<GradientInfo>& gradients,
                                                           visualization_msgs::MarkerArray& arr)
{
  if(gradients.size() != posed_decompositions.size() + posed_vector_decompositions.size()) {
    logWarn("Size mismatch between gradients %u and decompositions %u" , (unsigned int)gradients.size(), 
            (unsigned int) (posed_decompositions.size() + posed_vector_decompositions.size()));
    return;
  }
  for(unsigned int i = 0; i < gradients.size(); i++) {
    for(unsigned int j = 0; j < gradients[i].distances.size(); j++) {
      visualization_msgs::Marker arrow_mark;
      arrow_mark.header.frame_id = frame_id;
      arrow_mark.header.stamp = ros::Time::now();
      if(ns.empty()) {
        arrow_mark.ns = "self_coll_gradients";
      } else {
        arrow_mark.ns = ns;
      }
      arrow_mark.id = i*1000+j;
      double xscale = 0.0;
      double yscale = 0.0;
      double zscale = 0.0;
      if(gradients[i].distances[j] > 0.0 && gradients[i].distances[j] != DBL_MAX) {
        if(gradients[i].gradients[j].norm() > 0.0) {
          xscale = gradients[i].gradients[j].x()/gradients[i].gradients[j].norm();
          yscale = gradients[i].gradients[j].y()/gradients[i].gradients[j].norm();
          zscale = gradients[i].gradients[j].z()/gradients[i].gradients[j].norm();
        } else {
          logDebug("Negative length for %u %d %lf", i, arrow_mark.id, gradients[i].gradients[j].norm());
        }
      } else {
        logDebug("Negative dist %lf for %u %d", gradients[i].distances[j], i, arrow_mark.id);
      }
      arrow_mark.points.resize(2);
      if(i < posed_decompositions.size()) {
        arrow_mark.points[1].x = posed_decompositions[i]->getSphereCenters()[j].x();
        arrow_mark.points[1].y = posed_decompositions[i]->getSphereCenters()[j].y();
        arrow_mark.points[1].z = posed_decompositions[i]->getSphereCenters()[j].z();
      } else {
        arrow_mark.points[1].x = posed_vector_decompositions[i-posed_decompositions.size()]->getSphereCenters()[j].x();
        arrow_mark.points[1].y = posed_vector_decompositions[i-posed_decompositions.size()]->getSphereCenters()[j].y();
        arrow_mark.points[1].z = posed_vector_decompositions[i-posed_decompositions.size()]->getSphereCenters()[j].z();
      }
      arrow_mark.points[0] = arrow_mark.points[1];
      arrow_mark.points[0].x -= xscale*gradients[i].distances[j];
      arrow_mark.points[0].y -= yscale*gradients[i].distances[j];
      arrow_mark.points[0].z -= zscale*gradients[i].distances[j];
      arrow_mark.scale.x = 0.01;
      arrow_mark.scale.y = 0.03;
      arrow_mark.color.a = 1.0;
      if(gradients[i].types[j] == collision_detection::SELF) {
        arrow_mark.color.r = 1.0;
        arrow_mark.color.g = 0.2;
        arrow_mark.color.b = .5;
      } else if(gradients[i].types[j] == collision_detection::INTRA) {
        arrow_mark.color.r = .2;
        arrow_mark.color.g = 1.0;
        arrow_mark.color.b = .5;
      } else if(gradients[i].types[j] == collision_detection::ENVIRONMENT) {
        arrow_mark.color.r = .2;
        arrow_mark.color.g = .5;
        arrow_mark.color.b = 1.0;
      } else if(gradients[i].types[j] == collision_detection::NONE) {
        arrow_mark.color.r = 1.0;
        arrow_mark.color.g = .2;
        arrow_mark.color.b = 1.0;
      }
      arr.markers.push_back(arrow_mark);
    }
  }
}

void collision_detection::getCollisionMarkers(const std::string& frame_id,
                                                   const std::string& ns,
                                                   const ros::Duration& dur,
                                                   const std::vector<PosedBodySphereDecompositionPtr>& posed_decompositions,
                                                   const std::vector<PosedBodySphereDecompositionVectorPtr>& posed_vector_decompositions,
                                                   const std::vector<GradientInfo>& gradients,
                                                   visualization_msgs::MarkerArray& arr)
{
  if(gradients.size() != posed_decompositions.size() + posed_vector_decompositions.size()) {
    logWarn("Size mismatch between gradients %u and decompositions ",  (unsigned int)gradients.size(), 
            (unsigned int)(posed_decompositions.size() + posed_vector_decompositions.size()));
    return;
  }
  for(unsigned int i = 0; i < gradients.size(); i++) {
    for(unsigned int j = 0; j < gradients[i].types.size(); j++) {
      visualization_msgs::Marker sphere_mark;
      sphere_mark.type = visualization_msgs::Marker::SPHERE;
      sphere_mark.header.frame_id = frame_id;
      sphere_mark.header.stamp = ros::Time::now();
      if(ns.empty()) {
        sphere_mark.ns = "distance_collisions";
      } else {
        sphere_mark.ns = ns;
      }
      sphere_mark.id = i*1000+j;
      if(i < posed_decompositions.size()) {
        sphere_mark.scale.x = sphere_mark.scale.y = sphere_mark.scale.z = posed_decompositions[i]->getCollisionSpheres()[j].radius_*2.0;
        sphere_mark.pose.position.x = posed_decompositions[i]->getSphereCenters()[j].x();
        sphere_mark.pose.position.y = posed_decompositions[i]->getSphereCenters()[j].y();
        sphere_mark.pose.position.z = posed_decompositions[i]->getSphereCenters()[j].z();
      } else {
        sphere_mark.scale.x = sphere_mark.scale.y = sphere_mark.scale.z = posed_vector_decompositions[i-posed_decompositions.size()]->getCollisionSpheres()[j].radius_*2.0;
        sphere_mark.pose.position.x = posed_vector_decompositions[i-posed_decompositions.size()]->getSphereCenters()[j].x();
        sphere_mark.pose.position.y = posed_vector_decompositions[i-posed_decompositions.size()]->getSphereCenters()[j].y();
        sphere_mark.pose.position.z = posed_vector_decompositions[i-posed_decompositions.size()]->getSphereCenters()[j].z();
      }
      sphere_mark.pose.orientation.w = 1.0;
      sphere_mark.color.a = 1.0;
      if(gradients[i].types[j] == collision_detection::SELF) {
        sphere_mark.color.r = 1.0;
        sphere_mark.color.g = 0.2;
        sphere_mark.color.b = .5;
      } else if(gradients[i].types[j] == collision_detection::INTRA) {
        sphere_mark.color.r = .2;
        sphere_mark.color.g = 1.0;
        sphere_mark.color.b = .5;
      } else if(gradients[i].types[j] == collision_detection::ENVIRONMENT) {
        sphere_mark.color.r = .2;
        sphere_mark.color.g = .5;
        sphere_mark.color.b = 1.0;
      } else {
        sphere_mark.color.r = 1.0;
        sphere_mark.color.g = .2;
        sphere_mark.color.b = 1.0;
      }
      arr.markers.push_back(sphere_mark);
    }
  }
}
