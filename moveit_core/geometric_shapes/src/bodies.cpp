/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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

/* Author: Ioan Sucan */

#include "geometric_shapes/bodies.h"
#include "geometric_shapes/body_operations.h"

#include <ros/console.h>
extern "C"
{
#include <qhull/qhull.h>
#include <qhull/mem.h>
#include <qhull/qset.h>
#include <qhull/geom.h>
#include <qhull/merge.h>
#include <qhull/poly.h>
#include <qhull/io.h>
#include <qhull/stat.h>
}
#include <boost/math/constants/constants.hpp>
#include <limits>
#include <algorithm>
#include <Eigen/Geometry>

namespace bodies
{
namespace detail
{
static const double ZERO = 1e-9;

/** \brief Compute the square of the distance between a ray and a point
    Note: this requires 'dir' to be normalized */
static inline double distanceSQR(const Eigen::Vector3d& p, const Eigen::Vector3d& origin, const Eigen::Vector3d& dir)
{
  Eigen::Vector3d a = p - origin;
  double d = dir.dot(a);
  return a.squaredNorm() - d * d;
}

// temp structure for intersection points (used for ordering them)
struct intersc
{
  intersc(const Eigen::Vector3d &_pt, const double _tm) : pt(_pt), time(_tm) {}
  
  Eigen::Vector3d pt;
  double          time;
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// define order on intersection points
struct interscOrder
{
  bool operator()(const intersc &a, const intersc &b) const
  {
    return a.time < b.time;
  }
};
}
}

void bodies::Body::setDimensions(const shapes::Shape *shape)
{
  useDimensions(shape);
  updateInternalData();
}

bool bodies::Body::samplePointInside(random_numbers::RandomNumberGenerator &rng, unsigned int max_attempts, Eigen::Vector3d &result)
{
  BoundingSphere bs;
  computeBoundingSphere(bs);
  for (unsigned int i = 0 ; i < max_attempts ; ++i)
  {
    result = Eigen::Vector3d(rng.uniformReal(bs.center.x() - bs.radius, bs.center.x() + bs.radius),
                             rng.uniformReal(bs.center.y() - bs.radius, bs.center.y() + bs.radius),
                             rng.uniformReal(bs.center.z() - bs.radius, bs.center.z() + bs.radius));
    if (containsPoint(result))
      return true;
  }
  return false;
}

bool bodies::Sphere::containsPoint(const Eigen::Vector3d &p, bool verbose) const
{
  return (center_ - p).squaredNorm() < radius2_;
}

void bodies::Sphere::useDimensions(const shapes::Shape *shape) // radius
{
  radius_ = static_cast<const shapes::Sphere*>(shape)->radius;
}

std::vector<double> bodies::Sphere::getDimensions(void) const
{
  std::vector<double> d(1, radius_);
  return d;
}

void bodies::Sphere::updateInternalData(void)
{
  radiusU_ = radius_ * scale_ + padding_;
  radius2_ = radiusU_ * radiusU_;
  center_ = pose_.translation();
}

boost::shared_ptr<bodies::Body> bodies::Sphere::cloneAt(const Eigen::Affine3d &pose, double padding, double scale) const
{
  Sphere *s = new Sphere();
  s->radius_ = radius_;
  s->padding_ = padding;
  s->scale_ = scale;
  s->pose_ = pose;
  s->updateInternalData();
  return boost::shared_ptr<Body>(s);
}

double bodies::Sphere::computeVolume(void) const
{
  return 4.0 * boost::math::constants::pi<double>() * radiusU_ * radiusU_ * radiusU_ / 3.0;
}

void bodies::Sphere::computeBoundingSphere(BoundingSphere &sphere) const
{
  sphere.center = center_;
  sphere.radius = radiusU_;
}

void bodies::Sphere::computeBoundingCylinder(BoundingCylinder &cylinder) const
{
  cylinder.pose = pose_;
  cylinder.radius = radiusU_;
  cylinder.length = radiusU_;

}

bool bodies::Sphere::samplePointInside(random_numbers::RandomNumberGenerator &rng, unsigned int max_attempts, Eigen::Vector3d &result)
{
  for (unsigned int i = 0 ; i < max_attempts ; ++i)
  {
    const double minX = center_.x() - radiusU_; const double maxX = center_.x() + radiusU_;
    const double minY = center_.y() - radiusU_; const double maxY = center_.y() + radiusU_;
    const double minZ = center_.z() - radiusU_; const double maxZ = center_.z() + radiusU_;
    // we are sampling in a box; the probability of success after 20 attempts is 99.99996% given the ratio of box volume to sphere volume
    for (int j = 0 ; j < 20 ; ++j)
    {
      result = Eigen::Vector3d(rng.uniformReal(minX, maxX),
                               rng.uniformReal(minY, maxY),
                               rng.uniformReal(minZ, maxZ));
      if (containsPoint(result))
        return true;
    }
  }
  return false;
}

bool bodies::Sphere::intersectsRay(const Eigen::Vector3d& origin, const Eigen::Vector3d& dir, EigenSTL::vector_Vector3d *intersections, unsigned int count) const
{
  if (detail::distanceSQR(center_, origin, dir) > radius2_) return false;

  bool result = false;

  Eigen::Vector3d cp = origin - center_;
  double dpcpv = cp.dot(dir);

  Eigen::Vector3d w = cp - dpcpv * dir;
  Eigen::Vector3d Q = center_ + w;
  double x = radius2_ - w.squaredNorm();

  if (fabs(x) < detail::ZERO)
  {
    w = Q - origin;
    double dpQv = w.dot(dir);
    if (dpQv > detail::ZERO)
    {
      if (intersections)
        intersections->push_back(Q);
      result = true;
    }
  } else
    if (x > 0.0)
    {
      x = sqrt(x);
      w = dir * x;
      Eigen::Vector3d A = Q - w;
      Eigen::Vector3d B = Q + w;
      w = A - origin;
      double dpAv = w.dot(dir);
      w = B - origin;
      double dpBv = w.dot(dir);

      if (dpAv > detail::ZERO)
      {
        result = true;
        if (intersections)
        {
          intersections->push_back(A);
          if (count == 1)
            return result;
        }
      }

      if (dpBv > detail::ZERO)
      {
        result = true;
        if (intersections)
          intersections->push_back(B);
      }
    }
  return result;
}

bool bodies::Cylinder::containsPoint(const Eigen::Vector3d &p, bool verbose) const
{
  Eigen::Vector3d v = p - center_;
  double pH = v.dot(normalH_);

  if (fabs(pH) > length2_)
    return false;

  double pB1 = v.dot(normalB1_);
  double remaining = radius2_ - pB1 * pB1;

  if (remaining < 0.0)
    return false;
  else
  {
    double pB2 = v.dot(normalB2_);
    return pB2 * pB2 < remaining;
  }
}

void bodies::Cylinder::useDimensions(const shapes::Shape *shape) // (length, radius)
{
  length_ = static_cast<const shapes::Cylinder*>(shape)->length;
  radius_ = static_cast<const shapes::Cylinder*>(shape)->radius;
}

std::vector<double> bodies::Cylinder::getDimensions(void) const
{
  std::vector<double> d(2);
  d[0] = radius_;
  d[1] = length_;
  return d;
}

void bodies::Cylinder::updateInternalData(void)
{
  radiusU_ = radius_ * scale_ + padding_;
  radius2_ = radiusU_ * radiusU_;
  length2_ = scale_ * length_ / 2.0 + padding_;
  center_ = pose_.translation();
  radiusBSqr_ = length2_ * length2_ + radius2_;
  radiusB_ = sqrt(radiusBSqr_);

  Eigen::Matrix3d basis = pose_.rotation();
  normalB1_ = basis.col(0);
  normalB2_ = basis.col(1);
  normalH_  = basis.col(2);

  double tmp = -normalH_.dot(center_);
  d1_ = tmp + length2_;
  d2_ = tmp - length2_;
}

bool bodies::Cylinder::samplePointInside(random_numbers::RandomNumberGenerator &rng, unsigned int max_attempts, Eigen::Vector3d &result)
{
  // sample a point on the base disc of the cylinder
  double a = rng.uniformReal(-boost::math::constants::pi<double>(), boost::math::constants::pi<double>());
  double r = rng.uniformReal(-radiusU_, radiusU_);
  double x = cos(a) * r;
  double y = sin(a) * r;

  // sample e height
  double z = rng.uniformReal(-length2_, length2_);

  result = Eigen::Vector3d(x, y, z);
  return true;
}

boost::shared_ptr<bodies::Body> bodies::Cylinder::cloneAt(const Eigen::Affine3d &pose, double padding, double scale) const
{
  Cylinder *c = new Cylinder();
  c->length_ = length_;
  c->radius_ = radius_;
  c->padding_ = padding;
  c->scale_ = scale;
  c->pose_ = pose;
  c->updateInternalData();
  return boost::shared_ptr<Body>(c);
}

double bodies::Cylinder::computeVolume(void) const
{
  return 2.0 * boost::math::constants::pi<double>() * radius2_ * length2_;
}

void bodies::Cylinder::computeBoundingSphere(BoundingSphere &sphere) const
{
  sphere.center = center_;
  sphere.radius = radiusB_;
}

void bodies::Cylinder::computeBoundingCylinder(BoundingCylinder &cylinder) const
{
  cylinder.pose = pose_;
  cylinder.radius = radiusU_;
  cylinder.length = scale_*length_+padding_;
}

bool bodies::Cylinder::intersectsRay(const Eigen::Vector3d& origin, const Eigen::Vector3d& dir, EigenSTL::vector_Vector3d *intersections, unsigned int count) const
{
  if (detail::distanceSQR(center_, origin, dir) > radiusBSqr_) return false;

  std::vector<detail::intersc> ipts;

  // intersect bases
  double tmp = normalH_.dot(dir);
  if (fabs(tmp) > detail::ZERO)
  {
    double tmp2 = -normalH_.dot(origin);
    double t1 = (tmp2 - d1_) / tmp;

    if (t1 > 0.0)
    {
      Eigen::Vector3d p1(origin + dir * t1);
      Eigen::Vector3d v1(p1 - center_);
      v1 = v1 - normalH_.dot(v1) * normalH_;
      if (v1.squaredNorm() < radius2_ + detail::ZERO)
      {
        if (intersections == NULL)
          return true;

        detail::intersc ip(p1, t1);
        ipts.push_back(ip);
      }
    }

    double t2 = (tmp2 - d2_) / tmp;
    if (t2 > 0.0)
    {
      Eigen::Vector3d p2(origin + dir * t2);
      Eigen::Vector3d v2(p2 - center_);
      v2 = v2 - normalH_.dot(v2) * normalH_;
      if (v2.squaredNorm() < radius2_ + detail::ZERO)
      {
        if (intersections == NULL)
          return true;

        detail::intersc ip(p2, t2);
        ipts.push_back(ip);
      }
    }
  }

  if (ipts.size() < 2)
  {
    // intersect with infinite cylinder
    Eigen::Vector3d VD(normalH_.cross(dir));
    Eigen::Vector3d ROD(normalH_.cross(origin - center_));
    double a = VD.squaredNorm();
    double b = 2.0 * ROD.dot(VD);
    double c = ROD.squaredNorm() - radius2_;
    double d = b * b - 4.0 * a * c;
    if (d > 0.0 && fabs(a) > detail::ZERO)
    {
      d = sqrt(d);
      double e = -a * 2.0;
      double t1 = (b + d) / e;
      double t2 = (b - d) / e;

      if (t1 > 0.0)
      {
        Eigen::Vector3d p1(origin + dir * t1);
        Eigen::Vector3d v1(center_ - p1);

        if (fabs(normalH_.dot(v1)) < length2_ + detail::ZERO)
        {
          if (intersections == NULL)
            return true;

          detail::intersc ip(p1, t1);
          ipts.push_back(ip);
        }
      }

      if (t2 > 0.0)
      {
        Eigen::Vector3d p2(origin + dir * t2);
        Eigen::Vector3d v2(center_ - p2);

        if (fabs(normalH_.dot(v2)) < length2_ + detail::ZERO)
        {
          if (intersections == NULL)
            return true;
          detail::intersc ip(p2, t2);
          ipts.push_back(ip);
        }
      }
    }
  }

  if (ipts.empty())
    return false;

  std::sort(ipts.begin(), ipts.end(), detail::interscOrder());
  const unsigned int n = count > 0 ? std::min<unsigned int>(count, ipts.size()) : ipts.size();
  for (unsigned int i = 0 ; i < n ; ++i)
    intersections->push_back(ipts[i].pt);

  return true;
}

bool bodies::Box::samplePointInside(random_numbers::RandomNumberGenerator &rng, unsigned int /* max_attempts */, Eigen::Vector3d &result)
{
  result = pose_ * Eigen::Vector3d(rng.uniformReal(-length2_, length2_),
                                   rng.uniformReal(-width2_, width2_),
                                   rng.uniformReal(-height2_, height2_));
  return true;
}

bool bodies::Box::containsPoint(const Eigen::Vector3d &p, bool verbose) const
{
  Eigen::Vector3d v = p - center_;
  double pL = v.dot(normalL_);
  if (fabs(pL) > length2_)
    return false;

  double pW = v.dot(normalW_);
  if (fabs(pW) > width2_)
    return false;

  double pH = v.dot(normalH_);
  if (fabs(pH) > height2_)
    return false;

  return true;
}

void bodies::Box::useDimensions(const shapes::Shape *shape) // (x, y, z) = (length, width, height)
{
  const double *size = static_cast<const shapes::Box*>(shape)->size;
  length_ = size[0];
  width_  = size[1];
  height_ = size[2];
}

std::vector<double> bodies::Box::getDimensions(void) const
{
  std::vector<double> d(3);
  d[0] = length_;
  d[1] = width_;
  d[2] = height_;
  return d;
}

void bodies::Box::updateInternalData(void)
{
  double s2 = scale_ / 2.0;
  length2_ = length_ * s2 + padding_;
  width2_  = width_ * s2 + padding_;
  height2_ = height_ * s2 + padding_;

  center_  = pose_.translation();

  radius2_ = length2_ * length2_ + width2_ * width2_ + height2_ * height2_;
  radiusB_ = sqrt(radius2_);

  Eigen::Matrix3d basis = pose_.rotation();
  normalL_ = basis.col(0);
  normalW_ = basis.col(1);
  normalH_ = basis.col(2);

  const Eigen::Vector3d tmp(normalL_ * length2_ + normalW_ * width2_ + normalH_ * height2_);
  corner1_ = center_ - tmp;
  corner2_ = center_ + tmp;
}

boost::shared_ptr<bodies::Body> bodies::Box::cloneAt(const Eigen::Affine3d &pose, double padding, double scale) const
{
  Box *b = new Box();
  b->length_ = length_;
  b->width_ = width_;
  b->height_ = height_;
  b->padding_ = padding;
  b->scale_ = scale;
  b->pose_ = pose;
  b->updateInternalData();
  return boost::shared_ptr<Body>(b);
}

double bodies::Box::computeVolume(void) const
{
  return 8.0 * length2_ * width2_ * height2_;
}

void bodies::Box::computeBoundingSphere(BoundingSphere &sphere) const
{
  sphere.center = center_;
  sphere.radius = radiusB_;
}

void bodies::Box::computeBoundingCylinder(BoundingCylinder &cylinder) const
{
  double a, b;

  if(length2_ > width2_ && length2_ > height2_)
  {
    cylinder.length = length2_*2.0;
    a = width2_;
    b = height2_;
    Eigen::Affine3d rot(Eigen::AngleAxisd(90.0f * (M_PI/180.0f), Eigen::Vector3d::UnitY()));
    cylinder.pose = pose_*rot;
  }
  else
    if(width2_ > height2_)
    {
      cylinder.length = width2_*2.0;
      a = height2_;
      b = length2_;
      cylinder.radius = sqrt(height2_*height2_+length2_*length2_);
      Eigen::Affine3d rot(Eigen::AngleAxisd(90.0f * (M_PI/180.0f), Eigen::Vector3d::UnitX()));
      cylinder.pose = pose_*rot;
    }
    else
    {
      cylinder.length = height2_ * 2.0;
      a = width2_;
      b = length2_;
      cylinder.pose = pose_;
    }
  cylinder.radius = sqrt(a * a + b * b);
}

bool bodies::Box::intersectsRay(const Eigen::Vector3d& origin, const Eigen::Vector3d& dir, EigenSTL::vector_Vector3d *intersections, unsigned int count) const
{
  if (detail::distanceSQR(center_, origin, dir) > radius2_) return false;

  double t_near = -std::numeric_limits<double>::infinity();
  double t_far  = std::numeric_limits<double>::infinity();

  for (int i = 0; i < 3; i++)
  {
    const Eigen::Vector3d &vN = i == 0 ? normalL_ : (i == 1 ? normalW_ : normalH_);
    double dp = vN.dot(dir);

    if (fabs(dp) > detail::ZERO)
    {
      double t1 = vN.dot(corner1_ - origin) / dp;
      double t2 = vN.dot(corner2_ - origin) / dp;

      if (t1 > t2)
        std::swap(t1, t2);

      if (t1 > t_near)
        t_near = t1;

      if (t2 < t_far)
        t_far = t2;

      if (t_near > t_far)
        return false;

      if (t_far < 0.0)
        return false;
    }
    else
    {
      if (i == 0)
      {
        if ((std::min(corner1_.y(), corner2_.y()) > origin.y() ||
             std::max(corner1_.y(), corner2_.y()) < origin.y()) &&
            (std::min(corner1_.z(), corner2_.z()) > origin.z() ||
             std::max(corner1_.z(), corner2_.z()) < origin.z()))
          return false;
      }
      else
      {
        if (i == 1)
        {
          if ((std::min(corner1_.x(), corner2_.x()) > origin.x() ||
               std::max(corner1_.x(), corner2_.x()) < origin.x()) &&
              (std::min(corner1_.z(), corner2_.z()) > origin.z() ||
               std::max(corner1_.z(), corner2_.z()) < origin.z()))
            return false;
        }
        else
          if ((std::min(corner1_.x(), corner2_.x()) > origin.x() ||
               std::max(corner1_.x(), corner2_.x()) < origin.x()) &&
              (std::min(corner1_.y(), corner2_.y()) > origin.y() ||
               std::max(corner1_.y(), corner2_.y()) < origin.y()))
            return false;
      }
    }
  }

  if (intersections)
  {
    if (t_far - t_near > detail::ZERO)
    {
      intersections->push_back(t_near * dir + origin);
      if (count > 1)
        intersections->push_back(t_far  * dir + origin);
    }
    else
      intersections->push_back(t_far * dir + origin);
  }

  return true;
}

bool bodies::ConvexMesh::containsPoint(const Eigen::Vector3d &p, bool verbose) const
{
  if (!mesh_data_) return false;
  if (bounding_box_.containsPoint(p))
  {
    Eigen::Vector3d ip(i_pose_ * p);
    ip = mesh_data_->mesh_center_ + (ip - mesh_data_->mesh_center_) * scale_;
    return isPointInsidePlanes(ip);
  }
  else
    return false;
}

void bodies::ConvexMesh::useDimensions(const shapes::Shape *shape)
{
  mesh_data_.reset(new MeshData());
  const shapes::Mesh *mesh = static_cast<const shapes::Mesh*>(shape);

  double maxX = -std::numeric_limits<double>::infinity(), maxY = -std::numeric_limits<double>::infinity(), maxZ = -std::numeric_limits<double>::infinity();
  double minX =  std::numeric_limits<double>::infinity(), minY =  std::numeric_limits<double>::infinity(), minZ  = std::numeric_limits<double>::infinity();

  for(unsigned int i = 0; i < mesh->vertex_count ; ++i)
  {
    double vx = mesh->vertices[3 * i    ];
    double vy = mesh->vertices[3 * i + 1];
    double vz = mesh->vertices[3 * i + 2];

    if (maxX < vx) maxX = vx;
    if (maxY < vy) maxY = vy;
    if (maxZ < vz) maxZ = vz;

    if (minX > vx) minX = vx;
    if (minY > vy) minY = vy;
    if (minZ > vz) minZ = vz;
  }

  if (maxX < minX) maxX = minX = 0.0;
  if (maxY < minY) maxY = minY = 0.0;
  if (maxZ < minZ) maxZ = minZ = 0.0;

  mesh_data_->box_size_ = Eigen::Vector3d(maxX - minX, maxY - minY, maxZ - minZ);

  mesh_data_->box_offset_ = Eigen::Vector3d((minX + maxX) / 2.0, (minY + maxY) / 2.0, (minZ + maxZ) / 2.0);

  mesh_data_->planes_.clear();
  mesh_data_->triangles_.clear();
  mesh_data_->vertices_.clear();
  mesh_data_->mesh_radiusB_ = 0.0;
  mesh_data_->mesh_center_ = Eigen::Vector3d();

  double xdim = maxX - minX;
  double ydim = maxY - minY;
  double zdim = maxZ - minZ;

  double pose1;
  double pose2;

  unsigned int off1;
  unsigned int off2;

  /* compute bounding cylinder */
  double cyl_length;
  double maxdist = -std::numeric_limits<double>::infinity();
  if (xdim > ydim && xdim > zdim)
  {
    off1 = 1;
    off2 = 2;
    pose1 = mesh_data_->box_offset_.y();
    pose2 = mesh_data_->box_offset_.z();
    cyl_length = xdim;
  }
  else if(ydim > zdim)
  {
    off1 = 0;
    off2 = 2;
    pose1 = mesh_data_->box_offset_.x();
    pose2 = mesh_data_->box_offset_.z();
    cyl_length = ydim;
  }
  else
  {
    off1 = 0;
    off2 = 1;
    pose1 = mesh_data_->box_offset_.x();
    pose2 = mesh_data_->box_offset_.y();
    cyl_length = zdim;
  }

  /* compute convex hull */
  coordT *points = (coordT *)calloc(mesh->vertex_count*3, sizeof(coordT));
  for(unsigned int i = 0; i < mesh->vertex_count ; ++i)
  {
    points[3*i+0] = (coordT) mesh->vertices[3*i+0];
    points[3*i+1] = (coordT) mesh->vertices[3*i+1];
    points[3*i+2] = (coordT) mesh->vertices[3*i+2];
    
    double dista = mesh->vertices[3 * i + off1]-pose1;
    double distb = mesh->vertices[3 * i + off2]-pose2;
    double dist = sqrt(((dista*dista)+(distb*distb)));
    if(dist > maxdist)
      maxdist = dist;
  }
  mesh_data_->bounding_cylinder_.radius = maxdist;
  mesh_data_->bounding_cylinder_.length = cyl_length;

  FILE* null = fopen ("/dev/null","w");

  char flags[] = "qhull Tv";
  int exitcode = qh_new_qhull(3, mesh->vertex_count, points, true, flags, null, null);
  
  if(exitcode != 0) {
    ROS_WARN_STREAM("Convex hull creation failed");
    qh_freeqhull (!qh_ALL);
    int curlong, totlong;
    qh_memfreeshort (&curlong, &totlong);
    return;
  }

  int num_facets = qh num_facets;
  
  int num_vertices = qh num_vertices;
  mesh_data_->vertices_.reserve(num_vertices);
  Eigen::Vector3d sum(0, 0, 0);

  //necessary for FORALLvertices
  std::map<unsigned int, unsigned int> qhull_vertex_table;
  vertexT * vertex;
  FORALLvertices
  {
    Eigen::Vector3d vert(vertex->point[0],
                         vertex->point[1],
                         vertex->point[2]);
    qhull_vertex_table[vertex->id] = mesh_data_->vertices_.size();
    sum += vert;
    mesh_data_->vertices_.push_back(vert);
  }
  
  mesh_data_->mesh_center_ = sum / (double)(num_vertices);
  for (unsigned int j = 0 ; j < mesh_data_->vertices_.size() ; ++j)
  {
    double dist = (mesh_data_->vertices_[j] - mesh_data_->mesh_center_).squaredNorm();
    if (dist > mesh_data_->mesh_radiusB_)
      mesh_data_->mesh_radiusB_ = dist;
  }
  
  mesh_data_->mesh_radiusB_ = sqrt(mesh_data_->mesh_radiusB_);
  mesh_data_->triangles_.reserve(num_facets);

  //neccessary for qhull macro
  facetT * facet;
  FORALLfacets
  {
    Eigen::Vector4f planeEquation(facet->normal[0], facet->normal[1], facet->normal[2], facet->offset);
    mesh_data_->planes_.push_back(planeEquation);

    // Needed by FOREACHvertex_i_
    int vertex_n, vertex_i;
    FOREACHvertex_i_ ((*facet).vertices)
    {
      mesh_data_->triangles_.push_back(qhull_vertex_table[vertex->id]);
    }

  }
  qh_freeqhull(!qh_ALL);
  int curlong, totlong;
  qh_memfreeshort (&curlong, &totlong);
}

std::vector<double> bodies::ConvexMesh::getDimensions(void) const
{
  return std::vector<double>();
}

void bodies::ConvexMesh::updateInternalData(void)
{
  if (!mesh_data_)
    return;
  Eigen::Affine3d pose = pose_;
  pose.translation() = Eigen::Vector3d(pose_ * mesh_data_->box_offset_);

  boost::scoped_ptr<shapes::Box> box_shape(new shapes::Box(mesh_data_->box_size_.x(), mesh_data_->box_size_.y(), mesh_data_->box_size_.z()));
  bounding_box_.setDimensions(box_shape.get());
  bounding_box_.setPose(pose);
  bounding_box_.setPadding(padding_);
  bounding_box_.setScale(scale_);

  i_pose_ = pose_.inverse();
  center_ = pose_ * mesh_data_->mesh_center_;
  radiusB_ = mesh_data_->mesh_radiusB_ * scale_ + padding_;
  radiusBSqr_ = radiusB_ * radiusB_;

  // compute the scaled vertices, if needed
  if (padding_ == 0.0 && scale_ == 1.0)
    scaled_vertices_ = &mesh_data_->vertices_;
  else
  {
    if (!scaled_vertices_storage_)
      scaled_vertices_storage_.reset(new EigenSTL::vector_Vector3d());
    scaled_vertices_ = scaled_vertices_storage_.get();
    scaled_vertices_storage_->resize(mesh_data_->vertices_.size());
    for (unsigned int i = 0 ; i < mesh_data_->vertices_.size() ; ++i)
    {
      Eigen::Vector3d v(mesh_data_->vertices_[i] - mesh_data_->mesh_center_);
      double l = v.norm();
      scaled_vertices_storage_->at(i) = mesh_data_->mesh_center_ + v * (scale_ + (l > detail::ZERO ? padding_ / l : 0.0));
    }
  }
}
const std::vector<unsigned int>& bodies::ConvexMesh::getTriangles(void) const
{
  static const std::vector<unsigned int> empty;
  return mesh_data_ ? mesh_data_->triangles_ : empty;
}

const EigenSTL::vector_Vector3d& bodies::ConvexMesh::getVertices(void) const
{
  static const EigenSTL::vector_Vector3d empty;
  return mesh_data_ ? mesh_data_->vertices_ : empty;
}

const EigenSTL::vector_Vector3d& bodies::ConvexMesh::getScaledVertices(void) const
{
  return scaled_vertices_ ? *scaled_vertices_ : getVertices();
}

boost::shared_ptr<bodies::Body> bodies::ConvexMesh::cloneAt(const Eigen::Affine3d &pose, double padding, double scale) const
{
  ConvexMesh *m = new ConvexMesh();
  m->mesh_data_ = mesh_data_;
  m->padding_ = padding;
  m->scale_ = scale;
  m->pose_ = pose;
  m->updateInternalData();
  return boost::shared_ptr<Body>(m);
}

void bodies::ConvexMesh::computeBoundingSphere(BoundingSphere &sphere) const
{
  sphere.center = center_;
  sphere.radius = radiusB_;
}

void bodies::ConvexMesh::computeBoundingCylinder(BoundingCylinder &cylinder) const
{
  cylinder.length = mesh_data_ ? mesh_data_->bounding_cylinder_.length : 0.0;
  cylinder.radius = mesh_data_ ? mesh_data_->bounding_cylinder_.radius : 0.0;
  //need to do rotation correctly to get pose, which bounding box does
  BoundingCylinder cyl;
  bounding_box_.computeBoundingCylinder(cyl);
  cylinder.pose = cyl.pose;
}

bool bodies::ConvexMesh::isPointInsidePlanes(const Eigen::Vector3d& point) const
{
  unsigned int numplanes = mesh_data_->planes_.size();
  for (unsigned int i = 0 ; i < numplanes ; ++i)
  {
    const Eigen::Vector4f& plane = mesh_data_->planes_[i];
    Eigen::Vector3d plane_vec(plane.x(), plane.y(), plane.z());
    double dist = plane_vec.dot(point) + plane.w() - padding_ - 1e-6;
    if (dist > 0.0)
      return false;
  }
  return true;
}

unsigned int bodies::ConvexMesh::countVerticesBehindPlane(const Eigen::Vector4f& planeNormal) const
{
  unsigned int numvertices = mesh_data_->vertices_.size();
  unsigned int result = 0;
  for (unsigned int i = 0 ; i < numvertices ; ++i)
  {
    Eigen::Vector3d plane_vec(planeNormal.x(), planeNormal.y(), planeNormal.z());
    double dist = plane_vec.dot(mesh_data_->vertices_[i]) + planeNormal.w() - 1e-6;
    if (dist > 0.0)
      result++;
  }
  return result;
}

double bodies::ConvexMesh::computeVolume(void) const
{
  double volume = 0.0;
  if (mesh_data_)
    for (unsigned int i = 0 ; i < mesh_data_->triangles_.size() / 3 ; ++i)
    {
      const Eigen::Vector3d &v1 = mesh_data_->vertices_[mesh_data_->triangles_[3*i + 0]];
      const Eigen::Vector3d &v2 = mesh_data_->vertices_[mesh_data_->triangles_[3*i + 1]];
      const Eigen::Vector3d &v3 = mesh_data_->vertices_[mesh_data_->triangles_[3*i + 2]];
      volume += v1.x()*v2.y()*v3.z() + v2.x()*v3.y()*v1.z() + v3.x()*v1.y()*v2.z() - v1.x()*v3.y()*v2.z() - v2.x()*v1.y()*v3.z() - v3.x()*v2.y()*v1.z();
    }
  return fabs(volume)/6.0;
}

bool bodies::ConvexMesh::intersectsRay(const Eigen::Vector3d& origin, const Eigen::Vector3d& dir, EigenSTL::vector_Vector3d *intersections, unsigned int count) const
{
  if (!mesh_data_) return false;
  if (detail::distanceSQR(center_, origin, dir) > radiusBSqr_) return false;
  if (!bounding_box_.intersectsRay(origin, dir)) return false;

  // transform the ray into the coordinate frame of the mesh
  Eigen::Vector3d orig(i_pose_ * origin);
  Eigen::Vector3d dr(i_pose_ * dir);

  std::vector<detail::intersc> ipts;

  bool result = false;

  // for each triangle
  const unsigned int nt = mesh_data_->triangles_.size() / 3;
  for (unsigned int i = 0 ; i < nt ; ++i)
  {
    Eigen::Vector3d vec(mesh_data_->planes_[i].x(),
                        mesh_data_->planes_[i].y(),
                        mesh_data_->planes_[i].z());
    
    double tmp = vec.dot(dr);
    if (fabs(tmp) > detail::ZERO)
    {
                          
      double t = -(vec.dot(orig) + mesh_data_->planes_[i].w()) / tmp;
      if (t > 0.0)
      {
        const int i3 = 3 * i;
        const int v1 = mesh_data_->triangles_[i3 + 0];
        const int v2 = mesh_data_->triangles_[i3 + 1];
        const int v3 = mesh_data_->triangles_[i3 + 2];

        const Eigen::Vector3d &a = scaled_vertices_->at(v1);
        const Eigen::Vector3d &b = scaled_vertices_->at(v2);
        const Eigen::Vector3d &c = scaled_vertices_->at(v3);

        Eigen::Vector3d cb(c - b);
        Eigen::Vector3d ab(a - b);

        // intersection of the plane defined by the triangle and the ray
        Eigen::Vector3d P(orig + dr * t);

        // check if it is inside the triangle
        Eigen::Vector3d pb(P - b);
        Eigen::Vector3d c1(cb.cross(pb));
        Eigen::Vector3d c2(cb.cross(ab));
        if (c1.dot(c2) < 0.0)
          continue;

        Eigen::Vector3d ca(c - a);
        Eigen::Vector3d pa(P - a);
        Eigen::Vector3d ba(-ab);

        c1 = ca.cross(pa);
        c2 = ca.cross(ba);
        if (c1.dot(c2) < 0.0)
          continue;

        c1 = ba.cross(pa);
        c2 = ba.cross(ca);

        if (c1.dot(c2) < 0.0)
          continue;

        result = true;
        if (intersections)
        {
          detail::intersc ip(origin + dir * t, t);
          ipts.push_back(ip);
        }
        else
          break;
      }
    }
  }

  if (intersections)
  {
    std::sort(ipts.begin(), ipts.end(), detail::interscOrder());
    const unsigned int n = count > 0 ? std::min<unsigned int>(count, ipts.size()) : ipts.size();
    for (unsigned int i = 0 ; i < n ; ++i)
      intersections->push_back(ipts[i].pt);
  }

  return result;
}

bodies::BodyVector::BodyVector(void)
{
}

bodies::BodyVector::BodyVector(const std::vector<shapes::Shape*>& shapes,
                               const EigenSTL::vector_Affine3d& poses,
                               double padding)
{
  for(unsigned int i = 0; i < shapes.size(); i++)
    addBody(shapes[i], poses[i], padding);
}


bodies::BodyVector::~BodyVector(void)
{
  clear();
}

void bodies::BodyVector::clear(void)
{
  for(unsigned int i = 0; i < bodies_.size(); i++)
    delete bodies_[i];
  bodies_.clear();
}

void bodies::BodyVector::addBody(Body *body)
{
  bodies_.push_back(body);
  BoundingSphere sphere;
  body->computeBoundingSphere(sphere);
}

void bodies::BodyVector::addBody(const shapes::Shape *shape, const Eigen::Affine3d& pose, double padding)
{
  bodies::Body* body = bodies::createBodyFromShape(shape);
  body->setPose(pose);
  body->setPadding(padding);
  addBody(body);
}

std::size_t bodies::BodyVector::getCount(void) const
{
  return bodies_.size();
}

void bodies::BodyVector::setPose(unsigned int i, const Eigen::Affine3d& pose)
{
  if (i >= bodies_.size())
  {
    ROS_ERROR("There is no body at index %u", i);
    return;
  }

  bodies_[i]->setPose(pose);
}

const bodies::Body* bodies::BodyVector::getBody(unsigned int i) const
{
  if (i >= bodies_.size())
  {
    ROS_ERROR("There is no body at index %u", i);
    return NULL;
  }
  else
    return bodies_[i];
}

bool bodies::BodyVector::containsPoint(const Eigen::Vector3d &p, std::size_t &index, bool verbose) const
{
  for (std::size_t i = 0 ; i < bodies_.size() ; ++i)
    if (bodies_[i]->containsPoint(p, verbose))
    {
      index = i;
      return true;
    }
  return false;
}

bool bodies::BodyVector::containsPoint(const Eigen::Vector3d &p, bool verbose) const
{
  std::size_t dummy;
  return containsPoint(p, dummy, verbose);
}

bool bodies::BodyVector::intersectsRay(const Eigen::Vector3d& origin, const Eigen::Vector3d &dir, std::size_t &index, EigenSTL::vector_Vector3d *intersections, unsigned int count) const
{
  for (std::size_t i = 0 ; i < bodies_.size() ; ++i)
    if (bodies_[i]->intersectsRay(origin, dir, intersections, count))
    {
      index = i;
      return true;
    }
  return false;
}
