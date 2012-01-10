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

/* Author: Ioan Sucan, E. Gil Jones */

#ifndef GEOMETRIC_SHAPES_BODIES_
#define GEOMETRIC_SHAPES_BODIES_

#include "geometric_shapes/shapes.h"
#include <boost/scoped_ptr.hpp>
#include <random_numbers/random_numbers.h>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

/**
   This set of classes allows quickly detecting whether a given point
   is inside an object or not. This capability is useful when removing
   points from inside the robot (when the robot sees its arms, for
   example).
*/

namespace bodies
{

/** \brief Definition of a sphere that bounds another object */
struct BoundingSphere
{
  Eigen::Vector3f center;
  double    radius;
};

/** \brief Definition of a cylinder */
struct BoundingCylinder
{
  Eigen::Affine3f pose;
  double      radius;
  double      length;
};

class Body;
typedef boost::shared_ptr<Body> BodyPtr;
typedef boost::shared_ptr<const Body> BodyConstPtr;

/** \brief A body is a shape + its pose. Point inclusion, ray
    intersection can be tested, volumes and bounding spheres can
    be computed.*/
class Body
{
public:

  Body(void) : scale_(1.0), padding_(0.0), type_(shapes::UNKNOWN_SHAPE)
  {
    pose_.setIdentity();
  }

  virtual ~Body(void)
  {
  }

  /** \brief Get the type of shape this body represents */
  shapes::ShapeType getType(void) const
  {
    return type_;
  }

  /** \brief If the dimension of the body should be scaled, this
      method sets the scale. Default is 1.0 */
  void setScale(double scale)
  {
    scale_ = scale;
    updateInternalData();
  }

  /** \brief Retrieve the current scale */
  double getScale(void) const
  {
    return scale_;
  }

  /** \brief If constant padding should be added to the body, this
      method sets the padding. Default is 0.0 */
  void setPadding(double padd)
  {
    padding_ = padd;
    updateInternalData();
  }

  /** \brief Retrieve the current padding */
  double getPadding(void) const
  {
    return padding_;
  }

  /** \brief Set the pose of the body. Default is identity */
  void setPose(const Eigen::Affine3f &pose)
  {
    pose_ = pose;
    updateInternalData();
  }

  /** \brief Retrieve the pose of the body */
  const Eigen::Affine3f& getPose(void) const
  {
    return pose_;
  }

  /** \brief Get the dimensions associated to this body (as read from corresponding shape) */
  virtual std::vector<double> getDimensions(void) const = 0;

  /** \brief Set the dimensions of the body (from corresponding shape) */
  void setDimensions(const shapes::Shape *shape);

  /** \brief Check if a point is inside the body */
  bool containsPoint(double x, double y, double z, bool verbose = false) const
  {
    return containsPoint(Eigen::Vector3f(x, y, z), verbose);
  }

  /** \brief Check if a point is inside the body */
  virtual bool containsPoint(const Eigen::Vector3f &p, bool verbose = false) const = 0;

  /** \brief Check if a ray intersects the body, and find the
      set of intersections, in order, along the ray. A maximum
      number of intersections can be specified as well. If that
      number is 0, all intersections are returned */
  virtual bool intersectsRay(const Eigen::Vector3f& origin, const Eigen::Vector3f &dir, std::vector<Eigen::Vector3f> *intersections = NULL, unsigned int count = 0) const = 0;

  /** \brief Compute the volume of the body. This method includes
      changes induced by scaling and padding */
  virtual double computeVolume(void) const = 0;

  /** \brief Sample a point that is included in the body using a given random number generator. Sometimes multiple attempts need to be generated;
      the function terminates with failure (returns false) after \e max_attempts attempts. If the call is successful (returns true) the point is
      written to \e result */
  virtual bool samplePointInside(random_numbers::RandomNumberGenerator &rng, unsigned int max_attempts, Eigen::Vector3f &result);

  /** \brief Compute the bounding radius for the body, in its current
      pose. Scaling and padding are accounted for. */
  virtual void computeBoundingSphere(BoundingSphere &sphere) const = 0;

  /** \brief Compute the bounding cylinder for the body, in its current
      pose. Scaling and padding are accounted for. */
  virtual void computeBoundingCylinder(BoundingCylinder &cylinder) const = 0;

  BodyPtr cloneAt(const Eigen::Affine3f &pose) const
  {
    return cloneAt(pose, padding_, scale_);
  }

  virtual BodyPtr cloneAt(const Eigen::Affine3f &pose, double padding, double scaling) const = 0;

protected:

  virtual void updateInternalData(void) = 0;
  virtual void useDimensions(const shapes::Shape *shape) = 0;

  double            scale_;
  double            padding_;
  shapes::ShapeType type_;
  Eigen::Affine3f   pose_;

};

/** \brief Definition of a sphere */
class Sphere : public Body
{
public:
  Sphere(void) : Body()
  {
    type_ = shapes::SPHERE;
  }

  Sphere(const shapes::Shape *shape) : Body()
  {
    type_ = shapes::SPHERE;
    setDimensions(shape);
  }

  virtual ~Sphere(void)
  {
  }

  /** \brief Get the radius of the sphere */
  virtual std::vector<double> getDimensions(void) const;

  virtual bool containsPoint(const Eigen::Vector3f &p, bool verbose = false) const;
  virtual double computeVolume(void) const;
  virtual bool samplePointInside(random_numbers::RandomNumberGenerator &rng, unsigned int max_attempts, Eigen::Vector3f &result);
  virtual void computeBoundingSphere(BoundingSphere &sphere) const;
  virtual void computeBoundingCylinder(BoundingCylinder &cylinder) const;
  virtual bool intersectsRay(const Eigen::Vector3f& origin, const Eigen::Vector3f &dir, std::vector<Eigen::Vector3f> *intersections = NULL, unsigned int count = 0) const;

  virtual BodyPtr cloneAt(const Eigen::Affine3f &pose, double padding, double scale) const;

protected:

  virtual void useDimensions(const shapes::Shape *shape);
  virtual void updateInternalData(void);

  // shape-dependent data
  double    radius_;

  // pose/padding/scaling-dependent values & values computed for convenience and fast upcoming computations
  Eigen::Vector3f center_;
  double    radiusU_;
  double    radius2_;
};

/** \brief Definition of a cylinder */
class Cylinder : public Body
{
public:
  Cylinder(void) : Body()
  {
    type_ = shapes::CYLINDER;
  }

  Cylinder(const shapes::Shape *shape) : Body()
  {
    type_ = shapes::CYLINDER;
    setDimensions(shape);
  }

  virtual ~Cylinder(void)
  {
  }

  /** \brief Get the radius & length of the cylinder */
  virtual std::vector<double> getDimensions(void) const;

  virtual bool containsPoint(const Eigen::Vector3f &p, bool verbose = false) const;
  virtual double computeVolume(void) const;
  virtual bool samplePointInside(random_numbers::RandomNumberGenerator &rng, unsigned int max_attempts, Eigen::Vector3f &result);
  virtual void computeBoundingSphere(BoundingSphere &sphere) const;
  virtual void computeBoundingCylinder(BoundingCylinder &cylinder) const;
  virtual bool intersectsRay(const Eigen::Vector3f& origin, const Eigen::Vector3f &dir, std::vector<Eigen::Vector3f> *intersections = NULL, unsigned int count = 0) const;

  virtual BodyPtr cloneAt(const Eigen::Affine3f &pose, double padding, double scale) const;

protected:

  virtual void useDimensions(const shapes::Shape *shape);
  virtual void updateInternalData(void);

  // shape-dependent data
  double    length_;
  double    radius_;

  // pose/padding/scaling-dependent values & values computed for convenience and fast upcoming computations
  Eigen::Vector3f center_;
  Eigen::Vector3f normalH_;
  Eigen::Vector3f normalB1_;
  Eigen::Vector3f normalB2_;

  double    length2_;
  double    radiusU_;
  double    radiusB_;
  double    radiusBSqr_;
  double    radius2_;
  double    d1_;
  double    d2_;
};

/** \brief Definition of a box */
class Box : public Body
{
public:
  Box(void) : Body()
  {
    type_ = shapes::BOX;
  }

  Box(const shapes::Shape *shape) : Body()
  {
    type_ = shapes::BOX;
    setDimensions(shape);
  }

  virtual ~Box(void)
  {
  }

  /** \brief Get the length & width & height (x, y, z) of the box */
  virtual std::vector<double> getDimensions(void) const;

  virtual bool containsPoint(const Eigen::Vector3f &p, bool verbose = false) const;
  virtual double computeVolume(void) const;
  virtual bool samplePointInside(random_numbers::RandomNumberGenerator &rng, unsigned int max_attempts, Eigen::Vector3f &result);
  virtual void computeBoundingSphere(BoundingSphere &sphere) const;
  virtual void computeBoundingCylinder(BoundingCylinder &cylinder) const;
  virtual bool intersectsRay(const Eigen::Vector3f& origin, const Eigen::Vector3f &dir, std::vector<Eigen::Vector3f> *intersections = NULL, unsigned int count = 0) const;

  virtual BodyPtr cloneAt(const Eigen::Affine3f &pose, double padding, double scale) const;

protected:

  virtual void useDimensions(const shapes::Shape *shape); // (x, y, z) = (length, width, height)
  virtual void updateInternalData(void);

  // shape-dependent data
  double    length_;
  double    width_;
  double    height_;

  // pose/padding/scaling-dependent values & values computed for convenience and fast upcoming computations
  Eigen::Vector3f center_;
  Eigen::Vector3f normalL_;
  Eigen::Vector3f normalW_;
  Eigen::Vector3f normalH_;

  Eigen::Vector3f corner1_;
  Eigen::Vector3f corner2_;

  double    length2_;
  double    width2_;
  double    height2_;
  double    radiusB_;
  double    radius2_;
};

/** \brief Definition of a convex mesh. Convex hull is computed for a given shape::Mesh */
class ConvexMesh : public Body
{
public:

  ConvexMesh(void) : Body()
  {
    type_ = shapes::MESH;
    scaled_vertices_ = NULL;
  }

  ConvexMesh(const shapes::Shape *shape) : Body()
  {
    type_ = shapes::MESH;
    scaled_vertices_ = NULL;
    setDimensions(shape);
  }

  virtual ~ConvexMesh(void)
  {
  }

  /** \brief Returns an empty vector */
  virtual std::vector<double> getDimensions(void) const;

  virtual bool containsPoint(const Eigen::Vector3f &p, bool verbose = false) const;
  virtual double computeVolume(void) const;

  virtual void computeBoundingSphere(BoundingSphere &sphere) const;
  virtual void computeBoundingCylinder(BoundingCylinder &cylinder) const;
  virtual bool intersectsRay(const Eigen::Vector3f& origin, const Eigen::Vector3f &dir, std::vector<Eigen::Vector3f> *intersections = NULL, unsigned int count = 0) const;

  const std::vector<unsigned int>& getTriangles(void) const;
  const std::vector<Eigen::Vector3f>& getVertices(void) const;
  const std::vector<Eigen::Vector3f>& getScaledVertices(void) const;

  virtual BodyPtr cloneAt(const Eigen::Affine3f &pose, double padding, double scale) const;

protected:

  virtual void useDimensions(const shapes::Shape *shape);
  virtual void updateInternalData(void);

  /** \brief (Used mainly for debugging) Count the number of vertices behind a plane*/
  unsigned int countVerticesBehindPlane(const Eigen::Vector4f& planeNormal) const;

  /** \brief Check if a point is inside a set of planes that make up a convex mesh*/
  bool isPointInsidePlanes(const Eigen::Vector3f& point) const;

  struct MeshData
  {
    std::vector<Eigen::Vector4f>    planes_;
    std::vector<Eigen::Vector3f>    vertices_;
    std::vector<unsigned int> triangles_;
    Eigen::Vector3f                 mesh_center_;
    double                    mesh_radiusB_;
    Eigen::Vector3f                 box_offset_;
    Eigen::Vector3f                 box_size_;
    BoundingCylinder          bounding_cylinder_;
  };

  // shape-dependent data; keep this in one struct so that a cheap pointer copy can be done in cloneAt()
  boost::shared_ptr<MeshData> mesh_data_;

  // pose/padding/scaling-dependent values & values computed for convenience and fast upcoming computations
  Eigen::Affine3f                 i_pose_;
  Eigen::Vector3f                   center_;
  double                      radiusB_;
  double                      radiusBSqr_;
  Box                         bounding_box_;

  // pointer to an array of scaled vertices
  // if the padding is 0 & scaling is 1, then there is no need to have scaled vertices; we can just point to the vertices in mesh_data_
  // otherwise, point to scaled_vertices_storage_
  std::vector<Eigen::Vector3f>     *scaled_vertices_;

private:
  boost::scoped_ptr<std::vector<Eigen::Vector3f> > scaled_vertices_storage_;
};

/** @class BodyVector
 *  @brief A vector of Body objects
 */
class BodyVector
{
public:

  BodyVector(void);

  /** \brief Construct a body vector from a vector of shapes, a vector of poses and a padding */
  BodyVector(const std::vector<shapes::Shape*>& shapes, const std::vector<Eigen::Affine3f>& poses, double padding = 0.0);

  ~BodyVector();

  /** \brief Add a body*/
  void addBody(Body* body);

  /** \brief Add a body from a shape, a pose for the body and a padding*/
  void addBody(const shapes::Shape* shape, const Eigen::Affine3f& pose, double padding = 0.0);

  /** \brief Clear all bodies from the vector*/
  void clear(void);

  /** \brief Set the pose of a particular body in the vector of bodies*/
  void setPose(unsigned int i, const Eigen::Affine3f& pose);

  /** \brief Get the number of bodies in this vector*/
  std::size_t getCount(void) const;

  /** \brief Check if any of the bodies in the vector contains the input point*/
  bool containsPoint(const Eigen::Vector3f &p, bool verbose = false) const;

  /** \brief Check if any of the bodies intersects the ray defined by \e origin and \e dir.
      When the first intersection is found, this function terminates. The index of the body that
      does intersect the ray is set to \e index (set to -1 if no intersections were found). Optionally,
      the intersection points are computed and set to \e intersections */
  bool intersectsRay(const Eigen::Vector3f& origin, const Eigen::Vector3f &dir, int &index, std::vector<Eigen::Vector3f> *intersections = NULL, unsigned int count = 0) const;

  /** \brief Get the \e i th body in the vector*/
  const Body* getBody(unsigned int i) const;

  /** \brief Get the bounding sphere for the whole body*/
  double getBoundingSphereRadiusSquared(unsigned int i) const;

private:

  std::vector<Body*>  bodies_;
  std::vector<double> rsqrs_;

};

}

#endif
