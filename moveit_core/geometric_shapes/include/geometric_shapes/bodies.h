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

/** \author Ioan Sucan, E. Gil Jones */

#ifndef GEOMETRIC_SHAPES_BODIES_
#define GEOMETRIC_SHAPES_BODIES_

#include "geometric_shapes/shapes.h"
#include <LinearMath/btTransform.h>
#include <vector>

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
	btVector3 center;
	double    radius;
    };
    
    /** \brief Definition of a cylinder */
    struct BoundingCylinder
    {
	btTransform pose;
	double      radius;
	double      length;
    };
    
    /** \brief A body is a shape + its pose. Point inclusion, ray
	intersection can be tested, volumes and bounding spheres can
	be computed.*/
    class Body
    {
    public:
	
	Body(void)
	{
	    scale_ = 1.0;
	    padding_ = 0.0;
	    pose_.setIdentity();
	    type_ = shapes::UNKNOWN_SHAPE;
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
	void setPose(const btTransform &pose)
	{
	    pose_ = pose;
	    updateInternalData();
	}
	
	/** \brief Retrieve the pose of the body */
	const btTransform& getPose(void) const
	{
	    return pose_;
	}
	
	/** \brief Set the dimensions of the body (from corresponding shape) */
	void setDimensions(const shapes::Shape *shape)
	{
	    useDimensions(shape);
	    updateInternalData();
	}
	
	/** \brief Check is a point is inside the body */
	bool containsPoint(double x, double y, double z) const
	{
	    return containsPoint(btVector3(btScalar(x), btScalar(y), btScalar(z)));
	}
	
	/** \brief Check is a ray intersects the body, and find the
	    set of intersections, in order, along the ray. A maximum
	    number of intersections can be specified as well. If that
	    number is 0, all intersections are returned */
	virtual bool intersectsRay(const btVector3& origin, const btVector3 &dir, std::vector<btVector3> *intersections = NULL, unsigned int count = 0) const = 0;
	
	/** \brief Check is a point is inside the body */
	virtual bool containsPoint(const btVector3 &p, bool verbose = false) const = 0;	
	
	/** \brief Compute the volume of the body. This method includes
	    changes induced by scaling and padding */
	virtual double computeVolume(void) const = 0;
	
	/** \brief Compute the bounding radius for the body, in its current
	    pose. Scaling and padding are accounted for. */
	virtual void computeBoundingSphere(BoundingSphere &sphere) const = 0;
	
	/** \brief Compute the bounding cylinder for the body, in its current
	    pose. Scaling and padding are accounted for. */
	virtual void computeBoundingCylinder(BoundingCylinder &cylinder) const = 0;
	
    protected:
	
	virtual void updateInternalData(void) = 0;
	virtual void useDimensions(const shapes::Shape *shape) = 0;
	
	shapes::ShapeType type_;
	btTransform       pose_;	
	double            scale_;
	double            padding_;	
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
	
	virtual bool containsPoint(const btVector3 &p, bool verbose = false) const;
	virtual double computeVolume(void) const;
	virtual void computeBoundingSphere(BoundingSphere &sphere) const;
	virtual void computeBoundingCylinder(BoundingCylinder &cylinder) const;
	virtual bool intersectsRay(const btVector3& origin, const btVector3 &dir, std::vector<btVector3> *intersections = NULL, unsigned int count = 0) const;
	
    protected:
	
	virtual void useDimensions(const shapes::Shape *shape);
	virtual void updateInternalData(void);
	
	btVector3 center_;
	double    radius_;	
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
	
	virtual bool containsPoint(const btVector3 &p, bool verbose = false) const;
	virtual double computeVolume(void) const;
	virtual void computeBoundingSphere(BoundingSphere &sphere) const;
	virtual void computeBoundingCylinder(BoundingCylinder &cylinder) const;
	virtual bool intersectsRay(const btVector3& origin, const btVector3 &dir, std::vector<btVector3> *intersections = NULL, unsigned int count = 0) const;
	
    protected:
	
	virtual void useDimensions(const shapes::Shape *shape);
	virtual void updateInternalData(void);
	
	btVector3 center_;
	btVector3 normalH_;
	btVector3 normalB1_;
	btVector3 normalB2_;
	
	double    length_;
	double    length2_;	
	double    radius_;
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
	
	virtual bool containsPoint(const btVector3 &p, bool verbose = false) const;
	virtual double computeVolume(void) const;
	virtual void computeBoundingSphere(BoundingSphere &sphere) const;
	virtual void computeBoundingCylinder(BoundingCylinder &cylinder) const;
	virtual bool intersectsRay(const btVector3& origin, const btVector3 &dir, std::vector<btVector3> *intersections = NULL, unsigned int count = 0) const;
	
    protected:
	
	virtual void useDimensions(const shapes::Shape *shape); // (x, y, z) = (length, width, height)	    
	virtual void updateInternalData(void);
	
	btVector3 center_;
	btVector3 normalL_;
	btVector3 normalW_;
	btVector3 normalH_;
	
	btVector3 corner1_;
	btVector3 corner2_;
	
	double    length_;
	double    width_;
	double    height_;	
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
	}
	
	ConvexMesh(const shapes::Shape *shape) : Body()
	{	  
	    type_ = shapes::MESH;
	    setDimensions(shape);
	}
	
	virtual ~ConvexMesh(void)
	{
	}	
	
	virtual bool containsPoint(const btVector3 &p, bool verbose = false) const;
	virtual double computeVolume(void) const;
	
	virtual void computeBoundingSphere(BoundingSphere &sphere) const;
	virtual void computeBoundingCylinder(BoundingCylinder &cylinder) const;
	virtual bool intersectsRay(const btVector3& origin, const btVector3 &dir, std::vector<btVector3> *intersections = NULL, unsigned int count = 0) const;
	
	const std::vector<unsigned int>& getTriangles(void) const
	{
	    return triangles_;
	}
	
	const std::vector<btVector3>& getVertices(void) const
	{
	    return vertices_;
	}
	
	const std::vector<btVector3>& getScaledVertices(void) const
	{
	    return scaled_vertices_;
	}
	
    protected:
	
	virtual void useDimensions(const shapes::Shape *shape);
	virtual void updateInternalData(void);
	
	unsigned int countVerticesBehindPlane(const btVector4& planeNormal) const;
	bool isPointInsidePlanes(const btVector3& point) const;
	
	std::vector<btVector4>    planes_;
	std::vector<btVector3>    vertices_;
	std::vector<btVector3>    scaled_vertices_;
	std::vector<unsigned int> triangles_;
	btTransform               i_pose_;
	
	btVector3                 center_;
	btVector3                 mesh_center_;
	double                    radiusB_;
	double                    radiusBSqr_;
	double                    mesh_radiusB_;
	
	btVector3                 box_offset_;
	Box                       bounding_box_;
	BoundingCylinder          bounding_cylinder_;
    };
    
    /** \brief Compute a bounding sphere to enclose a set of bounding spheres */
    void mergeBoundingSpheres(const std::vector<BoundingSphere> &spheres, BoundingSphere &mergedSphere);
    
    class BodyVector
    {
    public:
	
	BodyVector(void);
	
	BodyVector(const std::vector<shapes::Shape*>& shapes, const std::vector<btTransform>& poses, double padding = 0.0);
	
	~BodyVector();
	
	void addBody(Body* body);
	void addBody(const shapes::Shape* shape, const btTransform& pose, double padding = 0.0);
	
	void clear(void);
	
	void setPose(unsigned int i, const btTransform& pose);
	
	std::size_t getSize() const
	{
	    return bodies_.size();
	}
	
	bool containsPoint(const btVector3 &p, bool verbose = false) const;

	const Body* getBody(unsigned int i) const;
	
	BoundingSphere getBoundingSphere(unsigned int i) const;
	double getBoundingSphereRadiusSquared(unsigned int i) const;
	
    private:
	
	std::vector<Body*>  bodies_;
	std::vector<double> rsqrs_;
	
    };
    
}

#endif
