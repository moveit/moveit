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

/** \author Ioan Sucan */

#include "geometric_shapes/bodies.h"
#include "geometric_shapes/body_operations.h"

#include <ros/console.h>
#include <LinearMath/btConvexHull.h>

#include <algorithm>
#include <iostream>
#include <cmath>

namespace bodies
{
    static const double ZERO = 1e-9;
    
    /** \brief Compute the square of the distance between a ray and a point 
	Note: this requires 'dir' to be normalized */
    static inline double distanceSQR(const btVector3& p, const btVector3& origin, const btVector3& dir)
    {
	btVector3 a = p - origin;
	double d = dir.dot(a);
	return a.length2() - d * d;
    }
    
    namespace detail
    {
	
	// temp structure for intersection points (used for ordering them)
	struct intersc
	{
	    intersc(const btVector3 &_pt, const double _tm) : pt(_pt), time(_tm) {}
	    
	    btVector3 pt;
	    double    time;
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

bool bodies::Sphere::containsPoint(const btVector3 &p, bool verbose) const 
{
    return (center_ - p).length2() < radius2_;
}

void bodies::Sphere::useDimensions(const shapes::Shape *shape) // radius
{
    radius_ = static_cast<const shapes::Sphere*>(shape)->radius;
}

void bodies::Sphere::updateInternalData(void)
{
    radiusU_ = radius_ * scale_ + padding_;
    radius2_ = radiusU_ * radiusU_;
    center_ = pose_.getOrigin();
}

double bodies::Sphere::computeVolume(void) const
{
    return 4.0 * M_PI * radiusU_ * radiusU_ * radiusU_ / 3.0;
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

bool bodies::Sphere::intersectsRay(const btVector3& origin, const btVector3& dir, std::vector<btVector3> *intersections, unsigned int count) const
{
    if (distanceSQR(center_, origin, dir) > radius2_) return false;
    
    bool result = false;
    
    btVector3 cp = origin - center_;
    double dpcpv = cp.dot(dir);
    
    btVector3 w = cp - dpcpv * dir;
    btVector3 Q = center_ + w;
    double x = radius2_ - w.length2();
    
    if (fabs(x) < ZERO)
    { 
	w = Q - origin;
	double dpQv = w.dot(dir);
	if (dpQv > ZERO)
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
	    btVector3 A = Q - w;
	    btVector3 B = Q + w;
	    w = A - origin;
	    double dpAv = w.dot(dir);
	    w = B - origin;
	    double dpBv = w.dot(dir);
	    
	    if (dpAv > ZERO)
	    {	
		result = true;
		if (intersections)
		{
		    intersections->push_back(A);
		    if (count == 1)
			return result;
		}
	    }
	    
	    if (dpBv > ZERO)
	    {
		result = true;
		if (intersections)
		    intersections->push_back(B);
	    }
	}
    return result;
}

bool bodies::Cylinder::containsPoint(const btVector3 &p, bool verbose) const 
{
    btVector3 v = p - center_;		
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

void bodies::Cylinder::updateInternalData(void)
{
    radiusU_ = radius_ * scale_ + padding_;
    radius2_ = radiusU_ * radiusU_;
    length2_ = scale_ * length_ / 2.0 + padding_;
    center_ = pose_.getOrigin();
    radiusBSqr_ = length2_ * length2_ + radius2_;
    radiusB_ = sqrt(radiusBSqr_);
    
    const btMatrix3x3& basis = pose_.getBasis();
    normalB1_ = basis.getColumn(0);
    normalB2_ = basis.getColumn(1);
    normalH_  = basis.getColumn(2);
    
    double tmp = -normalH_.dot(center_);
    d1_ = tmp + length2_;
    d2_ = tmp - length2_;
}

double bodies::Cylinder::computeVolume(void) const
{
    return 2.0 * M_PI * radius2_ * length2_;
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

bool bodies::Cylinder::intersectsRay(const btVector3& origin, const btVector3& dir, std::vector<btVector3> *intersections, unsigned int count) const
{
    if (distanceSQR(center_, origin, dir) > radiusBSqr_) return false;
    
    std::vector<detail::intersc> ipts;
    
    // intersect bases
    double tmp = normalH_.dot(dir);
    if (fabs(tmp) > ZERO)
    {
	double tmp2 = -normalH_.dot(origin);
	double t1 = (tmp2 - d1_) / tmp;
	
	if (t1 > 0.0)
	{
	    btVector3 p1(origin + dir * t1);
	    btVector3 v1(p1 - center_);
	    v1 = v1 - normalH_.dot(v1) * normalH_;
	    if (v1.length2() < radius2_ + ZERO)
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
	    btVector3 p2(origin + dir * t2);
	    btVector3 v2(p2 - center_);
	    v2 = v2 - normalH_.dot(v2) * normalH_;
	    if (v2.length2() < radius2_ + ZERO)
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
	btVector3 VD(normalH_.cross(dir));
	btVector3 ROD(normalH_.cross(origin - center_));
	double a = VD.length2();
	double b = 2.0 * ROD.dot(VD);
	double c = ROD.length2() - radius2_;
	double d = b * b - 4.0 * a * c;
	if (d > 0.0 && fabs(a) > ZERO)
	{
	    d = sqrt(d);
	    double e = -a * 2.0;
	    double t1 = (b + d) / e;
	    double t2 = (b - d) / e;
	    
	    if (t1 > 0.0)
	    {
		btVector3 p1(origin + dir * t1);
		btVector3 v1(center_ - p1);
		
		if (fabs(normalH_.dot(v1)) < length2_ + ZERO)
		{
		    if (intersections == NULL)
			return true;
		    
		    detail::intersc ip(p1, t1);
		    ipts.push_back(ip);
		}
	    }
	    
	    if (t2 > 0.0)
	    {
		btVector3 p2(origin + dir * t2);    
		btVector3 v2(center_ - p2);
		
		if (fabs(normalH_.dot(v2)) < length2_ + ZERO)
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

bool bodies::Box::containsPoint(const btVector3 &p, bool verbose) const 
{
    /*  if(verbose)
	fprintf(stderr,"Actual: %f,%f,%f \nDesired: %f,%f,%f \nTolerance:%f,%f,%f\n",p.x(),p.y(),p.z(),center_.x(),center_.y(),center_.z(),length2_,width2_,height2_);*/
    btVector3 v = p - center_;
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

void bodies::Box::updateInternalData(void) 
{
    double s2 = scale_ / 2.0;
    length2_ = length_ * s2 + padding_;
    width2_  = width_ * s2 + padding_;
    height2_ = height_ * s2 + padding_;
    
    center_  = pose_.getOrigin();
    
    radius2_ = length2_ * length2_ + width2_ * width2_ + height2_ * height2_;
    radiusB_ = sqrt(radius2_);
    
    const btMatrix3x3& basis = pose_.getBasis();
    normalL_ = basis.getColumn(0);
    normalW_ = basis.getColumn(1);
    normalH_ = basis.getColumn(2);
    
    const btVector3 tmp(normalL_ * length2_ + normalW_ * width2_ + normalH_ * height2_);
    corner1_ = center_ - tmp;
    corner2_ = center_ + tmp;
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
	btTransform rot(btQuaternion(btVector3(0,1,0),btRadians(90)));
	cylinder.pose = pose_*rot;
    }
    else
	if(width2_ > height2_)
	{
	    cylinder.length = width2_*2.0;
	    a = height2_;
	    b = length2_;
	    cylinder.radius = sqrt(height2_*height2_+length2_*length2_);
	    btTransform rot(btQuaternion(btVector3(1,0,0),btRadians(90)));
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

bool bodies::Box::intersectsRay(const btVector3& origin, const btVector3& dir, std::vector<btVector3> *intersections, unsigned int count) const
{  
    if (distanceSQR(center_, origin, dir) > radius2_) return false;
    
    double t_near = -INFINITY;
    double t_far  = INFINITY;
    
    for (int i = 0; i < 3; i++)
    {
	const btVector3 &vN = i == 0 ? normalL_ : (i == 1 ? normalW_ : normalH_);
	double dp = vN.dot(dir);
	
	if (fabs(dp) > ZERO)
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
	if (t_far - t_near > ZERO)
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

bool bodies::ConvexMesh::containsPoint(const btVector3 &p, bool verbose) const
{
    if (bounding_box_.containsPoint(p))
    {
	btVector3 ip(i_pose_ * p);
	ip = mesh_center_ + (ip - mesh_center_) * scale_;
	return isPointInsidePlanes(ip);
    }
    else
	return false;
}

void bodies::ConvexMesh::useDimensions(const shapes::Shape *shape)
{  
    const shapes::Mesh *mesh = static_cast<const shapes::Mesh*>(shape);
    
    double maxX = -INFINITY, maxY = -INFINITY, maxZ = -INFINITY;
    double minX =  INFINITY, minY =  INFINITY, minZ  = INFINITY;
    
    for(unsigned int i = 0; i < mesh->vertexCount ; ++i)
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
    
    shapes::Box *box_shape = new shapes::Box(maxX - minX, maxY - minY, maxZ - minZ);
    bounding_box_.setDimensions(box_shape);
    delete box_shape;
    
    box_offset_.setValue((minX + maxX) / 2.0, (minY + maxY) / 2.0, (minZ + maxZ) / 2.0);
    
    planes_.clear();
    triangles_.clear();
    vertices_.clear();
    mesh_radiusB_ = 0.0;
    mesh_center_.setValue(btScalar(0), btScalar(0), btScalar(0));
    
    double xdim = maxX - minX;
    double ydim = maxY - minY;
    double zdim = maxZ - minZ;
    
    double pose1;
    double pose2;
    
    unsigned int off1;
    unsigned int off2;
    
    double cyl_length;
    double maxdist = -INFINITY;
    
    if(xdim > ydim && xdim > zdim) {
	
	off1 = 1;
	off2 = 2;
	pose1 = box_offset_.y();
	pose2 = box_offset_.z();
	cyl_length = xdim;
    } else if(ydim > zdim) {
	off1 = 0;
	off2 = 2;
	pose1 = box_offset_.x();
	pose2 = box_offset_.z();
	cyl_length = ydim;
    } else {
	off1 = 0;
	off2 = 1;
	pose1 = box_offset_.x();
	pose2 = box_offset_.y();
	cyl_length = zdim;
    }
    
    
    btVector3 *vertices = new btVector3[mesh->vertexCount];
    for(unsigned int i = 0; i < mesh->vertexCount ; ++i)
    {
	vertices[i].setX(mesh->vertices[3 * i    ]);
	vertices[i].setY(mesh->vertices[3 * i + 1]);
	vertices[i].setZ(mesh->vertices[3 * i + 2]);
	
	double dista = mesh->vertices[3 * i + off1]-pose1;
	double distb = mesh->vertices[3 * i + off2]-pose2;
	double dist = sqrt(((dista*dista)+(distb*distb)));
	if(dist > maxdist) {
	    maxdist = dist;
	}
    }
    bounding_cylinder_.radius = maxdist;
    bounding_cylinder_.length = cyl_length;
    
    HullDesc hd(QF_TRIANGLES, mesh->vertexCount, vertices);
    HullResult hr;
    HullLibrary hl;
    if (hl.CreateConvexHull(hd, hr) == QE_OK)
    {
	//	std::cout << "Convex hull has " << hr.m_OutputVertices.size() << " vertices (down from " << mesh->vertexCount << "), " << hr.mNumFaces << " faces" << std::endl;
	
	vertices_.reserve(hr.m_OutputVertices.size());
	btVector3 sum(0, 0, 0);	
	
	for (int j = 0 ; j < hr.m_OutputVertices.size() ; ++j)
	{
	    vertices_.push_back(hr.m_OutputVertices[j]);
	    sum = sum + hr.m_OutputVertices[j];
	}
	
	mesh_center_ = sum / (double)(hr.m_OutputVertices.size());
	for (unsigned int j = 0 ; j < vertices_.size() ; ++j)
	{
	    double dist = vertices_[j].distance2(mesh_center_);
	    if (dist > mesh_radiusB_)
		mesh_radiusB_ = dist;
	}
	
	mesh_radiusB_ = sqrt(mesh_radiusB_);
	triangles_.reserve(hr.m_Indices.size());
	for (unsigned int j = 0 ; j < hr.mNumFaces ; ++j)
	{
	    const btVector3 &p1 = hr.m_OutputVertices[hr.m_Indices[j * 3    ]];
	    const btVector3 &p2 = hr.m_OutputVertices[hr.m_Indices[j * 3 + 1]];
	    const btVector3 &p3 = hr.m_OutputVertices[hr.m_Indices[j * 3 + 2]];
	    
	    btVector3 edge1 = (p2 - p1);
	    btVector3 edge2 = (p3 - p1);
	    
	    edge1.normalize();
	    edge2.normalize();
	    
	    btVector3 planeNormal = edge1.cross(edge2);
	    
	    if (planeNormal.length2() > btScalar(1e-6))
	    {
		planeNormal.normalize();
		btVector4 planeEquation(planeNormal.getX(), planeNormal.getY(), planeNormal.getZ(), -planeNormal.dot(p1));
		
		unsigned int behindPlane = countVerticesBehindPlane(planeEquation);
		if (behindPlane > 0)
		{
		    btVector4 planeEquation2(-planeEquation.getX(), -planeEquation.getY(), -planeEquation.getZ(), -planeEquation.getW());
		    unsigned int behindPlane2 = countVerticesBehindPlane(planeEquation2);
		    if (behindPlane2 < behindPlane)
		    {
			planeEquation.setValue(planeEquation2.getX(), planeEquation2.getY(), planeEquation2.getZ(), planeEquation2.getW());
			behindPlane = behindPlane2;
		    }
		}
		
		if (behindPlane > 0)
		    ROS_DEBUG("Approximate plane: %d of %d points are behind the plane", behindPlane, (int)vertices_.size());
		
		planes_.push_back(planeEquation);
		
		triangles_.push_back(hr.m_Indices[j * 3 + 0]);
		triangles_.push_back(hr.m_Indices[j * 3 + 1]);
		triangles_.push_back(hr.m_Indices[j * 3 + 2]);
	    }
	}
    }
    else
	ROS_ERROR("Unable to compute convex hull.");
    
    hl.ReleaseResult(hr);    
    delete[] vertices;
    
}

void bodies::ConvexMesh::updateInternalData(void) 
{
    btTransform pose = pose_;
    pose.setOrigin(pose_ * box_offset_);
    bounding_box_.setPose(pose);
    bounding_box_.setPadding(padding_);
    bounding_box_.setScale(scale_);
    
    i_pose_ = pose_.inverse();
    center_ = pose_ * mesh_center_;
    radiusB_ = mesh_radiusB_ * scale_ + padding_;
    radiusBSqr_ = radiusB_ * radiusB_;
    
    scaled_vertices_.resize(vertices_.size());
    for (unsigned int i = 0 ; i < vertices_.size() ; ++i)
    {
	btVector3 v(vertices_[i] - mesh_center_);
	btScalar l = v.length();
	scaled_vertices_[i] = mesh_center_ + v * (scale_ + (l > ZERO ? padding_ / l : 0.0));
    }
}

void bodies::ConvexMesh::computeBoundingSphere(BoundingSphere &sphere) const
{
    sphere.center = center_;
    sphere.radius = radiusB_;
}

void bodies::ConvexMesh::computeBoundingCylinder(BoundingCylinder &cylinder) const
{
    cylinder.length = bounding_cylinder_.length;
    cylinder.radius = bounding_cylinder_.radius;
    //need to do rotation correctly to get pose, which bounding box does
    BoundingCylinder cyl;
    bounding_box_.computeBoundingCylinder(cyl);
    cylinder.pose = cyl.pose;
}

bool bodies::ConvexMesh::isPointInsidePlanes(const btVector3& point) const
{
    unsigned int numplanes = planes_.size();
    for (unsigned int i = 0 ; i < numplanes ; ++i)
    {
	const btVector4& plane = planes_[i];
	btScalar dist = plane.dot(point) + plane.getW() - padding_ - btScalar(1e-6);
	if (dist > btScalar(0))
	    return false;
    }
    return true;
}

unsigned int bodies::ConvexMesh::countVerticesBehindPlane(const btVector4& planeNormal) const
{
    unsigned int numvertices = vertices_.size();
    unsigned int result = 0;
    for (unsigned int i = 0 ; i < numvertices ; ++i)
    {
	btScalar dist = planeNormal.dot(vertices_[i]) + planeNormal.getW() - btScalar(1e-6);
	if (dist > btScalar(0))
	    result++;
    }
    return result;
}

double bodies::ConvexMesh::computeVolume(void) const
{
    double volume = 0.0;
    for (unsigned int i = 0 ; i < triangles_.size() / 3 ; ++i)
    {
	const btVector3 &v1 = vertices_[triangles_[3*i + 0]];
	const btVector3 &v2 = vertices_[triangles_[3*i + 1]];
	const btVector3 &v3 = vertices_[triangles_[3*i + 2]];
	volume += v1.x()*v2.y()*v3.z() + v2.x()*v3.y()*v1.z() + v3.x()*v1.y()*v2.z() - v1.x()*v3.y()*v2.z() - v2.x()*v1.y()*v3.z() - v3.x()*v2.y()*v1.z();
    }
    return fabs(volume)/6.0;
}

bool bodies::ConvexMesh::intersectsRay(const btVector3& origin, const btVector3& dir, std::vector<btVector3> *intersections, unsigned int count) const
{
    if (distanceSQR(center_, origin, dir) > radiusBSqr_) return false;
    if (!bounding_box_.intersectsRay(origin, dir)) return false;
    
    // transform the ray into the coordinate frame of the mesh
    btVector3 orig(i_pose_ * origin);
    btVector3 dr(i_pose_.getBasis() * dir);
    
    std::vector<detail::intersc> ipts;
    
    bool result = false;
    
    // for each triangle 
    const unsigned int nt = triangles_.size() / 3;
    for (unsigned int i = 0 ; i < nt ; ++i)
    {
	btScalar tmp = planes_[i].dot(dr);
	if (fabs(tmp) > ZERO)
	{
	    double t = -(planes_[i].dot(orig) + planes_[i].getW()) / tmp;
	    if (t > 0.0)
	    {
		const int i3 = 3 * i;
		const int v1 = triangles_[i3 + 0];
		const int v2 = triangles_[i3 + 1];
		const int v3 = triangles_[i3 + 2];
		
		const btVector3 &a = scaled_vertices_[v1];
		const btVector3 &b = scaled_vertices_[v2];
		const btVector3 &c = scaled_vertices_[v3];
		
		btVector3 cb(c - b);
		btVector3 ab(a - b);
		
		// intersection of the plane defined by the triangle and the ray
		btVector3 P(orig + dr * t);
		
		// check if it is inside the triangle
		btVector3 pb(P - b);
		btVector3 c1(cb.cross(pb));
		btVector3 c2(cb.cross(ab));
		if (c1.dot(c2) < 0.0)
		    continue;
		
		btVector3 ca(c - a);
		btVector3 pa(P - a);
		btVector3 ba(-ab);
		
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
                               const std::vector<btTransform>& poses,
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
    rsqrs_.clear();
}

void bodies::BodyVector::addBody(Body *body)
{
    bodies_.push_back(body);
    BoundingSphere sphere;
    body->computeBoundingSphere(sphere);
    rsqrs_.push_back(sphere.radius * sphere.radius);
}

void bodies::BodyVector::addBody(const shapes::Shape *shape, const btTransform& pose, double padding)
{
    bodies::Body* body = bodies::createBodyFromShape(shape);
    body->setPose(pose);
    body->setPadding(padding);
    addBody(body);
}

void bodies::BodyVector::setPose(unsigned int i, const btTransform& pose)
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

bodies::BoundingSphere bodies::BodyVector::getBoundingSphere(unsigned int i) const
{
    BoundingSphere sphere;
    if (i >= bodies_.size())
    {
	ROS_ERROR("There is no body at index %u", i);
	sphere.radius = 0.0;
	sphere.center = btVector3(0.0, 0.0, 0.0);
    }
    else
	bodies_[i]->computeBoundingSphere(sphere);
    return sphere;
}

double bodies::BodyVector::getBoundingSphereRadiusSquared(unsigned int i) const
{
    if(i >= rsqrs_.size()) 
    {
	ROS_ERROR("There is no body at index %u", i);
	return -1.0;
    }
    else
	return rsqrs_[i];
}

bool bodies::BodyVector::containsPoint(const btVector3 &p, bool verbose) const
{
    for (unsigned int i = 0 ; i < bodies_.size() ; ++i) 
	if (bodies_[i]->containsPoint(p, verbose))
	    return true;
    return false;
}
