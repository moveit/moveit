/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage, Inc.
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

#include "geometric_shapes/shapes.h"
#include <ros/console.h>

shapes::Shape* shapes::Sphere::clone(void) const
{
    return new Sphere(radius);
}

shapes::Shape* shapes::Cylinder::clone(void) const
{
    return new Cylinder(radius, length);
}

shapes::Shape* shapes::Box::clone(void) const
{
    return new Box(size[0], size[1], size[2]);
}

shapes::Shape* shapes::Mesh::clone(void) const
{
    Mesh *dest = new Mesh(vertex_count, triangle_count);
    unsigned int n = 3 * vertex_count;
    for (unsigned int i = 0 ; i < n ; ++i)
        dest->vertices[i] = vertices[i];
    n = 3 * triangle_count;
    for (unsigned int i = 0 ; i < n ; ++i)
        dest->triangles[i] = triangles[i];
    if (normals)
        for (unsigned int i = 0 ; i < n ; ++i)
            dest->normals[i] = normals[i];
    else
    {
        delete[] dest->normals;
        dest->normals = NULL;
    }
    return dest;
}

shapes::StaticShape* shapes::Plane::clone(void) const
{
    return new Plane(a, b, c, d);
}

void shapes::Shape::scale(double scale)
{
    scaleAndPadd(scale, 0.0);
}

void shapes::Shape::padd(double padding)
{
    scaleAndPadd(1.0, padding);
}

void shapes::Sphere::scaleAndPadd(double scale, double padding)
{
    radius = radius * scale + padding;
}

void shapes::Cylinder::scaleAndPadd(double scale, double padding)
{
    radius = radius * scale + padding;
    length = length * scale + 2.0 * padding;
}

void shapes::Box::scaleAndPadd(double scale, double padding)
{
    double p2 = padding * 2.0;
    size[0] = size[0] * scale + p2;
    size[1] = size[1] * scale + p2;
    size[2] = size[2] * scale + p2;
}

void shapes::Mesh::scaleAndPadd(double scale, double padding)
{
    // find the center of the mesh
    double sx = 0.0, sy = 0.0, sz = 0.0;
    for (unsigned int i = 0 ; i < vertex_count ; ++i)
    {
        unsigned int i3 = i * 3;
        sx += vertices[i3];
        sy += vertices[i3 + 1];
        sz += vertices[i3 + 2];
    }
    sx /= (double)vertex_count;
    sy /= (double)vertex_count;
    sz /= (double)vertex_count;

    // scale the mesh
    for (unsigned int i = 0 ; i < vertex_count ; ++i)
    {
        unsigned int i3 = i * 3;

        // vector from center to the vertex
        double dx = vertices[i3] - sx;
        double dy = vertices[i3 + 1] - sy;
        double dz = vertices[i3 + 2] - sz;

        // length of vector
        double norm = sqrt(dx * dx + dy * dy + dz * dz);
        if (norm > 1e-6)
        {
            double fact = scale + padding/norm;
            vertices[i3] = sx + dx * fact;
            vertices[i3 + 1] = sy + dy * fact;
            vertices[i3 + 2] = sz + dz * fact;
        }
        else
        {
            double ndx = ((dx > 0) ? dx+padding : dx-padding);
            double ndy = ((dy > 0) ? dy+padding : dy-padding);
            double ndz = ((dz > 0) ? dz+padding : dz-padding);
            vertices[i3] = sx + ndx;
            vertices[i3 + 1] = sy + ndy;
            vertices[i3 + 2] = sz + ndz;
        }
    }
}

void shapes::Shape::print(std::ostream &out) const
{
    out << this << std::endl;
}

void shapes::StaticShape::print(std::ostream &out) const
{
    out << this << std::endl;
}

void shapes::Sphere::print(std::ostream &out) const
{
    out << "Sphere[radius=" << radius << "]" << std::endl;
}

void shapes::Cylinder::print(std::ostream &out) const
{
    out << "Cylinder[radius=" << radius << ", length=" << length << "]" << std::endl;
}

void shapes::Box::print(std::ostream &out) const
{
    out << "Box[x=length=" << size[0] << ", y=width=" << size[1] << "z=height=" << size[2] << "]" << std::endl;
}

void shapes::Mesh::print(std::ostream &out) const
{
    out << "Mesh[vertices=" << vertex_count << ", triangles=" << triangle_count << "]" << std::endl;
}

void shapes::Plane::print(std::ostream &out) const
{
    out << "Plane[a=" << a << ", b=" << b << ", c=" << c << ", d=" << d << "]" << std::endl;
}
