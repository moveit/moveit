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

#ifndef GEOMETRIC_SHAPES_SHAPE_OPERATIONS_
#define GEOMETRIC_SHAPES_SHAPE_OPERATIONS_

#include "geometric_shapes/shapes.h"
#include <moveit_msgs/Shape.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <LinearMath/btVector3.h>
#include <assimp/aiMesh.h>
#include <iostream>

namespace shapes
{

    /** \brief Load a mesh from a set of vertices. Triangles are
        constructed using index values from the triangles
        vector. Triangle k has vertices at index values triangles[3k],
        triangles[3k+1], triangles[3k+2]  */
    Mesh* createMeshFromVertices(const std::vector<btVector3> &vertices, const std::vector<unsigned int> &triangles);

    /** \brief Load a mesh from a set of vertices. Every 3 vertices
        are considered a triangle. Repeating vertices are identified
        and the set of triangle indices is constructed. The normal at
        each triangle is also computed */
    Mesh* createMeshFromVertices(const std::vector<btVector3> &source);

    /** \brief Load a mesh from a file that contains a mesh that can be loaded by assimp */
    Mesh* createMeshFromFilename(const std::string& filename, const btVector3 &scale = btVector3(1.0, 1.0, 1.0));

    /** \brief Load a mesh from an assimp datastructure */
    Mesh* createMeshFromAsset(const aiMesh* a, const aiMatrix4x4& transform, const btVector3& scale);

    /** \brief Create a copy of a shape */
    Shape* cloneShape(const Shape *shape);

    /** \brief Create a copy of a static shape */
    StaticShape* cloneShape(const StaticShape *shape);

    /** \brief Create a copy of a vector of shapes */
    std::vector<Shape*> cloneShapeVector(const std::vector<Shape*> &shapes);

    /** \brief Free the memory for a vector of shapes */
    void deleteShapeVector(std::vector<Shape*>& shapes);

    /** \brief Construct the shape that corresponds to the message. Return NULL on failure. */
    Shape* constructShapeFromMsg(const moveit_msgs::Shape &shape_msg);

    /** \brief Construct the message that corresponds to the shape and optionally add padding. Return false on failure. */
    bool constructMsgFromShape(const Shape* shape, moveit_msgs::Shape &shape_msg, double padding = 0.0);

    /** \brief Construct the marker that corresponds to the shape and optionally add padding. Return false on failure. */
    bool constructMarkerFromShape(const Shape* shape, visualization_msgs::Marker &mk, double padding = 0.0);

    /** \brief Construct the marker that corresponds to the shape shape message.  Return false on failure. */
    bool constructMarkerFromShape(const moveit_msgs::Shape &shape, visualization_msgs::Marker &mk);

    /** \brief Print information about this shape */
    void printShape(const Shape *shape, std::ostream &out = std::cout);

}

#endif
