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

#include "geometric_shapes/shape_operations.h"

#include <cstdio>
#include <cmath>
#include <algorithm>
#include <set>
#include <float.h>

#include <ros/console.h>
#include <resource_retriever/retriever.h>

#include <assimp/assimp.hpp>
#include <assimp/aiPostProcess.h>

#include <Eigen/Geometry>

#include <shape_conversions/shape_to_marker.h>

namespace shapes
{

namespace detail
{
struct myVertex
{
  Eigen::Vector3d    point;
  unsigned int index;
};

struct ltVertexValue
{
  bool operator()(const myVertex &p1, const myVertex &p2) const
  {
    const Eigen::Vector3d &v1 = p1.point;
    const Eigen::Vector3d &v2 = p2.point;
    if (v1.x() < v2.x())
      return true;
    if (v1.x() > v2.x())
      return false;
    if (v1.y() < v2.y())
      return true;
    if (v1.y() > v2.y())
      return false;
    if (v1.z() < v2.z())
      return true;
    return false;
  }
};

struct ltVertexIndex
{
  bool operator()(const myVertex &p1, const myVertex &p2) const
  {
    return p1.index < p2.index;
  }
};

}

Mesh* createMeshFromVertices(const std::vector<Eigen::Vector3d> &vertices, const std::vector<unsigned int> &triangles)
{
  unsigned int nt = triangles.size() / 3;
  Mesh *mesh = new Mesh(vertices.size(), nt);
  for (unsigned int i = 0 ; i < vertices.size() ; ++i)
  {
    mesh->vertices[3 * i    ] = vertices[i].x();
    mesh->vertices[3 * i + 1] = vertices[i].y();
    mesh->vertices[3 * i + 2] = vertices[i].z();
  }

  std::copy(triangles.begin(), triangles.end(), mesh->triangles);

  // compute normals
  for (unsigned int i = 0 ; i < nt ; ++i)
  {
    Eigen::Vector3d s1 = vertices[triangles[i * 3    ]] - vertices[triangles[i * 3 + 1]];
    Eigen::Vector3d s2 = vertices[triangles[i * 3 + 1]] - vertices[triangles[i * 3 + 2]];
    Eigen::Vector3d normal = s1.cross(s2);
    normal.normalize();
    mesh->normals[3 * i    ] = normal.x();
    mesh->normals[3 * i + 1] = normal.y();
    mesh->normals[3 * i + 2] = normal.z();
  }
  return mesh;
}

Mesh* createMeshFromVertices(const std::vector<Eigen::Vector3d> &source)
{
  if (source.size() < 3)
    return NULL;

  std::set<detail::myVertex, detail::ltVertexValue> vertices;
  std::vector<unsigned int>                         triangles;

  for (unsigned int i = 0 ; i < source.size() / 3 ; ++i)
  {
    // check if we have new vertices
    detail::myVertex vt;

    vt.point = source[3 * i];
    std::set<detail::myVertex, detail::ltVertexValue>::iterator p1 = vertices.find(vt);
    if (p1 == vertices.end())
    {
      vt.index = vertices.size();
      vertices.insert(vt);
    }
    else
      vt.index = p1->index;
    triangles.push_back(vt.index);

    vt.point = source[3 * i + 1];
    std::set<detail::myVertex, detail::ltVertexValue>::iterator p2 = vertices.find(vt);
    if (p2 == vertices.end())
    {
      vt.index = vertices.size();
      vertices.insert(vt);
    }
    else
      vt.index = p2->index;
    triangles.push_back(vt.index);

    vt.point = source[3 * i + 2];
    std::set<detail::myVertex, detail::ltVertexValue>::iterator p3 = vertices.find(vt);
    if (p3 == vertices.end())
    {
      vt.index = vertices.size();
      vertices.insert(vt);
    }
    else
      vt.index = p3->index;

    triangles.push_back(vt.index);
  }

  // sort our vertices
  std::vector<detail::myVertex> vt;
  vt.insert(vt.begin(), vertices.begin(), vertices.end());
  std::sort(vt.begin(), vt.end(), detail::ltVertexIndex());

  // copy the data to a mesh structure
  unsigned int nt = triangles.size() / 3;

  Mesh *mesh = new Mesh(vt.size(), nt);
  for (unsigned int i = 0 ; i < vt.size() ; ++i)
  {
    mesh->vertices[3 * i    ] = vt[i].point.x();
    mesh->vertices[3 * i + 1] = vt[i].point.y();
    mesh->vertices[3 * i + 2] = vt[i].point.z();
  }

  std::copy(triangles.begin(), triangles.end(), mesh->triangles);

  // compute normals
  for (unsigned int i = 0 ; i < nt ; ++i)
  {
    Eigen::Vector3d s1 = vt[triangles[i * 3    ]].point - vt[triangles[i * 3 + 1]].point;
    Eigen::Vector3d s2 = vt[triangles[i * 3 + 1]].point - vt[triangles[i * 3 + 2]].point;
    Eigen::Vector3d normal = s1.cross(s2);
    normal.normalize();
    mesh->normals[3 * i    ] = normal.x();
    mesh->normals[3 * i + 1] = normal.y();
    mesh->normals[3 * i + 2] = normal.z();
  }

  return mesh;
}

Mesh* createMeshFromResource(const std::string& resource, const Eigen::Vector3d &scale)
{
  resource_retriever::Retriever retriever;
  resource_retriever::MemoryResource res;
  try
  {
    res = retriever.get(resource);
  } 
  catch (resource_retriever::Exception& e)
  {
    ROS_ERROR("%s", e.what());
    return NULL;
  }

  if (res.size == 0)
  {
    ROS_WARN("Retrieved empty mesh for resource '%s'", resource.c_str());
    return NULL;
  }
  
  // Create an instance of the Importer class
  Assimp::Importer importer;
  
  // try to get a file extension
  std::string hint;
  std::size_t pos = resource.find_last_of(".");
  if (pos != std::string::npos)
  {
    hint = resource.substr(pos + 1);
    std::transform(hint.begin(), hint.end(), hint.begin(), ::tolower);
    
    // temp hack until everything is stl not stlb
    if (hint.find("stl") != std::string::npos)
      hint = "stl";
  }
  
  // And have it read the given file with some postprocessing
  const aiScene* scene = importer.ReadFileFromMemory(reinterpret_cast<void*>(res.data.get()), res.size,
                                                     aiProcess_Triangulate            |
                                                     aiProcess_JoinIdenticalVertices  |
                                                     aiProcess_SortByPType            |
                                                     aiProcess_OptimizeGraph          |
                                                     aiProcess_OptimizeMeshes, hint.c_str());
  if (!scene)
  {
    ROS_WARN_STREAM("Assimp reports no scene in " << resource);
    return NULL;
  }
  return createMeshFromAsset(scene, scale, resource);
}

static void extractMeshData(const aiScene *scene, const aiNode *node, aiMatrix4x4 transform, const Eigen::Vector3d &scale,
                            std::vector<Eigen::Vector3d> &vertices, std::vector<unsigned int> &triangles)
{
  transform *= node->mTransformation;
  for (unsigned int j = 0 ; j < node->mNumMeshes; ++j)
  {
    const aiMesh* a = scene->mMeshes[node->mMeshes[j]];
    for (unsigned int i = 0 ; i < a->mNumVertices ; ++i)
    {
      const aiVector3D &v = transform * a->mVertices[i];
      vertices.push_back(Eigen::Vector3d(v.x * scale.x(), v.y * scale.y(), v.z * scale.z()));
    }
    for (unsigned int i = 0 ; i < a->mNumFaces ; ++i)
      if (a->mFaces[i].mNumIndices == 3)
      {
        triangles.push_back(a->mFaces[i].mIndices[0]);
        triangles.push_back(a->mFaces[i].mIndices[1]);
        triangles.push_back(a->mFaces[i].mIndices[2]);
      }
  }
  
  for (unsigned int n = 0; n < node->mNumChildren; ++n)
    extractMeshData(scene, node->mChildren[n], transform, scale, vertices, triangles);
}

Mesh* createMeshFromAsset(const aiScene* scene, const Eigen::Vector3d &scale, const std::string &resource_name)
{
  if (!scene->HasMeshes())
  {
    ROS_WARN_STREAM("Assimp reports scene in " << resource_name << " has no meshes");
    return NULL;
  }
  std::vector<Eigen::Vector3d> vertices;
  std::vector<unsigned int> triangles;
  extractMeshData(scene, scene->mRootNode, aiMatrix4x4(), scale, vertices, triangles);
  if (vertices.empty())
  {
    ROS_WARN_STREAM("There are no vertices in the scene " << resource_name);
    return NULL;
  }
  if (triangles.empty())
  {
    ROS_WARN_STREAM("There are no triangles in the scene " << resource_name);
    return NULL;
  }
  
  return createMeshFromVertices(vertices, triangles);
}

Shape* constructShapeFromMsg(const shape_msgs::Shape &shape_msg)
{
  Shape *shape = NULL;
  if (shape_msg.type == shape_msgs::Shape::SPHERE)
  {
    if (shape_msg.dimensions.size() != 1)
      ROS_ERROR("Unexpected number of dimensions in sphere definition");
    else
      shape = new Sphere(shape_msg.dimensions[0]);
  }
  else
    if (shape_msg.type == shape_msgs::Shape::BOX)
    {
      if (shape_msg.dimensions.size() != 3)
        ROS_ERROR("Unexpected number of dimensions in box definition");
      else
        shape = new Box(shape_msg.dimensions[0], shape_msg.dimensions[1], shape_msg.dimensions[2]);
    }
    else
      if (shape_msg.type == shape_msgs::Shape::CYLINDER)
      {
        if (shape_msg.dimensions.size() != 2)
          ROS_ERROR("Unexpected number of dimensions in cylinder definition");
        else
          shape = new Cylinder(shape_msg.dimensions[0], shape_msg.dimensions[1]);
      }
      else
        if (shape_msg.type == shape_msgs::Shape::MESH)
        {
          if (shape_msg.dimensions.size() != 0)
            ROS_ERROR("Unexpected number of dimensions in mesh definition");
          else
          {
            if (shape_msg.triangles.size() % 3 != 0)
              ROS_ERROR("Number of triangle indices is not divisible by 3");
            else
            {
              if (shape_msg.triangles.empty() || shape_msg.vertices.empty())
                ROS_ERROR("Mesh definition is empty");
              else
              {
                std::vector<Eigen::Vector3d>    vertices(shape_msg.vertices.size());
                std::vector<unsigned int> triangles(shape_msg.triangles.size());
                for (unsigned int i = 0 ; i < shape_msg.vertices.size() ; ++i)
                  vertices[i] = Eigen::Vector3d(shape_msg.vertices[i].x, shape_msg.vertices[i].y, shape_msg.vertices[i].z);
                for (unsigned int i = 0 ; i < shape_msg.triangles.size() ; ++i)
                  triangles[i] = shape_msg.triangles[i];
                shape = createMeshFromVertices(vertices, triangles);
              }
            }
          }
        }

  if (shape == NULL)
    ROS_ERROR("Unable to construct shape corresponding to shape_msgect of type %d", (int)shape_msg.type);

  return shape;
}

bool constructMarkerFromShape(const Shape* shape, visualization_msgs::Marker &mk, bool use_mesh_triangle_list)
{
  shape_msgs::Shape shape_msg;
  if (constructMsgFromShape(shape, shape_msg))
  {
    bool ok = false;
    try
    {
      shape_conversions::constructMarkerFromShape(shape_msg, mk, use_mesh_triangle_list);
      ok = true;
    }
    catch (std::runtime_error &ex)
    {
      ROS_ERROR("%s", ex.what());
    }
    if (ok)
      return true;
  }
  return false;
}

bool constructMsgFromShape(const Shape* shape, shape_msgs::Shape &shape_msg)
{
  shape_msg.dimensions.clear();
  shape_msg.vertices.clear();
  shape_msg.triangles.clear();
  if (shape->type == SPHERE)
  {
    shape_msg.type = shape_msgs::Shape::SPHERE;
    shape_msg.dimensions.push_back(static_cast<const Sphere*>(shape)->radius);
  }
  else
    if (shape->type == BOX)
    {
      shape_msg.type = shape_msgs::Shape::BOX;
      const double* sz = static_cast<const Box*>(shape)->size;
      shape_msg.dimensions.push_back(sz[0]);
      shape_msg.dimensions.push_back(sz[1]);
      shape_msg.dimensions.push_back(sz[2]);
    }
    else
      if (shape->type == CYLINDER)
      {
        shape_msg.type = shape_msgs::Shape::CYLINDER;
        shape_msg.dimensions.push_back(static_cast<const Cylinder*>(shape)->radius);
        shape_msg.dimensions.push_back(static_cast<const Cylinder*>(shape)->length);
      }
      else
        if (shape->type == MESH)
        {
          shape_msg.type = shape_msgs::Shape::MESH;

          const Mesh *mesh = static_cast<const Mesh*>(shape);
          const unsigned int t3 = mesh->triangle_count * 3;

          shape_msg.vertices.resize(mesh->vertex_count);
          shape_msg.triangles.resize(t3);

          for (unsigned int i = 0 ; i < mesh->vertex_count ; ++i)
          {
            unsigned int i3 = i * 3;
            shape_msg.vertices[i].x = mesh->vertices[i3];
            shape_msg.vertices[i].y = mesh->vertices[i3 + 1];
            shape_msg.vertices[i].z = mesh->vertices[i3 + 2];
          }

          for (unsigned int i = 0 ; i < t3  ; ++i)
            shape_msg.triangles[i] = mesh->triangles[i];
        }
        else
        {
          ROS_ERROR("Unable to construct shape message for shape of type %d", (int)shape->type);
          return false;
        }

  return true;
}

StaticShape* constructShapeFromMsg(const shape_msgs::StaticShape &shape_msg)
{
  StaticShape *shape = NULL;
  if (shape_msg.type == shape_msgs::StaticShape::PLANE)
  {
    if (shape_msg.dimensions.size() == 4)
      shape = new Plane(shape_msg.dimensions[0], shape_msg.dimensions[1], shape_msg.dimensions[2], shape_msg.dimensions[3]);
    else
      ROS_ERROR("Unexpected number of dimensions in plane definition (got %d, expected 4)", (int)shape_msg.dimensions.size());
  }
  else
    ROS_ERROR("Unknown static shape type: %d", (int)shape_msg.type);
  return shape;
}

bool constructMsgFromShape(const StaticShape* shape, shape_msgs::StaticShape &shape_msg)
{
  shape_msg.dimensions.clear();
  if (shape->type == PLANE)
  {
    shape_msg.type = shape_msgs::StaticShape::PLANE;
    const Plane *p = static_cast<const Plane*>(shape);
    shape_msg.dimensions.push_back(p->a);
    shape_msg.dimensions.push_back(p->b);
    shape_msg.dimensions.push_back(p->c);
    shape_msg.dimensions.push_back(p->d);
  }
  else
  {
    ROS_ERROR("Unable to construct shape message for shape of type %d", (int)shape->type);
    return false;
  }
  return true;
}

}
