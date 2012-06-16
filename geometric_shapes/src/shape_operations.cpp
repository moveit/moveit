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

#include <shape_tools/shape_to_marker.h>

namespace shapes
{

namespace detail
{
struct myVertex
{
  Eigen::Vector3d point;
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

Shape* constructShapeFromMsg(const shape_msgs::Plane &shape_msg)
{ 
  return new Plane(shape_msg.coef[0], shape_msg.coef[1], shape_msg.coef[2], shape_msg.coef[3]);
}

Shape* constructShapeFromMsg(const shape_msgs::Mesh &shape_msg)
{      
  if (shape_msg.triangles.empty() || shape_msg.vertices.empty())
  {
    ROS_WARN("Mesh definition is empty");
    return NULL;
  }
  else
  {
    std::vector<Eigen::Vector3d> vertices(shape_msg.vertices.size());
    std::vector<unsigned int> triangles(shape_msg.triangles.size() * 3);
    for (unsigned int i = 0 ; i < shape_msg.vertices.size() ; ++i)
      vertices[i] = Eigen::Vector3d(shape_msg.vertices[i].x, shape_msg.vertices[i].y, shape_msg.vertices[i].z);
    for (unsigned int i = 0 ; i < shape_msg.triangles.size() ; ++i)
    {
      unsigned int i3 = i * 3;
      triangles[i3++] = shape_msg.triangles[i].vertex_indices[0];
      triangles[i3++] = shape_msg.triangles[i].vertex_indices[1];
      triangles[i3] = shape_msg.triangles[i].vertex_indices[2];
    }
    return createMeshFromVertices(vertices, triangles);
  }
}

Shape* constructShapeFromMsg(const shape_msgs::SolidPrimitive &shape_msg)
{
  Shape *shape = NULL;
  if (shape_msg.type == shape_msgs::SolidPrimitive::SPHERE)
  {
    shape = new Sphere(shape_msg.dimensions.x / 2.0);
  }
  else
    if (shape_msg.type == shape_msgs::SolidPrimitive::BOX)
    {
      shape = new Box(shape_msg.dimensions.x, shape_msg.dimensions.y, shape_msg.dimensions.z);
    }
    else
      if (shape_msg.type == shape_msgs::SolidPrimitive::CYLINDER)
      {
        shape = new Cylinder(shape_msg.dimensions.x / 2.0, shape_msg.dimensions.z);
      }
      else
        if (shape_msg.type == shape_msgs::SolidPrimitive::CONE)
        {
          shape = new Cone(shape_msg.dimensions.x / 2.0, shape_msg.dimensions.z);
        }
  if (shape == NULL)
    ROS_ERROR("Unable to construct shape corresponding to shape_msgect of type %d", (int)shape_msg.type);
  
  return shape;
}

class ShapeVisitorAlloc : public boost::static_visitor<Shape*>
{
public:
  
  Shape* operator()(const shape_msgs::Plane &shape_msg) const
  {
    return constructShapeFromMsg(shape_msg);
  }
  
  Shape* operator()(const shape_msgs::Mesh &shape_msg) const
  {
    return constructShapeFromMsg(shape_msg);
  }
  
  Shape* operator()(const shape_msgs::SolidPrimitive &shape_msg) const
  {
    return constructShapeFromMsg(shape_msg);
  }
};

Shape* constructShapeFromMsg(const ShapeMsg &shape_msg)
{
  return boost::apply_visitor(ShapeVisitorAlloc(), shape_msg);
}

class ShapeVisitorMarker : public boost::static_visitor<void>
{
public:
  
  ShapeVisitorMarker(visualization_msgs::Marker *marker, bool use_mesh_triangle_list) :
    boost::static_visitor<void>(),
    use_mesh_triangle_list_(use_mesh_triangle_list),
    marker_(marker)
  {
  }
  
  void operator()(const shape_msgs::Plane &shape_msg) const
  {
    throw std::runtime_error("No visual markers can be constructed for planes");
  }
  
  void operator()(const shape_msgs::Mesh &shape_msg) const
  {
    shape_tools::constructMarkerFromShape(shape_msg, *marker_, use_mesh_triangle_list_);
  }
  
  void operator()(const shape_msgs::SolidPrimitive &shape_msg) const
  {
    shape_tools::constructMarkerFromShape(shape_msg, *marker_);
  }
  
private:
  
  bool use_mesh_triangle_list_;
  visualization_msgs::Marker *marker_;
};

bool constructMarkerFromShape(const Shape* shape, visualization_msgs::Marker &marker, bool use_mesh_triangle_list)
{
  ShapeMsg shape_msg;
  if (constructMsgFromShape(shape, shape_msg))
  {
    bool ok = false;
    try
    { 
      boost::apply_visitor(ShapeVisitorMarker(&marker, use_mesh_triangle_list), shape_msg);
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

bool constructMsgFromShape(const Shape* shape, ShapeMsg &shape_msg)
{
  if (shape->type == SPHERE)
  {
    shape_msgs::SolidPrimitive s;
    s.type = shape_msgs::SolidPrimitive::SPHERE;
    s.dimensions.x = static_cast<const Sphere*>(shape)->radius * 2.0;
    s.dimensions.y = s.dimensions.z = 0.0;
    shape_msg = s;
  }
  else
    if (shape->type == BOX)
    { 
      shape_msgs::SolidPrimitive s;
      s.type = shape_msgs::SolidPrimitive::BOX;
      const double* sz = static_cast<const Box*>(shape)->size;
      s.dimensions.x = sz[0];
      s.dimensions.y = sz[1];
      s.dimensions.z = sz[2]; 
      shape_msg = s;
    }
    else
      if (shape->type == CYLINDER)
      { 
        shape_msgs::SolidPrimitive s;
        s.type = shape_msgs::SolidPrimitive::CYLINDER;
        s.dimensions.x = static_cast<const Cylinder*>(shape)->radius * 2.0;
        s.dimensions.y = 0.0;
        s.dimensions.z = static_cast<const Cylinder*>(shape)->length;
        shape_msg = s;
      }
      else
        if (shape->type == CONE)
        {   
          shape_msgs::SolidPrimitive s;
          s.type = shape_msgs::SolidPrimitive::CONE;
          s.dimensions.x = static_cast<const Cone*>(shape)->radius * 2.0; 
          s.dimensions.y = 0.0;
          s.dimensions.z = static_cast<const Cone*>(shape)->length;      
          shape_msg = s;
        }
        else
          if (shape->type == PLANE)
          {    
            shape_msgs::Plane s;
            const Plane *p = static_cast<const Plane*>(shape);
            s.coef[0] = p->a;
            s.coef[1] = p->b;
            s.coef[2] = p->c;
            s.coef[3] = p->d;  
            shape_msg = s;
          }
          else
            if (shape->type == MESH)
            {
              shape_msgs::Mesh s;
              const Mesh *mesh = static_cast<const Mesh*>(shape);
              s.vertices.resize(mesh->vertex_count);
              s.triangles.resize(mesh->triangle_count);
              
              for (unsigned int i = 0 ; i < mesh->vertex_count ; ++i)
              {
                unsigned int i3 = i * 3;
                s.vertices[i].x = mesh->vertices[i3];
                s.vertices[i].y = mesh->vertices[i3 + 1];
                s.vertices[i].z = mesh->vertices[i3 + 2];
              }
              
              for (unsigned int i = 0 ; i < s.triangles.size() ; ++i)
              { 
                unsigned int i3 = i * 3;
                s.triangles[i].vertex_indices[0] = mesh->triangles[i3];
                s.triangles[i].vertex_indices[1] = mesh->triangles[i3 + 1];
                s.triangles[i].vertex_indices[2] = mesh->triangles[i3 + 2];
              }
              shape_msg = s;
            }
            else
            {
              ROS_ERROR("Unable to construct shape message for shape of type %d", (int)shape->type);
              return false;
            }
  
  return true;
}

}
