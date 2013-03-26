/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Willow Garage, Inc.
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

/* Author: Suat Gedikli */

#include <moveit/mesh_filter/gl_mesh.h>
#include <geometric_shapes/shapes.h>
#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;
using shapes::Mesh;

mesh_filter::GLMesh::GLMesh (const Mesh& mesh, unsigned int mesh_label)
{
  vector<Vector3f> normals;
  averageVertexNormals(mesh, normals);
  
  mesh_label_ = mesh_label;
  list_ = glGenLists(1);
  glNewList (list_, GL_COMPILE );
    glBegin(GL_TRIANGLES);
    glColor4ubv ((GLubyte*)&mesh_label_);
      for (unsigned tIdx = 0; tIdx < mesh.triangle_count; ++tIdx)
      {
        unsigned v1 = mesh.triangles [3*tIdx];
        unsigned v2 = mesh.triangles [3*tIdx + 1];
        unsigned v3 = mesh.triangles [3*tIdx + 2];
        
        glNormal3f (normals[v1][0], normals[v1][1], normals[v1][2]);
        glVertex3f (mesh.vertices [v1 * 3], mesh.vertices [v1 * 3 + 1], mesh.vertices [v1 * 3 + 2]);
        
        glNormal3f (normals[v2][0], normals[v2][1], normals[v2][2]);
        glVertex3f (mesh.vertices [v2 * 3], mesh.vertices [v2 * 3 + 1], mesh.vertices [v2 * 3 + 2]);
        
        glNormal3f (normals[v3][0], normals[v3][1], normals[v3][2]);
        glVertex3f (mesh.vertices [v3 * 3], mesh.vertices [v3 * 3 + 1], mesh.vertices [v3 * 3 + 2]);
      }
    glEnd();
  glEndList();  
}

mesh_filter::GLMesh::~GLMesh ()
{
  glDeleteLists(1, list_);
}

void mesh_filter::GLMesh::render (const Affine3d& transform) const
{
  glMatrixMode (GL_MODELVIEW);
  glPushMatrix();
  if (!(transform.matrix ().Flags & RowMajorBit))
    glMultMatrixd (transform.matrix().data());
  else
    glMultTransposeMatrixd (transform.matrix().data());
  glCallList (list_);
  glPopMatrix();
}

void mesh_filter::GLMesh::averageVertexNormals (const Mesh& mesh, vector<Eigen::Vector3f>& normals)
{
  // get average normals!
  normals.resize (mesh.vertex_count, Vector3f (0, 0, 0));
  
  for (unsigned tIdx = 0; tIdx < mesh.triangle_count; ++tIdx)
  {
    unsigned tIdx3 = 3 * tIdx;
    unsigned tIdx3_1 = tIdx3 + 1;
    unsigned tIdx3_2 = tIdx3 + 2;
    
    unsigned v1 = mesh.triangles [tIdx3];
    unsigned v2 = mesh.triangles [tIdx3_1];
    unsigned v3 = mesh.triangles [tIdx3_2];

    normals [v1][0] += mesh.triangle_normals [tIdx3];
    normals [v1][1] += mesh.triangle_normals [tIdx3_1];
    normals [v1][2] += mesh.triangle_normals [tIdx3_2];

    normals [v2][0] += mesh.triangle_normals [tIdx3];
    normals [v2][1] += mesh.triangle_normals [tIdx3_1];
    normals [v2][2] += mesh.triangle_normals [tIdx3_2];
    
    normals [v3][0] += mesh.triangle_normals [tIdx3];
    normals [v3][1] += mesh.triangle_normals [tIdx3_1];
    normals [v3][2] += mesh.triangle_normals [tIdx3_2];
  }
  for (vector<Vector3f>::iterator nIt = normals.begin (); nIt != normals.end (); ++nIt)
  {
    if (nIt->squaredNorm () > 0)
      nIt->normalize ();
  }
}
