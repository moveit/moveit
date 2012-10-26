/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

/* Author: Adam Leeper */

#include "collision_detection/collision_common.h"
#include "collision_detection/collision_octomap_filter.h"
#include "collision_detection/collision_world.h"
#include "octomap/math/Vector3.h"
#include "octomap/math/Utils.h"
#include "octomap/octomap.h"


static const float ISO_VALUE = 0.5; // TODO magic number! (though, probably a good one).

// forward declarations
bool getMetaballSurfaceProperties(const octomap::point3d_list &cloud, const float &spacing, const octomath::Vector3 &contact_point, octomath::Vector3 &normal, float &depth, bool estimate_depth);
bool findSurface(const octomap::point3d_list &cloud, const float &spacing, const octomath::Vector3 &seed, octomath::Vector3 &surface_point, octomath::Vector3 &normal);
bool sampleCloud(const octomap::point3d_list &cloud, const float &spacing, const octomath::Vector3 &position, float &intensity, octomath::Vector3 &gradient);


void collision_detection::refineContactNormals(const CollisionWorld::ObjectConstPtr& object,
                                               CollisionResult &res, bool estimate_depth)
{
  if(!object)
  {
    ROS_ERROR("No valid Object passed in, cannot refine Normals!");
    return;
  }
  if(res.contact_count < 1)
  {
    ROS_WARN("There do not appear to be any contacts, so there is nothing to refine!");
    return;
  }

  // iterate through contacts
  for( collision_detection::CollisionResult::ContactMap::iterator it = res.contacts.begin(); it != res.contacts.end(); ++it)
  {
    std::string contact1 = it->first.first;
    std::string contact2 = it->first.second;
    std::string octomap_name = "";
    std::vector<collision_detection::Contact>& contact_vector = it->second;

    if(      contact1.find("octomap") != std::string::npos ) octomap_name = contact1;
    else if( contact2.find("octomap") != std::string::npos ) octomap_name = contact2;
    else
    {
      continue;
    }

    float cell_size = 0;
    if(!object->shapes_.empty())
    {
      const shapes::ShapeConstPtr& shape = object->shapes_[0];
      boost::shared_ptr<const shapes::OcTree> octree = boost::dynamic_pointer_cast<const shapes::OcTree>(shape);
      if(octree)
      {
        cell_size = octree->octree->getResolution();
        for(size_t contact_index = 0; contact_index < contact_vector.size(); contact_index++)
        {
          Eigen::Vector3d point =   contact_vector[contact_index].pos;
          Eigen::Vector3d normal =  contact_vector[contact_index].normal;

          octomath::Vector3 contact_point(point[0], point[1], point[2]);
          octomath::Vector3 contact_normal(normal[0], normal[1], normal[2]);
          octomath::Vector3 diagonal = octomath::Vector3(1,1,1);
          octomath::Vector3 bbx_min = contact_point - diagonal*cell_size; // TODO should this be a bit larger? (or smaller?)
          octomath::Vector3 bbx_max = contact_point + diagonal*cell_size;
          octomap::point3d_list node_centers;
          octree->octree->getOccupiedLeafsBBX(node_centers, bbx_min, bbx_max);

          octomath::Vector3 n;
          float depth;
          if(getMetaballSurfaceProperties(node_centers, cell_size, contact_point, n, depth, estimate_depth))
          {
            // only modify normal if the refinement predicts a "very different" result.
            if(contact_normal.angleTo(n) > (M_PI_2 - 0.35)) // TODO magic number!
              contact_vector[contact_index].normal = Eigen::Vector3d(n.x(), n.y(), n.z());

            if(estimate_depth)
              contact_vector[contact_index].depth = depth;

          }
        }
      }
    }
  }
}

bool getMetaballSurfaceProperties(const octomap::point3d_list &cloud, const float &spacing, const octomath::Vector3 &contact_point, octomath::Vector3 &normal, float &depth, bool estimate_depth)
{
  float intensity;
  if(estimate_depth)
  {
    octomath::Vector3 surface_point;
    if(findSurface(cloud, spacing, contact_point, surface_point, normal))
    {
      depth = normal.dot(surface_point - contact_point); // do we prefer this, or magnitude of surface - contact?
      return true;
    }
    else
    {
      return false;
    }
  }
  else // just get normals, no depth
  {
    octomath::Vector3 gradient;
    if(sampleCloud(cloud, spacing, contact_point, intensity, gradient))
    {
      normal = gradient.normalized();
      return true;
    }
    else
    {
      return false;
    }
  }
}

// --------------------------------------------------------------------------
// This algorithm is from Salisbury & Tarr's 1997 paper.  It will find the
// closest point on the surface starting from a seed point that is close by
// following the direction of the field gradient.
bool findSurface(const octomap::point3d_list &cloud, const float &spacing, const octomath::Vector3 &seed, octomath::Vector3 &surface_point, octomath::Vector3 &normal)
{
    const double epsilon = 1e-10;
    const int iterations = 10;
    float intensity = 0;

    octomath::Vector3 p = seed, dp, gs;
    for (int i = 0; i < iterations; ++i)
    {
      if(!sampleCloud(cloud, spacing, p, intensity, gs)) return false;
      double s = ISO_VALUE - intensity;
      dp = (gs * -s) * (1.0 / std::max(gs.dot(gs), epsilon));
      p = p + dp;
      if (dp.dot(dp) < epsilon)
      {
        surface_point = p;
        normal = gs.normalized();
        return true;
      }
    }
    return false;
//    return p;
}


bool sampleCloud(const octomap::point3d_list &cloud, const float &spacing, const octomath::Vector3 &position, float &intensity, octomath::Vector3 &gradient)
{
  intensity = 0.f;
  gradient = octomath::Vector3(0,0,0);

  float R = 1.5*spacing; // TODO magic number!
  //float T = 0.5; // TODO magic number!

  int NN = cloud.size();
  if(NN == 0)
  {
    return false;
  }

  // variables for Wyvill
  double a=0, b=0, c=0, R2=0, R4=0, R6=0, a1=0, b1=0, c1=0, a2=0, b2=0, c2=0;
  bool WYVILL = true;

  octomap::point3d_list::const_iterator it;
  for ( it = cloud.begin(); it != cloud.end(); it++ )
  {
    octomath::Vector3 v = (*it);

    if(WYVILL)
    {
      R2 = R*R;
      R4 = R2*R2;
      R6 = R4*R2;
      a = -4.0/9.0; b  = 17.0/9.0; c = -22.0/9.0;
      a1 = a/R6; b1 = b/R4; c1 = c/R2;
      a2 = 6*a1; b2 = 4*b1; c2 = 2*c1;
    }
    else
    {
      ROS_ERROR("This should not be called!");
    }

    double f_val = 0;
    octomath::Vector3 f_grad(0,0,0);

    octomath::Vector3 pos = position-v;
    double r = pos.norm();
    pos = pos*(1.0/r);
    if(r > R)  // must skip points outside valid bounds.
    {
      continue;
    }
    double r2 = r*r;
    double r3 = r*r2;
    double r4 = r2*r2;
    double r5 = r3*r2;
    double r6 = r3*r3;

    if(WYVILL)
    {
      f_val = (a1*r6 + b1*r4 + c1*r2 + 1);
      f_grad = pos*(a2*r5 + b2*r3 + c2*r);
    }
    else
    {
      ROS_ERROR("This should not be called!");
      double r_scaled = r/R;
      // TODO still need to address the scaling...
      f_val = pow((1-r_scaled),4)*(4*r_scaled + 1);
      f_grad = pos*(-4.0/R*pow(1.0-r_scaled,3)*(4.0*r_scaled+1.0)+4.0/R*pow(1-r_scaled,4));
    }

    // TODO:  The whole library should be overhauled to follow the "gradient points out"
    //        convention of implicit functions.
    intensity += f_val;
    gradient += f_grad;
  }
  // implicit surface gradient convention points out, so we flip it.
  gradient *= -1.0;
  return true; // it worked
}
