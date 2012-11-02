/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 *
 * $Id: model_registration.h 36182 2010-04-27 16:17:14Z rusu $
 *
 */
#ifndef MODEL_REGISTRATION_H_
#define MODEL_REGISTRATION_H_

#include <pcl/registration/registration.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

#include "tabletop_object_detector/model_fitter.h"

namespace tabletop_object_detector 
{
  //! Implementation of a model fitter using ICP registration from pcl
  template <typename T>
  class ICPRegistrationFitter : public ModelToCloudFitter
  {
    public:

    void initializeFromMesh(const shape_msgs::Shape &mesh) 
    {
      std::vector<tf::Vector3> btVectors;
      sampleMesh(mesh, btVectors, 0.002 );
      cloudmodel_.points.resize (btVectors.size ());
      for (int i = 0; i < (int)btVectors.size (); ++i)
      {
        cloudmodel_.points[i].x = btVectors[i].x ();
        cloudmodel_.points[i].y = btVectors[i].y ();
        cloudmodel_.points[i].z = btVectors[i].z ();
      }
    }
    
    ModelFitInfo fitPointCloud (const T &cloud)
    {
      pcl::IterativeClosestPoint<T, T> reg;
      reg.setMaximumIterations (10000);
      reg.setTransformationEpsilon (1e-5);
      //T::Ptr cloudptr;
      //cloudptr.reset (new T (cloud));
      //reg.setInputCloud (cloudptr);
      //reg.setInputModel (cloudmodel_);
      T cloud_out;
      reg.compute (cloud_out);
      
      double score = reg.getFitnessScore ();

      Eigen::Matrix4f tma = reg.getFinalTransformationMatrix (); 
      geometry_msgs::Pose p;
      p.position.x = tma (0, 3);
      p.position.y = tma (1, 3);
      p.position.z = tma (2, 3);
      Eigen::Quaternion<double> q (tma.block<3,3>(0,0));
      p.orientation.x = q.x ();
      p.orientation.y = q.y ();
      p.orientation.z = q.z ();
      p.orientation.w = q.w ();

      return ModelFitInfo(model_id_, p, score);
    }

  private:
    T cloudmodel_;
  };
} //namespace tabletop_object_detector

#endif
