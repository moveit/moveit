/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Intel Labs Pittsburgh
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
 *   * Neither the name of the Intel Labs nor the names of its
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

/* Author: Siddhartha Srinivasa */

#ifndef MOVEIT_DISTANCE_FIELD_PF_DISTANCE_FIELD_
#define MOVEIT_DISTANCE_FIELD_PF_DISTANCE_FIELD_

#include <moveit/distance_field/distance_field.h>

namespace distance_field
{

/**
 * \brief A DistanceField implementation that uses a raster scanning type method by Felzenszwalb et. al.
 *
 * Implementation of "Distance Transforms of Sampled Functions", Pedro F. Felzenszwalb and
 * Daniel P. Huttenlocher, Cornell Computing and Information Science TR2004-1963
 */
class PFDistanceField: public DistanceField<float>
{
public:
  PFDistanceField(double size_x, double size_y, double size_z, double resolution,
      double origin_x, double origin_y, double origin_z);

  virtual ~PFDistanceField();


  typedef std::vector<float> FloatArray;
  typedef std::vector<int>   IntArray;

  virtual void addPointsToField(const EigenSTL::vector_Vector3d &points);
  virtual void reset();

  const float DT_INF;

private:
  inline float sqr(float x) { return x*x; }
  void dt(const FloatArray& f, size_t nn, FloatArray& ft, IntArray& v, FloatArray& z);
  void computeDT();
  virtual double getDistance(const float& object) const;

};

////////////////////////// inline functions follow ////////////////////////////////////////

inline double PFDistanceField::getDistance(const float& object) const
{
  return sqrt(object)*this->resolution_[DIM_X];
}

}

#endif
