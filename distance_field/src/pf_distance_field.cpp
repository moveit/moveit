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

#include <moveit/distance_field/pf_distance_field.h>
#include <limits>

namespace distance_field
{

PFDistanceField::PFDistanceField(double size_x, double size_y, double size_z, double resolution,
    double origin_x, double origin_y, double origin_z):
  DistanceField<float>(size_x, size_y, size_z, resolution, origin_x, origin_y, origin_z, DT_INF),
  DT_INF(std::numeric_limits<float>::max())
{

}

PFDistanceField::~PFDistanceField()
{
}

void PFDistanceField::addPointsToField(const std::vector<Eigen::Vector3d> points)
{
  int x, y, z;
  float init = 0.0;
  for (unsigned int i=0; i<points.size(); ++i)
  {
    bool valid = worldToGrid(points[i].x(), points[i].y(), points[i].z(), x, y, z);
    if (!valid)
      continue;
    setCell(x,y,z, init);
  }
  computeDT();
}

void PFDistanceField::computeDT()
{

  size_t nx = num_cells_[DIM_X];
  size_t ny = num_cells_[DIM_Y];
  size_t nz = num_cells_[DIM_Z];

    size_t     maxdim = std::max( nx, std::max(ny, nz) );
    FloatArray f(maxdim), ft(maxdim), zz(maxdim+1);
    IntArray   v(maxdim);

    // along z
    for (size_t y=0; y<ny; ++y) {
        for (size_t x=0; x<nx; ++x) {
            for (size_t z=0; z<nz; ++z) {
                f[z] = getCell(x,y,z);
            }
            dt(f, nz, ft, v, zz);
            for (size_t z=0; z<nz; ++z) {
                setCell(x,y,z,ft[z]);
            }
        }
    }

    // along y
    for (size_t z=0; z<nz; ++z) {
        for (size_t x=0; x<nx; ++x) {
            for (size_t y=0; y<ny; ++y) {
                f[y] = getCell(x,y,z);
            }
            dt(f, ny, ft, v, zz);
            for (size_t y=0; y<ny; ++y) {
                setCell(x,y,z,ft[y]);
            }
        }
    }

    // along x
    for (size_t z=0; z<nz; ++z) {
        for (size_t y=0; y<ny; ++y) {
            for (size_t x=0; x<nx; ++x) {
                f[x] = getCell(x,y,z);
            }
            dt(f, nx, ft, v, zz);
            for (size_t x=0; x<nx; ++x) {
                setCell(x,y,z,ft[x]);
            }
        }
    }

}


void PFDistanceField::dt(const FloatArray& f,
        size_t nn,
        FloatArray& ft,
        IntArray& v,
        FloatArray& z) {

/*    assert(f.size() >= nn);
    assert(ft.size() >= nn);
    assert(v.size() >= nn);
    assert(z.size() >= nn+1);
*/
    int n = nn;

    int k = 0;

    v[0] = 0;

    z[0] = -DT_INF;
    z[1] =  DT_INF;

    for (int q=1; q<n; ++q) {
        float s = ((f[q]+sqr(q))-(f[v[k]]+sqr(v[k])))/(2*q-2*v[k]);
        while (s <= z[k]) {
            --k;
            s = ((f[q]+sqr(q))-(f[v[k]]+sqr(v[k])))/(2*q-2*v[k]);
        }
        ++k;
        v[k] = q;
        z[k] = s;
        z[k+1] = DT_INF;
    }

    k = 0;
    for (int q=0; q<n; ++q) {
        while (z[k+1] < q) { ++k; }
        ft[q] = sqr(q-v[k]) + f[v[k]];
    }

}


void PFDistanceField::reset()
{
  VoxelGrid<float>::reset(DT_INF);
}

}
