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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Julius Kammerl */

#include <moveit/rviz_plugin_render_tools/octomap_render.h>

DIAGNOSTIC_PUSH
SILENT_UNUSED_PARAM
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
DIAGNOSTIC_POP

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <rviz/ogre_helpers/point_cloud.h>

namespace moveit_rviz_plugin
{
typedef std::vector<rviz::PointCloud::Point> VPoint;
typedef std::vector<VPoint> VVPoint;

OcTreeRender::OcTreeRender(const std::shared_ptr<const octomap::OcTree>& octree,
                           OctreeVoxelRenderMode octree_voxel_rendering, OctreeVoxelColorMode octree_color_mode,
                           std::size_t max_octree_depth, Ogre::SceneNode* parent_node)
  : octree_(octree), colorFactor_(0.8)
{
  if (!max_octree_depth)
  {
    octree_depth_ = octree->getTreeDepth();
  }
  else
  {
    octree_depth_ = std::min(max_octree_depth, (std::size_t)octree->getTreeDepth());
  }

  scene_node_ = parent_node->createChildSceneNode();

  cloud_.resize(octree_depth_);

  for (std::size_t i = 0; i < octree_depth_; ++i)
  {
    std::stringstream sname;
    sname << "PointCloud Nr." << i;
    cloud_[i] = new rviz::PointCloud();
    cloud_[i]->setName(sname.str());
    cloud_[i]->setRenderMode(rviz::PointCloud::RM_BOXES);
    scene_node_->attachObject(cloud_[i]);
  }

  octreeDecoding(octree, octree_voxel_rendering, octree_color_mode);
}

OcTreeRender::~OcTreeRender()
{
  scene_node_->detachAllObjects();

  for (std::size_t i = 0; i < octree_depth_; ++i)
  {
    delete cloud_[i];
  }
}

void OcTreeRender::setPosition(const Ogre::Vector3& position)
{
  scene_node_->setPosition(position);
}

void moveit_rviz_plugin::OcTreeRender::setOrientation(const Ogre::Quaternion& orientation)
{
  scene_node_->setOrientation(orientation);
}

// method taken from octomap_server package
void OcTreeRender::setColor(double z_pos, double min_z, double max_z, double color_factor,
                            rviz::PointCloud::Point* point)
{
  int i;
  double m, n, f;

  double s = 1.0;
  double v = 1.0;

  double h = (1.0 - std::min(std::max((z_pos - min_z) / (max_z - min_z), 0.0), 1.0)) * color_factor;

  h -= floor(h);
  h *= 6;
  i = floor(h);
  f = h - i;
  if (!(i & 1))
    f = 1 - f;  // if i is even
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i)
  {
    case 6:
    case 0:
      point->setColor(v, n, m);
      break;
    case 1:
      point->setColor(n, v, m);
      break;
    case 2:
      point->setColor(m, v, n);
      break;
    case 3:
      point->setColor(m, n, v);
      break;
    case 4:
      point->setColor(n, m, v);
      break;
    case 5:
      point->setColor(v, m, n);
      break;
    default:
      point->setColor(1, 0.5, 0.5);
      break;
  }
}

void OcTreeRender::octreeDecoding(const std::shared_ptr<const octomap::OcTree>& octree,
                                  OctreeVoxelRenderMode octree_voxel_rendering, OctreeVoxelColorMode octree_color_mode)
{
  VVPoint point_buf;
  point_buf.resize(octree_depth_);

  // get dimensions of octree
  double min_x, min_y, min_z, max_x, max_y, max_z;
  octree->getMetricMin(min_x, min_y, min_z);
  octree->getMetricMax(max_x, max_y, max_z);

  unsigned int render_mode_mask = static_cast<unsigned int>(octree_voxel_rendering);

  size_t point_count = 0;
  {
    // traverse all leafs in the tree:
    for (octomap::OcTree::iterator it = octree->begin(octree_depth_), end = octree->end(); it != end; ++it)
    {
      bool display_voxel = false;

      // the left part evaluates to 1 for free voxels and 2 for occupied voxels
      if (((int)octree->isNodeOccupied(*it) + 1) & render_mode_mask)
      {
        // check if current voxel has neighbors on all sides -> no need to be displayed
        bool all_neighbors_found = true;

        octomap::OcTreeKey key;
        octomap::OcTreeKey n_key = it.getKey();

        for (key[2] = n_key[2] - 1; all_neighbors_found && key[2] <= n_key[2] + 1; ++key[2])
        {
          for (key[1] = n_key[1] - 1; all_neighbors_found && key[1] <= n_key[1] + 1; ++key[1])
          {
            for (key[0] = n_key[0] - 1; all_neighbors_found && key[0] <= n_key[0] + 1; ++key[0])
            {
              if (key != n_key)
              {
                octomap::OcTreeNode* node = octree->search(key);

                // the left part evaluates to 1 for free voxels and 2 for occupied voxels
                if (!(node && (((int)octree->isNodeOccupied(node)) + 1) & render_mode_mask))
                {
                  // we do not have a neighbor => break!
                  all_neighbors_found = false;
                }
              }
            }
          }
        }

        display_voxel |= !all_neighbors_found;
      }

      if (display_voxel)
      {
        rviz::PointCloud::Point new_point;

        new_point.position.x = it.getX();
        new_point.position.y = it.getY();
        new_point.position.z = it.getZ();

        float cell_probability;

        switch (octree_color_mode)
        {
          case OCTOMAP_Z_AXIS_COLOR:
            setColor(new_point.position.z, min_z, max_z, colorFactor_, &new_point);
            break;
          case OCTOMAP_PROBABLILTY_COLOR:
            cell_probability = it->getOccupancy();
            new_point.setColor((1.0f - cell_probability), cell_probability, 0.0);
            break;
          default:
            break;
        }

        // push to point vectors
        unsigned int depth = it.getDepth();
        point_buf[depth - 1].push_back(new_point);

        ++point_count;
      }
    }
  }

  for (size_t i = 0; i < octree_depth_; ++i)
  {
    double size = octree->getNodeSize(i + 1);

    cloud_[i]->clear();
    cloud_[i]->setDimensions(size, size, size);

    cloud_[i]->addPoints(&point_buf[i].front(), point_buf[i].size());
    point_buf[i].clear();
  }
}
}  // namespace moveit_rviz_plugin
