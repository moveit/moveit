#ifndef MOVEIT_OCCUPANCY_MAP_H
#define MOVEIT_OCCUPANCY_MAP_H

#include <boost/shared_ptr.hpp>
#include <octomap/octomap.h>

namespace occupancy_map_monitor
{
  typedef octomap::OcTreeNode OccMapNode;
  typedef octomap::OcTree OccMapTree;
  typedef boost::shared_ptr<OccMapTree> OccMapTreePtr;
  typedef boost::shared_ptr<const OccMapTree> OccMapTreeConstPtr;
}

#endif /* MOVEIT_OCCUPANCY_MAP_H */
