#ifndef MOVEIT_OCCUPANCY_MAP_H
#define MOVEIT_OCCUPANCY_MAP_H

#include <boost/shared_ptr.hpp>
#include <octomap/octomap.h>

namespace occupancy_map_monitor
{
  typedef octomap::OcTree OccMapTree;
  typedef boost::shared_ptr<OccMapTree> OccMapTreePtr;
}

#endif /* MOVEIT_OCCUPANCY_MAP_H */
