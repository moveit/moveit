#ifndef OCCUPANCY_MAP_MONITOR_OCTOMAP_MARKERS_H
#define OCCUPANCY_MAP_MONITOR_OCTOMAP_MARKERS_H

#include <visualization_msgs/MarkerArray.h>
#include <moveit/occupancy_map_monitor/occupancy_map.h>

namespace occupancy_map_monitor
{

void make_filtered_marker_array(const OccMapTreeConstPtr &tree, const ros::Time &stamp, const std::string &frame_id, const std::string &ns, visualization_msgs::MarkerArray &marker_arr, const boost::function<bool (const OccMapNode&)> filter_func);

void make_occupied_cells_marker_array(const OccMapTreeConstPtr &tree, const ros::Time &stamp, const std::string &frame_id, const std::string &ns, visualization_msgs::MarkerArray &marker_arr);

void make_free_cells_marker_array(const OccMapTreeConstPtr &tree, const ros::Time &stamp, const std::string &frame_id, const std::string &ns, visualization_msgs::MarkerArray &marker_arr);

} /* namespace occupancy_map_monitor */

#endif // OCCUPANCY_MAP_MONITOR_OCTOMAP_MARKERS_H
