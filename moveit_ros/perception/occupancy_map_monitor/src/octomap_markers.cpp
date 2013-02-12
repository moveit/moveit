#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <visualization_msgs/MarkerArray.h>
#include <moveit/occupancy_map_monitor/occupancy_map.h>

namespace occupancy_map_monitor
{

void make_filtered_marker_array(const OccMapTreeConstPtr &tree, const ros::Time &stamp, const std::string &frame_id, const std::string &ns, visualization_msgs::MarkerArray &marker_arr, const boost::function<bool (const OccMapNode&)> filter_func)
{
  int tree_depth = tree->getTreeDepth();

  /* each array stores all cubes of a different size, one for each depth level */
  marker_arr.markers.resize(tree_depth + 1);

  /* now, traverse all leaves in the tree */
  for (OccMapTree::iterator it = tree->begin(tree_depth), end = tree->end(); it != end; ++it)
  {
    //(tree->isNodeOccupied(*it))
    if(filter_func(*it))
    {
      double x = it.getX();
      double y = it.getY();
      double z = it.getZ();

      unsigned idx = it.getDepth();
      assert(idx < marker_arr.markers.size());

      geometry_msgs::Point cube_center;
      cube_center.x = x;
      cube_center.y = y;
      cube_center.z = z;
      marker_arr.markers[idx].pose.orientation.w = 1.0;
      
      marker_arr.markers[idx].points.push_back(cube_center);
    }
  }

  /* finish MarkerArray */
  for (unsigned i = 0; i < marker_arr.markers.size(); ++i)
  {
    double size = tree->getNodeSize(i);

    marker_arr.markers[i].header.frame_id = frame_id;
    marker_arr.markers[i].header.stamp = stamp;
    marker_arr.markers[i].ns = ns;
    marker_arr.markers[i].id = i;
    marker_arr.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
    marker_arr.markers[i].scale.x = size;
    marker_arr.markers[i].scale.y = size;
    marker_arr.markers[i].scale.z = size;
    marker_arr.markers[i].color.r = 0.0;
    marker_arr.markers[i].color.g = 0.0;
    marker_arr.markers[i].color.b = 1.0;
    marker_arr.markers[i].color.a = 0.5;

    if (marker_arr.markers[i].points.size() > 0)
      marker_arr.markers[i].action = visualization_msgs::Marker::ADD;
    else
      marker_arr.markers[i].action = visualization_msgs::Marker::DELETE;
  }
}

void make_occupied_cells_marker_array(const OccMapTreeConstPtr &tree, const ros::Time &stamp, const std::string &frame_id, const std::string &ns, visualization_msgs::MarkerArray &marker_arr)
{
  bool (OccMapTree::*isNodeOccupied) (const OccMapNode &) const  =  &OccMapTree::isNodeOccupied;
  make_filtered_marker_array(tree, stamp, frame_id, ns, marker_arr, boost::bind(isNodeOccupied, tree.get(), _1));
}

}  /* namespace occupancy_map_monitor */
