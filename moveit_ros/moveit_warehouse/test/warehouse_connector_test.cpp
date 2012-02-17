#include <moveit_warehouse/warehouse_connector.h>
#include <moveit_warehouse/warehouse.h>

int main(int argc, char** argv) {

  ros::init(argc, argv, "warehouse_test");

  moveit_warehouse::WarehouseConnector wc;
  boost::shared_ptr<moveit_warehouse::PlanningSceneStorage> planning_scene_storage;
  wc.connectToDatabase("/u/gjones/.ros/arm_navigation_dbs/moveit_db", planning_scene_storage);

  ros::WallDuration(1.0).sleep();

  wc.connectToDatabase("/u/gjones/.ros/arm_navigation_dbs/moveit_2_db", planning_scene_storage);

  ros::waitForShutdown();

}
