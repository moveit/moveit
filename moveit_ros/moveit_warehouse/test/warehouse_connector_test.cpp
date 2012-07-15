#include <moveit_warehouse/warehouse_connector.h>
#include <moveit_warehouse/warehouse.h>

int main(int argc, char** argv) 
{

  ros::init(argc, argv, "warehouse_test");

  moveit_warehouse::WarehouseConnector wc("/opt/ros/fuerte/stacks/warehousewg/mongodb/mongo/bin/mongod");
  boost::shared_ptr<moveit_warehouse::PlanningSceneStorage> planning_scene_storage;
  wc.connectToDatabase("moveit_db", planning_scene_storage);

  ros::WallDuration(1.0).sleep();

  wc.connectToDatabase("moveit_2_db", planning_scene_storage);

  ros::waitForShutdown();

}
