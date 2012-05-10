#include <planning_scene_monitor/planning_scene_monitor.h>
#include <kinematics_constraint_aware/inverse_kinematics_sanity_checker.h>
#include <kinematics_plugin_loader/kinematics_plugin_loader_helpers.h>

int main(int argc, char **argv) {

  ros::init(argc, argv, "inverse_kinematics_sanity_checker");

  boost::shared_ptr<kinematics_plugin_loader::KinematicsPluginLoader> kinematics_plugin_loader_(new kinematics_plugin_loader::KinematicsPluginLoader());
  
  boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_(new planning_scene_monitor::PlanningSceneMonitor("robot_description", kinematics_plugin_loader_));

  std::map<std::string, kinematics::KinematicsBasePtr> solver_map;
  kinematics_plugin_loader::generateKinematicsLoaderMap(planning_scene_monitor_->getPlanningScene()->getKinematicModel(),
                                                        planning_scene_monitor_->getPlanningScene()->getSrdfModel(),
                                                        kinematics_plugin_loader_,
                                                        solver_map);

  InverseKinematicsSanityChecker sanity(solver_map,
                                        planning_scene_monitor_->getPlanningScene()->getKinematicModel());

  sanity.runTest("arm",
                 10000);


  ros::shutdown();
}
