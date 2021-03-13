#include <pybind11/pybind11.h>
#include <urdf_parser/urdf_parser.h>
#include <moveit/robot_model/robot_model.h>
#include <srdfdom/model.h>

namespace py = pybind11;

// each  of these functions define the bindings for a namespace/subfolder within moveit_core
void def_collision_detection_bindings(py::module& contact);

void def_robot_model_bindings(py::module& m);

void def_robot_state_bindings(py::module& m);

void def_transforms_bindings(py::module& m);

void def_planning_scene_bindings(py::module& m);

void def_kinematic_constraints_bindings(py::module& m);

auto load_robot_model(const std::string& urdf_path, const std::string& srdf_path)
{
  auto urdf_model = urdf::parseURDFFile(urdf_path);
  auto srdf_model = std::make_shared<srdf::Model>();
  srdf_model->initFile(*urdf_model, srdf_path);
  auto robot_model = std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
  return robot_model;
}

PYBIND11_MODULE(pymoveit_core, m)
{
  auto collision_detection_m = m.def_submodule("collision_detection");
  auto planning_scene_m = m.def_submodule("planning_scene");
  auto robot_model_m = m.def_submodule("robot_model");
  auto robot_state_m = m.def_submodule("robot_state");
  auto kinematic_constraints_m = m.def_submodule("kinematic_constraints");
  auto transforms_m = m.def_submodule("transforms");

  def_collision_detection_bindings(collision_detection_m);
  def_robot_model_bindings(robot_model_m);
  def_robot_state_bindings(robot_state_m);
  def_planning_scene_bindings(planning_scene_m);
  def_transforms_bindings(transforms_m);
  def_kinematic_constraints_bindings(kinematic_constraints_m);

  // convenience function
  m.def("load_robot_model", &load_robot_model);
}
