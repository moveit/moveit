#include <pybind11/pybind11.h>
#include <urdf_model/model.h>
#include <moveit/python/pybind_rosmsg_typecasters.h>

namespace py = pybind11;

void def_collision_detect_bindings(py::module& contact);

void def_robot_model_bindings(py::module& m);

void def_robot_state_bindings(py::module& m);

void def_transforms_bindings(py::module& m);

void def_planning_scene_bindings(py::module& m);

void def_kinematic_constraints_bindings(py::module& m);

PYBIND11_MODULE(pymoveit, m)
{
  m.doc() = "moveit core";

  auto collision_detection_m = m.def_submodule("collision_detection");
  auto planning_scene_m = m.def_submodule("planning_scene");
  auto robot_model_m = m.def_submodule("robot_model");
  auto robot_state_m = m.def_submodule("robot_state");
  auto kinematic_constraints_m = m.def_submodule("kinematic_constraints");
  auto transforms_m = m.def_submodule("transforms");


  def_collision_detect_bindings(collision_detection_m);
  def_robot_model_bindings(robot_model_m);
  def_robot_state_bindings(robot_state_m);
  def_planning_scene_bindings(planning_scene_m);
  def_transforms_bindings(transforms_m);
  def_kinematic_constraints_bindings(kinematic_constraints_m);
}
