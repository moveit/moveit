#include <pybind11/pybind11.h>
#include <urdf_parser/urdf_parser.h>
#include <urdf_model/model.h>
#include <random_numbers/random_numbers.h>
#include <srdfdom/model.h>
#include <moveit/python/pybind_rosmsg_typecasters.h>

namespace py = pybind11;

void def_collision_detect_bindings(py::module& contact);

void def_robot_model_bindings(py::module& m);

void def_robot_state_bindings(py::module& m);

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
  auto urdf_m = m.def_submodule("urdf");
  auto random_numbers_m = m.def_submodule("random_numbers");

  py::class_<random_numbers::RandomNumberGenerator>(random_numbers_m, "RandomNumbers")
      .def(py::init<>())
      .def(py::init<boost::uint32_t>(), py::arg("seed"))
      //
      ;

  def_collision_detect_bindings(collision_detection_m);
  def_robot_model_bindings(robot_model_m);
  def_robot_state_bindings(robot_state_m);
  def_planning_scene_bindings(planning_scene_m);
  def_kinematic_constraints_bindings(kinematic_constraints_m);

  urdf_m.def("parseURDFFile", &urdf::parseURDFFile);
  py::class_<urdf::ModelInterface, urdf::ModelInterfaceSharedPtr>(urdf_m, "ModelInterface")
      .def(py::init<>())
      .def("getName", &urdf::ModelInterface::getName);

  auto srdf_m = m.def_submodule("srdf", "srdf");
  py::class_<srdf::Model>(srdf_m, "Model").def(py::init<>()).def("initFile", &srdf::Model::initFile);
}
