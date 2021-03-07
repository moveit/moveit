#include <pybind11/pybind11.h>
#include <urdf_parser/urdf_parser.h>
#include <urdf_model/model.h>
#include <srdfdom/model.h>

namespace py = pybind11;

void def_collision_detect_bindings(py::module &m);

void def_robot_model_bindings(py::module &m);

void def_planning_scene_bindings(py::module &m);

PYBIND11_MODULE(pymoveit, m)
{
    m.doc() = "moveit core";

    auto collision_detection_m = m.def_submodule("collision_detection");
    auto planning_scene_m = m.def_submodule("planning_scene");
    auto robot_model_m = m.def_submodule("robot_model");

    def_collision_detect_bindings(collision_detection_m);
    def_robot_model_bindings(robot_model_m);
    def_planning_scene_bindings(planning_scene_m);

    auto urdf_m = m.def_submodule("urdf", "urdf");
    urdf_m.def("parseURDFFile", &urdf::parseURDFFile);
    py::class_<urdf::ModelInterface, urdf::ModelInterfaceSharedPtr>(urdf_m, "ModelInterface")
            .def(py::init<>())
            .def("getName", &urdf::ModelInterface::getName);

    auto srdf_m = m.def_submodule("srdf", "srdf");
    py::class_<srdf::Model>(srdf_m, "Model")
            .def(py::init<>())
            .def("initFile", &srdf::Model::initFile);

    // This is how you depend on other python modules (none at the moment)
    // py::module::import("other_module");

}
