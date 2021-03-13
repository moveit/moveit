#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <moveit/robot_model/robot_model.h>
#include <urdf_model/model.h>
#include <moveit/python/pybind_rosmsg_typecasters.h>

namespace py = pybind11;
using namespace robot_model;

void def_robot_model_bindings(py::module& m)
{
  m.doc() = "Definition of a kinematic model. Not thread safe, however multiple instances can be created.";
  py::class_<RobotModel, RobotModelPtr>(m, "RobotModel")
      .def(py::init<const urdf::ModelInterfaceSharedPtr&, const srdf::ModelConstSharedPtr&>(), py::arg("urdf_model"),
           py::arg("srdf_model"))
      .def("getName", &RobotModel::getName)
      .def("getLinkModelNames", &RobotModel::getLinkModelNames)
      .def("getJointModelNames", &RobotModel::getJointModelNames)
      //
      ;

  py::class_<JointModelGroup, JointModelGroupPtr>(m, "JointModelGroup")
      .def("getLinkModelNames", &JointModelGroup::getLinkModelNames)
      .def("getJointModelNames", &JointModelGroup::getJointModelNames)
      //
      ;
}
