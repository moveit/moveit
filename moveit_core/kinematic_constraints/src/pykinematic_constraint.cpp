#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <moveit/python/pybind_rosmsg_typecasters.h>

#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/kinematic_constraints/utils.h>

namespace py = pybind11;

using namespace kinematic_constraints;

auto CONSTRUCT_GOAL_CONSTRAINTS1 = py::overload_cast<const std::string&,                 //
                                                     const geometry_msgs::PoseStamped&,  //
                                                     double,                             //
                                                     double>(&constructGoalConstraints);

auto ADD1 = py::overload_cast<const moveit_msgs::Constraints&,  //
                              const moveit::core::Transforms&>(&KinematicConstraintSet::add);

auto DECIDE1 = py::overload_cast<const moveit::core::RobotState&, bool>(&KinematicConstraintSet::decide, py::const_);

void def_kinematic_constraints_bindings(py::module& m)
{
  m.doc() = "Class for joint, position, visibility, and other constraints";
  m.def("constructGoalConstraints",
        CONSTRUCT_GOAL_CONSTRAINTS1,      //
        py::arg("link_name"),             //
        py::arg("pose"),                  //
        py::arg("tolerance_pos") = 1e-3,  //
        py::arg("tolerance_angle") = 1e-2);

  py::class_<ConstraintEvaluationResult, std::shared_ptr<ConstraintEvaluationResult>>(m, "ConstraintEvaluationResult")
      .def(py::init<>())
      .def_readwrite("satisfied", &ConstraintEvaluationResult::satisfied)
      .def_readwrite("distance", &ConstraintEvaluationResult::distance)
      //
      ;

  py::class_<KinematicConstraintSet, KinematicConstraintSetPtr>(m, "KinematicConstraintSet")
      .def(py::init<robot_model::RobotModelConstPtr>(), py::arg("robot_model"))
      .def("add", ADD1)
      .def("decide", DECIDE1, py::arg("state"), py::arg("verbose") = false);
}
