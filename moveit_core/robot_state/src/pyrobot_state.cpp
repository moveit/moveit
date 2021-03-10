#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <moveit/python/pybind_rosmsg_typecasters.h>

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

namespace py = pybind11;
using namespace robot_state;

auto SET_TO_RANDOM_POSITIONS1 = py::overload_cast<>(&RobotState::setToRandomPositions);
auto SET_TO_RANDOM_POSITIONS2 = py::overload_cast<const JointModelGroup*>(&RobotState::setToRandomPositions);
auto SET_TO_RANDOM_POSITIONS3 = py::overload_cast<const JointModelGroup*,                 //
                                                  random_numbers::RandomNumberGenerator&  //
                                                  >(&RobotState::setToRandomPositions);

auto SET_JOINT_GROUP_POSITIONS1 = py::overload_cast<const std::string&,         //
                                                    const std::vector<double>&  //
                                                    >(&RobotState::setJointGroupPositions);
auto SET_JOINT_GROUP_POSITIONS2 = py::overload_cast<const JointModelGroup*,     //
                                                    const std::vector<double>&  //
                                                    >(&RobotState::setJointGroupPositions);

auto SATISFIES_BOUNDS1 = py::overload_cast<const JointModelGroup*, double>(&RobotState::satisfiesBounds, py::const_);

auto ROBOT_STATE_MSG_TO_ROBOT_STATE1 = py::overload_cast<const Transforms&,               //
                                                         const moveit_msgs::RobotState&,  //
                                                         RobotState&,                     //
                                                         bool                             //
                                                         >(robotStateMsgToRobotState);
auto ROBOT_STATE_MSG_TO_ROBOT_STATE2 = py::overload_cast<const moveit_msgs::RobotState&,  //
                                                         RobotState&,                     //
                                                         bool                             //
                                                         >(robotStateMsgToRobotState);

void def_robot_state_bindings(py::module& m)
{
  m.doc() = "Representation of a robot's state. This includes position, velocity, acceleration and effort.";
  py::class_<RobotState, RobotStatePtr>(m, "RobotState")
      .def(py::init<const robot_model::RobotModelConstPtr&>(), py::arg("robot_model"))
      .def("setToRandomPositions", SET_TO_RANDOM_POSITIONS1)
      .def("setToRandomPositions", SET_TO_RANDOM_POSITIONS2)
      .def("setToRandomPositions", SET_TO_RANDOM_POSITIONS3)
      .def("getJointModelGroup", &RobotState::getJointModelGroup, py::return_value_policy::reference)
      .def("getJointModel", &RobotState::getJointModel, py::return_value_policy::reference)
      .def("getLinkModel", &RobotState::getLinkModel, py::return_value_policy::reference)
      .def("getVariableNames", &RobotState::getVariableNames)
      .def("getVariablePositions", py::overload_cast<>(&RobotState::getVariablePositions, py::const_),
           py::return_value_policy::reference)
      .def("getVariableCount", &RobotState::getVariableCount)
      .def("hasVelocities", &RobotState::hasVelocities)
      .def("setJointGroupPositions", SET_JOINT_GROUP_POSITIONS1)
      .def("setJointGroupPositions", SET_JOINT_GROUP_POSITIONS2)
      .def("satisfiesBounds", SATISFIES_BOUNDS1, py::arg("joint_model_group"), py::arg("margin") = 0.0)
      .def("update", &RobotState::update, py::arg("force") = false)
      .def("printStateInfo",
           [](const RobotState& s) {
             std::stringstream ss;
             s.printStateInfo(ss);
             return ss.str();
           })
      .def("printStatePositions",
           [](const RobotState& s) {
             std::stringstream ss;
             s.printStatePositions(ss);
             return ss.str();
           })
      .def("__repr__",
           [](const RobotState& s) {
             std::stringstream ss;
             s.printStatePositions(ss);
             return ss.str();
           })
      //
      ;

  m.def("jointStateToRobotState", &jointStateToRobotState);
  m.def("robotStateMsgToRobotState", ROBOT_STATE_MSG_TO_ROBOT_STATE1);
  m.def("robotStateMsgToRobotState", ROBOT_STATE_MSG_TO_ROBOT_STATE2);
  m.def(
      "robotStateToRobotStateMsg",
      [](const RobotState& state, bool copy_attached_bodies) {
        moveit_msgs::RobotState state_msg;
        robotStateToRobotStateMsg(state, state_msg, copy_attached_bodies);
        return state_msg;
      },
      py::arg("state"), py::arg("copy_attached_bodies") = true);
}
