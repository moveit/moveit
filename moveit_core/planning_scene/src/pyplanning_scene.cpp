#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <moveit/python/pybind_rosmsg_typecasters.h>

#include <moveit/planning_scene/planning_scene.h>

namespace py = pybind11;
using namespace planning_scene;

void def_planning_scene_bindings(py::module& m)
{
  m.doc() = "The planning scene represents the state of the world and the robot, "
            "and can be used for collision checking";
  py::class_<PlanningScene, PlanningScenePtr>(m, "PlanningScene")
      .def(py::init<const moveit::core::RobotModelConstPtr&, const collision_detection::WorldPtr&>(),
           py::arg("robot_model"), py::arg("world") = collision_detection::WorldPtr(new collision_detection::World()))
      .def("checkSelfCollision",
           py::overload_cast<const collision_detection::CollisionRequest&, collision_detection::CollisionResult&>(
               &PlanningScene::checkSelfCollision, py::const_))
      .def("checkSelfCollision",
           py::overload_cast<const collision_detection::CollisionRequest&, collision_detection::CollisionResult&,
                             const robot_state::RobotState&, const collision_detection::AllowedCollisionMatrix&>(
               &PlanningScene::checkSelfCollision, py::const_))
      .def("checkCollision",
           py::overload_cast<const collision_detection::CollisionRequest&, collision_detection::CollisionResult&>(
               &PlanningScene::checkCollision, py::const_))
      .def("checkCollision",
           py::overload_cast<const collision_detection::CollisionRequest&, collision_detection::CollisionResult&,
                             const robot_state::RobotState&, const collision_detection::AllowedCollisionMatrix&>(
               &PlanningScene::checkCollision, py::const_))
      .def("getCurrentStateNonConst", &PlanningScene::getCurrentStateNonConst)
      .def("getCurrentState", &PlanningScene::getCurrentState)
      .def("getAllowedCollisionMatrix", &PlanningScene::getAllowedCollisionMatrix)
      .def("isStateConstrained",
           py::overload_cast<const moveit_msgs::RobotState&, const kinematic_constraints::KinematicConstraintSet&, bool>(
               &PlanningScene::isStateConstrained, py::const_),
           py::arg("state"), py::arg("constr"), py::arg("verbose") = false)
      .def(
          "isStateConstrained",
          py::overload_cast<const moveit::core::RobotState&, const kinematic_constraints::KinematicConstraintSet&, bool>(
              &PlanningScene::isStateConstrained, py::const_),
          py::arg("state"), py::arg("constr"), py::arg("verbose") = false)
      .def("isStateConstrained",
           py::overload_cast<const moveit_msgs::RobotState&, const moveit_msgs::Constraints&, bool>(
               &PlanningScene::isStateConstrained, py::const_),
           py::arg("state"), py::arg("constr"), py::arg("verbose") = false)
      .def("isStateConstrained",
           py::overload_cast<const moveit::core::RobotState&, const moveit_msgs::Constraints&, bool>(
               &PlanningScene::isStateConstrained, py::const_),
           py::arg("state"), py::arg("constr"), py::arg("verbose") = false)
      //      .def("setStateFeasibilityPredicate", &PlanningScene::setStateFeasibilityPredicate)
      .def("isStateValid",
           py::overload_cast<const moveit_msgs::RobotState&, const std::string&, bool>(&PlanningScene::isStateValid,
                                                                                       py::const_),
           py::arg("state"), py::arg("group"), py::arg("verbose") = false)
      .def("isStateValid",
           py::overload_cast<const moveit::core::RobotState&, const std::string&, bool>(&PlanningScene::isStateValid,
                                                                                        py::const_),
           py::arg("state"), py::arg("group"), py::arg("verbose") = false)
      .def("isStateValid",
           py::overload_cast<const moveit_msgs::RobotState&, const moveit_msgs::Constraints&, const std::string&, bool>(
               &PlanningScene::isStateValid, py::const_),
           py::arg("state"), py::arg("constr"), py::arg("group"), py::arg("verbose") = false)
      .def("isStateValid",
           py::overload_cast<const moveit::core::RobotState&, const moveit_msgs::Constraints&, const std::string&, bool>(
               &PlanningScene::isStateValid, py::const_),
           py::arg("state"), py::arg("constr"), py::arg("group"), py::arg("verbose") = false)
      .def("isStateValid",
           py::overload_cast<const moveit::core::RobotState&, const kinematic_constraints::KinematicConstraintSet&,
                             const std::string&, bool>(&PlanningScene::isStateValid, py::const_),
           py::arg("state"), py::arg("constr"), py::arg("group"), py::arg("verbose") = false)
      .def("setCurrentState", py::overload_cast<const moveit_msgs::RobotState&>(&PlanningScene::setCurrentState))
      .def("setCurrentState", py::overload_cast<const robot_state::RobotState&>(&PlanningScene::setCurrentState))
      //
      ;
}