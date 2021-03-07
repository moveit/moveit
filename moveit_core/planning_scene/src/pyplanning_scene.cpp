#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>
#include <moveit/planning_scene/planning_scene.h>

namespace py = pybind11;


void def_planning_scene_bindings(py::module &m)
{
    m.doc() = "The planning scene represents the state of the world and the robot, "
              "and can be used for collision checking";
    py::class_<planning_scene::PlanningScene, planning_scene::PlanningScenePtr>(m, "PlanningScene")
            .def(py::init<const moveit::core::RobotModelConstPtr &, const collision_detection::WorldPtr &>(),
                 py::arg("robot_model"),
                 py::arg("world") = collision_detection::WorldPtr(new collision_detection::World())
            ) // This comment prevents clang-format from moving the semicolon
            ;
}