#include <pybind11/pybind11.h>
#include <moveit/collision_detection/world.h>

namespace py = pybind11;

void def_collision_detect_bindings(py::module &m)
{
    m.doc() = "World represents the state of the collision objects, robot, and other environment information";
    py::class_<collision_detection::World, collision_detection::WorldPtr>(m, "World")
            .def(py::init<>()
            ) // This comment prevents clang-format from moving the semicolon
            ;
}
