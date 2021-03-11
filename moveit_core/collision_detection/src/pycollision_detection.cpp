#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <moveit/collision_detection/world.h>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/python/pybind_rosmsg_typecasters.h>

namespace py = pybind11;

using namespace collision_detection;

auto SET_ENTRY1 = py::overload_cast<const std::string&, const std::string&, bool>(&AllowedCollisionMatrix::setEntry);

void def_collision_detect_bindings(py::module& m)
{
  m.doc() = "contains collision detection, the world, and allowed collision matrices";
  py::enum_<BodyType>(m, "BodyType")
      .value("ROBOT_ATTACHED", BodyType::ROBOT_ATTACHED)
      .value("ROBOT_LINK", BodyType::ROBOT_LINK)
      .value("WORLD_OBJECT", BodyType::WORLD_OBJECT)
      .export_values();
  py::class_<Contact>(m, "Contact")
      .def(py::init<>())
      .def_readwrite("body_name_1", &Contact::body_name_1)
      .def_readwrite("body_name_2", &Contact::body_name_2)
      .def_readwrite("body_type_1", &Contact::body_type_1)
      .def_readwrite("body_type_2", &Contact::body_type_2)
      .def_readwrite("depth", &Contact::depth)
      .def_property_readonly("nearest_points",
                             [](const Contact & contact) {
                               std::vector<Eigen::Vector3d> v{ contact.nearest_points[0], contact.nearest_points[1] };
                               return v;
                             })
      .def_readwrite("normal", &Contact::normal)
      .def_readwrite("percent_interpolation", &Contact::percent_interpolation)
      .def_readwrite("pos", &Contact::pos)
    //
      ;
  py::class_<CollisionRequest>(m, "CollisionRequest")
      .def(py::init<>())
      .def_readwrite("contacts", &CollisionRequest::contacts)
      .def_readwrite("cost", &CollisionRequest::cost)
      .def_readwrite("distance", &CollisionRequest::distance)
      .def_readwrite("group_name", &CollisionRequest::group_name)
      .def_readwrite("is_done", &CollisionRequest::is_done)
      .def_readwrite("max_contacts", &CollisionRequest::max_contacts)
      .def_readwrite("max_contacts_per_pair", &CollisionRequest::max_contacts_per_pair)
      .def_readwrite("max_cost_sources", &CollisionRequest::max_cost_sources)
      .def_readwrite("verbose", &CollisionRequest::verbose)
      //
      ;
  py::class_<CollisionResult>(m, "CollisionResult")
      .def(py::init<>())
      .def_readwrite("collision", &CollisionResult::collision)
      .def_readwrite("contact_count", &CollisionResult::contact_count)
      .def_readwrite("contacts", &CollisionResult::contacts)
      .def_readwrite("cost_sources", &CollisionResult::cost_sources)
      .def_readwrite("distance", &CollisionResult::distance)
      .def("clear", &CollisionResult::clear)
      //
      ;
  py::class_<AllowedCollisionMatrix>(m, "AllowedCollisionMatrix").def(py::init<>()).def("setEntry", SET_ENTRY1)
      //
      ;
  py::class_<World, WorldPtr>(m, "World").def(py::init<>());
}
