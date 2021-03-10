#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <moveit/python/pybind_rosmsg_typecasters.h>

#include <moveit/transforms/transforms.h>

namespace py = pybind11;
using namespace moveit::core;

void def_transforms_bindings(py::module& m)
{
  m.doc() = "Provides an implementation of a snapshot of a transform tree that can be easily queried for transforming "
            "different quantities. Transforms are maintained as a list of transforms to a particular frame. All stored "
            "transforms are considered fixed.";
  py::class_<Transforms, TransformsPtr>(m, "Transforms").def(py::init<const std::string&>())
      //
      ;
}
