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
  py::class_<Transforms, TransformsPtr>(m, "Transforms")
      .def(py::init<const std::string&>())
      .def("canTransform", &Transforms::canTransform)
      .def("getTargetFrame", &Transforms::getTargetFrame)
      .def("getTransform", &Transforms::getTransform)
      .def("isFixedFrame", &Transforms::isFixedFrame)
      .def("getAllTransforms", &Transforms::getAllTransforms)
      .def("setTransform", py::overload_cast<const Eigen::Isometry3d&, const std::string&>(&Transforms::setTransform))
      .def("setTransform", py::overload_cast<const geometry_msgs::TransformStamped&>(&Transforms::setTransform))
      .def("setTransforms", &Transforms::setTransforms)
      .def("setAllTransforms", &Transforms::setAllTransforms)
      .def("transformVector3", &Transforms::transformVector3)
      .def("transformQuaternion", &Transforms::transformQuaternion)
      .def("transformRotationMatrix", &Transforms::transformRotationMatrix)
      .def("transformPose", &Transforms::transformPose)
      //
      ;
}
