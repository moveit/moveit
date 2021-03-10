#pragma once

#include <pybind11/pybind11.h>
#include <ros/duration.h>
#include <ros/serialization.h>

/** Provide pybind11 type converters for ROS types */

namespace moveit
{
namespace python
{
PYBIND11_EXPORT pybind11::object createMessage(const std::string& ros_msg_name);
PYBIND11_EXPORT bool convertible(const pybind11::handle& h, const char* ros_msg_name);

}  // namespace python
}  // namespace moveit

namespace pybind11
{
namespace detail
{
/// Convert ros::Duration / ros::WallDuration into a float
template <typename T>
struct DurationCaster
{
  // C++ -> Python
  static handle cast(T&& src, return_value_policy /* policy */, handle /* parent */)
  {
    return PyFloat_FromDouble(src.toSec());
  }

  // Python -> C++
  bool load(handle src, bool convert)
  {
    if (hasattr(src, "to_sec"))
    {
      value = T(src.attr("to_sec")().cast<double>());
    }
    else if (convert)
    {
      value = T(src.cast<double>());
    }
    else
      return false;
    return true;
  }

  PYBIND11_TYPE_CASTER(T, _("Duration"));
};

template <>
struct type_caster<ros::Duration> : DurationCaster<ros::Duration>
{
};

template <>
struct type_caster<ros::WallDuration> : DurationCaster<ros::WallDuration>
{
};

/// Convert ROS message types (C++ <-> python)
template <typename T>
struct type_caster<T, enable_if_t<ros::message_traits::IsMessage<T>::value>>
{
  // C++ -> Python
  static handle cast(const T& src, return_value_policy /* policy */, handle /* parent */)
  {
    // serialize src into (python) buffer
    std::size_t size = ros::serialization::serializationLength(src);
    object pbuffer = reinterpret_steal<object>(PyBytes_FromStringAndSize(nullptr, size));
    ros::serialization::OStream stream(reinterpret_cast<uint8_t*>(PyBytes_AsString(pbuffer.ptr())), size);
    ros::serialization::serialize(stream, src);
    // deserialize python type from buffer
    object msg = moveit::python::createMessage(ros::message_traits::DataType<T>::value());
    msg.attr("deserialize")(pbuffer);
    return msg.release();
  }

  // Python -> C++
  bool load(handle src, bool convert)
  {
    if (!moveit::python::convertible(src, ros::message_traits::DataType<T>::value()))
      return false;
    // serialize src into (python) buffer
    object pstream = module::import("io").attr("BytesIO")();
    src.attr("serialize")(pstream);
    object pbuffer = pstream.attr("getvalue")();
    // deserialize C++ type from buffer
    char* cbuffer = nullptr;
    Py_ssize_t size;
    PyBytes_AsStringAndSize(pbuffer.ptr(), &cbuffer, &size);
    ros::serialization::IStream cstream(const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(cbuffer)), size);
    ros::serialization::deserialize(cstream, value);
    return true;
  }

  PYBIND11_TYPE_CASTER(T, _<T>());
};
}  // namespace detail
}  // namespace pybind11
