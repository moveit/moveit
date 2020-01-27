/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#ifndef MOVEIT_PY_BINDINGS_TOOLS_SERIALIZE_MSG_
#define MOVEIT_PY_BINDINGS_TOOLS_SERIALIZE_MSG_

#include <ros/ros.h>
#include <Python.h>
#include <boost/python.hpp>
#include <string>
#include <stdexcept>

namespace moveit
{
namespace py_bindings_tools
{
/** \brief Convert a ROS message to a string */
template <typename T>
std::string serializeMsg(const T& msg)
{
  // we use the fact char is same size as uint8_t;
  assert(sizeof(uint8_t) == sizeof(char));
  std::size_t size = ros::serialization::serializationLength(msg);
  std::string result(size, '\0');
  if (size)
  {
    // we convert the message into a string because that is easy to sent back & forth with Python
    // This is fine since C0x because &string[0] is guaranteed to point to a contiguous block of memory
    ros::serialization::OStream stream_arg(reinterpret_cast<uint8_t*>(&result[0]), size);
    ros::serialization::serialize(stream_arg, msg);
  }
  return result;
}

/** \brief Convert a string to a ROS message */
template <typename T>
void deserializeMsg(const std::string& data, T& msg)
{
  assert(sizeof(uint8_t) == sizeof(char));
  ros::serialization::IStream stream_arg(const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(&data[0])), data.size());
  ros::serialization::deserialize(stream_arg, msg);
}

#if PY_MAJOR_VERSION >= 3 || defined(DOXYGEN)

/** \brief C++ Wrapper class for Python 3 \c Bytes Object */
class ByteString : public boost::python::object
{
public:
  // constructors for bp::handle and friends
  BOOST_PYTHON_FORWARD_OBJECT_CONSTRUCTORS(ByteString, boost::python::object)
  ByteString() : boost::python::object(boost::python::handle<>(PyBytes_FromString("")))
  {
  }
  explicit ByteString(const char* s) : boost::python::object(boost::python::handle<>(PyBytes_FromString(s)))
  {
  }
  explicit ByteString(const std::string& s)
    : boost::python::object(boost::python::handle<>(PyBytes_FromStringAndSize(s.c_str(), s.size())))
  {
  }
};

/** \brief Convert a Python 3 Bytestring to a ROS message */
template <typename T>
void deserializeMsg(const ByteString& data, T& msg)
{
  const std::string s = boost::python::extract<std::string>(data);
  deserializeMsg(s, msg);
}

/** \brief Convert a ROS message to a Python 3 ByteString */
template <typename T>
ByteString serializeMsgByteString(const T& msg)
{
  return ByteString(serializeMsg(msg));
}
#endif

// for old Python version
#if PY_MAJOR_VERSION < 3 || defined(DOXYGEN)
/** \brief Convenience wrapper replacement for Python 2 */
using ByteString = std::string;

/** \brief Convenience wrapper replacement for Python 2 */
template <typename T>
std::string serializeMsgByteString(const T& msg)
{
  return serializeMsg(msg);
}

#endif
}
}

#if PY_MAJOR_VERSION >= 3
namespace boost
{
namespace python
{
namespace converter
{
// only accept python 3 Bytes instance when used as C++ function parameter
template <>
struct object_manager_traits<moveit::py_bindings_tools::ByteString>
    : pytype_object_manager_traits<&PyBytes_Type, moveit::py_bindings_tools::ByteString>
{
};
}
}
}
#endif

#endif
