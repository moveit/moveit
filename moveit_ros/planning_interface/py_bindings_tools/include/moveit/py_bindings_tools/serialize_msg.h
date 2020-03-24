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
}
}

#endif
