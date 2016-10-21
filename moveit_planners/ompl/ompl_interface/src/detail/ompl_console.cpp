/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

#include <ros/console.h>
#include <ompl/util/Console.h>

namespace ompl_inteface
{
class OutputHandlerROS : public ompl::msg::OutputHandler
{
public:
  OutputHandlerROS() : OutputHandler()
  {
  }

  virtual void log(const std::string &text, ompl::msg::LogLevel level, const char *filename, int line)
  {
    switch (level)
    {
      case ompl::msg::LOG_INFO:
      {
        ROSCONSOLE_DEFINE_LOCATION(true, ::ros::console::levels::Info, std::string(ROSCONSOLE_ROOT_LOGGER_NAME) + ".omp"
                                                                                                                  "l");
        if (ROS_UNLIKELY(__rosconsole_define_location__enabled))
        {
          ::ros::console::print(NULL, __rosconsole_define_location__loc.logger_,
                                __rosconsole_define_location__loc.level_, filename, line, "", "%s", text.c_str());
        }
      }
      break;
      case ompl::msg::LOG_WARN:
      {
        ROSCONSOLE_DEFINE_LOCATION(true, ::ros::console::levels::Warn, std::string(ROSCONSOLE_ROOT_LOGGER_NAME) + ".omp"
                                                                                                                  "l");
        if (ROS_UNLIKELY(__rosconsole_define_location__enabled))
        {
          ::ros::console::print(NULL, __rosconsole_define_location__loc.logger_,
                                __rosconsole_define_location__loc.level_, filename, line, "", "%s", text.c_str());
        }
      }
      break;
      case ompl::msg::LOG_ERROR:
      {
        ROSCONSOLE_DEFINE_LOCATION(true, ::ros::console::levels::Error,
                                   std::string(ROSCONSOLE_ROOT_LOGGER_NAME) + ".ompl");
        if (ROS_UNLIKELY(__rosconsole_define_location__enabled))
        {
          ::ros::console::print(NULL, __rosconsole_define_location__loc.logger_,
                                __rosconsole_define_location__loc.level_, filename, line, "", "%s", text.c_str());
        }
      }
      break;
      default:
        // debug
        {
          ROSCONSOLE_DEFINE_LOCATION(true, ::ros::console::levels::Debug,
                                     std::string(ROSCONSOLE_ROOT_LOGGER_NAME) + ".ompl");
          if (ROS_UNLIKELY(__rosconsole_define_location__enabled))
          {
            ::ros::console::print(NULL, __rosconsole_define_location__loc.logger_,
                                  __rosconsole_define_location__loc.level_, filename, line, "", "%s", text.c_str());
          }
        }
        break;
    }
  }
};

struct RegisterOH
{
  RegisterOH()
  {
    static OutputHandlerROS oh_ros;
    ompl::msg::useOutputHandler(&oh_ros);
    ompl::msg::setLogLevel(ompl::msg::LOG_DEBUG);
  }
};

static RegisterOH proxy;
}
