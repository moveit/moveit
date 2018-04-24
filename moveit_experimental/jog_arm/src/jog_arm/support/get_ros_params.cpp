///////////////////////////////////////////////////////////////////////////////
//      Title     : get_ros_params.cpp
//      Project   : jog_arm
//      Created   : 3/27/2018
//      Author    : Andy Zelenak
//      Platforms : Ubuntu 64-bit
//      Copyright : Copyright© The University of Texas at Austin, 2014-2017. All
//      rights reserved.
//
//          All files within this directory are subject to the following, unless
//          an alternative
//          license is explicitly included within the text of each file.
//
//          This software and documentation constitute an unpublished work
//          and contain valuable trade secrets and proprietary information
//          belonging to the University. None of the foregoing material may be
//          copied or duplicated or disclosed without the express, written
//          permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
//          AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
//          INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
//          PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
//          THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
//          NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
//          THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
//          University be liable for incidental, special, indirect, direct or
//          consequential damages or loss of profits, interruption of business,
//          or related expenses which may arise from use of software or
//          documentation,
//          including but not limited to those resulting from defects in
//          software
//          and/or documentation, or loss or inaccuracy of data of any kind.
//
///////////////////////////////////////////////////////////////////////////////

#include "jog_arm/support/get_ros_params.h"

std::string get_ros_params::getStringParam(const std::string& name, ros::NodeHandle& n)
{
  std::string s;
  if (!n.getParam(name, s))
    ROS_ERROR_STREAM_NAMED("getStringParam", "YAML config file does not "
                                             "contain parameter "
                                                 << name);
  return s;
}

double get_ros_params::getDoubleParam(const std::string& name, ros::NodeHandle& n)
{
  double value;
  if (!n.getParam(name, value))
    ROS_ERROR_STREAM_NAMED("getDoubleParam", "YAML config file does not "
                                             "contain parameter "
                                                 << name);
  return value;
}

double get_ros_params::getIntParam(const std::string& name, ros::NodeHandle& n)
{
  int value;
  if (!n.getParam(name, value))
    ROS_ERROR_STREAM_NAMED("getIntParam", "YAML config file does not "
                                          "contain parameter "
                                              << name);
  return value;
}

bool get_ros_params::getBoolParam(const std::string& name, ros::NodeHandle& n)
{
  bool value;
  if (!n.getParam(name, value))
    ROS_ERROR_STREAM_NAMED("getBoolParam", "YAML config file does not contain parameter " << name);
  return value;
}
