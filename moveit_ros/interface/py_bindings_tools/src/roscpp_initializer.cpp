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
*   * Neither the name of the Willow Garage nor the names of its
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

#include "moveit/py_bindings_tools/roscpp_initializer.h"
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>

struct InitProxy
{
  InitProxy()
  {
    char **fake_argv = new char*[1];
    fake_argv[0] = strdup("moveit_python_wrappers");
    int fake_argc = 1;
    ros::init(fake_argc, fake_argv, "moveit_python_wrappers", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
    delete[] fake_argv[0];
    delete[] fake_argv;
  }
  
  ~InitProxy()
  { 
    if (ros::isInitialized() && !ros::isShuttingDown())
      ros::shutdown();
  }
};

moveit_py_bindings_tools::ROScppInitializer::ROScppInitializer()
{
  // ensure we do not accidentally initialize ROS multiple times per process
  static boost::mutex lock;
  boost::mutex::scoped_lock slock(lock);
  
  // once per process, we start a spinner
  static bool once = true;
  if (once)
  {
    once = false;
    static boost::shared_ptr<InitProxy> proxy;
    
    // if ROS (cpp) is not initialized, we initialize it
    if (!ros::isInitialized())
      proxy.reset(new InitProxy());
    
    static ros::AsyncSpinner spinner(1);
    spinner.start();
  }
}
