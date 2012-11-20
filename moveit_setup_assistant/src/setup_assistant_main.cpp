/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Author: Dave Coleman

#include "widgets/setup_assistant_widget.h"
#include <ros/ros.h>
#include <QApplication>
#include <QMessageBox>
#include <boost/program_options.hpp>
#include <signal.h>

static void siginthandler(int param)
{
  QApplication::quit();
}

int main(int argc, char **argv)
{ 
  // Parse parameters
  namespace po = boost::program_options;

  // Declare the supported options
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "Show help message")
    ("debug", "Run in debug/test mode")
    ("urdf_path", po::value<std::string>(), "Optional, relative path to URDF in URDF package")
    ("config_pkg", po::value<std::string>(), "Optional, pass in existing config package to load");

  // Process options
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);    

  if (vm.count("help"))
  {
    std::cout << desc << std::endl;
    return 1;
  }

  // Start ROS Node
  ros::init(argc, argv, "moveit_setup_assistant", ros::init_options::NoSigintHandler);

  // ROS Spin
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;

  // Create Qt Application
  QApplication qtApp(argc, argv);

  // Load Qt Widget
  moveit_setup_assistant::SetupAssistantWidget saw( NULL, vm );
  saw.setMinimumWidth(1024);
  saw.setMinimumHeight(768);
  //  saw.setWindowState( Qt::WindowMaximized );

  saw.show();

  signal(SIGINT, siginthandler);

  // Wait here until Qt App is finished
  const int result = qtApp.exec();

  // Shutdown ROS
  ros::shutdown();    

  return result;
}


