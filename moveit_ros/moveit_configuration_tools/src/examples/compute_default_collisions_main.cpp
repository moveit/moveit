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

#include <moveit_configuration_tools/widgets/compute_default_collisions_widget.h>
#include <QApplication>
#include <QtGui>
//#include <boost/thread.hpp>

using namespace std;

int collision_progress = 0;

int main(int argc, char **argv)
{
  // Seed Random
  srand(time(NULL));

  // Start ROS Node
  //ros::init(argc, argv, "compute_default_collisions", ros::init_options::NoSigintHandler);
  ros::init(argc, argv, "compute_default_collisions");

  //ros::AsyncSpinner spinner(1);
  //spinner.start();  
 
  // Create Qt Application
  QApplication qtApp(argc, argv);

  // Load Qt Widget
  ComputeDefaultCollisionsWidget * cdcm = new ComputeDefaultCollisionsWidget();
  qtApp.setActiveWindow(cdcm);
  //cdcm->showMaximized();
  cdcm->setMinimumWidth(1024);
  cdcm->setMinimumHeight(768);
  cdcm->show();

  // Error check
  /*if(!cdcm->isInited())
  {
    ROS_WARN_STREAM("Unable to initialize Qt Widget. Exiting");
    exit(0);
    }*/

  // For sending transforms and markers
  //  boost::thread spin_thread(boost::bind(&spin_function));
  //ros::shutdown();    

  return qtApp.exec();
}
