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
//#include <moveit_configuration_tools/tools/compute_default_collisions.h>
//#include <planning_scene_monitor/planning_scene_monitor.h>
#include <QApplication>
#include <QtGui>

using namespace std;

static const std::string ROBOT_DESCRIPTION="robot_description";

int main(int argv, char **args)
{
  /*
  ros::init(argc, argv, "compute_default_collisions", ros::init_options::AnonymousName);

  ros::AsyncSpinner spinner(1);
  spinner.start();  
  */

  //unsigned int num_trials = 10000;
  //const bool verbose = false; // Output benchmarking and statistics

  // Load robot description
  //planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION);

  // Find the default collision matrix - all links that are allowed to collide
  //const std::map<std::string, std::set<std::string> > &disabled_links = 
  //  moveit_configuration_tools::computeDefaultCollisions(psm.getPlanningScene(), true, num_trials, verbose);

  // Output results to an XML file
  //moveit_configuration_tools::outputDisabledCollisionsXML( disabled_links );

  //ros::shutdown();    

  QApplication app(argv, args);

  ComputeDefaultCollisionsWidget cdcm;
  cdcm.show();

  return app.exec();
}
