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

/* Author: Dave Coleman, Ioan Sucan */

#include <planning_scene_monitor/planning_scene_monitor.h>
#include "moveit_configuration_tools/compute_default_collision_matrix.h"
#include "moveit_configuration_tools/benchmark_timer.h"
//Temporary:
#include <iostream>b
#include <fstream>

static const std::string ROBOT_DESCRIPTION="robot_description";
BenchmarkTimer BTimer; // used for analyzing results

int main(int argc, char **argv)
{
  ros::init(argc, argv, "compute_default_collision_matrix", ros::init_options::AnonymousName);

  ros::AsyncSpinner spinner(1);
  spinner.start();  

  int trials = 1000;
  if( argc > 1 )
  {
    trials = atoi(argv[1]);
    ROS_INFO("Number of trials %d", trials);
  }

  // Write reslts to file
  //std::ofstream results;
  //results.open("/u/dcoleman/collision_data.csv");  

  //int trials = 1000; //for(int trials = 10; trials <= 1000; trials += 50)
  //{
  //int disabled_links_count = 0;

  std::cout << "TRIALS " << trials << " --------------------------- \n\n\n";

  // Do 10 tests
  //for(int ii = 0; ii < 10; ++ii)
  //{
  // Setup benchmark timer
  BTimer = BenchmarkTimer();
  BTimer.start("Total"); 
   
  // Load robot description
  planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION);

  // Find the default collision matrix - all links that are allowed to collide
  const std::map<std::string, std::vector<std::string> > &disabled_links = 
    moveit_configuration_tools::computeDefaultCollisionMatrix(psm.getPlanningScene(), false, trials);

  // Output the yaml file
  // TODO: convert to proper yaml file output method
  unsigned int n = 0;
  for (std::map<std::string, std::vector<std::string> >::const_iterator it = disabled_links.begin() ; it != disabled_links.end() ; ++it)
  {    
    for (std::size_t i = 0 ; i < it->second.size() ; ++i)
    {
      std::cout << "\t<disable_collisions link1=\"" << it->first << "\" link2=\"" << it->second[i] << "\" />" << std::endl;
      n++;
    }
  }
  
  // Benchmarking Results
  BTimer.end("Total"); 
  BTimer.printTimes(); // output results   
  // number of links disabled from collision checking
  std::cout << n << std::endl << std::endl;  

  /*disabled_links_cout += n;
    }
    results << trials << " " << disabled_links_count;
    }
    results.close();
  */

  ros::shutdown();    
  return 0;
}
