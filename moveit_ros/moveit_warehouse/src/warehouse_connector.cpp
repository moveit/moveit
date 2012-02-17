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

/* Author: E. Gil Jones */

#include <mongo_ros/message_collection.h>
#include <sys/types.h>
#include <signal.h>
#include <unistd.h>
#include <ros/package.h>
#include <moveit_warehouse/warehouse_connector.h>

namespace moveit_warehouse
{

WarehouseConnector::WarehouseConnector() {
  child_pid_ = 0;
}

WarehouseConnector::~WarehouseConnector() {
  if(child_pid_ != 0) {
    kill(child_pid_, SIGTERM);
  }
}

bool WarehouseConnector::connectToDatabase(const std::string& dirname, 
                                           boost::shared_ptr<PlanningSceneStorage>& planning_scene_storage) {
  
  if(child_pid_ != 0) {
    kill(child_pid_, SIGTERM);
  }
  
  child_pid_ = fork();
  if(child_pid_ == 0) {
    std::string mongoexec = "/opt/ros/fuerte/stacks/warehousewg/mongodb/mongo/bin/mongod";
    char** argv = new char*[4];
    
    argv[0] = new char[25];
    sprintf(argv[0], "mongod");
    
    argv[1] = new char[25];
    sprintf(argv[1], "--dbpath");
    
    argv[2] = new char[1024];
    sprintf(argv[2], "%s", dirname.c_str());
    
    argv[3] = NULL;
    
    //ros::package::getPath("mongodb");//+"/bin/mongod";
    int code = execv(mongoexec.c_str(), argv);
    delete[] argv[0];
    delete[] argv[1];
    delete[] argv[2];
    delete[] argv;
    ROS_INFO_STREAM("Deleting " << code << " errno " << errno << " string errno " << strerror(errno));
    //std::string("--dbpath").c_str(), "/u/gjones/.ros/arm_navigation_dbs/pr2_manip_db", NULL);
  } else {
    //sleep so mongod has time to come up
    ros::WallDuration(1.0).sleep();
    planning_scene_storage.reset(new PlanningSceneStorage());
  }
  return true;
}

}
