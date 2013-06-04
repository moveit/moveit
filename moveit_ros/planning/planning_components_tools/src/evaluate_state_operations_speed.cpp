/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Willow Garage, Inc.
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

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/profiler/profiler.h>
#include <ros/ros.h>

static const std::string ROBOT_DESCRIPTION = "robot_description";

int main(int argc, char **argv)
{
    ros::init(argc, argv, "evaluate_state_operations_speed");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    robot_model_loader::RobotModelLoader rml(ROBOT_DESCRIPTION);
    ros::Duration(0.5).sleep();
    
    robot_model::RobotModelConstPtr robot_model = rml.getModel();
    if (robot_model)
    {
      static const int N = 1000;
      robot_state::RobotState state(robot_model);

      ROS_INFO("Testing model '%s':", robot_model->getName().c_str());
      
      moveit::Profiler::Clear();
      moveit::Profiler::Start();
      
      ROS_INFO("Evaluating FK Default ...");
      for (int i = 0 ; i < N ; ++i)
      {
        moveit::Profiler::Begin("FK Default");
        state.setToDefaultValues();
        moveit::Profiler::End("FK Default");
      }

      ROS_INFO("Evaluating FK Random ...");
      for (int i = 0 ; i < N ; ++i)
      {
        moveit::Profiler::Begin("FK Random");
        state.setToRandomValues();
        moveit::Profiler::End("FK Random");
      } 
      std::vector<robot_state::RobotState*> copies(N, NULL);  
      ROS_INFO("Evaluating Copy State ...");
      for (int i = 0 ; i < N ; ++i)
      {
        moveit::Profiler::Begin("Copy State");
        copies[i] = new robot_state::RobotState(state);
        moveit::Profiler::End("Copy State");
      }

      ROS_INFO("Evaluating Free State ...");
      for (int i = 0 ; i < N ; ++i)
      {
        moveit::Profiler::Begin("Free State");
        delete copies[i];
        moveit::Profiler::End("Free State");
      }
      
      const std::vector<std::string> &groups = robot_model->getJointModelGroupNames();
      for (std::size_t j = 0 ; j < groups.size() ; ++j)
      {
        robot_state::JointStateGroup *jsg = state.getJointStateGroup(groups[j]);
        ROS_INFO("%s: Evaluating FK Default ...", groups[j].c_str());
        std::string pname = groups[j] + ":FK Default";
        for (int i = 0 ; i < N ; ++i)
        {
          moveit::Profiler::Begin(pname);
          jsg->setToDefaultValues();
          moveit::Profiler::End(pname);
        }
        
        ROS_INFO("%s: Evaluating FK Random ...", groups[j].c_str());
        pname = groups[j] + ":FK Random";
        for (int i = 0 ; i < N ; ++i)
        {
          moveit::Profiler::Begin(pname);
          jsg->setToRandomValues();
          moveit::Profiler::End(pname);
        }
      }
      
      moveit::Profiler::Stop();
      moveit::Profiler::Status();
    }
    else
      ROS_ERROR("Unable to initialize robot model.");
    
    ros::shutdown();
    return 0;
}
