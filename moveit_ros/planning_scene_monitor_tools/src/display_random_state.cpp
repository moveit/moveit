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

#include <planning_scene_monitor/planning_scene_monitor.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "display_random_state");
  
  bool valid = false;
  bool invalid = false;
  for (int i = 0 ; i < argc ; ++i)
  {
    if (strcmp(argv[i], "--valid") == 0)
    {
      valid = true;
      break;
    }
    if (strcmp(argv[i], "--invalid") == 0)
    {
      invalid = true;
      break;
    }
  }
  
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  ros::NodeHandle nh;
  planning_models_loader::KinematicModelLoader::Options opt;
  opt.robot_description_ = "robot_description";
  planning_models_loader::KinematicModelLoaderPtr kml(new planning_models_loader::KinematicModelLoader(opt));
  planning_scene_monitor::PlanningSceneMonitor psm(kml);
  psm.startWorldGeometryMonitor();
  psm.startSceneMonitor();
  ros::Publisher pub_scene = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  
  ros::Duration(0.5).sleep();
  
  do 
  {
    if(!psm.getPlanningScene())
    {
      ROS_ERROR("Planning scene did not load properly, exiting...");
      break;
    }
    
    std::cout << "Type a number and hit Enter. That number of ";
    if (valid)
      std::cout << "valid ";
    else
      if (invalid)
        std::cout << "invalid ";
    std::cout << "states will be randomly generated at an interval of one second and published as a planning scene." << std::endl;
    std::size_t n;
    std::cin >> n;
    
    for (std::size_t i = 0 ; i < n ; ++i)
    {
      if (valid)
      {
        bool found = false;
        unsigned int attempts = 0;
        do 
        {
          attempts++;
          psm.getPlanningScene()->getCurrentState().setToRandomValues();
          collision_detection::CollisionRequest req;
          collision_detection::CollisionResult res;
          psm.getPlanningScene()->checkCollision(req, res);
          found = !res.collision;
        } while (!found && attempts < 100);
        if (!found)
        {
          std::cout << "Unable to find valid state" << std::cout;
          continue;
        }
      }
      else
        if (invalid)
        {
          bool found = false;
          unsigned int attempts = 0;
          do 
          {
            attempts++;
            psm.getPlanningScene()->getCurrentState().setToRandomValues();
            collision_detection::CollisionRequest req;
            collision_detection::CollisionResult res;
            psm.getPlanningScene()->checkCollision(req, res);
            found = res.collision;
          } while (!found && attempts < 100);
          if (!found)
          {
            std::cout << "Unable to find invalid state" << std::cout;
            continue;
          }
        }
        else
          psm.getPlanningScene()->getCurrentState().setToRandomValues();

      psm.getPlanningScene()->getCurrentState().setToDefaultValues();

      
      // r0
      double v[7] = { 2.09642, 0.90423, -0.420767, -0.195229, 2.32139 ,-1.5101, -1.02677 };
      std::vector<double> vv; vv.insert(vv.end(), v, v+7);
      psm.getPlanningScene()->getCurrentState().getJointStateGroup("left_arm")->setStateValues(vv);
      double q[7] = {-1.3675, 0.224576 ,-1.81731, -1.37681, 2.778 ,-0.983775, -1.19062 } ;
      std::vector<double> qq; qq.insert(qq.end(), q, q+7);
      psm.getPlanningScene()->getCurrentState().getJointStateGroup("right_arm")->setStateValues(qq);
            

      /*
      // r7
      double v[7] = {  -0.146608 ,0.838234, -0.201172, -0.33163, 1.55722 ,-1.09131 ,0.793076};
      std::vector<double> vv; vv.insert(vv.end(), v, v+7);
      psm.getPlanningScene()->getCurrentState().getJointStateGroup("left_arm")->setStateValues(vv);
      double q[7] = { 0.0692123, 0.310049, -2.50733, -0.752757, -1.93879, -1.67769, -0.489014 } ;
      std::vector<double> qq; qq.insert(qq.end(), q, q+7);
      psm.getPlanningScene()->getCurrentState().getJointStateGroup("right_arm")->setStateValues(qq);
      */            

      /*
      // r10 (tucked)
      double v[7] = {0.0654119 ,1.27547, 1.81169, -1.6877, -1.70563 ,-0.134536, -0.0694477 };
      std::vector<double> vv; vv.insert(vv.end(), v, v+7);
      psm.getPlanningScene()->getCurrentState().getJointStateGroup("left_arm")->setStateValues(vv);
      double q[7] = {-0.0544564 ,1.08707 ,-1.52602, -2.1115 ,-1.40485, -1.83555, 0.214071  } ;
      std::vector<double> qq; qq.insert(qq.end(), q, q+7);
      psm.getPlanningScene()->getCurrentState().getJointStateGroup("right_arm")->setStateValues(qq);
      */      

      /*
      // r9
      double v[7] = { 0.467443, 0.300118, -0.281953, -1.1776, 0.867952, -0.520881, 2.39421};
      std::vector<double> vv; vv.insert(vv.end(), v, v+7);
      psm.getPlanningScene()->getCurrentState().getJointStateGroup("left_arm")->setStateValues(vv);
      double q[7] = {0.513607 ,0.362847, -2.99738, -2.00923, 0.13881, -1.24666, -0.234934  } ;
      std::vector<double> qq; qq.insert(qq.end(), q, q+7);
      psm.getPlanningScene()->getCurrentState().getJointStateGroup("right_arm")->setStateValues(qq);
      */

      /*
      // r4
      double v[7] = {0.300959, 0.440359, 0.927072, -0.46064, 1.67819, -0.337447, -2.24238};
      std::vector<double> vv; vv.insert(vv.end(), v, v+7);
      psm.getPlanningScene()->getCurrentState().getJointStateGroup("left_arm")->setStateValues(vv);
      double q[7] = {  0.287, 0.872195, -1.59107, -1.77545, 2.87086, -0.381915, -2.22692} ;
      std::vector<double> qq; qq.insert(qq.end(), q, q+7);
      psm.getPlanningScene()->getCurrentState().getJointStateGroup("right_arm")->setStateValues(qq);
      */
      moveit_msgs::PlanningScene psmsg;
      psm.getPlanningScene()->getPlanningSceneMsg(psmsg);
      pub_scene.publish(psmsg);
      psm.getPlanningScene()->getCurrentState().printStateInfo();
      
      sleep(1);        
    }
  } while (nh.ok());    
  
  ros::shutdown();
  return 0;
}
