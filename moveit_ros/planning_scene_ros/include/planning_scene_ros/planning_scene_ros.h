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

#ifndef PLANNING_SCENE_PLANNING_SCENE_ROS_
#define PLANNING_SCENE_PLANNING_SCENE_ROS_

#include <ros/ros.h>
#include <tf/tf.h>
#include <planning_scene/planning_scene.h>
#include "planning_scene_ros/current_state_monitor.h"

namespace planning_scene_ros
{

    class PlanningSceneROS : public planning_scene::PlanningScene
    {
    public:
        PlanningSceneROS(const std::string &robot_description, tf::Transformer *tf= NULL);

        const std::string& getRobotDescription(void) const
        {
            return robot_description_;
        }

        const CurrentStateMonitorPtr& getStateMonitor(void) const
        {
            return csm_;
        }

        void startStateMonitor(void);
        void stopStateMonitor(void);

    protected:

        bool loadRobotFromParamServer(void);
        void configureDefaultCollisionMatrix(void);
        void configureDefaultPadding(void);

        ros::NodeHandle        nh_;
        tf::Transformer       *tf_;
        std::string            robot_description_;
        double                 default_robot_padd_;
        double                 default_robot_scale_;
        double                 default_object_padd_;
        double                 default_attached_padd_;

      boost::shared_ptr<urdf::Model>            urdf_;
      boost::shared_ptr<srdf::Model>            srdf_;

        CurrentStateMonitorPtr csm_;
    };

}

#endif
