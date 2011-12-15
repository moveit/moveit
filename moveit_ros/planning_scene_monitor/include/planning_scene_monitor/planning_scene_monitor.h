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

#ifndef PLANNING_SCENE_MONITOR_PLANNING_SCENE_MONITOR_
#define PLANNING_SCENE_MONITOR_PLANNING_SCENE_MONITOR_

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <planning_scene/planning_scene.h>
#include "planning_scene_monitor/robot_model_loader.h"
#include "planning_scene_monitor/current_state_monitor.h"

namespace planning_scene_monitor
{
    /**
       Subscribes to:

       planning_scene
       planning_scene_diff
     */
    class PlanningSceneMonitor
    {
    public:
        PlanningSceneMonitor(const std::string &robot_description, tf::Transformer *tf);
        PlanningSceneMonitor(const planning_scene::PlanningSceneConstPtr &parent, const std::string &robot_description, tf::Transformer *tf);
        ~PlanningSceneMonitor(void);

        const planning_scene::PlanningScenePtr& getPlanningScene(void)
        {
            return scene_;
        }

        const planning_scene::PlanningSceneConstPtr& getPlanningScene(void) const
        {
            return scene_const_;
        }

        const std::string& getRobotDescription(void) const
        {
            return robot_description_;
        }

        const CurrentStateMonitorPtr& getStateMonitor(void) const
        {
            return csm_;
        }

        void updateFixedTransforms(void);

        void startStateMonitor(const std::string &joint_states_topic = "joint_states");
        void stopStateMonitor(void);
	void useMonitoredState(void);
	
	void startSceneMonitor(const std::string &scene_topic = "planning_scene",
			       const std::string &scene_diff_topic = "planning_scene_diff");
	void stopSceneMonitor(void);
	
	void startWorldGeometryMonitor(const std::string &collision_objects_topic = "collision_object",
				       const std::string &attached_objects_topic = "attached_collision_object",
				       const std::string &collision_map_topic = "collision_map");
	void stopWorldGeometryMonitor(void);
	
	/** \brief Return the time when the last update was made to the planning scene (by the monitor) */
	const ros::Time& getLastUpdate(void) const
	{
	    return last_update_;
	}
	
        void lockScene(void);
        void unlockScene(void);

    protected:

        void initialize(const planning_scene::PlanningSceneConstPtr &parent, const std::string &robot_description);
        void configureDefaultCollisionMatrix(void);
        void configureDefaultPadding(void);

        void newPlanningSceneCallback(const moveit_msgs::PlanningSceneConstPtr &scene);
        void newPlanningSceneDiffCallback(const moveit_msgs::PlanningSceneConstPtr &scene);
        void collisionObjectCallback(const moveit_msgs::CollisionObjectConstPtr &obj);
        void attachObjectCallback(const moveit_msgs::AttachedCollisionObjectConstPtr &obj);
        void collisionMapCallback(const moveit_msgs::CollisionMapConstPtr &map);

        planning_scene::PlanningScenePtr      scene_;
        planning_scene::PlanningSceneConstPtr scene_const_;
        boost::mutex                          scene_update_mutex_;

        ros::NodeHandle                       nh_;
        ros::NodeHandle                       root_nh_;
        tf::Transformer                      *tf_;
        std::string                           robot_description_;
        double                                default_robot_padd_;
        double                                default_robot_scale_;
        double                                default_object_padd_;
        double                                default_attached_padd_;

        ros::Subscriber                       planning_scene_subscriber_;
        ros::Subscriber                       planning_scene_diff_subscriber_;

        message_filters::Subscriber<moveit_msgs::CollisionObject>
                                             *collision_object_subscriber_;
        tf::MessageFilter<moveit_msgs::CollisionObject>
                                             *collision_object_filter_;
        message_filters::Subscriber<moveit_msgs::AttachedCollisionObject>
                                             *attached_collision_object_subscriber_;
        message_filters::Subscriber<moveit_msgs::CollisionMap>
                                             *collision_map_subscriber_;
        tf::MessageFilter<moveit_msgs::CollisionMap>
                                             *collision_map_filter_;

        CurrentStateMonitorPtr                csm_;

	ros::Time                             last_update_;
    };

    typedef boost::shared_ptr<PlanningSceneMonitor> PlanningSceneMonitorPtr;
    typedef boost::shared_ptr<const PlanningSceneMonitor> PlanningSceneMonitorConstPtr;

}

#endif
