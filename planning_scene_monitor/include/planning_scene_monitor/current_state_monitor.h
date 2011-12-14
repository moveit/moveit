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

#ifndef PLANNING_SCENE_MONITOR_CURRENT_STATE_MONITOR_
#define PLANNING_SCENE_MONITOR_CURRENT_STATE_MONITOR_

#include <ros/ros.h>
#include <tf/tf.h>
#include <planning_models/kinematic_state.h>
#include <sensor_msgs/JointState.h>
#include <boost/function.hpp>
#include <boost/thread/mutex.hpp>

namespace planning_scene_monitor
{

    typedef boost::function<void(const sensor_msgs::JointStateConstPtr &joint_state)> JointStateUpdateCallback;

    class CurrentStateMonitor
    {
    public:

        CurrentStateMonitor(const planning_models::KinematicModelConstPtr &kmodel, tf::Transformer *tf);

        void startStateMonitor(void);
        void stopStateMonitor(void);
        bool haveCompleteState(void) const;
        bool haveCompleteState(const ros::Duration &age) const;

        planning_models::KinematicStatePtr getCurrentState(void) const;
        std::map<std::string, double> getCurrentStateValues(void) const;
        void setOnStateUpdateCallback(const JointStateUpdateCallback &callback);

        /** \brief When a joint value is received to be out of bounds, it is changed slightly to fit within bounds,
            if the difference is less than the specified error. */
        double getBoundsError(void) const
        {
            return error_;
        }

        /** \brief When a joint value is received to be out of bounds, it is changed slightly to fit within bounds,
            if the difference is less than the specified error. */
        void setBoundsError(double error)
        {
            error_ = error;
        }

    private:

        void jointStateCallback(const sensor_msgs::JointStateConstPtr &joint_state);

        ros::NodeHandle                              nh_;
        tf::Transformer                             *tf_;
        planning_models::KinematicModelConstPtr      kmodel_;
        planning_models::KinematicState              kstate_;
        planning_models::KinematicState::JointState *root_;
        std::map<std::string, ros::Time>             joint_time_;
        bool                                         state_monitor_started_;
        double                                       error_;
        ros::Subscriber                              joint_state_subscriber_;

        mutable boost::mutex                         state_update_lock_;
        JointStateUpdateCallback                     on_state_update_callback_;
    };

    typedef boost::shared_ptr<CurrentStateMonitor> CurrentStateMonitorPtr;
    typedef boost::shared_ptr<const CurrentStateMonitor> CurrentStateMonitorConstPtr;
}

#endif
