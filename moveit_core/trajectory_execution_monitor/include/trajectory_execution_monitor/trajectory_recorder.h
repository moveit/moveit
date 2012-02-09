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

/** \author E. Gil Jones, Ken Anderson */

#ifndef _TRAJECTORY_RECORDER_H_
#define _TRAJECTORY_RECORDER_H_

#include <ros/ros.h>
#include <boost/function.hpp>

#include <trajectory_msgs/JointTrajectory.h>

namespace trajectory_execution_monitor
{

/// \brief Function that gets called when new state information arrives
typedef boost::function<bool(	const ros::Time& time, const std::map<std::string, double>&,
                              const std::map<std::string,double>&)> NewStateCallbackFunction;

/// \brief Records the trajectory by calling registered callback functions as new states arrive.
class TrajectoryRecorder {

public:

  TrajectoryRecorder(const std::string& recorder_name) : recorder_name_(recorder_name) 
  {};

  /// \brief Register a callback function to get called when a new state arrives.
  void registerCallback(const std::string& name, const NewStateCallbackFunction& callback) {
    callback_map_[name] = callback;
  };

  /// \brief Deregisters a callback that is no longer to be called when a new state arrives.
  /// Do NOT call this from inside NewStateCallbackFunction,
  /// or you may find yourself invalidating an iterator and segfaulting.
  void deregisterCallback(const std::string& name) {
    callback_map_.erase(name);
  };

  /// \brief Deregisters a callback that is no longer to be called when a new state arrives.
  /// This function is should be called when the calling function is a NewStateCallbackFunction.
  void delayedDeregisterCallback(const std::string& name) {
    deregister_list_.push_back(name);
  }

  /// \brief The name of the recorder
  const std::string& getName() const {
    return recorder_name_;
  }
    
protected:

  /// \brief Call all of the registered callbacks.
  void callCallbacks(const ros::Time& time,
                     const std::map<std::string, double>& joint_positions,
                     const std::map<std::string, double>& joint_velocities) ;
  
  std::string recorder_name_;
  std::map<std::string, NewStateCallbackFunction> callback_map_;
  std::vector<std::string> deregister_list_;

};

}

#endif
