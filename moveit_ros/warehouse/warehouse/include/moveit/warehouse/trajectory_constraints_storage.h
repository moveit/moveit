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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Mario Prats, Ioan Sucan */

#ifndef MOVEIT_MOVEIT_WAREHOUSE_TRAJECTORY_CONSTRAINTS_STORAGE_
#define MOVEIT_MOVEIT_WAREHOUSE_TRAJECTORY_CONSTRAINTS_STORAGE_

#include "moveit/warehouse/moveit_message_storage.h"
#include <moveit/macros/class_forward.h>
#include <moveit_msgs/TrajectoryConstraints.h>

namespace moveit_warehouse
{

typedef mongo_ros::MessageWithMetadata<moveit_msgs::TrajectoryConstraints>::ConstPtr TrajectoryConstraintsWithMetadata;
typedef boost::shared_ptr<mongo_ros::MessageCollection<moveit_msgs::TrajectoryConstraints> > TrajectoryConstraintsCollection;

MOVEIT_CLASS_FORWARD(TrajectoryConstraintsStorage);

class TrajectoryConstraintsStorage : public MoveItMessageStorage
{
public:

  static const std::string DATABASE_NAME;

  static const std::string CONSTRAINTS_ID_NAME;
  static const std::string CONSTRAINTS_GROUP_NAME;
  static const std::string ROBOT_NAME;


  /** \brief Initialize the trajectory constraints storage to connect to a specified \e host and \e port for the MongoDB.
      If defaults are used for the parameters (empty host name, 0 port), the constructor looks for ROS params specifying
      which host/port to use. NodeHandle::searchParam() is used starting from ~ to look for warehouse_port and warehouse_host.
      If no values are found, the defaults are left to be the ones MongoDB uses.
      If \e wait_seconds is above 0, then a maximum number of seconds can elapse until connection is successful, or a runtime exception is thrown. */
  TrajectoryConstraintsStorage(const std::string &host = "", const unsigned int port = 0, double wait_seconds = 5.0);

  void addTrajectoryConstraints(const moveit_msgs::TrajectoryConstraints &msg, const std::string &name, const std::string &robot = "", const std::string &group = "");
  bool hasTrajectoryConstraints(const std::string &name, const std::string &robot = "", const std::string &group = "") const;
  void getKnownTrajectoryConstraints(std::vector<std::string> &names, const std::string &robot = "", const std::string &group = "") const;
  void getKnownTrajectoryConstraints(const std::string &regex, std::vector<std::string> &names, const std::string &robot = "", const std::string &group = "") const;

  /** \brief Get the constraints named \e name. Return false on failure. */
  bool getTrajectoryConstraints(TrajectoryConstraintsWithMetadata &msg_m, const std::string &name, const std::string &robot = "", const std::string &group = "") const;

  void renameTrajectoryConstraints(const std::string &old_name, const std::string &new_name, const std::string &robot = "", const std::string &group = "");

  void removeTrajectoryConstraints(const std::string &name, const std::string &robot = "", const std::string &group = "");

  void reset(void);

private:

  void createCollections(void);

  TrajectoryConstraintsCollection constraints_collection_;

};
}

#endif
