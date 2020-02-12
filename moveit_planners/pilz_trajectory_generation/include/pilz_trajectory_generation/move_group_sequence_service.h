/*
 * Copyright (c) 2018 Pilz GmbH & Co. KG
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef SEQUENCE_SERVICE_CAPABILITY_H
#define SEQUENCE_SERVICE_CAPABILITY_H

#include <moveit/move_group/move_group_capability.h>

#include <pilz_msgs/GetMotionSequence.h>

namespace pilz_trajectory_generation
{

// Forward declarations
class CommandListManager;

/**
 * @brief Provide service to blend multiple trajectories in the form of a MoveGroup capability (plugin).
 */
class MoveGroupSequenceService : public move_group::MoveGroupCapability
{
public:

  MoveGroupSequenceService();
  ~MoveGroupSequenceService();

  virtual void initialize() override;

private:
  bool plan(pilz_msgs::GetMotionSequence::Request &req,
            pilz_msgs::GetMotionSequence::Response &res);

private:
  ros::ServiceServer sequence_service_;
  std::unique_ptr<CommandListManager> command_list_manager_ ;

};

}



#endif // SEQUENCE_SERVICE_CAPABILITY_H
