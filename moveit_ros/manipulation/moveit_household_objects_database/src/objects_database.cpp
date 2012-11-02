/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

// Author(s): Matei Ciocarlie

#include "moveit_household_objects_database/objects_database.h"

#include <database_interface/db_filters.h>

#include "moveit_household_objects_database/database_task.h"

using namespace database_interface;

namespace moveit_household_objects_database {

bool ObjectsDatabase::acquireNextTask(std::vector< boost::shared_ptr<DatabaseTask> > &task)
{
  //first get and mark (atomically) the task id
  DatabaseTaskID id_example;
  std::vector< boost::shared_ptr<DatabaseTaskID> > id_vec;
  if ( !getList<DatabaseTaskID>(id_vec, id_example, "") )
  {
    ROS_ERROR("Failed to get the id of the next task to be run");
    return false;
  }
  if (id_vec.empty())
  {
    //no task to be run
    return true;
  }
  if (id_vec.size() != 1)
  {
    ROS_ERROR("Next task acquisition returned more than one result");
    return false;
  }
  //get the actual task id
  int id = id_vec[0]->id_.get();
  std::stringstream idstr;
  idstr << id;
  std::string where_clause("dbase_task_id=" + idstr.str());
  //fill in the rest
  if (!getList<DatabaseTask>(task, where_clause) || task.size() != 1 )
  {
    ROS_ERROR("Acquire next task: failed to populate entry");
    return false;
  }
  return true;
}

}//namespace
