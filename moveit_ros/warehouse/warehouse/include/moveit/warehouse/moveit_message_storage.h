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

#ifndef MOVEIT_MOVEIT_WAREHOUSE_MOVEIT_MESSAGE_STORAGE_
#define MOVEIT_MOVEIT_WAREHOUSE_MOVEIT_MESSAGE_STORAGE_

#include <mongo_ros/message_collection.h>
#include <vector>
#include <string>

namespace moveit_warehouse
{

/** \brief This class provides the mechanism to connect to a MongoDB and reads needed ROS parameters when appropriate. */
class MoveItMessageStorage
{
public:
  /** \brief Initialize the storage to connect to a specified \e host and \e port for the MongoDB. 
      If defaults are used for the parameters (empty host name, 0 port), the constructor looks for ROS params specifying 
      which host/port to use. NodeHandle::searchParam() is used starting from ~ to look for warehouse_port and warehouse_host.
      If these params are not found either, a final attempt is made to look for the param values under /moveit_warehouse/warehouse_*.
      If no values are found, the defaults are left to be the ones MongoDB uses. 
      If \e wait_seconds is above 0, then a maximum number of seconds can elapse until connection is successful, or a runtime exception is thrown. */
  MoveItMessageStorage(const std::string &host = "", const unsigned int port = 0, double wait_seconds = 5.0);

  virtual ~MoveItMessageStorage();
  
  const std::string& getDatabaseHost() const
  {
    return db_host_;    
  }
  
  unsigned int getDatabasePort() const
  {
    return db_port_;
  }
  
protected:
  
  /// Keep only the \e names that match \e regex
  void filterNames(const std::string &regex, std::vector<std::string> &names) const;
  
  void drop(const std::string &db);
  
  std::string  db_host_;
  unsigned int db_port_;
  double       timeout_;
};
}

#endif
