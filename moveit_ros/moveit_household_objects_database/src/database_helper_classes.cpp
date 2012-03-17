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

#include "moveit_household_objects_database/database_helper_classes.h"

namespace moveit_household_objects_database {

std::istream& operator >> (std::istream &str, DatabasePose &dhp)
{
  std::vector<double> vec;
  database_interface::operator >> (str,vec);
  if (str.fail()) return str;
  if (vec.size() != 7)
  {
    str.clear(std::ios::failbit);
    return str;
  }
  dhp.pose_.position.x = vec[0]; dhp.pose_.position.y = vec[1]; dhp.pose_.position.z = vec[2];
  dhp.pose_.orientation.x = vec[4]; dhp.pose_.orientation.y = vec[5]; dhp.pose_.orientation.z = vec[6];
  dhp.pose_.orientation.w = vec[3];
  return str;
}

std::ostream& operator << (std::ostream &str, const DatabasePose &dhp)
{
  std::vector<double> vec(7);
  vec[0] = dhp.pose_.position.x;   vec[1] = dhp.pose_.position.y;   vec[2] = dhp.pose_.position.z; 
  vec[4] = dhp.pose_.orientation.x; vec[5] = dhp.pose_.orientation.y; vec[6] = dhp.pose_.orientation.z; 
  vec[3] = dhp.pose_.orientation.w; 
  return database_interface::operator << (str,vec);
}

std::istream& operator >> (std::istream &str, DatabaseHandPosture &dhp)
{
  std::vector<double> vec;
  database_interface::operator >> (str,vec);
  if (str.fail()) return str;
  if (vec.empty())
  {
    str.clear(std::ios::failbit);
    return str;
  }
  dhp.joint_angles_.clear();
  dhp.joint_angles_.insert( dhp.joint_angles_.begin(), vec.begin(), vec.end() );
  return str;
}

std::ostream& operator << (std::ostream &str, const DatabaseHandPosture &dhp)
{
  std::vector<double> vec;
  vec.insert( vec.begin(), dhp.joint_angles_.begin(), dhp.joint_angles_.end() );
  return database_interface::operator << (str,vec);
}

} //namespace
