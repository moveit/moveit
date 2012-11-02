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

// Author(s): Peter Brook

#ifndef _DATABASE_HELPER_CLASSES_H_
#define _DATABASE_HELPER_CLASSES_H_

#include <database_interface/db_class.h>
#include <geometry_msgs/Pose.h>

namespace database_interface {

//! Specialized version for binary conversion from a database binary blob to a vector of float
template <>
class DBField< std::vector<float> > : public DBFieldData< std::vector<float> >
{
 public:
  DBField(Type type, DBClass *owner, std::string name, std::string table_name, bool write_permission) : 
    DBFieldData< std::vector<float> >(type, owner, name, table_name, write_permission) {}

 DBField(DBClass *owner, const DBField< std::vector<float> > *other) : 
    DBFieldData< std::vector<float> >(owner, other) 
      {
	this->copy(other);
      }
  
  virtual bool fromBinary(const char* binary, size_t length) 
  {
    //recall that the size given here is in bytes
    if (!length)
    {
      data_.clear();
      return true;
    }
    //check if this is indeed a multiple of a number of floats
    if ( length % sizeof(float) != 0)
    {
      std::cerr << "Binary conversion of " << length << " bytes to vector<float> failed\n";
      return false;
    }
    data_.resize(length / sizeof(float));
    memcpy(&(data_[0]), binary, length);
    return true;
  }

  virtual bool toBinary(const char* &binary, size_t &length) const 
  {
    length = sizeof(float) * data_.size();
    if (!data_.empty())
    {
      binary = reinterpret_cast<const char*>(&(data_[0]));
    }
    return true;
  }
};

//! Specialized version for binary conversion from a database binary blob to a vector of uint8
template <>
class DBField< std::vector<uint8_t> > : public DBFieldData< std::vector<uint8_t> >
{
 public:
  DBField(Type type, DBClass *owner, std::string name, std::string table_name, bool write_permission) :
    DBFieldData< std::vector<uint8_t> >(type, owner, name, table_name, write_permission) {}

 DBField(DBClass *owner, const DBField< std::vector<uint8_t> > *other) :
    DBFieldData< std::vector<uint8_t> >(owner, other)
      {
	this->copy(other);
      }

  virtual bool fromBinary(const char* binary, size_t length)
  {
    //recall that the size given here is in bytes
    if (!length)
    {
      data_.clear();
      return true;
    }
    //check if this is indeed a multiple of a number of uint8's
    if ( length % sizeof(uint8_t) != 0)
    {
      std::cerr << "Binary conversion of " << length << " bytes to vector<uint8_t> failed\n";
      return false;
    }
    data_.resize(length / sizeof(uint8_t));
    memcpy(&(data_[0]), binary, length);
    return true;
  }

  virtual bool toBinary(const char* &binary, size_t &length) const
  {
    length = sizeof(uint8_t) * data_.size();
    if (!data_.empty())
    {
      binary = reinterpret_cast<const char*>(&(data_[0]));
    }
    return true;
  }
};




} //namespace database_interface

namespace moveit_household_objects_database {

/* Database convention is currently as follows:

   - joints are represented in an array with the value of each DOF of the robot: {j1}
     (only one joint for the PR2)

   - position and orientation are also in an array, with pose as a translation and quaternion:
     {tx, ty, tz, qw, qx, qy, qz}
     WARNING: in the database, the w component of the quaternion is first!!!
*/

//! A wrapper for the geometry_msgs::pose so we can store it in the database
/*! Right now, in the database, the pose is stored in a weird way (see above), so we need a
  wrapper to take care of the conversion. In the future, hopefully we'll get rid of this.
 */
class DatabasePose
{
 public:
  geometry_msgs::Pose pose_;
};

std::istream& operator >> (std::istream &str, DatabasePose &dhp);
std::ostream& operator << (std::ostream &str, const DatabasePose &dhp);

//! A wrapper for a vector of doubles, used in the database to store the hand joint angles
/*! Right now, in the database, the posture is stored in a weird way (see above), so we need a
  wrapper to take care of the conversion. In the future, hopefully we'll get rid of this.
 */
class DatabaseHandPosture
{
 public:
  std::vector<double> joint_angles_;
};

std::istream& operator >> (std::istream &str, DatabaseHandPosture &dhp);
std::ostream& operator << (std::ostream &str, const DatabaseHandPosture &dhp);

} //namespace

#endif
