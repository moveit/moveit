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

#ifndef _DATABASE_MESH_H_
#define _DATABASE_MESH_H_

#include <database_interface/db_class.h>

namespace database_interface {

//! Specialized version for binary conversion from a database binary blob to a vector of int
template <>
class DBField< std::vector<int> > : public DBFieldData< std::vector<int> >
{
 public:
  DBField(Type type, DBClass *owner, std::string name, std::string table_name, bool write_permission) : 
    DBFieldData< std::vector<int> >(type, owner, name, table_name, write_permission) {}

 DBField(DBClass *owner, const DBField< std::vector<int> > *other) : DBFieldData< std::vector<int> >(owner, other) 
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
    //check if this is indeed a multiple of a number of integers
    if ( length % sizeof(int) != 0)
    {
      std::cerr << "Binary conversion of " << length << " bytes to vector<int> failed\n";
      return false;
    }
    data_.resize(length / sizeof(int));
    memcpy(&(data_[0]), binary, length);
    return true;
  }

  virtual bool toBinary(const char* &binary, size_t &length) const 
  {
    length = sizeof(int) * data_.size();
    if (!data_.empty())
    {
      binary = reinterpret_cast<const char*>(&(data_[0]));
    }
    return true;
  }
};

//! Specialized version for binary conversion from a database binary blob to a vector of double
template <>
class DBField< std::vector<double> > : public DBFieldData< std::vector<double> >
{
 public:
  DBField(Type type, DBClass *owner, std::string name, std::string table_name, bool write_permission) : 
    DBFieldData< std::vector<double> >(type, owner, name, table_name, write_permission) {}

 DBField(DBClass *owner, const DBField< std::vector<double> > *other) : 
    DBFieldData< std::vector<double> >(owner, other) 
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
    //check if this is indeed a multiple of a number of doubles
    if ( length % sizeof(double) != 0)
    {
      std::cerr << "Binary conversion of " << length << " bytes to vector<double> failed\n";
      return false;
    }
    data_.resize(length / sizeof(double));
    memcpy(&(data_[0]), binary, length);
    return true;
  }

  virtual bool toBinary(const char* &binary, size_t &length) const 
  {
    length = sizeof(double) * data_.size();
    if (!data_.empty())
    {
      binary = reinterpret_cast<const char*>(&(data_[0]));
    }
    return true;
  }
};

}

namespace moveit_household_objects_database {

class DatabaseMesh : public database_interface::DBClass
{
 private:

 public:
  //! The original model id
  database_interface::DBField<int> id_;
  //! List of vertices
  database_interface::DBField< std::vector<double> > vertices_;
  //! List of triangles
  database_interface::DBField< std::vector<int> > triangles_;

 DatabaseMesh() :
  id_(database_interface::DBFieldBase::TEXT, this, "original_model_id", "mesh", true),
  vertices_(database_interface::DBFieldBase::BINARY, this, "mesh_vertex_list", "mesh", true),
  triangles_(database_interface::DBFieldBase::BINARY, this, "mesh_triangle_list", "mesh", true)
    {
      primary_key_field_ = &id_;
      fields_.push_back(&vertices_);
      fields_.push_back(&triangles_);

      setAllFieldsWriteToDatabase(false);
      setAllFieldsReadFromDatabase(false);
      id_.setWriteToDatabase(true);
      id_.setReadFromDatabase(true);
    }
  ~DatabaseMesh(){}
};

} //namespace

#endif
