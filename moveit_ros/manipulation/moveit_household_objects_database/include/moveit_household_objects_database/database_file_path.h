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

#ifndef _DATABASE_FILE_PATH_H_
#define _DATABASE_FILE_PATH_H_

#include <database_interface/db_class.h>


namespace moveit_household_objects_database {

class DatabaseFilePath : public database_interface::DBClass
{
 private:

 public:

  database_interface::DBField<int> id_;
  database_interface::DBField<int> original_model_id_;
  database_interface::DBField<std::string> file_type_;
  database_interface::DBField<std::string> file_path_;

 DatabaseFilePath() :
  id_(database_interface::DBFieldBase::TEXT, this, "file_path_id", "file_path", true),
  original_model_id_(database_interface::DBFieldBase::TEXT, this, "original_model_id", "file_path", true),
  file_type_(database_interface::DBFieldBase::TEXT, this, "file_type", "file_path", true),
  file_path_(database_interface::DBFieldBase::TEXT, this, "file_path_path", "file_path", true)
    {
      primary_key_field_ = &id_;
      fields_.push_back(&original_model_id_);
      fields_.push_back(&file_type_);
      fields_.push_back(&file_path_);
      setAllFieldsWriteToDatabase(true);
      setAllFieldsReadFromDatabase(true);
      id_.setWriteToDatabase(false);
      id_.setSequenceName("file_path_sequence");
      
    }
  ~DatabaseFilePath(){}
};

}
#endif
