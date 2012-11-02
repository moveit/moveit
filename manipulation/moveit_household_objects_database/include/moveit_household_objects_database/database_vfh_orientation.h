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

// Author(s): Aitor Aldoma

#ifndef _DATABASE_VFH_ORIENTATION_H_
#define _DATABASE_VFH_ORIENTATION_H_

#include <database_interface/db_class.h>

#include "moveit_household_objects_database/database_helper_classes.h"

namespace moveit_household_objects_database {

class DatabaseVFHOrientation : public database_interface::DBClass
{
 private:

 public:
  //! The id of this descriptor 
  database_interface::DBField<int> vfh_orientation_id_;
  //! The id of the view that this descriptor was computed for
  database_interface::DBField<int> view_id_;
  //! The id of the view that this descriptor was computed for
  database_interface::DBField<int> vfh_id_;
  //! The data itself
  database_interface::DBField< std::vector<uint8_t> > vfh_orientation_descriptor_;

 DatabaseVFHOrientation() :
  vfh_orientation_id_(database_interface::DBFieldBase::TEXT, this, "vfh_orientation_id", "vfh_orientation", true),
  view_id_(database_interface::DBFieldBase::TEXT, this, "view_id", "vfh_orientation", true),
  vfh_id_(database_interface::DBFieldBase::TEXT, this, "vfh_id", "vfh_orientation", true),
  vfh_orientation_descriptor_(database_interface::DBFieldBase::BINARY, this, "vfh_orientation_descriptor", "vfh_orientation", true)
  {
    primary_key_field_ = &vfh_orientation_id_;
    fields_.push_back(&view_id_);
    fields_.push_back(&vfh_id_);
    fields_.push_back(&vfh_orientation_descriptor_);

    vfh_orientation_id_.setSequenceName("vfh_orientation_id_seq");
    
    setAllFieldsWriteToDatabase(true);
    setAllFieldsReadFromDatabase(true);
    vfh_orientation_id_.setWriteToDatabase(false);
    vfh_orientation_descriptor_.setWriteToDatabase(false);
    vfh_orientation_descriptor_.setReadFromDatabase(false);
  }
  ~DatabaseVFHOrientation(){}
};

} //namespace moveit_household_objects_database

#endif
