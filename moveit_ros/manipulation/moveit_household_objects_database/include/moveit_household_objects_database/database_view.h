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

#ifndef _DATABASE_VIEW_H_
#define _DATABASE_VIEW_H_

#include <database_interface/db_class.h>

#include "moveit_household_objects_database/database_helper_classes.h"

namespace moveit_household_objects_database {

class DatabaseView : public database_interface::DBClass
{
 private:

 public:
  //! The id of the view
  database_interface::DBField<int> view_id_;
  //! The original model id
  database_interface::DBField<int> scaled_model_id_;
  //! The transform for the camera for this view
  database_interface::DBField<DatabasePose> view_transform_;
  //! The iteration of this view
  database_interface::DBField<int> iteration_;
  //! The data itself
  database_interface::DBField< std::vector<uint8_t> > view_point_cloud_data_;

  database_interface::DBField< std::vector<float> > centroid_;

 DatabaseView() :
  view_id_(database_interface::DBFieldBase::TEXT, this, "view_id", "view", true),
  scaled_model_id_(database_interface::DBFieldBase::TEXT, this, "scaled_model_id", "view", true),
  view_transform_(database_interface::DBFieldBase::TEXT, this, "view_transform", "view", true),
  iteration_(database_interface::DBFieldBase::TEXT, this, "iteration", "view", true),
  view_point_cloud_data_(database_interface::DBFieldBase::BINARY, this, "view_point_cloud_data", "view", true),
  centroid_(database_interface::DBFieldBase::TEXT, this, "centroid", "view", true)
  {
    primary_key_field_ = &view_id_;
    fields_.push_back(&scaled_model_id_);
    fields_.push_back(&view_transform_);
    fields_.push_back(&iteration_);
    fields_.push_back(&view_point_cloud_data_);
    fields_.push_back(&centroid_);

    view_id_.setSequenceName("view_id_seq");
    
    setAllFieldsWriteToDatabase(true);
    setAllFieldsReadFromDatabase(true);
    //don't write the id as it has a sequence
    view_id_.setWriteToDatabase(false);
    view_point_cloud_data_.setWriteToDatabase(false);
    view_point_cloud_data_.setReadFromDatabase(false);
  }
  ~DatabaseView(){}
};

} //namespace moveit_household_objects_database

#endif
