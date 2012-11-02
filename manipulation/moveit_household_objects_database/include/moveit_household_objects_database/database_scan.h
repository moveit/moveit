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

#ifndef _DATABASE_SCAN_H_
#define _DATABASE_SCAN_H_

#include <database_interface/db_class.h>

namespace moveit_household_objects_database {

class DatabasePose;

class DatabaseScan : public database_interface::DBClass
{
 private:
 public:
  //primary key
  database_interface::DBField<int> id_;
  //other fields
  database_interface::DBField<int> scaled_model_id_;
  database_interface::DBField<std::string> scan_bagfile_location_;
  database_interface::DBField<std::string> scan_source_;
  database_interface::DBField<DatabasePose> object_pose_;
  database_interface::DBField<std::string> frame_id_;
  database_interface::DBField<std::string> cloud_topic_;
  DatabaseScan() :
    id_(database_interface::DBFieldBase::TEXT, this, "scan_id", "scan", true),
    scaled_model_id_(database_interface::DBFieldBase::TEXT, this, "scaled_model_id", "scan", true),
    scan_bagfile_location_(database_interface::DBFieldBase::TEXT, this, "scan_bagfile_location", "scan", true),
    scan_source_(database_interface::DBFieldBase::TEXT, this, "scan_source", "scan", true),
    object_pose_(database_interface::DBFieldBase::TEXT, this, "object_pose", "scan", true),
    frame_id_(database_interface::DBFieldBase::TEXT, this, "frame_id", "scan", true),
    cloud_topic_(database_interface::DBFieldBase::TEXT, this, "cloud_topic", "scan", true)
  {
    primary_key_field_ = &id_;
    fields_.push_back(&scaled_model_id_);
    fields_.push_back(&scan_bagfile_location_);
    fields_.push_back(&scan_source_);
    fields_.push_back(&object_pose_);
    fields_.push_back(&frame_id_);
    fields_.push_back(&cloud_topic_);

    setAllFieldsReadFromDatabase(true);
    setAllFieldsWriteToDatabase(true);

    id_.setSequenceName("scan_scan_id_seq");
    id_.setWriteToDatabase(false);
  }
};

} //namespace

#endif
