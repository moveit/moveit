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

#ifndef _DATABASE_SCALED_MODEL_H_
#define _DATABASE_SCALED_MODEL_H_

#include <database_interface/db_class.h>

namespace moveit_household_objects_database {

class DatabaseScaledModel : public database_interface::DBClass
{
 private:

 public:
  //primary key
  database_interface::DBField<int> id_;
  //fields from the scaled_model table
  database_interface::DBField<double> scale_;
  database_interface::DBField<int> original_model_id_;

  //fields from the original_model table
  //maybe at some point we'll inherit these directly
  database_interface::DBField<std::string> model_;
  database_interface::DBField<std::string> maker_;
  database_interface::DBField< std::vector<std::string> > tags_;
  database_interface::DBField<std::string> source_;
  database_interface::DBField<std::string> acquisition_method_;
  
 DatabaseScaledModel()  : 
    id_(database_interface::DBFieldBase::TEXT, this, "scaled_model_id", "scaled_model", true),
    scale_(database_interface::DBFieldBase::TEXT, this, "scaled_model_scale", "scaled_model", true),
    original_model_id_(database_interface::DBFieldBase::TEXT, this, "original_model_id", "scaled_model", true),
    //fields "inherited" from the original model have no write access
    model_(database_interface::DBFieldBase::TEXT, this, "original_model_model" ,"original_model", false),
    maker_(database_interface::DBFieldBase::TEXT, this, "original_model_maker", "original_model", false),
    tags_(database_interface::DBFieldBase::TEXT, this, "original_model_tags", "original_model", false),
    source_(database_interface::DBFieldBase::TEXT, this, "original_model_source", "original_model", false),
    acquisition_method_(database_interface::DBFieldBase::TEXT, this, "acquisition_method_name", "original_model", false)
  {
    primary_key_field_ = &id_;
    fields_.push_back(&scale_);
    fields_.push_back(&original_model_id_);

    fields_.push_back(&model_);
    fields_.push_back(&maker_);
    fields_.push_back(&tags_);
    fields_.push_back(&source_);
    fields_.push_back(&acquisition_method_);

    setAllFieldsReadFromDatabase(true);
    setAllFieldsWriteToDatabase(false);
    scale_.setWriteToDatabase(true);
    original_model_id_.setWriteToDatabase(true);

    id_.setSequenceName("model_model_id_seq");
    id_.setWriteToDatabase(false);

    foreign_keys_.insert( std::pair<std::string, database_interface::DBFieldBase*>("original_model", &original_model_id_) );
  }
};

} //namespace model_database

#endif
