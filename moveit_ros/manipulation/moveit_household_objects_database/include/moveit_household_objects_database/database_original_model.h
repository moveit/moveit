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

#ifndef _DATABASE_ORIGINAL_MODEL_H_
#define _DATABASE_ORIGINAL_MODEL_H_

#include <database_interface/db_class.h>

namespace moveit_household_objects_database {

//! The C++ version of an original model entry in the database
/*! Note that:
  - all data stored in the database is declared as database_interface::DBField<...>
*/
class DatabaseOriginalModel : public database_interface::DBClass
{
 private:

 public:
  //primary key
  database_interface::DBField<int> id_;
  //fields from the original_model table
  database_interface::DBField<std::string> model_;
  database_interface::DBField<std::string> maker_;
  database_interface::DBField< std::vector<std::string> > tags_;
  database_interface::DBField<std::string> source_;
  database_interface::DBField<std::string> description_;
  database_interface::DBField<std::string> barcode_;
  database_interface::DBField<std::string> acquisition_method_;
  database_interface::DBField<std::string> recognition_id_;
  database_interface::DBField<bool> concave_filled_;

  //! Places all the fields in the fields_ vector and sets foreign keys and sequences
  /*! Note that:
    - the address of the id_ field is noted as primary key
    - the addresses of all the other fields are stored in fields_
    - all the needed foreign keys are inserted for referencing fields that live in different
      table than out primary key
    - the sequence name used by our primary key field is noted
   */
  void initFields()
  {
    //set the primary key
    primary_key_field_ = &id_;
    //and the rest of the fields
    fields_.push_back(&model_);
    fields_.push_back(&maker_);
    fields_.push_back(&tags_);
    fields_.push_back(&source_);
    fields_.push_back(&description_);
    fields_.push_back(&barcode_);
    fields_.push_back(&acquisition_method_);
    fields_.push_back(&recognition_id_);
    fields_.push_back(&concave_filled_);
    
    //sequences
    id_.setSequenceName("unscaled_model_unscaled_model_id_seq");
  }

  //! Initializes permissions for the fields
  /*! Note that:
    - default behavior is set for which fields are to be read from / written to the database.
      That this can always be changed later for any particular instance of this class, if you 
      want to save or load any particular fields.
   */
  void initPermissions()
  {
    //by default, most fields are not used, so they should not be read from or written to database
    //WARNING: do this AFTER inserting your data into the fields_ vector
    setAllFieldsReadFromDatabase(false);
    setAllFieldsWriteToDatabase(false);
    //the fields that are usually used:
    //primary key id_ only syncs from database; it has a sequence which is used by default on insertions
    id_.setReadFromDatabase(true);
    //others that only sync to instance (we don't save them by default, but we try to retrieve them)
    tags_.setReadFromDatabase(true);
    barcode_.setReadFromDatabase(true);
    //the rest sync both ways; we'll put here those that have a NOT NULL constraint in the database
    model_.setReadWrite(true);
    maker_.setReadWrite(true);
    source_.setReadWrite(true);
    acquisition_method_.setReadWrite(true);
  }

  //! Constructs the fields, then calls initFields() followed by initPermissions()
  /*! Note that:
    - all fields are intialized with the type, owner (this), column name and table name
   */
 DatabaseOriginalModel()  : 
    id_(database_interface::DBFieldBase::TEXT, this, "original_model_id", "original_model", true),
    model_(database_interface::DBFieldBase::TEXT, this, "original_model_model" ,"original_model", true),
    maker_(database_interface::DBFieldBase::TEXT, this, "original_model_maker", "original_model", true),
    tags_(database_interface::DBFieldBase::TEXT, this, "original_model_tags", "original_model", true),
    source_(database_interface::DBFieldBase::TEXT, this, "original_model_source", "original_model", true),
    description_(database_interface::DBFieldBase::TEXT, this, "original_model_description", "original_model", true),
    barcode_(database_interface::DBFieldBase::TEXT, this, "original_model_barcode", "original_model", true),
    acquisition_method_(database_interface::DBFieldBase::TEXT, this, "acquisition_method_name", "original_model", true),
    recognition_id_(database_interface::DBFieldBase::TEXT, this, "original_model_recognition_id", 
                    "original_model", true),
    concave_filled_(database_interface::DBFieldBase::TEXT, this, "original_model_concave_filled", 
		    "original_model", true)
  {
    initFields();
    initPermissions();
  }

    //! Copy-constructs the fields based on the copied instance fields, then calls initFields()
    /*! The data itself in the fields, as well as the permissions, gets copied in the 
      field copy construction.
    */
 DatabaseOriginalModel(const DatabaseOriginalModel *other) : 
    id_(this, &other->id_),
    model_(this, &other->model_),
    maker_(this, &other->maker_),
    tags_(this, &other->tags_),
    source_(this, &other->source_),
    description_(this, &other->description_),
    barcode_(this, &other->barcode_),
    acquisition_method_(this, &other->acquisition_method_),
    recognition_id_(this, &other->recognition_id_),
    concave_filled_(this, &other->concave_filled_)
      {
	initFields();
	//no need to call initPermissions() since field permission are copied over from other
      }
};

} //namespace model_database

#endif
