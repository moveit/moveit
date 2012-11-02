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

#ifndef _DATABASE_GRASP_PAIR_H_
#define _DATABASE_GRASP_PAIR_H_

#include <database_interface/db_class.h>

namespace moveit_household_objects_database {

//! Contains a database record of a grasp
class DatabaseGraspPair : public DBClass
{
 private:

 public:
  //! The primary key, id of this grasp in the database
  database_interface::DBField<int> pair_id_;

  //! The id of the first grasp in the pair
  database_interface::DBField<int> grasp1_id_;

  //! The id of the second grasp in the pair
  database_interface::DBField<int> grasp2_id_;

  //! Only initialized fields
  DatabaseGraspPair() :
   pair_id_(database_interface::DBFieldBase::TEXT, this, "pair_id", "grasp_pair", true),
   grasp1_id_(database_interface::DBFieldBase::TEXT, this, "grasp1_id", "grasp_pair", true),
   grasp2_id_(database_interface::DBFieldBase::TEXT, this, "grasp2_id", "grasp_pair", true)
  {
    //primary key field
    primary_key_field_ = &pair_id_;

    //all the other fields
    fields_.push_back(&grasp1_id_);
    fields_.push_back(&grasp2_id_);

    //by default, all fields here are used, and many of then not-null, so sync both ways
    setAllFieldsReadFromDatabase(true);
    setAllFieldsWriteToDatabase(true);

    //sequences
    pair_id_.setSequenceName("pair_id_seq");

    //primary key only syncs from database; it has a sequence which is used by default on insertions
    pair_id_.setWriteToDatabase(false);
  }

  //! Empty stub
  ~DatabaseGraspPair(){}
};

} //namespace

#endif
