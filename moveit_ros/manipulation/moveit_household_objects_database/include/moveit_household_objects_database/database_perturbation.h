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

//! \author Peter Brook

#ifndef _DATABASE_GRASP_ANALYSIS_H_
#define _DATABASE_GRASP_ANALYSIS_H_

#include <vector>

#include <database_interface/db_class.h>

namespace moveit_household_objects_database {

//! Contains a database record of a grasp
class DatabasePerturbation : public database_interface::DBClass
{
 private:
  const std::string table_name_;
 public:
  //! The primary key, id of this perturbation in the database
  database_interface::DBField<int> perturbation_id_;

  //! The id of the grasp that this is analyzing
  database_interface::DBField<int> grasp_id_;

  //! The id of the energy function we are using
  database_interface::DBField<int> energy_function_id_;

  //! The deltas (6 double values) used to perturb the final grasp pose
  database_interface::DBField<std::vector<double> > deltas_;

  //! The score the perturbed grasp achieved
  database_interface::DBField<double> score_;

  //! The final pose at which energy functions were evaluated. This is stored so that we don't have to reapply reactive grasping methods
  database_interface::DBField<DatabasePose> final_pose_;
  //! Only initialized fields
  DatabasePerturbation() :
    table_name_("grasp_analysis"),
   perturbation_id_(database_interface::DBFieldBase::TEXT, this, "perturbation_id", table_name_, true),
   grasp_id_(database_interface::DBFieldBase::TEXT, this, "grasp_id", table_name_, true),
   energy_function_id_(database_interface::DBFieldBase::TEXT, this, "energy_function_id", table_name_, true),
   deltas_(database_interface::DBFieldBase::TEXT, this, "deltas", table_name_, true),
   score_(database_interface::DBFieldBase::TEXT, this, "score", table_name_, true),
   final_pose_(database_interface::DBFieldBase::TEXT, this, "final_position", table_name_, true)
  {
    //primary key field
    primary_key_field_ = &perturbation_id_;

    //all the other fields
    fields_.push_back(&grasp_id_);
    fields_.push_back(&energy_function_id_);
    fields_.push_back(&deltas_);
    fields_.push_back(&score_);
    fields_.push_back(&final_pose_);
    //by default, all fields here are used, and many of then not-null, so sync both ways
    setAllFieldsReadFromDatabase(true);
    setAllFieldsWriteToDatabase(true);

    //sequences
    perturbation_id_.setSequenceName("grasp_analysis_perturbation_id_seq");

    //primary key only syncs from database; it has a sequence which is used by default on insertions
    perturbation_id_.setWriteToDatabase(false);
  }

  //! Empty stub
  ~DatabasePerturbation(){}
};

typedef boost::shared_ptr<DatabasePerturbation> DatabasePerturbationPtr;

} //namespace

#endif
