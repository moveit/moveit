/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

// Author(s): Mehmet Dogar (mdogar@cs.cmu.edu)

#ifndef _DATABASE_OBJECT_PATHS_H_
#define _DATABASE_OBJECT_PATHS_H_

#include <iostream>

#include <geometry_msgs/Pose.h>

#include <database_interface/db_class.h>

#include <boost/shared_ptr.hpp>

#include "moveit_household_objects_database/database_helper_classes.h"

namespace moveit_household_objects_database {

//! Contains a database record of a capture region.
class DatabaseObjectPaths : public database_interface::DBClass
{
 private:

 public:
  //! The primary key, id of this capture region in the database.
  database_interface::DBField<int> id_;

  //! The scaled model that this capture region is for.
  database_interface::DBField<int> scaled_model_id_;

  database_interface::DBField<int> object_db_id_;

  //! The description (usually name) of the object this capture region is for.
  database_interface::DBField<std::string> object_description_;
  //! The unique hash generated from the object geometry.
  database_interface::DBField<std::string> object_geometry_hash_;
  //! The unique hash generated from the robot geometry.
  database_interface::DBField<std::string> robot_geometry_hash_;

  //! The coefficient of friction between the hand and the object used to compute this capture region.
  database_interface::DBField<double> hand_object_coefficient_of_friction_;
  //! The pushing distance used to compute this capture region.
  database_interface::DBField<double> pushing_distance_;
  //! The heuristic used to decide when the object is in the hand.
  database_interface::DBField<double> in_hand_distance_;

  //! The resolution the capture region is computed in lateral offset.
  database_interface::DBField<double> y_resolution_;
  //! The starting point in lateral offset.
  database_interface::DBField<double> start_y_offset_;
  //! The number of steps in lateral offset.
  database_interface::DBField<int> n_y_steps_;

  //! The rotational resolution the capture region is computed for.
  database_interface::DBField<double> rotation_resolution_;
  //! The number of steps in rotations.
  database_interface::DBField<int> n_rotations_;

  //! The motion type of the object for every lateral offset - rotation - pressure distribution tuple, during the push.
  database_interface::DBField< std::vector< std::vector< std::vector< std::vector< int > > > > > motion_types_;

  //! The poses of the object for every lateral offset - rotation - pressure distribution tuple, during the push.
  database_interface::DBField< std::vector< std::vector< std::vector< std::vector< DatabasePose > > > > > poses_;

  //! The length of each computed path.
  database_interface::DBField< std::vector< std::vector< std::vector< int > > > > path_lengths_;


  //! The radius of the cylinder bounding the object along the axis normal to thepushing surface.
  database_interface::DBField<double> radius_of_cylinder_bounding_the_object_;
  //! The coordinate the frame this capture region is computed in relative to the object's original coordinate frame.
  database_interface::DBField<DatabasePose> object_pushing_frame_in_object_frame_;

  //! The height of the contact on the object.
  database_interface::DBField<double> fingertip_frame_to_pushing_surface_distance_;
  //! The maximum width of the fingers.
  database_interface::DBField<double> finger_width_;

  //! The width of the aperture of the hand for the object paths computation.
  database_interface::DBField<double> aperture_width_; 

  //! The minimum value for the pressure distribution scale. Must be between 0-1.
  database_interface::DBField<double> min_pressure_distribution_scale_;
  //! The maximum value for the pressure distribution scale. Must be between 0-1.
  database_interface::DBField<double> max_pressure_distribution_scale_;
  //! The resolution to change the pressure distribution scale. Must be between 0-1.
  database_interface::DBField<double> pressure_distribution_scale_resolution_;
  //! Number of different pressure distributions scales used during simulation.
  database_interface::DBField<int> n_pressure_distributions_;

  //! The pushing resolution to use when storing the object path data.
  database_interface::DBField<double> path_storage_resolution_;


  //! Only initialized fields
  DatabaseObjectPaths() :
    id_(database_interface::DBFieldBase::TEXT, this, "object_paths_id", "object_paths", true),
    scaled_model_id_(database_interface::DBFieldBase::TEXT, this, "scaled_model_id", "object_paths", true),
    object_db_id_(database_interface::DBFieldBase::TEXT, this, "object_db_id", "object_paths", true),
    object_description_(database_interface::DBFieldBase::TEXT, this, "object_description", "object_paths", true),
    object_geometry_hash_(database_interface::DBFieldBase::TEXT, this, "object_geometry_hash", "object_paths", true),
    robot_geometry_hash_(database_interface::DBFieldBase::TEXT, this, "robot_geometry_hash", "object_paths", true),
    hand_object_coefficient_of_friction_(database_interface::DBFieldBase::TEXT, this, "hand_object_coefficient_of_friction", "object_paths", true),
    pushing_distance_(database_interface::DBFieldBase::TEXT, this, "pushing_distance", "object_paths", true),
    in_hand_distance_(database_interface::DBFieldBase::TEXT, this, "in_hand_distance", "object_paths", true),
    y_resolution_(database_interface::DBFieldBase::TEXT, this, "y_resolution", "object_paths", true),
    start_y_offset_(database_interface::DBFieldBase::TEXT, this, "start_y_offset", "object_paths", true),
    n_y_steps_(database_interface::DBFieldBase::TEXT, this, "n_y_steps", "object_paths", true),
    rotation_resolution_(database_interface::DBFieldBase::TEXT, this, "rotation_resolution", "object_paths", true),
    n_rotations_(database_interface::DBFieldBase::TEXT, this, "n_rotations", "object_paths", true),
    motion_types_(database_interface::DBFieldBase::TEXT, this, "motion_types", "object_paths", true),
    poses_(database_interface::DBFieldBase::TEXT, this, "poses", "object_paths", true),
    path_lengths_(database_interface::DBFieldBase::TEXT, this, "path_lengths", "object_paths", true),
    radius_of_cylinder_bounding_the_object_(database_interface::DBFieldBase::TEXT, this, "radius_of_cylinder_bounding_the_object", "object_paths", true),
    object_pushing_frame_in_object_frame_(database_interface::DBFieldBase::TEXT, this, "object_pushing_frame_in_object_frame", "object_paths", true),
    fingertip_frame_to_pushing_surface_distance_(database_interface::DBFieldBase::TEXT, this, "fingertip_frame_to_pushing_surface_distance", "object_paths", true),
    finger_width_(database_interface::DBFieldBase::TEXT, this, "finger_width", "object_paths", true),
    aperture_width_(database_interface::DBFieldBase::TEXT, this, "aperture_width", "object_paths", true),
    min_pressure_distribution_scale_(database_interface::DBFieldBase::TEXT, this, "min_pressure_distribution_scale", "object_paths", true),
    max_pressure_distribution_scale_(database_interface::DBFieldBase::TEXT, this, "max_pressure_distribution_scale", "object_paths", true),
    pressure_distribution_scale_resolution_(database_interface::DBFieldBase::TEXT, this, "pressure_distribution_scale_resolution", "object_paths", true),
    n_pressure_distributions_(database_interface::DBFieldBase::TEXT, this, "n_pressure_distributions", "object_paths", true),
    path_storage_resolution_(database_interface::DBFieldBase::TEXT, this, "path_storage_resolution", "object_paths", true)
  {
    //primary key field
    primary_key_field_ = &id_;
    //all the other fields
    fields_.push_back(&scaled_model_id_);
    fields_.push_back(&object_db_id_);
    fields_.push_back(&object_description_);
    fields_.push_back(&object_geometry_hash_);
    fields_.push_back(&robot_geometry_hash_);
    fields_.push_back(&hand_object_coefficient_of_friction_);
    fields_.push_back(&pushing_distance_);
    fields_.push_back(&in_hand_distance_);
    fields_.push_back(&y_resolution_);
    fields_.push_back(&start_y_offset_);
    fields_.push_back(&n_y_steps_);
    fields_.push_back(&rotation_resolution_);
    fields_.push_back(&n_rotations_);
    fields_.push_back(&motion_types_);
    fields_.push_back(&poses_);
    fields_.push_back(&path_lengths_);
    fields_.push_back(&radius_of_cylinder_bounding_the_object_);
    fields_.push_back(&object_pushing_frame_in_object_frame_);
    fields_.push_back(&fingertip_frame_to_pushing_surface_distance_);
    fields_.push_back(&finger_width_);
    fields_.push_back(&aperture_width_);
    fields_.push_back(&min_pressure_distribution_scale_);
    fields_.push_back(&max_pressure_distribution_scale_);
    fields_.push_back(&pressure_distribution_scale_resolution_);
    fields_.push_back(&n_pressure_distributions_);
    fields_.push_back(&path_storage_resolution_);

    //sequences
    id_.setSequenceName("object_paths_id_seq");

    //by default, all fields here are used, and many of then not-null, so sync both ways
    setAllFieldsReadFromDatabase(true);
    setAllFieldsWriteToDatabase(true);
    //primary key id_ only syncs from database; it has a sequence which is used by default on insertions
    id_.setWriteToDatabase(false);
  }

  //! Empty stub
  ~DatabaseObjectPaths(){}
};

typedef boost::shared_ptr<DatabaseObjectPaths> DatabaseObjectPathsPtr;

} //namespace

#endif
