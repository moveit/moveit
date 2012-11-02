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

#ifndef _OBJECTS_DATABASE_H_
#define _OBJECTS_DATABASE_H_

#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

//for ROS error messages
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>

#include <shape_msgs/Mesh.h>

//#include <moveit_household_objects_database_msgs/DatabaseScan.h>

#include <database_interface/postgresql_database.h>

#include "moveit_household_objects_database/database_original_model.h"
#include "moveit_household_objects_database/database_scaled_model.h"
#include "moveit_household_objects_database/database_grasp.h"
#include "moveit_household_objects_database/database_mesh.h"
#include "moveit_household_objects_database/database_perturbation.h"
#include "moveit_household_objects_database/database_scan.h"
#include "moveit_household_objects_database/database_view.h"
#include "moveit_household_objects_database/database_vfh.h"
#include "moveit_household_objects_database/database_vfh_orientation.h"
#include "moveit_household_objects_database/database_file_path.h"
#include "moveit_household_objects_database/database_task.h"
#include "moveit_household_objects_database/database_capture_region.h"
#include "moveit_household_objects_database/database_object_paths.h"

namespace moveit_household_objects_database
{

  class DatabaseTask;

  //! A slight specialization of the general database interface with a few convenience functions added
  class ObjectsDatabase : public database_interface::PostgresqlDatabase
  {
  public:
    //! Attempts to connect to the specified database
    ObjectsDatabase (std::string host, std::string port, std::string user, std::string password, std::string dbname) :
      PostgresqlDatabase (host, port, user, password, dbname)
    {
    }

    //! Attempts to connect to the specified database
    ObjectsDatabase (const database_interface::PostgresqlDatabaseConfig &config) :
      PostgresqlDatabase (config)
    {
    }

    //! Just a stub for now
    ~ObjectsDatabase ()
    {
    }

    //! Acquires the next experiment to be executed from the list of tasks in the database
    /*! Also marks it as RUNNING in an atomic fashion, so that it is not acquired by
     another process.*/
    virtual bool
    acquireNextTask (std::vector<boost::shared_ptr<DatabaseTask> > &task);

    //------- helper functions wrapped around the general versions for convenience -------
    //----------------- or for cases where where_clauses are needed ----------------------

    //! Gets a list of all the original models in the database
    bool
    getOriginalModelsList (std::vector<boost::shared_ptr<DatabaseOriginalModel> > &models) const
    {
      DatabaseOriginalModel example;
      return getList<DatabaseOriginalModel> (models, example, "");
    }

    //! Gets a list of all the scaled models in the database
    bool
    getScaledModelsList (std::vector<boost::shared_ptr<DatabaseScaledModel> > &models) const
    {
      DatabaseScaledModel example;
      return getList<DatabaseScaledModel> (models, example, "");
    }

    //! Gets a list of scaled models based on acquisition method
    bool
    getScaledModelsByAcquisition (std::vector<boost::shared_ptr<DatabaseScaledModel> > &models,
                                  std::string acquisition_method) const
    {
      std::string where_clause ("acquisition_method_name='" + acquisition_method + "'");
      DatabaseScaledModel example;
      //this should be set by default, but let's make sure again
      example.acquisition_method_.setReadFromDatabase (true);
      return getList<DatabaseScaledModel> (models, example, where_clause);
    }

    virtual bool
    getScaledModelsBySet (std::vector<boost::shared_ptr<DatabaseScaledModel> > &models, 
                          std::string model_set_name) const
    {
      if (model_set_name.empty ())
        return getScaledModelsList (models);
      std::string where_clause = std::string ("original_model_id IN (SELECT original_model_id FROM "
        "model_set WHERE model_set_name = '") + model_set_name + std::string ("')");
      DatabaseScaledModel example;
      return getList<DatabaseScaledModel> (models, example, where_clause);
    }

    //! Returns the number of original models in the database
    bool
    getNumOriginalModels (int &num_models) const
    {
      DatabaseOriginalModel example;
      return countList (&example, num_models, "");
    }

    bool
    getScaledModelsByRecognitionId(std::vector<boost::shared_ptr<DatabaseScaledModel> > &models, 
                                   std::string recognition_id) const
    {
      std::string where_clause = 
        std::string("original_model_id IN (SELECT original_model_id FROM "
                    "original_model WHERE original_model_recognition_id = '")
        + recognition_id + std::string("')");
      DatabaseScaledModel example;
      return getList<DatabaseScaledModel> (models, example, where_clause);
    }

    //! Returns the path that geometry paths are relative to
    bool
    getModelRoot (std::string& root) const
    {
      return getVariable ("'MODEL_ROOT'", root);
    }

    //! Gets a list of all models with the requested tags in the database
    bool
    getModelsListByTags (std::vector<boost::shared_ptr<DatabaseOriginalModel> > &models, 
                         std::vector<std::string> tags) const
    {
      DatabaseOriginalModel example;
      std::string where_clause ("(");
      for (size_t i = 0; i < tags.size (); i++)
      {
        if (i != 0)
          where_clause += "AND ";
        where_clause += "'" + tags[i] + "' = ANY (original_model_tags)";
      }
      where_clause += ")";
      return getList<DatabaseOriginalModel> (models, example, where_clause);
    }

    //! Gets the list of all the grasps for a scaled model id
    bool
    getGrasps (int scaled_model_id, std::string hand_name, std::vector<boost::shared_ptr<DatabaseGrasp> > &grasps) const
    {
      DatabaseGrasp example;
      std::stringstream id;
      id << scaled_model_id;
      std::string where_clause ("scaled_model_id=" + id.str () + " AND hand_name='" + hand_name + "'");
      //+ " AND grasp_energy >= 0 AND grasp_energy < 10" );
      return getList<DatabaseGrasp> (grasps, example, where_clause);
    }

    //! Gets the list of only those grasps that are cluster reps a database model
    bool
    getClusterRepGrasps (int scaled_model_id, std::string hand_name,
                         std::vector<boost::shared_ptr<DatabaseGrasp> > &grasps) const
    {
      DatabaseGrasp example;
      std::stringstream id;
      id << scaled_model_id;
      std::string where_clause ("scaled_model_id=" + id.str () + " AND hand_name='" + hand_name + "'"
          + " AND grasp_cluster_rep=true AND grasp_energy >= 0 AND grasp_energy < 10");
      return getList<DatabaseGrasp> (grasps, example, where_clause);
    }

    //! Gets  the mesh for a scaled model
    bool
    getScaledModelMesh (int scaled_model_id, DatabaseMesh &mesh) const
    {
      //first get the original model id
      DatabaseScaledModel scaled_model;
      scaled_model.id_.data () = scaled_model_id;
      if (!loadFromDatabase (&scaled_model.original_model_id_))
      {
        ROS_ERROR ("Failed to get original model for scaled model id %d", scaled_model_id);
        return false;
      }
      mesh.id_.data () = scaled_model.original_model_id_.data ();
      if (!loadFromDatabase (&mesh.triangles_) || !loadFromDatabase (&mesh.vertices_))
      {
        ROS_ERROR ("Failed to load mesh from database for scaled model %d, resolved to original model %d",
                   scaled_model_id, mesh.id_.data ());
        return false;
      }
      return true;
    }

    //! Gets the mesh for a scaled model as a shape_msgs::Shape
    bool
    getScaledModelMesh (int scaled_model_id, shape_msgs::Mesh &shape) const
    {
      DatabaseMesh mesh;
      if (!getScaledModelMesh (scaled_model_id, mesh))
        return false; 
      for (size_t i = 0; i < mesh.triangles_.data ().size () / 3; i++)
      {
        shape_msgs::MeshTriangle t;
        t.vertex_indices[0] = mesh.triangles_.data ().at (3 * i + 0);
        t.vertex_indices[1] = mesh.triangles_.data ().at (3 * i + 1);
        t.vertex_indices[2] = mesh.triangles_.data ().at (3 * i + 2);
        shape.triangles.push_back(t);
      }
      shape.vertices.clear ();
      if (mesh.vertices_.data ().size () % 3 != 0)
      {
        ROS_ERROR ("Get scaled model mesh: size of vertices vector is not a multiple of 3");
        return false;
      }
      for (size_t i = 0; i < mesh.vertices_.data ().size () / 3; i++)
      {
        geometry_msgs::Point p;
        p.x = mesh.vertices_.data ().at (3 * i + 0);
        p.y = mesh.vertices_.data ().at (3 * i + 1);
        p.z = mesh.vertices_.data ().at (3 * i + 2);
        shape.vertices.push_back (p);
      }
      return true;
    }

    // bool
    // getModelScans (int scaled_model_id, std::string source,
    //                std::vector<moveit_household_objects_database_msgs::DatabaseScan> &matching_scan_list) const
    // {
    //   DatabaseScan scan;
    //   std::vector<boost::shared_ptr<DatabaseScan> > matching_scans;
    //   std::string where_clause ("scaled_model_id = " + boost::lexical_cast<std::string> (scaled_model_id) + " AND "
    //       + "scan_source = '" + source + "'");
    //   getList<DatabaseScan> (matching_scans, where_clause);

    //   BOOST_FOREACH(const boost::shared_ptr<DatabaseScan> &scan, matching_scans)
    //         {
    //           moveit_household_objects_database_msgs::DatabaseScan out_scan;
    //           out_scan.model_id = scan->scaled_model_id_.get ();
    //           out_scan.scan_source = scan->scan_source_.get ();
    //           out_scan.bagfile_location = scan->scan_bagfile_location_.get ();

    //           out_scan.pose.pose = scan->object_pose_.get ().pose_;
    //           out_scan.pose.header.frame_id = scan->frame_id_.get ();
    //           out_scan.cloud_topic = scan->cloud_topic_.get ();
    //           matching_scan_list.push_back (out_scan);
    //         }

    //   return true;
    // }

    /*!
     * These two functions use the ANY(ARRAY[ids]) syntax because those were the most performant in speed tests.
     */

    //! Gets the perturbations for all grasps for a given scaled model
    bool
    getAllPerturbationsForModel (int scaled_model_id, std::vector<DatabasePerturbationPtr> &perturbations)
    {
      std::string where_clause = std::string ("grasp_id = ANY(ARRAY(SELECT grasp_id FROM grasp WHERE "
        "scaled_model_id = " + boost::lexical_cast<std::string> (scaled_model_id) + std::string ("))"));
      DatabasePerturbation example;
      return getList<DatabasePerturbation> (perturbations, example, where_clause);
    }

    bool
    getPerturbationsForGrasps (const std::vector<int> &grasp_ids, std::vector<DatabasePerturbationPtr> &perturbations)
    {
      std::vector<std::string> grasp_id_strs;
      grasp_id_strs.reserve (grasp_ids.size ());
      BOOST_FOREACH(int id, grasp_ids)
            {
              grasp_id_strs.push_back (boost::lexical_cast<std::string, int> (id));
            }
      std::string where_clause = std::string ("grasp_id = ANY(ARRAY[" + boost::algorithm::join (grasp_id_strs, ", ")
          + "])");
      DatabasePerturbation example;
      return getList<DatabasePerturbation> (perturbations, example, where_clause);
    }

    bool
    getVFHDescriptors (std::vector<boost::shared_ptr<DatabaseVFH> > &vfh)
    {
      DatabaseVFH example;
      if (!getList<DatabaseVFH> (vfh, example, ""))
        return false;
      for (size_t i = 0; i < vfh.size (); i++)
      {
        if (!loadFromDatabase (&(vfh[i]->vfh_descriptor_)))
        {
          ROS_ERROR ("Failed to load descriptor data for vfh id %d", vfh[i]->vfh_id_.data ());
        }
      }
    }

    bool
   getViewFromViewIdNoData (int view_id, boost::shared_ptr<DatabaseView> &view)
   {
     std::vector<boost::shared_ptr<DatabaseView> > views;
     std::stringstream where2;
     where2 << "view_id =" << view_id;
     std::string where_clause2 (where2.str ());
     getList (views, where_clause2);
     view = views[0];
     return true;
   }

    bool
    getViewFromVFHId (int vfh_id, boost::shared_ptr<DatabaseView> &view)
    {
      std::vector<boost::shared_ptr<DatabaseVFH> > vfhs;
      std::stringstream where;
      where << "vfh_id =" << vfh_id;
      std::string where_clause (where.str ());
      if (!getList (vfhs, where_clause)) {
        return false;
      }

      std::vector<boost::shared_ptr<DatabaseView> > views;
      std::stringstream where2;
      where2 << "view_id =" << vfhs[0]->view_id_.data ();
      std::string where_clause2 (where2.str ());
      getList (views, where_clause2);
      if (!loadFromDatabase (&views[0]->view_point_cloud_data_)) {
        ROS_ERROR ("Failed to load view point cloud data for view id %d", vfhs[0]->view_id_.data ());
      }
      view = views[0];
      return true;
    }

    bool
    getViewFromVFHIdNoData (int vfh_id, boost::shared_ptr<DatabaseView> &view)
    {
      std::vector<boost::shared_ptr<DatabaseVFH> > vfhs;
      std::stringstream where;
      where << "vfh_id =" << vfh_id;
      std::string where_clause (where.str ());
      if (!getList (vfhs, where_clause)) {
        return false;
      }

      std::vector<boost::shared_ptr<DatabaseView> > views;
      std::stringstream where2;
      where2 << "view_id =" << vfhs[0]->view_id_.data ();
      std::string where_clause2 (where2.str ());
      getList (views, where_clause2);
      view = views[0];
      return true;
    }

    bool
    getVFHFromView (boost::shared_ptr<DatabaseVFH> & vfh, 
                    boost::shared_ptr<moveit_household_objects_database::DatabaseView> &view)
    {
      std::vector<boost::shared_ptr<DatabaseVFH> > vfhs;
      std::stringstream where;
      where << "view_id =" << view->view_id_.data();
      std::string where_clause (where.str ());
      if (!getList (vfhs, where_clause)) {
        return false;
      }

      vfh = vfhs[0];
      if (!loadFromDatabase (&vfh->vfh_descriptor_)) {
        ROS_ERROR ("Failed to load VFH descriptor for %d", vfh->vfh_id_.data ());
      }
      return true;
    }

    bool
    getOrientationRollFromVFHId (int vfh_id, boost::shared_ptr<DatabaseVFHOrientation> &roll_histogram)
    {
      std::vector<boost::shared_ptr<DatabaseVFH> > vfhs;
      std::stringstream where;
      where << "vfh_id =" << vfh_id;
      std::string where_clause (where.str ());
      if (!getList (vfhs, where_clause)) {
        return false;
      }

      /*std::vector<boost::shared_ptr<DatabaseView> > views;
      std::stringstream where2;
      where2 << "view_id =" << vfhs[0]->view_id_.data ();
      std::string where_clause2 (where2.str ());
      getList (views, where_clause2);*/

      //Once we have the view, load vfh orientation
      std::vector<boost::shared_ptr<DatabaseVFHOrientation> > rolls;
      std::stringstream where3;
      where3 << "vfh_id =" << vfhs[0]->vfh_id_.data ();
      std::string where_clause3 (where3.str ());
      if (!getList (rolls, where_clause3)) {
        return false;
      }

      //std::cout << "rolls size:" << rolls.size() << std::endl;
      if (rolls.size() == 0) {
        return false;
      }

      if (!loadFromDatabase (&rolls[0]->vfh_orientation_descriptor_)) {
        ROS_ERROR ("Failed to load VFH roll orientation histogram => id %d", rolls[0]->vfh_orientation_id_.data ());
      }
      roll_histogram = rolls[0];
      return true;
    }

    bool
    getOrientationRollFromVFHThroughViewId (int vfh_id, boost::shared_ptr<DatabaseVFHOrientation> &roll_histogram)
    {
      std::vector<boost::shared_ptr<DatabaseVFH> > vfhs;
      std::stringstream where;
      where << "vfh_id =" << vfh_id;
      std::string where_clause (where.str ());
      if (!getList (vfhs, where_clause)) {
        return false;
      }

      std::vector<boost::shared_ptr<DatabaseView> > views;
      std::stringstream where2;
      where2 << "view_id =" << vfhs[0]->view_id_.data ();
      std::string where_clause2 (where2.str ());
      getList (views, where_clause2);

      //Once we have the view, load vfh orientation
      std::vector<boost::shared_ptr<DatabaseVFHOrientation> > rolls;
      std::stringstream where3;
      where3 << "view_id =" << views[0]->view_id_.data ();
      std::string where_clause3 (where3.str ());
      getList (rolls, where_clause3);

      if (!loadFromDatabase (&rolls[0]->vfh_orientation_descriptor_)) {
        ROS_ERROR ("Failed to load VFH roll orientation histogram => id %d", rolls[0]->vfh_orientation_id_.data ());
      }
      roll_histogram = rolls[0];
      return true;
    }

    bool
    getModelScaleFromId (int scaled_model_id, boost::shared_ptr<DatabaseScaledModel> &scale_model)
    {
      std::vector<boost::shared_ptr<DatabaseScaledModel> > scaled_models;
      std::stringstream where;
      where << "scaled_model_id =" << scaled_model_id;
      std::string where_clause (where.str ());
      if (!getList (scaled_models, where_clause))
        return false;

      scale_model = scaled_models[0];
      return true;
    }

    bool
    getModelPathFromViewId (boost::shared_ptr<DatabaseView> &view, std::string &path)
    {

      boost::shared_ptr<DatabaseScaledModel> scaled_model;
      getModelScaleFromId (view->scaled_model_id_.data (), scaled_model);
      //get path...
      std::vector<boost::shared_ptr<DatabaseFilePath> > file_paths;
      std::stringstream where3;
      where3 << "original_model_id =" << scaled_model->original_model_id_.data ();
      std::string where_clause3 (where3.str ());
      if (!getList (file_paths, where_clause3))
        return false;

      path = file_paths[0]->file_path_.data ();
      return true;
    }

    bool
    getModelScale (boost::shared_ptr<DatabaseView> &view, double * scale_factor)
    {
      boost::shared_ptr<DatabaseScaledModel> scaled_model;
      getModelScaleFromId (view->scaled_model_id_.data (), scaled_model);
      *scale_factor = scaled_model->scale_.data ();
      return true;
    }

    bool
    getViewsFromScaledModelId (int scaled_model_id, int iteration, std::vector<boost::shared_ptr<DatabaseView> > & views)
    {
      std::stringstream where;
      where << "scaled_model_id =" << scaled_model_id << " AND iteration =" << iteration;
      std::string where_clause (where.str ());
      if (!getList (views, where_clause)) {
        return false;
      }

      return true;
    }

    //! Gets the list of all capture regions for an object and hand pair.
    bool
    getCaptureRegions (int scaled_model_id, std::string robot_geometry_hash, std::vector<boost::shared_ptr<DatabaseCaptureRegion> > &capture_regions) const
    {
      DatabaseCaptureRegion example;
      std::stringstream id;
      id << scaled_model_id;
      std::string where_clause ("scaled_model_id=" + id.str () + " AND robot_geometry_hash='" + robot_geometry_hash + "'" );
      return getList<DatabaseCaptureRegion> (capture_regions, example, where_clause);
    }

    //! Gets the list of all object paths for an object and hand pair.
    bool
    getObjectPaths (int scaled_model_id, std::string robot_geometry_hash, std::vector<boost::shared_ptr<DatabaseObjectPaths> > &object_paths) const
    {
      DatabaseObjectPaths example;
      std::stringstream id;
      id << scaled_model_id;
      std::string where_clause ("object_db_id=" + id.str () + " AND robot_geometry_hash='" + robot_geometry_hash + "'" );
      return getList<DatabaseObjectPaths> (object_paths, example, where_clause);
    }


  };
  typedef boost::shared_ptr<ObjectsDatabase> ObjectsDatabasePtr;
}//namespace

#endif
