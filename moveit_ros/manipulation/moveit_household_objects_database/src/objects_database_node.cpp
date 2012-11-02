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

//! Wraps around the most common functionality of the objects database and offers it
//! as ROS services

#include <vector>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>

#include <moveit_manipulation_msgs/GetModelList.h>
#include <moveit_manipulation_msgs/GetModelMesh.h>

#include <moveit_household_objects_database/objects_database.h>

const std::string GET_MODELS_SERVICE_NAME = "get_model_list";
const std::string GET_MESH_SERVICE_NAME = "get_model_mesh";
const std::string GET_DESCRIPTION_SERVICE_NAME = "get_model_description";
const std::string GRASP_PLANNING_SERVICE_NAME = "database_grasp_planning";
const std::string GET_SCANS_SERVICE_NAME = "get_model_scans";
const std::string SAVE_SCAN_SERVICE_NAME = "save_model_scan";
const std::string TRANSLATE_ID_SERVICE_NAME = "translate_id";
const std::string GRASP_PLANNING_ACTION_NAME = "database_grasp_planning";

using namespace moveit_manipulation_msgs;
using namespace moveit_household_objects_database;

//! Wraps around database connection to provide database-related services through ROS
/*! Contains very thin wrappers for getting a list of scaled models and for getting the mesh
  of a model, as well as a complete server for the grasp planning service */
class ObjectsDatabaseNode
{
private:
  //! Node handle in the private namespace
  ros::NodeHandle priv_nh_;

  //! Node handle in the root namespace
  ros::NodeHandle root_nh_;

  //! Server for the get models service
  ros::ServiceServer get_models_srv_;

  //! Server for the get mesh service
  ros::ServiceServer get_mesh_srv_;

  //! The database connection itself
  ObjectsDatabase *database_;

  //! Callback for the get models service
  bool getModelsCB(GetModelList::Request &request, GetModelList::Response &response)
  {
    if (!database_)
    {
      response.return_code.code = response.return_code.DATABASE_NOT_CONNECTED;
      return true;
    }
    std::vector< boost::shared_ptr<DatabaseScaledModel> > models;
    if (!database_->getScaledModelsBySet(models, request.model_set))
    {
      response.return_code.code = response.return_code.DATABASE_QUERY_ERROR;
      return true;
    }
    for (size_t i=0; i<models.size(); i++)
    {
      response.model_ids.push_back( models[i]->id_.data() );
    }
    response.return_code.code = response.return_code.SUCCESS;
    return true;
  }

  //! Callback for the get mesh service
  bool getMeshCB(GetModelMesh::Request &request, GetModelMesh::Response &response)
  {
    if (!database_)
    {
      response.return_code.code = response.return_code.DATABASE_NOT_CONNECTED;
      return true;
    }
    if ( !database_->getScaledModelMesh(request.model_id, response.mesh) )
    {
      response.return_code.code = response.return_code.DATABASE_QUERY_ERROR;
      return true;
    }
    response.return_code.code = response.return_code.SUCCESS;
    return true;
  }

public:
  ObjectsDatabaseNode() : priv_nh_("~"), root_nh_("")
  {
    //initialize database connection
    std::string database_host, database_port, database_user, database_pass, database_name;
    root_nh_.param<std::string>("/household_objects_database/database_host", database_host, "");
    int port_int;
    root_nh_.param<int>("/household_objects_database/database_port", port_int, -1);
    std::stringstream ss; ss << port_int; database_port = ss.str();
    root_nh_.param<std::string>("/household_objects_database/database_user", database_user, "");
    root_nh_.param<std::string>("/household_objects_database/database_pass", database_pass, "");
    root_nh_.param<std::string>("/household_objects_database/database_name", database_name, "");
    database_ = new ObjectsDatabase(database_host, database_port, database_user, database_pass, database_name);
    if (!database_->isConnected())
    {
      ROS_ERROR("ObjectsDatabaseNode: failed to open model database on host "
		"%s, port %s, user %s with password %s, database %s. Unable to do grasp "
		"planning on database recognized objects. Exiting.",
		database_host.c_str(), database_port.c_str(), 
		database_user.c_str(), database_pass.c_str(), database_name.c_str());
      delete database_; database_ = NULL;
    }

    //advertise services
    get_models_srv_ = priv_nh_.advertiseService(GET_MODELS_SERVICE_NAME, &ObjectsDatabaseNode::getModelsCB, this);    
    get_mesh_srv_ = priv_nh_.advertiseService(GET_MESH_SERVICE_NAME, &ObjectsDatabaseNode::getMeshCB, this);    
  }


  ~ObjectsDatabaseNode()
  {
    delete database_;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "objects_database_node");
  ObjectsDatabaseNode node;
  ros::spin();
  return 0;  
}
