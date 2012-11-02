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

#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include <boost/filesystem.hpp>

#include "moveit_household_objects_database/objects_database.h"
#include "moveit_household_objects_database/database_original_model.h"
#include "moveit_household_objects_database/database_scaled_model.h"
#include "moveit_household_objects_database/database_file_path.h"

#include "mesh_loader.h"

void usage()
{
  std::cerr << "Usage: insert_model geometry_file thumbnail_file\n";
}

std::string getNonEmptyString(const std::string &display_name)
{
  std::string ret_string;
  while(1)
  {
    std::cout << "Enter " << display_name << ": ";
    std::getline(std::cin, ret_string);
    if (!ret_string.empty()) break;
    std::cout << "Non-empty string required \n";
  }
  return ret_string;
}

std::string getString(const std::string &display_name)
{
  std::string ret_string;
  std::cout << "Enter " << display_name << " (leave empty for none):";
  std::getline(std::cin, ret_string);
  return ret_string;
}

bool copyFile(std::string old_path, std::string new_root, std::string &filename)
{
  //get just the filename part of the original file
  size_t slash_position = old_path.find_last_of('/');
  if (slash_position == std::string::npos) 
  {
    //no slash
    filename = old_path;
  } 
  else if (slash_position >= old_path.size() - 1)
  {
    //slash is the last character
    std::cerr << "Failed to parse input filename: " << old_path << "\n";
    return false;
  }
  else
  {
    //skip the last slash
    filename = old_path.substr(slash_position+1, std::string::npos);
  }
  std::string new_path(new_root);
  if (new_path.at(new_path.size() - 1) != '/') 
  {
    new_path.append("//");
  }
  new_path.append(filename);
  if ( boost::filesystem::exists(new_path) )
  {
    std::cerr << "File " << new_path << " already exists; skipping copy.\n";
    return true;
  }  
  boost::filesystem::copy_file(old_path, new_path);
  if ( !boost::filesystem::exists(new_path) )
  {
    std::cerr << "Failed to copy file " << filename << " to " << new_path << "\n";
    return false;
  }
  return true;
}

bool loadGeometry(moveit_household_objects_database::DatabaseMesh &mesh, std::string filename)
{
  return true;
}

void getOriginalModelInfo(moveit_household_objects_database::DatabaseOriginalModel &original_model)
{
  std::vector<std::string> tags;

  /*
  //hard-coded for als object entries
  std::string name = getNonEmptyString("name");
  original_model.source_.data() = "3dwarehouse";
  original_model.acquisition_method_.data() = "cad";
  original_model.maker_.data() = "na";
  original_model.model_.data() = name;
  original_model.description_.get() = name;
  tags.push_back(name);
  tags.push_back("als_object");
  */
  /*
  original_model.source_.data() = getNonEmptyString("object source");
  original_model.acquisition_method_.data() = getNonEmptyString("acquisition method");
  original_model.maker_.data() = getNonEmptyString("object maker");
  original_model.model_.data() = getNonEmptyString("object model");
  original_model.barcode_.get() = getString("object barcode");
  original_model.description_.get() = getString("object description");
  std::cout << "Enter tags one at a time; enter an empty line when done\n";
  while(1)
  {
    std::string tag;
    std::getline(std::cin,tag);
    if (!tag.empty()) tags.push_back(tag);
    else break;
  }
  */

  //hard-coded for TOD objects
  original_model.source_.data() = "Household";
  original_model.acquisition_method_.data() = "TOD";
  original_model.maker_.data() = getNonEmptyString("object maker");
  original_model.model_.data() = getNonEmptyString("object model");
  original_model.barcode_.get() = getString("object barcode");
  original_model.description_.get() = getString("object description");
  std::cout << "Enter tags one at a time; enter an empty line when done\n";
  while(1)
  {
    std::string tag;
    std::getline(std::cin,tag);
    if (!tag.empty()) tags.push_back(tag);
    else break;
  }

  if (!tags.empty())
  {
    original_model.tags_.data() = tags;
    original_model.tags_.setWriteToDatabase(true);
  }
  if (!original_model.barcode_.get().empty())
  {
    original_model.barcode_.setWriteToDatabase(true);
  }
  if (!original_model.description_.get().empty()) 
  {
    original_model.description_.setWriteToDatabase(true);
  }

}

int main(int argc, char **argv)
{
  //connect to database
  //hard-coded for now
  moveit_household_objects_database::ObjectsDatabase database("wgs36", "5432", "willow", "willow", "household_objects");
  if (!database.isConnected())
  {
    std::cerr << "Database failed to connect";
    return -1;
  }

  //parse the input
  //first argument is always the geometry filename which must exist
  if (argc < 3) 
  {
    usage();
    return -1;
  }

  //check that the geometry file exists
  std::string geometry_filename(argv[1]);
  if ( !boost::filesystem::exists(geometry_filename) )
  {
    std::cerr << "Geometry file " << geometry_filename << " not found\n";
    return -1;
  }

  //check that thumbnail file exists 
  std::string thumbnail_filename(argv[2]);
  if ( !boost::filesystem::exists(thumbnail_filename) )
  {
    std::cout << "Thumbnail file " << thumbnail_filename << " not found\n";
    return -1;
  }

  //read in the geometry of the model from the ply file
  moveit_household_objects_database::DatabaseMesh mesh;
  moveit_household_objects_database::PLYModelLoader loader;
  if (loader.readFromFile(geometry_filename, mesh.vertices_.data(), mesh.triangles_.data()) < 0)
  {
    std::cerr << "Failed to read geometry from file\n";
    return -1;
  }
  if (mesh.vertices_.data().empty() || mesh.triangles_.data().empty())
  {
    std::cerr << "No geometry read from file\n";
    return -1;
  }
  
  //the original model we will insert
  moveit_household_objects_database::DatabaseOriginalModel original_model;
  //ask use for info to populate fields 
  getOriginalModelInfo(original_model);

  //copy the file(s) over to the model root
  std::string model_root;
  if (!database.getModelRoot(model_root) || model_root.empty())
  {
    std::cerr << "Failed to get model root from database\n";
    return -1;
  }
  std::string geometry_relative_filename;
  if (!copyFile(geometry_filename, model_root, geometry_relative_filename)) return -1;
  std::string thumbnail_relative_filename;
  if (!copyFile(thumbnail_filename, model_root, thumbnail_relative_filename)) return -1;

  //insert the original model into the database
  if (!database.insertIntoDatabase(&original_model))
  {
    std::cerr << "Failed to insert original model in database\n";
    return -1;
  }
  int original_model_id = original_model.id_.data();

  //insert the geometry
  mesh.id_.data() = original_model_id;
  if (!database.insertIntoDatabase(&mesh))
  {
    std::cerr << "Failed to insert mesh in database\n";
    return -1;
  }
  if (!database.saveToDatabase(&mesh.triangles_) || !database.saveToDatabase(&mesh.vertices_))
  {
    std::cerr << "Failed to write mesh data to database\n";
    return -1;
  }

  // insert a scaled model at range 1.0
  moveit_household_objects_database::DatabaseScaledModel scaled_model;
  scaled_model.original_model_id_.data() = original_model_id;
  scaled_model.scale_.data() = 1.0;
  if (!database.insertIntoDatabase(&scaled_model))
  {
    std::cerr << "Failed to insert scaled model in database\n";
    return -1;
  }
  
  //insert the paths to the files in the database
  moveit_household_objects_database::DatabaseFilePath geometry_file_path;
  geometry_file_path.original_model_id_.data() = original_model_id;
  geometry_file_path.file_type_.data() = "GEOMETRY_BINARY_PLY";
  geometry_file_path.file_path_.data() = geometry_relative_filename;
  if (!database.insertIntoDatabase(&geometry_file_path))
  {
    std::cerr << "Failed to insert geometry file path in database\n";
    return -1;
  }  
  moveit_household_objects_database::DatabaseFilePath thumbnail_file_path;
  thumbnail_file_path.original_model_id_.data() = original_model_id;
  thumbnail_file_path.file_type_.data() = "THUMBNAIL_BINARY_PNG";
  thumbnail_file_path.file_path_.data() = thumbnail_relative_filename;
  if (!database.insertIntoDatabase(&thumbnail_file_path))
  {
    std::cerr << "Failed to insert thumbnail file path in database\n";
    return -1;
  }

  std::cerr << "Insertion succeeded\n";
  return 0;
}
