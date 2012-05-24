/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

/* Author: Ioan Sucan */

#include "moveit_configuration_tools/compute_default_collision_matrix.h"
#include "moveit_configuration_tools/benchmark_timer.h"
#include <set>

extern BenchmarkTimer BTimer;

// *********************************************************************
// Main call for computing default collision matrix
// *********************************************************************
std::map<std::string, std::vector<std::string> > 
moveit_configuration_tools::computeDefaultCollisionMatrix(
                                                          const planning_scene::PlanningSceneConstPtr &parent_scene, 
                                                          bool include_never_colliding)
{
  // Trial count variables
  static const unsigned int small_trial_count = 1000;
  static const unsigned int small_trial_limit = (unsigned int)((double)small_trial_count * 0.95);
  static const unsigned int small_trial_connected_limit = (unsigned int)((double)small_trial_count * 0.75);
  
  // Create new instance of planning scene using pointer
  planning_scene::PlanningScene scene(parent_scene);

  // Create collision detection object
  collision_detection::CollisionRequest req;
  req.contacts = true;
  req.max_contacts = 2;
  req.max_contacts_per_pair = 1;
  req.verbose = false;

  // Create collision matrix
  collision_detection::AllowedCollisionMatrix &acm = scene.getAllowedCollisionMatrix();

  // Create result vars
  std::map<std::string, std::vector<std::string> > result;
  std::set<std::pair<std::string, std::string> > seen_colliding;
  
  // For each link, compute the set of other links it connects to via a single joint (adjacent links) 
  // or via a chain of joints and links with no geometry

  LinkGraph lg; // LinkGraph is a custom type defined at top

  // Create Connection Graph
  BTimer.start("GenConnG"); // Benchmarking Timer - temporary
  moveit_configuration_tools::computeConnectionGraph(scene.getKinematicModel()->getRootLink(), lg);
  BTimer.end("GenConnG"); // Benchmarking Timer - temporary

  // UPDATE NUMBER OF CONTACTS TO CONSIDER ----------------------------------------------------------
  ROS_INFO("Computing the number of contacts we should consider...");

  bool done = false; 

  BTimer.start("ContactConsider"); // Benchmarking Timer - temporary  
  // update the number of contacts we should consider in collision detection:
  while (!done)
  {
    done = true;

    // Check for SELF collisions
    collision_detection::CollisionResult res;
    scene.getCurrentState().setToRandomValues();
    scene.checkSelfCollision(req, res);

    // Sum the number of collisions
    unsigned int nc = 0;
    for (collision_detection::CollisionResult::ContactMap::const_iterator it = res.contacts.begin() ; it != res.contacts.end() ; ++it)
    {
      nc += it->second.size();
    }
    
    // Check if the number of contacts is greater than the max count
    if (nc >= req.max_contacts)
    {
      req.max_contacts *= 2; // double the max contacts
      done = false;
    }
  }
  BTimer.end("ContactConsider"); // Benchmarking Timer - temporary  
  
  // ALWAYS IN COLLISION --------------------------------------------------------------------
  // compute the links that are always in collision
  ROS_INFO("Computing pairs of links that are always in collision...");
  done = false;

  BTimer.start("AlwaysColl"); // Benchmarking Timer - temporary  
  while (!done)
  {
    std::map<std::pair<std::string, std::string>, unsigned int> count;    

    for (unsigned int i = 0 ; i < small_trial_count ; ++i)
    {
      // Check for collisions
      collision_detection::CollisionResult res;
      scene.getCurrentState().setToRandomValues();
      scene.checkSelfCollision(req, res);

      // Sum the number of collisions
      unsigned int nc = 0;
      for (collision_detection::CollisionResult::ContactMap::const_iterator it = res.contacts.begin() ; it != res.contacts.end() ; ++it)
      {
        count[it->first]++;
        seen_colliding.insert(it->first);
        nc += it->second.size();
      }
      
      // Check if the number of contacts is greater than the max count
      if (nc >= req.max_contacts)
        req.max_contacts *= 2; // double the max contacts
    }

    // ADD QUALIFYING LINKS TO COLLISION MATRIX -----------------------------------------------
    // if a pair of links is almost always in collision (small_trial_limit) or they are adjacent 
    // and very often in collision (small_trial_connected_limit)
    // add those links to the collision matrix

    unsigned int found = 0;
    
    // Loop through every pair of collisions?
    for (std::map<std::pair<std::string, std::string>, unsigned int>::const_iterator it = count.begin() ; it != count.end() ; ++it)
    {      
      if (it->second > small_trial_connected_limit)
      {
        bool ok = it->second > small_trial_limit;
        if (!ok)
        {
          // check if links are connected by a joint or a chain of joints & links with no geometry
          const std::set<const planning_models::KinematicModel::LinkModel*> &nbh = lg.at(scene.getKinematicModel()->getLinkModel(it->first.first));
          ok = nbh.find(scene.getKinematicModel()->getLinkModel(it->first.second)) != nbh.end();
        }
        if (ok)
        {
          if (it->first.first < it->first.second)
            result[it->first.first].push_back(it->first.second);
          else
            result[it->first.second].push_back(it->first.first);
          acm.setEntry(it->first.first, it->first.second, true);
          found++;
        }
      }
    }
    
    // if no updates were made to the collision matrix, we stop
    if (found == 0)
      done = true;
    else
      ROS_INFO("Found %u allowed collisions", found);
  }
  BTimer.end("AlwaysColl"); // Benchmarking Timer - temporary  
  


  // NEVER IN COLLISION -------------------------------------------------------------------
  BTimer.start("NeverColl"); // Benchmarking Timer - temporary  
  if (include_never_colliding)
  {
    // get the pairs of links that are never in collision
    for (int k = 0 ; k < 5 ; ++k)
    {
      bool update = true;
      while (update)
      {
        update = false;
        ROS_INFO("Still seeing updates on possibly colliding links ...");
        for (unsigned int i = 0 ; i < small_trial_count ; ++i)
        {
          collision_detection::CollisionResult res;
          scene.getCurrentState().setToRandomValues();
          scene.checkSelfCollision(req, res);
          for (collision_detection::CollisionResult::ContactMap::const_iterator it = res.contacts.begin() ; it != res.contacts.end() ; ++it)
          {
            if (seen_colliding.insert(it->first).second)
              update = true;
          }
        }
      }   
    }
    
    const std::vector<std::string> &names = scene.getKinematicModel()->getLinkModelNamesWithCollisionGeometry();
    for (std::size_t i = 0 ; i < names.size() ; ++i)
    {
      for (std::size_t j = i + 1 ; j < names.size() ; ++j)
        if (seen_colliding.find(std::make_pair(names[i], names[j])) == seen_colliding.end())
        {
          if (names[i] < names[j])
            result[names[i]].push_back(names[j]);
          else
            result[names[j]].push_back(names[i]);
        }
    }
  }
  BTimer.end("NeverColl"); // Benchmarking Timer - temporary  
  
  return result;
}

// *********************************************************************
// Recursively build the edge list of graph connections
// *********************************************************************
void moveit_configuration_tools::computeConnectionGraphRec(const planning_models::KinematicModel::LinkModel *link, moveit_configuration_tools::LinkGraph &edges)
{
  if (link)
  {
    for (std::size_t i = 0 ; i < link->getChildJointModels().size() ; ++i)
    {
      const planning_models::KinematicModel::LinkModel *next = link->getChildJointModels()[i]->getChildLinkModel();
      
      edges[next].insert(link);
      edges[link].insert(next);
      moveit_configuration_tools::computeConnectionGraphRec(next, edges);      
    }
  }
  else
  {
    ROS_ERROR("Joint exists in URDF with no link!");
  }  
}

// *********************************************************************
// Build the connection graph and then check 
// *********************************************************************
void moveit_configuration_tools::computeConnectionGraph(const planning_models::KinematicModel::LinkModel *link, moveit_configuration_tools::LinkGraph &edges)
{
  edges.clear();
  moveit_configuration_tools::computeConnectionGraphRec(link, edges);
  bool update = true;
  while (update)
  {
    update = false;
    // Check if each edge has a shape
    for (moveit_configuration_tools::LinkGraph::const_iterator it = edges.begin() ; it != edges.end() ; ++it)
      if (!it->first->getShape())
      {        
        // Create new link
        std::vector<const planning_models::KinematicModel::LinkModel*> v;

        // Take second link in edge and add to new link
        for (std::set<const planning_models::KinematicModel::LinkModel*>::const_iterator jt = it->second.begin() ; jt != it->second.end() ; ++jt)
          v.push_back(*jt);

        // Check if we need to keep looping?
        for (std::size_t i = 0 ; i < v.size() ; ++i)
          for (std::size_t j = i + 1 ; j < v.size() ; ++j)
          {
            if (edges[v[i]].insert(v[j]).second)
              update = true;
            if (edges[v[j]].insert(v[i]).second)
              update = true;
          }
      }
  }
}


