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

// ******************************************************************************
// Main call for computing default collision matrix
// ******************************************************************************
std::map<std::string, std::vector<std::string> >  // an adj list
moveit_configuration_tools::computeDefaultCollisionMatrix(const planning_scene::PlanningSceneConstPtr &parent_scene, 
                                                          bool include_never_colliding)
{
  // Trial count variables
  static const unsigned int small_trial_count = 1000;
  static const unsigned int small_trial_limit = (unsigned int)((double)small_trial_count * 0.95);
  static const unsigned int small_trial_connected_limit = (unsigned int)((double)small_trial_count * 0.75);
  
  // Create new instance of planning scene using pointer
  planning_scene::PlanningScene scene(parent_scene);

  // Create collision detection request object
  collision_detection::CollisionRequest req;
  req.contacts = true;
  req.max_contacts = 2; // max number of contacts to compute. 2 is just a guess? TODO: change to number of links^2?
  req.max_contacts_per_pair = 1;
  req.verbose = false;

  // Create structure for tracking which collisions are allowed
  // 
  // AllowedCollisionMatrix: Definition of a structure for the allowed collision matrix. 
  // All elements in the collision world are referred to by their names.
  // This class represents which collisions are allowed to happen and which are not. */
  collision_detection::AllowedCollisionMatrix &acm = scene.getAllowedCollisionMatrix();

  // Create 'resultAdjList' to that contains a link as a key and an ordered list of links that are connected. An edgelist
  std::map<std::string, std::vector<std::string> > resultAdjList;

  // Track unique edges that have been found to be in collision in some state
  std::set<std::pair<std::string, std::string> > links_seen_colliding;
  

  // FIND CONNECTING LINKS ------------------------------------------------------------------------
  // For each link, compute the set of other links it connects to via a single joint (adjacent links) 
  // or via a chain of joints with intermediate links with no geometry (like a socket joint)

  LinkGraph lg; // LinkGraph is a custom type of a map with a LinkModel as key and a set of LinkModels as second

  // Create Connection Graph
  BTimer.start("GenConnG"); // Benchmarking Timer - temporary
  moveit_configuration_tools::computeConnectionGraph(scene.getKinematicModel()->getRootLink(), lg);
  BTimer.end("GenConnG"); // Benchmarking Timer - temporary

  ROS_INFO("Generated connection graph with %d links", int(lg.size()));

  // UPDATE NUMBER OF CONTACTS TO CONSIDER ----------------------------------------------------------
  // update the number of contacts we should consider in collision detection:
  ROS_INFO("Computing the number of contacts we should consider...");
  BTimer.start("ContactConsider"); // Benchmarking Timer - temporary  

  bool done = false; 
  while (!done)
  {
    done = true;

    // Check for collisions in a random state
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
      req.max_contacts *= 2; // double the max contacts that the CollisionRequest checks for
      std::cout << "Doubling max_contacts to " << req.max_contacts << std::endl;

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

    // Do a large number of tests
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
        links_seen_colliding.insert(it->first);
        nc += it->second.size();
      }
      
      // Check if the number of contacts is greater than the max count
      if (nc >= req.max_contacts)
      {
        req.max_contacts *= 2; // double the max contacts that the CollisionRequest checks for
        std::cout << "Doubling max_contacts to " << req.max_contacts << std::endl;
      }
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
            resultAdjList[it->first.first].push_back(it->first.second);
          else
            resultAdjList[it->first.second].push_back(it->first.first);
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

        // Do a large number of tests
        for (unsigned int i = 0 ; i < small_trial_count ; ++i)
        {
          collision_detection::CollisionResult res;
          scene.getCurrentState().setToRandomValues();
          scene.checkSelfCollision(req, res);
          for (collision_detection::CollisionResult::ContactMap::const_iterator it = res.contacts.begin() ; it != res.contacts.end() ; ++it)
          {
            if (links_seen_colliding.insert(it->first).second) // the second is a bool determining if it was already in 
              update = true;
          }
        }
      }   
    }
    
    const std::vector<std::string> &names = scene.getKinematicModel()->getLinkModelNamesWithCollisionGeometry();
    for (std::size_t i = 0 ; i < names.size() ; ++i)
    {
      for (std::size_t j = i + 1 ; j < names.size() ; ++j)
        if (links_seen_colliding.find(std::make_pair(names[i], names[j])) == links_seen_colliding.end())
        {
          if (names[i] < names[j])
            resultAdjList[names[i]].push_back(names[j]);
          else
            resultAdjList[names[j]].push_back(names[i]);
        }
    }
  }
  BTimer.end("NeverColl"); // Benchmarking Timer - temporary  
  
  return resultAdjList;
}

// ******************************************************************************
// Build the robot links connection graph and then check for links with no geomotry
// ******************************************************************************
void 
moveit_configuration_tools::computeConnectionGraph(const planning_models::KinematicModel::LinkModel *start_link, 
                                                   moveit_configuration_tools::LinkGraph &edges)
{
  edges.clear(); // make sure the edges structure is clear

  // Recurively build adj list of link connections
  moveit_configuration_tools::computeConnectionGraphRec(start_link, edges);

  // Repeatidly check for links with no geometry and remove them, then re-check until no more removals are detected
  bool update = true; // track if a no geometry link was found
  while (update)
  {
    update = false; // default

    // Check if each edge has a shape
    for (moveit_configuration_tools::LinkGraph::const_iterator edge_it = edges.begin() ; edge_it != edges.end() ; ++edge_it)
    {
      if (!edge_it->first->getShape()) // link in adjList "edges" does not have shape, remove!
      {        
        // Temporary list for connected links
        std::vector<const planning_models::KinematicModel::LinkModel*> temp_list;

        // Copy link's parent and child links to temp_list
        for (std::set<const planning_models::KinematicModel::LinkModel*>::const_iterator adj_it = edge_it->second.begin(); 
             adj_it != edge_it->second.end(); 
             ++adj_it)
        {
          temp_list.push_back(*adj_it);
        }

        // Make all preceeding and succeeding links to the no-shape link fully connected
        // so that they don't collision check with themselves
        for (std::size_t i = 0 ; i < temp_list.size() ; ++i)
        {
          for (std::size_t j = i + 1 ; j < temp_list.size() ; ++j)
          {
            // for each LinkModel in temp_list, find its location in the edges structure and insert the rest 
            // of the links into its unique set.
            // if the LinkModel is not already in the set, update is set to true so that we keep searching
            if (edges[temp_list[i]].insert(temp_list[j]).second) 
              update = true;
            if (edges[temp_list[j]].insert(temp_list[i]).second)
              update = true;
          }
        }
      }
    }
  }
}


// ******************************************************************************
// Recursively build the adj list of link connections
// ******************************************************************************
void 
moveit_configuration_tools::computeConnectionGraphRec(const planning_models::KinematicModel::LinkModel *start_link, 
                                                      moveit_configuration_tools::LinkGraph &edges)
{
  if (start_link) // check that the link is a valid pointer
  {
    // Loop through every link attached to start_link
    for (std::size_t i = 0 ; i < start_link->getChildJointModels().size() ; ++i)
    {
      const planning_models::KinematicModel::LinkModel *next = start_link->getChildJointModels()[i]->getChildLinkModel();
      
      // Bi-directional connection
      edges[next].insert(start_link);
      edges[start_link].insert(next);
      
      // Iterate with subsequent link
      moveit_configuration_tools::computeConnectionGraphRec(next, edges);      
    }
  }
  else
  {
    ROS_ERROR("Joint exists in URDF with no link!");
  }  
}

