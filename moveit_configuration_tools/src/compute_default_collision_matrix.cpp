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

/* Author: Dave Coleman */

#include "moveit_configuration_tools/compute_default_collision_matrix.h"
#include "moveit_configuration_tools/benchmark_timer.h"
#include <boost/math/special_functions/binomial.hpp> // for statistics at end
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>

extern BenchmarkTimer BTimer;

boost::mutex _access; // used for threading

// LinkGraph defines a Link's model and a set of unique links it connects
typedef std::map<const planning_models::KinematicModel::LinkModel*, std::set<const planning_models::KinematicModel::LinkModel*> > LinkGraph;

// StringAdjList is an adjacency list structure containing links in string-based form
typedef std::map<std::string, std::set<std::string> > StringAdjList;

// Unique set of pairs of links in string-based form
typedef std::set<std::pair<std::string, std::string> > StringPairSet;

// ******************************************************************************************
// Static Prototypes
// ******************************************************************************************

/**
 * \brief Build the robot links connection graph and then check for links with no geomotry
 * \param link The root link to begin a breadth first search on
 * \param link_graph A representation of all bi-direcitonal joint connections between links in robot_description
 */
static void computeConnectionGraph(const planning_models::KinematicModel::LinkModel *link, LinkGraph &link_graph);

/**
 * \brief Recursively build the adj list of link connections
 * \param link The root link to begin a breadth first search on
 * \param link_graph A representation of all bi-direcitonal joint connections between links in robot_description
 */
static void computeConnectionGraphRec(const planning_models::KinematicModel::LinkModel *link, LinkGraph &link_graph);

/**
 * \brief Disable collision checking for adjacent links, or adjacent with no geometry links between them
 * \param link_graph A representation of all bi-direcitonal joint connections between links in robot_description
 * \param disabled_links An adjacency list of all links to be disabled, with pairs ordered alphabetically
 * \param acm Matrix of collisions that are disabled in the collision_detection library
 * \return number of adjacent links found and disabled
 */
static unsigned int disableAdjacentLinks(LinkGraph &link_graph, StringAdjList &disabled_links, 
                                         collision_detection::AllowedCollisionMatrix &acm);


/**
 * \brief Disable all collision checks that occur when the robot is started in its default state
 * \param scene A reference to the robot in the planning scene
 * \param disabled_links An adjacency list of all links to be disabled, with pairs ordered alphabetically
 * \param acm Matrix of collisions that are disabled in the collision_detection library
 * \param req A reference to a collision request that is already initialized
 * \return number of default collision links found and disabled
 */
static unsigned int disableDefaultCollisions(planning_scene::PlanningScene &scene, StringAdjList &disabled_links, 
                                             collision_detection::AllowedCollisionMatrix &acm, collision_detection::CollisionRequest &req);

/**
 * \brief Compute the links that are always in collision
 * \param scene A reference to the robot in the planning scene
 * \param disabled_links An adjacency list of all links to be disabled, with pairs ordered alphabetically
 * \param acm Matrix of collisions that are disabled in the collision_detection library
 * \param req A reference to a collision request that is already initialized
 * \param links_seen_colliding Set of links that have at some point been seen in collision
 * \return number of always in collision links found and disabled
 */
static unsigned int disableAlwaysInCollision(planning_scene::PlanningScene &scene, StringAdjList &disabled_links, 
                                             collision_detection::AllowedCollisionMatrix &acm, collision_detection::CollisionRequest &req,
                                             StringPairSet &links_seen_colliding);

/**
 * \brief Get the pairs of links that are never in collision
 * \param scene A reference to the robot in the planning scene
 * \param disabled_links An adjacency list of all links to be disabled, with pairs ordered alphabetically
 * \param acm Matrix of collisions that are disabled in the collision_detection library
 * \param req A reference to a collision request that is already initialized
 * \param links_seen_colliding Set of links that have at some point been seen in collision
 * \return number of never in collision links found and disabled
 */
static unsigned int disableNeverInCollision(planning_scene::PlanningScene &scene, StringAdjList &disabled_links, 
                                            collision_detection::AllowedCollisionMatrix &acm, const collision_detection::CollisionRequest &req,
                                            StringPairSet &links_seen_colliding);

// ******************************************************************************************
// Generates an adjacency list of links that are always and never in collision, to speed up collision detection
// ******************************************************************************************
std::map<std::string, std::set<std::string> >  // an adj list
moveit_configuration_tools::computeDefaultCollisionMatrix(const planning_scene::PlanningSceneConstPtr &parent_scene, 
                                                          bool include_never_colliding, int trials, bool verbose)
{
  // Create new instance of planning scene using pointer
  planning_scene::PlanningScene scene(parent_scene);

  // Create structure for tracking which collisions are allowed
  // AllowedCollisionMatrix: Definition of a structure for the allowed collision matrix. 
  // All elements in the collision world are referred to by their names.
  // This class represents which collisions are allowed to happen and which are not. */
  collision_detection::AllowedCollisionMatrix &acm = scene.getAllowedCollisionMatrix();
  std::cout << "Intial ACM size " << acm.getSize() << std::endl;

  //acm.print(std::cout);

  // Map of disabled collisions that contains a link as a key and an ordered list of links that are connected. An adjaceny list.
  //std::map<std::string, std::set<std::string> > disabled_links;
  StringAdjList disabled_links;

  // Track unique edges that have been found to be in collision in some state
  StringPairSet links_seen_colliding;
  

  // FIND CONNECTING LINKS ------------------------------------------------------------------------
  // For each link, compute the set of other links it connects to via a single joint (adjacent links) 
  // or via a chain of joints with intermediate links with no geometry (like a socket joint)

  LinkGraph link_graph; // LinkGraph is a custom type of a map with a LinkModel as key and a set of LinkModels as second

  // Create Connection Graph
  BTimer.start("Compute Connection Graph"); // Benchmarking Timer - temporary
  computeConnectionGraph(scene.getKinematicModel()->getRootLink(), link_graph);
  BTimer.end("Compute Connection Graph"); // Benchmarking Timer - temporary

  // DISABLE ALL ADJACENT LINK COLLISIONS ---------------------------------------------------------
  // if 2 links are adjacent, or adjacent with a zero-shape between them, disable collision checking for them
  BTimer.start("Disable Adjacent Links"); // Benchmarking Timer - temporary
  unsigned int num_adjacent = 0; //disableAdjacentLinks( link_graph, disabled_links, acm );
  BTimer.end("Disable Adjacent Links"); // Benchmarking Timer - temporary

  // INITIAL CONTACTS TO CONSIDER GUESS -----------------------------------------------------------
  // Create collision detection request object
  collision_detection::CollisionRequest req;
  req.contacts = true;
  req.max_contacts = int(link_graph.size()); // max number of contacts to compute. initial guess is number of links on robot
  req.max_contacts_per_pair = 1;
  req.verbose = false;


  // DISABLE "DEFAULT" COLLISIONS --------------------------------------------------------
  // Disable all collision checks that occur when the robot is started in its default state
  BTimer.start("Default Collisions"); // Benchmarking Timer - temporary
  unsigned int num_default = 0; //disableDefaultCollisions(scene, disabled_links, acm, req);
  BTimer.end("Default Collisions"); // Benchmarking Timer - temporary


  // ALWAYS IN COLLISION --------------------------------------------------------------------
  // Compute the links that are always in collision
  BTimer.start("Always in Collision"); // Benchmarking Timer - temporary
  unsigned int num_always = 0; //disableAlwaysInCollision(scene, disabled_links, acm, req, links_seen_colliding);
  BTimer.end("Always in Collision"); // Benchmarking Timer - temporary  

  ROS_INFO("Links seen colliding total = %d", int(links_seen_colliding.size()));


  // NEVER IN COLLISION -------------------------------------------------------------------
  // Get the pairs of links that are never in collision
  BTimer.start("Never in Collision"); // Benchmarking Timer - temporary  
  unsigned int num_never;
  if (include_never_colliding)
  {
    num_never = disableNeverInCollision(scene, disabled_links, acm, req, links_seen_colliding);
  }
  BTimer.end("Never in Collision"); // Benchmarking Timer - temporary  
  std::cout << "DISABLED " << disabled_links.size() << "\n";
  ROS_INFO("Links seen colliding total = %d", int(links_seen_colliding.size()));


  if(verbose)
  {
    std::cout << "ACM size is now " << acm.getSize() << std::endl;
    //acm.print(std::cout);

    // Calculate number of disabled links:
    unsigned int num_disabled = 0;
    for (std::map<std::string, std::set<std::string> >::const_iterator it = disabled_links.begin() ; it != disabled_links.end() ; ++it)
      for (std::set<std::string>::const_iterator link2_it = it->second.begin(); link2_it != it->second.end();  ++link2_it)
        ++num_disabled;

    std::cout << "-------------------------------------------------------------------------------\n";
    std::cout << "Statistics: \n";
    unsigned int num_links = int(link_graph.size());
    double num_possible = boost::math::binomial_coefficient<double>(num_links, 2); // n choose 2
    unsigned int num_sometimes = num_possible - num_disabled;

    printf("%6d : %s\n",   num_links, "Total Links");
    printf("%6.0f : %s\n", num_possible, "Total possible collisions");
    printf("%6d : %s\n",   num_always, "Always in collision");
    printf("%6d : %s\n",   num_never, "Never in collision");
    printf("%6d : %s\n",   num_default, "Default in collision");
    printf("%6d : %s\n",   num_adjacent, "Adjacent links disabled");
    printf("%6d : %s\n",   num_sometimes, "Sometimes in collision");
    printf("%6d : %s\n",   num_disabled, "TOTAL DISABLED");

    std::cout << "Copy to Spreadsheet:\n";
    std::cout << num_links << "\t" << num_possible << "\t" << num_always << "\t" << num_never 
              << "\t" << num_default << "\t" << num_adjacent << "\t" << num_sometimes 
              << "\t" << num_disabled << std::endl;

  }

  return disabled_links;
}

// ******************************************************************************************
// Build the robot links connection graph and then check for links with no geomotry
// ******************************************************************************************
void computeConnectionGraph(const planning_models::KinematicModel::LinkModel *start_link, LinkGraph &link_graph)
{
  link_graph.clear(); // make sure the edges structure is clear

  // Recurively build adj list of link connections
  computeConnectionGraphRec(start_link, link_graph);

  // Repeatidly check for links with no geometry and remove them, then re-check until no more removals are detected
  bool update = true; // track if a no geometry link was found
  while (update)
  {
    update = false; // default

    // Check if each edge has a shape
    for (LinkGraph::const_iterator edge_it = link_graph.begin() ; edge_it != link_graph.end() ; ++edge_it)
    {
      if (!edge_it->first->getShape()) // link in adjList "link_graph" does not have shape, remove!
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
            // for each LinkModel in temp_list, find its location in the link_graph structure and insert the rest 
            // of the links into its unique set.
            // if the LinkModel is not already in the set, update is set to true so that we keep searching
            if (link_graph[temp_list[i]].insert(temp_list[j]).second) 
              update = true;
            if (link_graph[temp_list[j]].insert(temp_list[i]).second)
              update = true;
          }
        }
      }
    }
  }
  ROS_INFO("Generated connection graph with %d links", int(link_graph.size()));
}


// ******************************************************************************************
// Recursively build the adj list of link connections
// ******************************************************************************************
void computeConnectionGraphRec(const planning_models::KinematicModel::LinkModel *start_link, LinkGraph &link_graph)
{
  if (start_link) // check that the link is a valid pointer
  {
    // Loop through every link attached to start_link
    for (std::size_t i = 0 ; i < start_link->getChildJointModels().size() ; ++i)
    {
      const planning_models::KinematicModel::LinkModel *next = start_link->getChildJointModels()[i]->getChildLinkModel();
      
      // Bi-directional connection
      link_graph[next].insert(start_link);
      link_graph[start_link].insert(next);
      
      // Iterate with subsequent link
      computeConnectionGraphRec(next, link_graph);      
    }
  }
  else
  {
    ROS_ERROR("Joint exists in URDF with no link!");
  }  
}

// ******************************************************************************************
// Disable collision checking for adjacent links, or adjacent with no geometry links between them
// ******************************************************************************************
unsigned int disableAdjacentLinks(LinkGraph &link_graph, StringAdjList &disabled_links, collision_detection::AllowedCollisionMatrix &acm)
{
  int num_disabled = 0;
  for (LinkGraph::const_iterator link_graph_it = link_graph.begin() ; link_graph_it != link_graph.end() ; ++link_graph_it)
  {
    // disable all connected links to current link by looping through them
    for (std::set<const planning_models::KinematicModel::LinkModel*>::const_iterator adj_it = link_graph_it->second.begin(); 
         adj_it != link_graph_it->second.end(); 
         ++adj_it)
    {
      //  ROS_INFO("Disabled std::cout << "LinkModel " << 
      //ROS_INFO("Disabled %s to %s", link_graph_it->first->getName().c_str(), (*adj_it)->getName().c_str() );

      // compare the string names of the two links and add the lesser alphabetically, s.t. the pair is only added once
      if (link_graph_it->first->getName() < (*adj_it)->getName() )
        disabled_links[ link_graph_it->first->getName() ].insert( (*adj_it)->getName() );
      else
        disabled_links[ (*adj_it)->getName() ].insert( link_graph_it->first->getName() );
      acm.setEntry( link_graph_it->first->getName(), (*adj_it)->getName(), true); // disable link checking in the collision matrix
      
      ++num_disabled;
    }
  }
  ROS_INFO("Disabled %d adjancent links from collision checking", num_disabled);
  
  return num_disabled;
}

// ******************************************************************************************
// Disable all collision checks that occur when the robot is started in its default state
// ******************************************************************************************
unsigned int disableDefaultCollisions(planning_scene::PlanningScene &scene, StringAdjList &disabled_links, 
                                      collision_detection::AllowedCollisionMatrix &acm, collision_detection::CollisionRequest &req)
{
  // Setup environment
  collision_detection::CollisionResult res;
  scene.getCurrentState().setToDefaultValues(); // set to default values of 0 OR half between low and high joint values
  scene.checkSelfCollision(req, res);

  // For each collision in default state, always add to disabled links set
  int num_disabled = 0;
  for (collision_detection::CollisionResult::ContactMap::const_iterator it = res.contacts.begin() ; it != res.contacts.end() ; ++it)
  {
    // compare the string names of the two links and add the lesser alphabetically, s.t. the pair is only added once
    if (it->first.first < it->first.second) 
      disabled_links[it->first.first].insert(it->first.second);
    else
      disabled_links[it->first.second].insert(it->first.first);

    acm.setEntry(it->first.first, it->first.second, true); // disable link checking in the collision matrix

    //ROS_INFO("Disabled %s to %s", it->first.first.c_str(), it->first.second.c_str());

    ++num_disabled;
  }

  ROS_INFO("Disabled %d links that are in collision in default state", num_disabled);  

  return num_disabled;
}

// ******************************************************************************************
// Compute the links that are always in collision
// ******************************************************************************************
unsigned int disableAlwaysInCollision(planning_scene::PlanningScene &scene, StringAdjList &disabled_links, 
                                      collision_detection::AllowedCollisionMatrix &acm, collision_detection::CollisionRequest &req,
                                      StringPairSet &links_seen_colliding)
{
  // Trial count variables
  static const unsigned int small_trial_count = 1000;
  static const unsigned int small_trial_limit = (unsigned int)((double)small_trial_count * 0.95);
  
  ROS_INFO("Computing pairs of links that are always in collision...");
  bool done = false;
  unsigned int num_disabled = 0;
    
  while (!done)
  {
    // DO 'small_trial_count' COLLISION CHECKS AND RECORD STATISTICS ---------------------------------------
    std::map<std::pair<std::string, std::string>, unsigned int> collision_count;    

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
        collision_count[it->first]++;
        links_seen_colliding.insert(it->first);
        nc += it->second.size();
      }
      
      // Check if the number of contacts is greater than the max count
      if (nc >= req.max_contacts)
      {
        req.max_contacts *= 2; // double the max contacts that the CollisionRequest checks for
        ROS_INFO("Doubling max_contacts to %d", int(req.max_contacts));
      }
    }

    // >= 95% OF TIME IN COLLISION DISABLE -----------------------------------------------------
    // Disable collision checking for links that are >= 95% of the time in collision
    int found = 0;

    // Loop through every pair of link collisions and disable if it meets the threshold
    for (std::map<std::pair<std::string, std::string>, unsigned int>::const_iterator it = collision_count.begin() ; it != collision_count.end() ; ++it)
    {      
      /*std::cout << it->first.first;
        std::cout << std::setw(50) << it->first.second;
        std::cout << std::setw(50) << it->second << "\n";*/

      // Disable these two links permanently      
      if (it->second > small_trial_limit)
      {
        //std::cout << "\t DISABLED AT 95% CHECK !!!!!!!!!!!!!!!!!!!!!!!!!! \n";

        // compare the string names of the two links and add the lesser alphabetically, s.t. the pair is only added once
        if (it->first.first < it->first.second) 
          disabled_links[it->first.first].insert(it->first.second);
        else
          disabled_links[it->first.second].insert(it->first.first);

        acm.setEntry(it->first.first, it->first.second, true); // disable link checking in the collision matrix

        num_disabled++;
        found ++;
      }

    }
    
    // if no updates were made to the collision matrix, we stop
    if (found == 0)
      done = true;

    ROS_INFO("Disabled %u collision checks", found);
    std::cout << "LOOPING ALWAYS COLLISION CHECKER ------------------------------------ \n\n";
  }

  return num_disabled;
}

// ******************************************************************************************
// Get the pairs of links that are never in collision
// ******************************************************************************************
unsigned int disableNeverInCollisionBACKUP(planning_scene::PlanningScene &scene, StringAdjList &disabled_links, 
                                           collision_detection::AllowedCollisionMatrix &acm, const collision_detection::CollisionRequest &req,
                                           StringPairSet &links_seen_colliding)
{
  static const unsigned int small_trial_count = 10000;
  unsigned int num_never = 0;

  int total_checks = 0;
  int total_checks_used = 0;
  for (int k = 0 ; k < 50 ; ++k) // loop 10 times just to be sure. 10 is arbitary
  {
    ROS_INFO("K Loop %d", k);

    bool update = true;
    while (update)
    {
      update = false;
      //ROS_INFO("Still seeing updates on possibly colliding links ...");
      int new_links_seen_colliding_count = 0;

      // Do a large number of tests
      for (unsigned int i = 0 ; i < small_trial_count ; ++i)
      {
        ++total_checks;
        collision_detection::CollisionResult res;
        scene.getCurrentState().setToRandomValues();
        scene.checkSelfCollision(req, res);

        for (collision_detection::CollisionResult::ContactMap::const_iterator it = res.contacts.begin() ; it != res.contacts.end() ; ++it)
        {
          acm.setEntry(it->first.first, it->first.second, true); // disable link checking in the collision matrix

          if (links_seen_colliding.insert(it->first).second) // the second is a bool determining if it was already in 
          {
            update = true; // this collision has not yet been inserted into the list
            //std::cout << "       Also is new in list" << std::endl;
            //std::cout << std::endl << "Collision between " << it->first.first << " " << it->first.second << std::endl;
            ++ new_links_seen_colliding_count;
            total_checks_used = total_checks;
          }
        }
      }
      std::cout << "New links seen colliding " <<  new_links_seen_colliding_count << " ----- links seen colliding total: " 
                << links_seen_colliding.size() << "\n";
    }   
  }
    
  // Get the names of the link models that have some collision geometry associated to themselves
  const std::vector<std::string> &names = scene.getKinematicModel()->getLinkModelNamesWithCollisionGeometry();
  ROS_INFO("LINK MODELS WITH GEOMETRY: %d", int(names.size()));
    
  // Loop through every combination of name pairs n^2
  for (std::size_t i = 0 ; i < names.size() ; ++i)
  {
    for (std::size_t j = i + 1 ; j < names.size() ; ++j)
    {
      // Check if current pair has been seen colliding ever
      if (links_seen_colliding.find(std::make_pair(names[i], names[j])) == links_seen_colliding.end())
      {
        ++num_never;
        
        // Add to disabled list
        if (names[i] < names[j])
          disabled_links[names[i]].insert(names[j]);
        else
          disabled_links[names[j]].insert(names[i]);

        //ROS_INFO("Disabled %s to %s", names[i].c_str(), names[j].c_str());
      }
    }
  }
  ROS_INFO("Found %d links that are never in collision", num_never);
  ROS_INFO("Total checks used was %d", total_checks_used);

  return num_never;
}


void disableNeverInCollisionThread( int thread_id, unsigned int num_trials, StringPairSet * links_seen_colliding,
                                    planning_scene::PlanningScene &scene, collision_detection::AllowedCollisionMatrix &acm, 
                                    const collision_detection::CollisionRequest &req)
                                    
{
  StringPairSet my_links_seen_colliding; // disabled links just for this thread
  boost::posix_time::milliseconds workTime( 2000 );

  std::cout << "Worker: " << thread_id << " running" << std::endl;

  // Do a large number of tests
  for (unsigned int i = 0 ; i < num_trials ; ++i)
  {
    collision_detection::CollisionResult res;
    scene.getCurrentState().setToRandomValues();
    scene.checkSelfCollision(req, res);

    for (collision_detection::CollisionResult::ContactMap::const_iterator it = res.contacts.begin() ; it != res.contacts.end() ; ++it)
    {
      acm.setEntry(it->first.first, it->first.second, true); // disable link checking in the collision matrix

      if (my_links_seen_colliding.insert(it->first).second) // the second is a bool determining if it was already in 
      {
        //p    update = true; // this collision has not yet been inserted into the list
        //std::cout << "       Also is new in list" << std::endl;
        std::cout << "Worker " << thread_id << "\t Collision between " << it->first.first << " " << it->first.second << std::endl;
        //total_checks_used = total_checks;
      }
    }
  }

  // ---------------------------------------------------------
  // Join our list with the main list. Lock the main list
  {
    boost::mutex::scoped_lock lock(_access);

    std::cout << "HAS LOCK: " << thread_id << std::endl;

    for (StringPairSet::const_iterator set_it = my_links_seen_colliding.begin() ; set_it != my_links_seen_colliding.end() ; ++set_it)
    {    
      // Insert pair from this thread's disabled_links to the main disabled_links list
      (my_links_seen_colliding).insert( std::pair<std::string,std::string>( "Dave", "Coleman")  );

      std::cout << "Combining " << "\t" << set_it->first<< "\t" << set_it->second << std::endl;
    }
    // Pause for fun
    boost::this_thread::sleep(workTime);
  }


  std::cout << "Worker: " << thread_id << " finished" << std::endl;

}

// ******************************************************************************************
// Get the pairs of links that are never in collision
// ******************************************************************************************
unsigned int disableNeverInCollision(planning_scene::PlanningScene &scene, StringAdjList &disabled_links, 
                                     collision_detection::AllowedCollisionMatrix &acm, const collision_detection::CollisionRequest &req,
                                     StringPairSet &links_seen_colliding)
{
  static const unsigned int num_trials = 10;
  unsigned int num_never = 0;

  //  int total_checks = 0;
  //  int total_checks_used = 0;
  /*for (int k = 0 ; k < 50 ; ++k) // loop 10 times just to be sure. 10 is arbitary
    {
    ROS_INFO("K Loop %d", k);

    bool update = true;
    while (update)
    {
    update = false;*/

  //ROS_INFO("Still seeing updates on possibly colliding links ...");



  std::cout << "main: startup" << std::endl;


  boost::thread_group bgroup; // create a group of threads
  for(int i = 0; i < 2; ++i) // TODO: change to 4
    bgroup.create_thread( boost::bind( disableNeverInCollisionThread, i, num_trials, links_seen_colliding, scene, acm, req ) );

  std::cout << "main: waiting for thread" << std::endl  << std::endl;

  bgroup.join_all();

  std::cout  << std::endl << "main: done" << std::endl;



  //    }   
  //  }
    
  // Get the names of the link models that have some collision geometry associated to themselves
  const std::vector<std::string> &names = scene.getKinematicModel()->getLinkModelNamesWithCollisionGeometry();
  ROS_INFO("LINK MODELS WITH GEOMETRY: %d", int(names.size()));
    
  // Loop through every combination of name pairs n^2
  for (std::size_t i = 0 ; i < names.size() ; ++i)
  {
    for (std::size_t j = i + 1 ; j < names.size() ; ++j)
    {
      // Check if current pair has been seen colliding ever
      if (links_seen_colliding.find(std::make_pair(names[i], names[j])) == links_seen_colliding.end())
      {
        ++num_never;
        /* 
        // Add to disabled list
        if (names[i] < names[j])
        disabled_links[names[i]].insert(names[j]);
        else
        disabled_links[names[j]].insert(names[i]);
        */
        //ROS_INFO("Disabled %s to %s", names[i].c_str(), names[j].c_str());
      }
    }
  }
  ROS_INFO("Found %d links that are never in collision", num_never);
  //ROS_INFO("Total checks used was %d", total_checks_used);

  return num_never;
}
