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

#include "moveit_configuration_tools/tools/compute_longest_valid_segment_fraction.h"
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>

namespace moveit_configuration_tools
{

double computeLongestValidSegmentFraction(const planning_scene::PlanningSceneConstPtr &parent_scene, unsigned int *progress, 
                                          const unsigned int trials, const bool verbose)
{
  return computeLongestValidSegmentFraction(parent_scene, "", progress, trials, verbose);
}
/*

static void addCollisionObject(const planning_scene::PlanningScenePtr &scene, unsigned int trials_valid_invalid, unsigned int trials)
{
  static const std::string ID = "__OBJ_TEST_CHECK_LONGEST_VALID_SEGMENT_FRACTION";
  
  // create a box as tall as the robot, but 1cm on each side.
  std::vector<double> aabb;
  scene->getCurrentState().computeAABB(aabb);
  shapes::ShapePtr shape(new shapes::Box(0.01, 0.01, std::max(0.1, aabb[5] - aabb[4])));
  
  // place the box on the corner of the robot, at first
  Eigen::Affine3d pose;
  pose.setIdentity();
  pose.translation().x() = aabb[0];
  pose.translation().y() = aabb[2];
  
  // add the box to the scene
  scene->getCollisionWorld()->addToObject(ID, shape, pose);
  
  collision_detection::CollisionRequest req;
  random_numbers::RandomNumberGenerator rng;
  planning_models::KinematicState test(scene->getCurrentState());
  bool found = false;
  do
  {
    unsigned int attempts = 0;  
    bool found_valid = false;
    bool found_invalid = false;
    do 
    {
      ++attempts;
      // check world collision
      collision_detection::CollisionResult res;
      scene->getCollisionWorld()->checkRobotCollision(req, res, *scene->getCollisionRobot(),
                                                      test, scene->getAllowedCollisionMatrix());
      if (res.collision)
        found_invalid = true;
      else
        found_valid = true;
    } while (!(found_valid && found_invalid) && attempts < trials_valid_invalid);
    if (found_valid && found_invalid)
      found = true;
    else
    {
      pose.translation().x() = aabb[0];
      pose.translation().y() = aabb[2];
      scene->getCollisionWorld()->moveShapeInObject(ID, shape, pose);
    }
  } while (!found);
  
}

static bool generateValidState(const planning_scene::PlanningSceneConstPtr &scene, planning_models::KinematicState &state, unsigned int trials)
{
  unsigned int attempts = 0;
  bool found = false;
  do
  {
    ++attempts;
    state.setToRandomValues();
    if (!scene->isStateColliding(state))
      found = true;
  } while (!found && attempts < trials);
  return found;
}

static bool generateInvalidState(const planning_scene::PlanningSceneConstPtr &scene, planning_models::KinematicState &state)
{
  
}
*/
double computeLongestValidSegmentFraction(const planning_scene::PlanningSceneConstPtr &parent_scene, const std::string &group, unsigned int *progress, 
                                          const unsigned int trials, const bool verbose)
{
  /*
  const planning_scene::PlanningSceneConstPtr scene = parent_scene;
  
  // check to see if there are any objects in the collision world that the robot can potentially collide with
  if (parent_scene->getCollisionWorld()->getObjectsCount() == 0)
  {
    // there are no objects, so we add one
    planning_scene::PlanningScenePtr new_scene(new planning_scene::PlanningScene(parent_scene));
    addCollisionObject(new_scene, 100, 100);
    scene = new_scene;
  }
  else
  {
    // sample at random to see if the robot will hit something in the environment
    planning_models::KinematicState test(parent_scene->getCurrentState());
    collision_detection::CollisionRequest req;
    bool found = false;
    unsigned int attempts = 0;
    do 
    {
      ++attempts;
      test.setToRandomValues();
      collision_detection::CollisionResult res;
      parent_scene->getCollisionWorld()->checkRobotCollision(req, res, *parent_scene->getCollisionRobot(),
                                                             test, parent_scene->getAllowedCollisionMatrix());
      if (res.collision)
      {
        res.clear();
        parent_scene->getCollisionRobot()->checkSelfCollision(req, res, test, parent_scene->getAllowedCollisionMatrix());
        if (!res.collision)
          found = true;
      }
    } while (!found && attempts < trials);
    
    if (!found)
    {
      planning_scene::PlanningScenePtr new_scene(new planning_scene::PlanningScene(parent_scene));
      addCollisionObject(new_scene, 100, 100);
      scene = new_scene;
    }
  }
  
  // at this point we know we have a scene where the robot can move and sometimes it collides with an object
  // next, we generate random motions that cross the object in their path

  planning_models::KinematicState s1(parent_scene->getCurrentState());
  planning_models::KinematicState s2(parent_scene->getCurrentState());
  
  bool found = false;  
  if (generateValidState(scene, s1, trials))
    found = generateValidState(scene, s2, trials);
  if (found)
  {
    double d = s1.distance(s2);
    
  }
  
  // 0.001 -- 0.2
  */
  return 0.05;
}


}
