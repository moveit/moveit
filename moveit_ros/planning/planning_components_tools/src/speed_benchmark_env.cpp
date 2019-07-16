/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Jens Petit
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
 *   * Neither the name of the copyright holder nor the names of its
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

/* Author: Jens Petit */

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/collision_plugin_loader/collision_plugin_loader.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <moveit/collision_detection_fcl/collision_detector_allocator_fcl.h>
#include <geometric_shapes/shape_operations.h>
#include <random_numbers/random_numbers.h>

#include <moveit/collision_detection_bullet/collision_env_bullet.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/utils/robot_model_test_utils.h>

static const std::string ROBOT_DESCRIPTION = "robot_description";
static const int MAX_SEARCH_FACTOR_CLUTTER = 3;
static const int MAX_SEARCH_FACTOR_STATES = 30;

enum class RobotStateSelector
{
  IN_COLLISION,
  NOT_IN_COLLISION,
  RANDOM,
};

enum class CollisionDetector
{
  FCL,
  BULLET,
};

enum class CollisionObjectType
{
  MESH,
  BOX,
};

/** \brief Clutters the world of the planning scene with random objects in a certain area around the origin. All added
 *  objects are not in collision.
*
*   \param planning_scene The planning scene
*   \param num_objects The number of objects to be cluttered
*   \param CollisionObjectType Type of object to clutter (mesh or box) */
void clutterWorld(collision_detection::CollisionEnvPtr& env, const size_t num_objects, CollisionObjectType type,
                  robot_state::RobotStatePtr& current_state)
{
  ROS_INFO("Cluttering scene...");

  random_numbers::RandomNumberGenerator num_generator = random_numbers::RandomNumberGenerator(123);

  // allow all robot links to be in collision for world check
  collision_detection::AllowedCollisionMatrix acm{ collision_detection::AllowedCollisionMatrix(
      env->getRobotModel()->getLinkModelNames(), true) };

  // set the robot state to home position
  collision_detection::CollisionRequest req;
  current_state->setToDefaultValues(current_state->getJointModelGroup("panda_arm"), "home");
  current_state->update();

  // load panda link5 as world collision object
  std::string name;
  shapes::ShapeConstPtr shape;
  std::string kinect = "package://moveit_resources/panda_description/meshes/collision/link5.stl";

  Eigen::Quaterniond quat;
  Eigen::Isometry3d pos{ Eigen::Isometry3d::Identity() };

  size_t added_objects{ 0 };
  size_t i{ 0 };
  // create random objects until as many added as desired or quit if too many attempts
  while (added_objects < num_objects && i < num_objects * MAX_SEARCH_FACTOR_CLUTTER)
  {
    // add with random size and random position
    pos.translation().x() = num_generator.uniformReal(-1.0, 1.0);
    pos.translation().y() = num_generator.uniformReal(-1.0, 1.0);
    pos.translation().z() = num_generator.uniformReal(0.0, 1.0);

    quat.x() = num_generator.uniformReal(-1.0, 1.0);
    quat.y() = num_generator.uniformReal(-1.0, 1.0);
    quat.z() = num_generator.uniformReal(-1.0, 1.0);
    quat.w() = num_generator.uniformReal(-1.0, 1.0);
    quat.normalize();
    pos.rotate(quat);

    switch (type)
    {
      case CollisionObjectType::MESH:
      {
        shapes::Mesh* mesh = shapes::createMeshFromResource(kinect);
        mesh->scale(num_generator.uniformReal(0.3, 1.0));
        shape.reset(mesh);
        name = "mesh";
        break;
      }
      case CollisionObjectType::BOX:
      {
        shape.reset(new shapes::Box(num_generator.uniformReal(0.05, 0.2), num_generator.uniformReal(0.05, 0.2),
                                    num_generator.uniformReal(0.05, 0.2)));
        name = "box";
        break;
      }
    }

    name.append(std::to_string(i));
    env->getWorld()->addToObject(name, shape, pos);

    // try if it isn't in collision if yes, ok, if no remove.
    collision_detection::CollisionResult res;
    env->checkRobotCollision(req, res, *current_state, acm);

    if (!res.collision)
    {
      added_objects++;
    }
    else
    {
      ROS_DEBUG_STREAM("Object was in collision, remove");
      env->getWorld()->removeObject(name);
    }

    i++;
  }
  ROS_INFO_STREAM("Cluttered the planning scene with " << added_objects << " objects");
}

/** \brief Runs a collision detection benchmark and measures the time.
*
*   \param trials The number of repeated collision checks for each state
*   \param scene The planning scene
*   \param CollisionDetector The type of collision detector
*   \param only_self Flag for only self collision check performed */
void runCollisionDetection(unsigned int trials, const collision_detection::CollisionEnvPtr& env,
                           const std::vector<robot_state::RobotState>& states, bool only_self)
{
  collision_detection::AllowedCollisionMatrix acm{ collision_detection::AllowedCollisionMatrix(
      env->getRobotModel()->getLinkModelNames(), true) };

  ROS_INFO_STREAM("Starting detection using Bullet env");

  collision_detection::CollisionResult res;
  collision_detection::CollisionRequest req;

  // for world collision request detailed information
  if (!only_self)
  {
    req.contacts = true;
    req.max_contacts = 999;
    req.max_contacts_per_pair = 99;
  }

  ros::WallTime start = ros::WallTime::now();
  for (unsigned int i = 0; i < trials; ++i)
  {
    for (auto& state : states)
    {
      res.clear();

      if (only_self)
      {
        env->checkSelfCollision(req, res, state, acm);
      }
      else
      {
        env->checkRobotCollision(req, res, state, acm);
      }
    }
  }
  double duration = (ros::WallTime::now() - start).toSec();
  ROS_INFO("Performed %lf collision checks per second", (double)trials * states.size() / duration);
  ROS_INFO_STREAM("Total number was " << trials * states.size() << " checks.");
  ROS_INFO_STREAM("We had " << states.size() << " different robot states which were "
                            << (res.collision ? "in collison " : "not in collision ") << "with " << res.contact_count);

  // color collided objects red
  for (auto contact : res.contacts)
  {
    ROS_INFO_STREAM("Between: " << contact.first.first << " and " << contact.first.second);
  }
}

/** \brief Samples valid states of the robot which can be in collision if desired.
 *  \param desired_states Specifier for type for desired state
 *  \param num_states Number of desired states
 *  \param scene The planning scene
 *  \param robot_states Result vector */
void findStates(const RobotStateSelector desired_states, unsigned int num_states,
                robot_state::RobotStatePtr& current_state, const collision_detection::CollisionEnvPtr& env,
                std::vector<robot_state::RobotState>& robot_states)
{
  collision_detection::CollisionRequest req;

  size_t i{ 0 };
  while (robot_states.size() < num_states && i < num_states * MAX_SEARCH_FACTOR_STATES)
  {
    current_state->setToRandomPositions();
    current_state->update();
    collision_detection::CollisionResult res;
    env->checkSelfCollision(req, res, *current_state);
    ROS_INFO_STREAM("Found state " << (res.collision ? "in collision" : "not in collision"));

    switch (desired_states)
    {
      case RobotStateSelector::IN_COLLISION:
        if (res.collision)
          robot_states.push_back(*current_state);
        break;
      case RobotStateSelector::NOT_IN_COLLISION:
        if (!res.collision)
          robot_states.push_back(*current_state);
        break;
      case RobotStateSelector::RANDOM:
        robot_states.push_back(*current_state);
        break;
    }
    i++;
  }

  if (robot_states.empty())
  {
    ROS_ERROR_STREAM("Did not find any correct states.");
  }
}

int main(int argc, char** argv)
{
  robot_model::RobotModelPtr robot_model;
  ros::init(argc, argv, "collision_env_speed");
  ros::NodeHandle node_handle;

  unsigned int trials = 1;

  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::WallDuration sleep_t(2.5);

  robot_model = moveit::core::loadTestingRobotModel("panda");
  collision_detection::CollisionEnvPtr c_env{ new collision_detection::CollisionEnvBullet(robot_model) };

  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;

  collision_detection::AllowedCollisionMatrix acm;
  // Use default collision operations in the SRDF to setup the acm
  const std::vector<std::string>& collision_links = robot_model->getLinkModelNamesWithCollisionGeometry();
  acm.setEntry(collision_links, collision_links, false);

  // allow collisions for pairs that have been disabled
  const std::vector<srdf::Model::DisabledCollision>& dc = robot_model->getSRDF()->getDisabledCollisionPairs();
  for (const srdf::Model::DisabledCollision& it : dc)
    acm.setEntry(it.link1_, it.link2_, true);

  robot_state::RobotStatePtr current_state{ new robot_state::RobotState(c_env->getRobotModel()) };
  c_env->checkCollision(req, res, *current_state, acm);

  ROS_INFO("Starting...");

  if (true)
  {
    ros::Duration(0.5).sleep();

    current_state->setToDefaultValues(current_state->getJointModelGroup("panda_arm"), "home");
    current_state->update();

    std::vector<robot_state::RobotState> sampled_states;
    sampled_states.push_back(*current_state);

    ROS_INFO("Starting benchmark: Robot in empty world, only self collision check");
    runCollisionDetection(trials, c_env, sampled_states, true);

    clutterWorld(c_env, 100, CollisionObjectType::MESH, current_state);

    ROS_INFO("Starting benchmark: Robot in cluttered world, no collision with world");
    runCollisionDetection(trials, c_env, sampled_states, false);

    double joint_2 = 1.5;
    current_state->setJointPositions("panda_joint2", &joint_2);
    current_state->update();

    std::vector<robot_state::RobotState> sampled_states_2;
    sampled_states_2.push_back(*current_state);

    ROS_INFO("Starting benchmark: Robot in cluttered world, in collision with world");
    runCollisionDetection(trials, c_env, sampled_states_2, false);

    moveit_msgs::PlanningScene planning_scene_msg;
  }
  else
  {
    ROS_ERROR("Planning scene not configured");
  }

  return 0;
}
