/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Universitaet Hamburg
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Rafael A. Rojas*/

#include <gtest/gtest.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene/collision_object.h>
#include <moveit/utils/message_checks.h>
#include <moveit/utils/robot_model_test_utils.h>
#include <urdf_parser/urdf_parser.h>
#include <string>
#include <ros/package.h>
#include <tf2_eigen/tf2_eigen.h>
#include <moveit/rdf_loader/rdf_loader.h>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

TEST(URDF_TO_COLLISION_OBJECT, LoadFromFile)
{
  std::string path = ros::package::getPath("moveit_resources_fanuc_description") + "/urdf/fanuc.urdf";
  urdf::Model urdf_model;
  urdf_model.initFile(path);

  // use rdf_loader::RDFLoader with urdf and srdf to instantiate a robot model and check the kinematics

  KDL::Tree tree;
  KDL::Chain chain;
  kdl_parser::treeFromUrdfModel(urdf_model, tree);

  tree.getChain("base_link", "tool0", chain);

  KDL::ChainFkSolverPos_recursive fk_solver(chain);

  KDL::JntArray joint_array;
  joint_array.data = Eigen::VectorXd::Zero(6);
  // chain.getSegment(0).
  for (std::size_t i = 0; i < chain.getNrOfSegments(); i++)
  {
    KDL::Frame frame;
    chain.getSegment(0).getName();
    fk_solver.ChainFkSolverPos::JntToCart(joint_array, frame, i);
  }

  moveit_msgs::CollisionObject collision_object = planning_scene::urdf_to_collision_object(urdf_model);

  std::cout << "KDL Segments " << tree.getNrOfSegments() << "  collision object frames "
            << collision_object.primitives.size() << "\n";
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
