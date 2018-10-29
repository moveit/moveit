/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Bryce Willey.
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
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF AD w/ 2VISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Bryce Willey */

#include <ros/ros.h>
#include <boost/algorithm/string_regex.hpp>
#include <boost/math/constants/constants.hpp>
#include <geometry_msgs/Pose.h>
#include "moveit/utils/robot_model_builder.h"

namespace moveit
{
namespace core
{
    moveit::core::RobotModelPtr loadRobot(std::string robot_name)
    {
        moveit::core::RobotModelPtr robot_model;
        boost::filesystem::path res_path(MOVEIT_TEST_RESOURCES_DIR);

        urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDFFile((res_path / robot_name / "urdf/robot.xml").string());
        if (urdf_model == nullptr)
        {
            ROS_ERROR("Cannot find URDF for %s. Make sure moveit_resources/your robot description is installed",
                      robot_name.c_str());
            return robot_model;
        }
        srdf::ModelSharedPtr srdf_model;
        srdf_model.reset(new srdf::Model());
        srdf_model->initFile(*urdf_model, (res_path / robot_name / "srdf/robot.xml").string());
        robot_model.reset(new moveit::core::RobotModel(urdf_model, srdf_model));
        return robot_model;
    }

    RobotModelBuilder::RobotModelBuilder(std::string name, std::string base_link_name) :
        urdf_model_(new urdf::ModelInterface()), srdf_writer_(new srdf::SRDFWriter())
    {
        urdf_model_->clear();
        urdf_model_->name_ = name;

        urdf::LinkSharedPtr base_link;
        base_link.reset(new urdf::Link);
        base_link->name = base_link_name;
        urdf_model_->links_.insert(std::make_pair(base_link_name, base_link));

        srdf_writer_->robot_name_ = name;
    }

  void RobotModelBuilder::add(std::string section, std::string type, std::vector<geometry_msgs::Pose> origins)
    {
        std::vector<std::string> link_names;
        boost::split_regex(link_names, section, boost::regex("->"));
        if (link_names.empty())
        {
            ROS_ERROR("No links specified (empty section?)");
            return;
        }
        // First link should already be added.
        if (not urdf_model_->getLink(link_names[0]))
        {
            ROS_ERROR("Link %s not present in builder yet!", link_names[0].c_str());
            return;
        }

        // Iterate through each link.
        for (size_t i = 1; i < link_names.size(); i++)
        {
            // These links shouldn't be present already.
            if (urdf_model_->getLink(link_names[i]))
            {
                ROS_ERROR("Link %s is already specified", link_names[i].c_str());
                return;
            }
            urdf::LinkSharedPtr link;
            link.reset(new urdf::Link);
            link->name = link_names[i];
            urdf_model_->links_.insert(std::make_pair(link_names[i], link));
            urdf::JointSharedPtr joint;
            joint.reset(new urdf::Joint);
            joint->name = link_names[i - 1] + "-" + link_names[i] + "-joint";
            // Default to Identity transform for origins.
            geometry_msgs::Pose o = origins[i - 1];
            joint->parent_to_joint_origin_transform.clear();
            joint->parent_to_joint_origin_transform.position =
              urdf::Vector3(o.position.x, o.position.y, o.position.z);
            joint->parent_to_joint_origin_transform.rotation =
              urdf::Rotation(o.orientation.x, o.orientation.y, o.orientation.z, o.orientation.w);

            joint->parent_link_name = link_names[i - 1];
            joint->child_link_name = link_names[i];
            if (type == "planar")
                joint->type = urdf::Joint::PLANAR;
            else if (type == "floating")
                joint->type = urdf::Joint::FLOATING;
            else if (type == "revolute")
                joint->type = urdf::Joint::REVOLUTE;
            else if (type == "continuous")
                joint->type = urdf::Joint::CONTINUOUS;
            else if (type == "prismatic")
                joint->type = urdf::Joint::PRISMATIC;
            else if (type == "fixed")
                joint->type = urdf::Joint::FIXED;
            else
            {
                ROS_ERROR("No such joint type as %s", type.c_str());
                return;
            }

            joint->axis = urdf::Vector3(1.0, 0.0, 0.0);
            if (joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::PRISMATIC)
            {
              urdf::JointLimitsSharedPtr limits;
              limits.reset(new urdf::JointLimits);
              limits->lower = -boost::math::constants::pi<double>();
              limits->upper = boost::math::constants::pi<double>();

              joint->limits = limits;
            }
            urdf_model_->joints_.insert(std::make_pair(joint->name, joint));
        }
    }

  void RobotModelBuilder::addLinkBox(std::string link_name, geometry_msgs::Point size, geometry_msgs::Pose origin)
  {
    if (not urdf_model_->getLink(link_name))
      {
        ROS_ERROR("Link %s not present in builder yet!", link_name.c_str());
        return;
      }
    urdf::LinkSharedPtr link;
    urdf_model_->getLink(link_name, link);

    urdf::CollisionSharedPtr coll;
    coll.reset(new urdf::Collision);
    coll->origin.position = urdf::Vector3(origin.position.x, origin.position.y, origin.position.z);
    coll->origin.rotation = urdf::Rotation(origin.orientation.x, origin.orientation.y, origin.orientation.z, origin.orientation.w);
    urdf::BoxSharedPtr geometry;
    geometry.reset(new urdf::Box);
    geometry->dim = urdf::Vector3(size.x, size.y, size.z);
    coll->geometry = geometry;
    link->collision_array.push_back(coll);
  }

  void RobotModelBuilder::addLinkMesh(std::string link_name, std::string filename, geometry_msgs::Pose origin)
  {
    if (not urdf_model_->getLink(link_name))
      {
        ROS_ERROR("Link %s not present in builder yet!", link_name.c_str());
        return;
      }
    urdf::LinkSharedPtr link;
    urdf_model_->getLink(link_name, link);

    urdf::CollisionSharedPtr coll;
    coll.reset(new urdf::Collision);
    coll->origin.position = urdf::Vector3(origin.position.x, origin.position.y, origin.position.z);
    coll->origin.rotation = urdf::Rotation(origin.orientation.x, origin.orientation.y, origin.orientation.z, origin.orientation.w);
    urdf::MeshSharedPtr geometry;
    geometry.reset(new urdf::Mesh);
    geometry->filename = filename;
    coll->geometry = geometry;
    link->collision_array.push_back(coll);
  }

  void RobotModelBuilder::addVirtualJoint(std::string parent_frame, std::string child_link, std::string type, std::string name)
    {
      srdf::Model::VirtualJoint new_virtual_joint;
      if (name == "")
        {
            new_virtual_joint.name_ = parent_frame + "-" + child_link + "-virtual_joint";
        }
      else
        {
          new_virtual_joint.name_ = name;
        }
        new_virtual_joint.type_ = type;
        new_virtual_joint.parent_frame_ = parent_frame;
        new_virtual_joint.child_link_ = child_link;
        srdf_writer_->virtual_joints_.push_back(new_virtual_joint);
    }

    void RobotModelBuilder::addGroupChain(std::string base_link, std::string tip_link, std::string name)
    {
        srdf::Model::Group new_group;
        if (name == "")
        {
            new_group.name_ = base_link + "-" + tip_link + "-chain-group";
        }
        else
        {
            new_group.name_ = name;
        }
        new_group.chains_.push_back(std::make_pair(base_link, tip_link));
        srdf_writer_->groups_.push_back(new_group);
    }

    void RobotModelBuilder::addGroup(std::vector<std::string> links, std::vector<std::string> joints, std::string name)
    {
        srdf::Model::Group new_group;
        new_group.name_ = name;
        new_group.links_ = links;
        new_group.joints_ = joints;
        srdf_writer_->groups_.push_back(new_group);
    }

    moveit::core::RobotModelPtr RobotModelBuilder::build()
    {
        moveit::core::RobotModelPtr robot_model;
        std::map<std::string, std::string> parent_link_tree;
        parent_link_tree.clear();

        try
        {
            urdf_model_->initTree(parent_link_tree);
        }
        catch (urdf::ParseError &e)
        {
            ROS_ERROR("Failed to build tree: %s", e.what());
            return robot_model;
        }

        // find the root link
        try
        {
            urdf_model_->initRoot(parent_link_tree);
        }
        catch(urdf::ParseError &e)
        {
            ROS_ERROR("Failed to find root link: %s", e.what());
            return robot_model;
        }
        srdf_writer_->updateSRDFModel(*urdf_model_);
        robot_model.reset(new moveit::core::RobotModel(urdf_model_, srdf_writer_->srdf_model_));
        return robot_model;
    }
}
}
