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

#ifndef MOVEIT_CORE_UTILS_TEST_
#define MOVEIT_CORE_UTILS_TEST_

/** \file test_utils.h
 *  \brief convenience functions and classes used for testing and making simple robot models.
 */

#include <urdf_parser/urdf_parser.h>
#include <urdf_model/model.h>
#include <urdf_model/joint.h>
#include <srdfdom/srdf_writer.h>
#include <urdf/model.h>
#include <fstream>
#include <boost/filesystem/path.hpp>
#include <moveit_resources/config.h>
#include <moveit/robot_model/robot_model.h>
#include <geometry_msgs/Pose.h>

namespace moveit
{
namespace core
{

    /** \brief Loads a robot from moveit_resources.
     * \param[in] robot_name The name of the robot package in moveit_resources to load.
     *            For example, "panda_description", or "faunc_description".
     * \returns a RobotModel constructed from robot_name's URDF and SRDF.
     */
    moveit::core::RobotModelPtr loadRobot(std::string robot_name);

    /** \brief Easily build different robot models for testing.
     *  Should only be used for testing, as many model fields will be left at their defaults.
     */
    class RobotModelBuilder
    {
    public:
        RobotModelBuilder(std::string name, std::string base_link_name);

        /**
         * Adds new joints and links to the robot model.
         * Example: builder.add("a->b->c", "prismatic");
         */
        void add(std::string section, std::string type, std::vector<geometry_msgs::Pose> origins = {});

        void addLinkMesh(std::string link_name, std::string filename, geometry_msgs::Pose origin);
        void addLinkBox(std::string link_name, geometry_msgs::Point size, geometry_msgs::Pose origin);

        void addVirtualJoint(std::string parent_frame, std::string child_link, std::string type, std::string name="");
        void addGroupChain(std::string base_link, std::string tip_link, std::string name="");
        void addGroup(std::vector<std::string> links, std::vector<std::string> joints, std::string name);

        /**
         * Builds and returns the specified robot model.
         */
        moveit::core::RobotModelPtr build();

    private:
        urdf::ModelInterfaceSharedPtr urdf_model_;
        srdf::SRDFWriterPtr srdf_writer_;
    };

}
}

#endif
