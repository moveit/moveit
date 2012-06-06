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

#ifndef MOVEIT_ROS_MOVEIT_CONFIGURATION_TOOLS_TOOLS_COMPUTE_DEFAULT_COLLISIONS_
#define MOVEIT_ROS_MOVEIT_CONFIGURATION_TOOLS_TOOLS_COMPUTE_DEFAULT_COLLISIONS_

#include <planning_scene/planning_scene.h>
//#include <map>
//#include <vector>
//#include <string>

namespace moveit_configuration_tools
{

/**
 * \brief Generate an adjacency list of links that are always and never in collision, to speed up collision detection
 * \param parent_scene A reference to the robot in the planning scene
 * \param include_never_colliding Optional flag to disable the check for links that are never in collision
 * \param trials Optional ability to set the number random collision checks that are made. Increase the probability of correctness
 * \return Adj List of unique set of pairs of links in string-based form
 */
std::map<std::string, std::set<std::string> > 
computeDefaultCollisions(const planning_scene::PlanningSceneConstPtr &parent_scene, unsigned int *progress, 
                         const bool include_never_colliding = true, const unsigned int trials = 10000, const bool verbose = false);

/**
 * \brief Generate xml format of disabled links for use in an SRDF
 * \param Adj List of unique set of pairs of links in string-based form 
 */
void outputDisabledCollisionsXML(const std::map<std::string, std::set<std::string> > &disabled_links);
}

#endif
