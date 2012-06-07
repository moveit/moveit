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


#ifndef MOVEIT_ROS_MOVEIT_CONFIGURATION_TOOLS_TOOLS_COMPUTE_LONGEST_VALID_SEGMENT_FRACTION_
#define MOVEIT_ROS_MOVEIT_CONFIGURATION_TOOLS_TOOLS_COMPUTE_LONGEST_VALID_SEGMENT_FRACTION_

#include <planning_scene/planning_scene.h>

namespace moveit_configuration_tools
{

/** \brief Compute the maximum length of a path segment that is considered valid if its endpoints are valid, as a fraction of the space extent, as computed by planning_models::KinematicModel::JointModel::getMaximumExtent(). The computation is performed for an entire kinematic state. */
double computeLongestValidSegmentFraction(const planning_scene::PlanningSceneConstPtr &parent_scene, unsigned int *progress, 
                                          const unsigned int trials = 10000, const bool verbose = false);

/** \brief Compute the maximum length of a path segment that is considered valid if its endpoints are valid, as a fraction of the space extent, as computed by planning_models::KinematicModel::JointModel::getMaximumExtent() .The computation is performed for a particular group */
double computeLongestValidSegmentFraction(const planning_scene::PlanningSceneConstPtr &parent_scene, const std::string &group, unsigned int *progress, 
                                          const unsigned int trials = 10000, const bool verbose = false);

}

#endif
