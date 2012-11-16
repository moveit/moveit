//Software License Agreement (BSD License)

//Copyright (c) 2011, Willow Garage, Inc.
//All rights reserved.

//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions
//are met:

// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of Willow Garage, Inc. nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.

//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//POSSIBILITY OF SUCH DAMAGE.

#ifndef _INVERSE_KINEMATICS_SANITY_CHECKER_
#define _INVERSE_KINEMATICS_SANITY_CHECKER_

#include <random_numbers/random_numbers.h>
#include <kinematics_base/kinematics_base.h>
#include <planning_scene/planning_scene.h>

class InverseKinematicsSanityChecker {

public:

  InverseKinematicsSanityChecker(const std::map<std::string, kinematics::KinematicsBasePtr>& solver_map,
                                 const planning_scene::PlanningSceneConstPtr& scene);
  
  void runTest(const std::string& group,
               std::vector<std::pair<std::vector<double>, std::vector<double> > >& wrong_solutions,
               unsigned int samples = 10000,
               bool normalize_between_pi = false) const;

  std::vector<double> sampleJointValues(const std::vector<moveit_msgs::JointLimits>& limits) const;

protected:

  mutable random_numbers::RandomNumberGenerator rng_;
  std::map<std::string, kinematics::KinematicsBasePtr>  solver_map_;  
  planning_scene::PlanningSceneConstPtr planning_scene_;

};

#endif
