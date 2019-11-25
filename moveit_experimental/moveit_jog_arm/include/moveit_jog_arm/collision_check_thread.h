/*
     Title     : collision_check_thread.h
     Project   : moveit_jog_arm
     Created   : 1/11/2019
     Author    : Brian O'Neil, Andy Zelenak, Blake Anderson

BSD 3-Clause License

Copyright (c) 2019, Los Alamos National Security, LLC
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include "jog_arm_data.h"
#include "low_pass_filter.h"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

namespace moveit_jog_arm
{
class CollisionCheckThread
{
public:
  CollisionCheckThread(const moveit_jog_arm::JogArmParameters parameters,
                       moveit_jog_arm::JogArmShared& shared_variables, pthread_mutex_t& mutex,
                       const robot_model_loader::RobotModelLoaderPtr& model_loader_ptr);
};
}
