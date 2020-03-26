/*******************************************************************************
 *      Title     : status_codes.h
 *      Project   : moveit_jog_arm
 *      Created   : 2/25/2019
 *      Author    : Andy Zelenak
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Los Alamos National Security, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#pragma once

#include <string>
#include <unordered_map>

namespace moveit_jog_arm
{
enum StatusCode : int8_t
{
  NO_WARNING = 0,
  DECELERATE_FOR_SINGULARITY = 1,
  HALT_FOR_SINGULARITY = 2,
  DECELERATE_FOR_COLLISION = 3,
  HALT_FOR_COLLISION = 4,
  JOINT_BOUND = 5
};

const std::unordered_map<uint, std::string>
    JOG_ARM_STATUS_CODE_MAP({ { NO_WARNING, "No warnings" },
                              { DECELERATE_FOR_SINGULARITY, "Close to a singularity, decelerating" },
                              { HALT_FOR_SINGULARITY, "Very close to a singularity, emergency stop" },
                              { DECELERATE_FOR_COLLISION, "Close to a collision, decelerating" },
                              { HALT_FOR_COLLISION, "Collision detected, emergency stop" },
                              { JOINT_BOUND, "Close to a joint bound (position or velocity), halting" } });
}  // end namespace trackjoint
