/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015,Fraunhofer IPA
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
 *   * Neither the name of Fraunhofer IPA nor the names of its
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

/* Author: Mathias Lüdtke */

#ifndef MOVEIT_MOVEIT_SETUP_ASSISTANT_TOOLS_FILE_LOADER_
#define MOVEIT_MOVEIT_SETUP_ASSISTANT_TOOLS_FILE_LOADER_

#include <string>
#include <vector>

namespace moveit_setup_assistant
{
/// detemine if given path points to a xacro file
bool isXacroFile(const std::string& path);

/// load file from given path into buffer
bool loadFileToString(std::string& buffer, const std::string& path);

/// run xacro with the given args on the file, return result in buffer
bool loadXacroFileToString(std::string& buffer, const std::string& path, const std::vector<std::string>& xacro_args);

/// helper that branches between loadFileToString() and loadXacroFileToString() based on result of isXacroFile()
bool loadXmlFileToString(std::string& buffer, const std::string& path, const std::vector<std::string>& xacro_args);
}

#endif
