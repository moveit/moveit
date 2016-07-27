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

#include <moveit/setup_assistant/tools/file_loader.h>

#include <fstream>
#include <streambuf>

namespace moveit_setup_assistant
{
bool isXacroFile(const std::string& path)
{
  // TODO: implement case-insensitive search
  return path.find(".xacro") != std::string::npos;
}

bool loadFileToString(std::string& buffer, const std::string& path)
{
  if (path.empty())
    return false;

  std::ifstream stream(path.c_str());
  if (!stream.good())
    return false;

  // Load the file to a string using an efficient memory allocation technique
  stream.seekg(0, std::ios::end);
  buffer.reserve(stream.tellg());
  stream.seekg(0, std::ios::beg);
  buffer.assign((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
  stream.close();

  return true;
}

bool loadXacroFileToString(std::string& buffer, const std::string& path, const std::vector<std::string>& xacro_args)
{
  if (path.empty())
    return false;

  std::string cmd = "rosrun xacro xacro ";
  for (std::vector<std::string>::const_iterator it = xacro_args.begin(); it != xacro_args.end(); ++it)
    cmd += *it + " ";
  cmd += path;

  FILE* pipe = popen(cmd.c_str(), "r");
  if (!pipe)
    return false;

  char pipe_buffer[128];
  while (!feof(pipe))
  {
    if (fgets(pipe_buffer, 128, pipe) != NULL)
      buffer += pipe_buffer;
  }
  pclose(pipe);

  return true;
}

bool loadXmlFileToString(std::string& buffer, const std::string& path, const std::vector<std::string>& xacro_args)
{
  if (isXacroFile(path))
  {
    return loadXacroFileToString(buffer, path, xacro_args);
  }
  else
  {
    return loadFileToString(buffer, path);
  }
}
}
