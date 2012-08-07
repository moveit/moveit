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

#include <move_group_interface/move_group.h>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/join.hpp>
#include <ros/ros.h>

void printHelp(void)
{
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_cmdline", ros::init_options::AnonymousName);
  
  if (argc < 2)
  {
    std::cerr << "Usage: " << argv[0] << " <group_name>" << std::endl;
    return 1;
  }
  
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  move_group_interface::MoveGroup group(argv[1]);
  std::cout << std::endl;
  std::cout << "Waiting for commands. Type help to get a list of known commands." << std::endl << std::endl;
  
  while (ros::ok())
  {
    std::cout << "> ";
    std::string cmd;
    std::getline(std::cin, cmd); 
    if (!std::cin.good())
      break;
    boost::trim(cmd);
    if (cmd == "help")
      printHelp();
    else
      if (cmd == "quit" || cmd == "exit" || cmd == "q")
        break;
      else
        if (cmd.substr(0, 6) == "record")
        {
          std::string name = cmd.substr(6);
          boost::trim(name);
          group.rememberJointValues(name);
          std::cout << "Remembered current joint values under the name '" << name << "'." << std::endl;
        }
        else
          if (cmd.substr(0, 2) == "go")
          {
            std::string name = cmd.substr(2);
            boost::trim(name);
            if (group.setNamedTarget(name))
            {
              if (group.move())
                std::cout << "Moved to '" << name << "'" << std::endl;
              else
                std::cout << "Failed while moving to '" << name << "'" << std::endl;
            }
            else
              std::cout << "'" << name << "' is unknown" << std::endl;
          }
          else
            if (cmd.substr(0, 3) == "set")
            {
              std::string line = cmd.substr(3);
              boost::trim(line);
              std::stringstream s(line);
              std::string name; s >> name;
              std::vector<double> jv;
              std::string jvs;
              while (s.good() && !s.eof())
              {
                double v; s >> v;
                jvs = jvs + " " + boost::lexical_cast<std::string>(v);
                jv.push_back(v);
              }
              group.rememberJointValues(name, jv);
              std::cout << "Remembered specified joint values [" << jvs << " ] under the name '" << name << "'." << std::endl;
            }
            else
              if (cmd.substr(0, 4) == "rand")
	      {
		group.setRandomTarget();
		if (group.move())
		  std::cout << "Moved to random target" << std::endl;
		else
		  std::cout << "Failed while moving to random target." << std::endl;
	      }
	      else
	      if (cmd.substr(0, 3) == "del")
	      {      
                std::string name = cmd.substr(3);
                boost::trim(name);
                group.forgetJointValues(name);
              }
              else
                if (cmd == "show")
                {
                  const std::map<std::string, std::vector<double> >& jv = group.getRememberedJointValues();
                  for (std::map<std::string, std::vector<double> >::const_iterator it = jv.begin() ; it != jv.end() ; ++it)
                  {
                    std::cout << it->first << " = [";
                    std::string v;
                    for (std::size_t i = 0; i < it->second.size() ; ++i)
                      v = v + " " + boost::lexical_cast<std::string>(it->second[i]);
                    std::cout << v << " ]" << std::endl;
                  }
                }
                else
                {
                  const std::map<std::string, std::vector<double> >& jv = group.getRememberedJointValues();
                  std::map<std::string, std::vector<double> >::const_iterator it = jv.find(cmd);
                  if (it != jv.end())
                  {
                    std::string v;
                    for (std::size_t i = 0; i < it->second.size() ; ++i)
                      v = v + " " + boost::lexical_cast<std::string>(it->second[i]);
                    std::cout << v << std::endl;
                  }
                  else
                    std::cerr << "Unknown command: '" << cmd << "'" << std::endl;
                }
  }
  std::cout << "\nBye bye!" << std::endl;
  return 0;
}
