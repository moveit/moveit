/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage, Inc.
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

#ifndef MOVEIT_PLANNING_RDF_LOADER_
#define MOVEIT_PLANNING_RDF_LOADER_

#include <urdf/model.h>
#include <srdfdom/model.h>
#include <boost/shared_ptr.hpp>
#include <tinyxml.h>

namespace rdf_loader
{
/** @class RDFLoader
 *  @brief Default constructor
 *  @param robot_description The string name corresponding to the ROS param where the URDF is loaded*/
class RDFLoader
{ 
public:
  /** @brief Default constructor
   *  @param robot_description The string name corresponding to the ROS param where the URDF is loaded; the SRDF is assumed to be at the same param name + the "_semantic" suffix */
  RDFLoader(const std::string &robot_description = "robot_description");

   /** \brief Initialize the robot model from a string representation of the URDF and SRDF documents */
  RDFLoader(const std::string &urdf_string, const std::string &srdf_string);

  /** \brief Initialize the robot model from a parsed XML representation of the URDF and SRDF */
  RDFLoader(TiXmlDocument *urdf_doc, TiXmlDocument *srdf_doc);

  /** @brief Get the resolved parameter name for the robot description */
  const std::string& getRobotDescription() const
  {
    return robot_description_;
  }
  
  /** @brief Get the parsed URDF model*/
  const boost::shared_ptr<urdf::ModelInterface>& getURDF() const
  {
    return urdf_;
  }

  /** @brief Get the parsed SRDF model*/
  const boost::shared_ptr<srdf::Model>& getSRDF() const
  {
    return srdf_;
  }
  
private:
  
  std::string                             robot_description_;
  boost::shared_ptr<srdf::Model>          srdf_;
  boost::shared_ptr<urdf::ModelInterface> urdf_;
  
};

typedef boost::shared_ptr<RDFLoader> RDFLoaderPtr;
typedef boost::shared_ptr<const RDFLoader> RDFLoaderConstPtr;

}
#endif
