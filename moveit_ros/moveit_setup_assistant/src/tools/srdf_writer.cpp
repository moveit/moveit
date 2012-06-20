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

#include <tinyxml.h>
#include <moveit_setup_assistant/tools/srdf_writer.h>

namespace moveit_setup_assistant
{

// ******************************************************************************************
// Constructor
// ******************************************************************************************
SRDFWriter::SRDFWriter()
{
}

// ******************************************************************************************
// Destructor
// ******************************************************************************************
SRDFWriter::~SRDFWriter()
{
}

// ******************************************************************************************
// Load SRDF data from a pre-populated string
// ******************************************************************************************


bool SRDFWriter::initString( const urdf::ModelInterface &robot_model, const std::string &srdf_string )
{
  // Load from SRDF Model
  srdf::Model srdf_model;

  // Error check
  if( !srdf_model.initString( robot_model, srdf_string ) )
  {
    return false; // error loading file. improper format?
  }

  // Copy all read-only data from srdf model to this object
  disabled_collisions_ = srdf_model.getDisabledCollisionPairs();
  groups_ = srdf_model.getGroups();
  virtual_joints_ = srdf_model.getVirtualJoints();
  end_effectors_ = srdf_model.getEndEffectors();
  group_states_ = srdf_model.getGroupStates();

  return true;
}
// ******************************************************************************************
// 
// ******************************************************************************************



// ******************************************************************************************
// 
// ******************************************************************************************



// ******************************************************************************************
// 
// ******************************************************************************************

}
