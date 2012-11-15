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
#ifndef MOVEIT_MOVEIT_SETUP_ASSISTANT_TOOLS_SRDF_WRITER_
#define MOVEIT_MOVEIT_SETUP_ASSISTANT_TOOLS_SRDF_WRITER_


#include <boost/shared_ptr.hpp>
#include <srdfdom/model.h> // use their struct datastructures

namespace moveit_setup_assistant
{

// ******************************************************************************************
// ******************************************************************************************
// Class
// ******************************************************************************************
// ******************************************************************************************

class SRDFWriter
{
public:
  // ******************************************************************************************
  // Public Functions
  // ******************************************************************************************
  /** 
   * Constructor
   */
  SRDFWriter();

  /** 
   * Destructor
   */
  ~SRDFWriter();
  
  /** 
   * Initialize the SRDF writer with an exisiting SRDF file (optional)
   * 
   * @param urdf_model a preloaded urdf model reference
   * @param srdf_string the text contents of an SRDF file
   * 
   * @return bool if initialization was successful
   */
  bool initString( const urdf::ModelInterface &robot_model, const std::string &srdf_string );

  /** 
   * Update the SRDF Model class using a new SRDF string
   * 
   * @param robot_model a loaded URDF model
   */
  void updateSRDFModel( const urdf::ModelInterface &robot_model );

  /** 
   * Generate SRDF XML of all contained data and save to file
   * 
   * @param file_path - string path location to save SRDF
   * @return bool - true if save was successful
   */
  bool writeSRDF( const std::string &file_path );

  /** 
   * Get a string of a generated SRDF document
   * 
   * @return string of XML of current SRDF contents
   */
  std::string getSRDFString();

  /** 
   * Generate SRDF XML of all contained data 
   * 
   * @return TinyXML document that contains current SRDF data in this class
   */
  TiXmlDocument generateSRDF();

  /** 
   * Generate XML for SRDF groups
   * 
   * @param root - TinyXML root element to attach sub elements to
   */
  void createGroupsXML( TiXmlElement *root );

  /** 
   * Generate XML for SRDF disabled collisions of robot link pairs
   * 
   * @param root  - TinyXML root element to attach sub elements to
   */
  void createDisabledCollisionsXML( TiXmlElement *root );

  /** 
   * Generate XML for SRDF group states of each joint's position
   * 
   * @param root  - TinyXML root element to attach sub elements to
   */
  void createGroupStatesXML( TiXmlElement *root );

  /** 
   * Generate XML for SRDF end effectors
   * 
   * @param root  - TinyXML root element to attach sub elements to
   */
  void createEndEffectorsXML( TiXmlElement *root );

  /** 
   * Generate XML for SRDF virtual joints
   * 
   * @param root  - TinyXML root element to attach sub elements to
   */
  void createVirtualJointsXML( TiXmlElement *root );
  
  /** 
   * Generate XML for SRDF passive joints
   * 
   * @param root  - TinyXML root element to attach sub elements to
   */
  void createPassiveJointsXML( TiXmlElement *root );
  
  // ******************************************************************************************
  // Group Datastructures
  // ******************************************************************************************

  std::vector<srdf::Model::Group>             groups_;
  std::vector<srdf::Model::GroupState>        group_states_;
  std::vector<srdf::Model::VirtualJoint>      virtual_joints_;
  std::vector<srdf::Model::EndEffector>       end_effectors_;
  std::vector<srdf::Model::DisabledCollision> disabled_collisions_;
  std::vector<srdf::Model::PassiveJoint>      passive_joints_;

  // Store the SRDF Model for updating the kinematic_model
  boost::shared_ptr<srdf::Model>              srdf_model_;

  // Robot name
  std::string robot_name_;

};

// ******************************************************************************************
// Typedef
// ******************************************************************************************

/// Create a shared pointer for passing this data object between widgets
typedef boost::shared_ptr<SRDFWriter> SRDFWriterPtr;


}

#endif
