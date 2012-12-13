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
#include <ros/console.h>
#include <moveit/setup_assistant/tools/srdf_writer.h>

namespace moveit_setup_assistant
{

// ******************************************************************************************
// Constructor
// ******************************************************************************************
SRDFWriter::SRDFWriter()
{
  // Intialize the SRDF model
  srdf_model_.reset( new srdf::Model() );
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
  // Error check
  if( !srdf_model_->initString( robot_model, srdf_string ) )
  {
    return false; // error loading file. improper format?
  }

  // Copy all read-only data from srdf model to this object
  disabled_collisions_ = srdf_model_->getDisabledCollisionPairs();
  groups_ = srdf_model_->getGroups();
  virtual_joints_ = srdf_model_->getVirtualJoints();
  end_effectors_ = srdf_model_->getEndEffectors();
  group_states_ = srdf_model_->getGroupStates();
  passive_joints_ = srdf_model_->getPassiveJoints();

  // Copy the robot name b/c the root xml element requires this attribute
  robot_name_ = robot_model.getName();

  return true;
}

// ******************************************************************************************
// Update the SRDF Model class using a new SRDF string
// ******************************************************************************************
void SRDFWriter::updateSRDFModel( const urdf::ModelInterface &robot_model )
{
  // Get an up to date SRDF String
  const std::string srdf_string = getSRDFString();

  // Error check
  if( !srdf_model_->initString( robot_model, srdf_string ) )
  {
    ROS_ERROR( "Unable to update the SRDF Model" );
    exit(1);
  }
}

// ******************************************************************************************
// Save to file a generated SRDF document
// ******************************************************************************************
bool SRDFWriter::writeSRDF( const std::string &file_path )
{
  // Generate the SRDF
  TiXmlDocument document = generateSRDF();

  // Save to file
  return document.SaveFile( file_path );
}

// ******************************************************************************************
// Get a string of a generated SRDF document
// ******************************************************************************************
std::string SRDFWriter::getSRDFString()
{
  // Generate the SRDF
  TiXmlDocument document = generateSRDF();

  // Setup printer
  TiXmlPrinter printer;
  printer.SetIndent( "    " );
  document.Accept( &printer );
  
  // Return string
  return printer.CStr();
}

// ******************************************************************************************
// Generate SRDF XML of all contained data 
// ******************************************************************************************
TiXmlDocument SRDFWriter::generateSRDF()
{
  TiXmlDocument document;
  TiXmlDeclaration* decl = new TiXmlDeclaration( "1.0", "", "" );  
  document.LinkEndChild( decl ); 
  
  // Convenience comments
  TiXmlComment * comment = new TiXmlComment( "This does not replace URDF, and is not an extension of URDF.\n    This is a format for representing semantic information about the robot structure.\n    A URDF file must exist for this robot as well, where the joints and the links that are referenced are defined\n" );
  document.LinkEndChild( comment );  
  
  // Root
  TiXmlElement* robot_root = new TiXmlElement("robot");
  robot_root->SetAttribute("name", robot_name_ ); // robot name
  document.LinkEndChild( robot_root );

  // Add Groups
  createGroupsXML( robot_root );
  
  // Add Group States
  createGroupStatesXML( robot_root );

  // Add End Effectors
  createEndEffectorsXML( robot_root );

  // Add Virtual Joints
  createVirtualJointsXML( robot_root );  

  // Add Passive Joints
  createPassiveJointsXML( robot_root );  

  // Add Disabled Collisions
  createDisabledCollisionsXML( robot_root );

  // Save
  return document;
}

// ******************************************************************************************
// Generate XML for SRDF groups
// ******************************************************************************************
void SRDFWriter::createGroupsXML( TiXmlElement *root )
{
  // Convenience comments
  if( groups_.size() ) // only show comments if there are corresponding elements
  {
    TiXmlComment *comment;
    comment = new TiXmlComment( "GROUPS: Representation of a set of joints and links. This can be useful for specifying DOF to plan for, defining arms, end effectors, etc" );  
    root->LinkEndChild( comment );  
    comment = new TiXmlComment( "LINKS: When a link is specified, the parent joint of that link (if it exists) is automatically included" );
    root->LinkEndChild( comment );  
    comment = new TiXmlComment( "JOINTS: When a joint is specified, the child link of that joint (which will always exist) is automatically included" );
    root->LinkEndChild( comment );  
    comment = new TiXmlComment( "CHAINS: When a chain is specified, all the links along the chain (including endpoints) are included in the group. Additionally, all the joints that are parents to included links are also included. This means that joints along the chain and the parent joint of the base link are included in the group");
    root->LinkEndChild( comment );
    comment = new TiXmlComment( "SUBGROUPS: Groups can also be formed by referencing to already defined group names" );
    root->LinkEndChild( comment );  
  }

  // Loop through all of the top groups
  for( std::vector<srdf::Model::Group>::iterator group_it = groups_.begin(); 
       group_it != groups_.end();  ++group_it )
  {
    
    // Create group element
    TiXmlElement *group = new TiXmlElement("group");
    group->SetAttribute("name", group_it->name_ ); // group name
    root->LinkEndChild(group);

    // LINKS
    for( std::vector<std::string>::const_iterator link_it = group_it->links_.begin();
         link_it != group_it->links_.end(); ++link_it )
    {
      TiXmlElement *link = new TiXmlElement("link");
      link->SetAttribute("name", *link_it ); // link name
      group->LinkEndChild( link );
    }

    // JOINTS
    for( std::vector<std::string>::const_iterator joint_it = group_it->joints_.begin();
         joint_it != group_it->joints_.end(); ++joint_it )
    {
      TiXmlElement *joint = new TiXmlElement("joint");
      joint->SetAttribute("name", *joint_it ); // joint name
      group->LinkEndChild( joint );
    }

    // CHAINS
    for( std::vector<std::pair<std::string,std::string> >::const_iterator chain_it = group_it->chains_.begin();
         chain_it != group_it->chains_.end(); ++chain_it )
    {
      TiXmlElement *chain = new TiXmlElement("chain");
      chain->SetAttribute("base_link", chain_it->first );
      chain->SetAttribute("tip_link", chain_it->second );
      group->LinkEndChild( chain );
    }

    // SUBGROUPS
    for( std::vector<std::string>::const_iterator subgroup_it = group_it->subgroups_.begin();
         subgroup_it != group_it->subgroups_.end(); ++subgroup_it )
    {
      TiXmlElement *subgroup = new TiXmlElement("group");
      subgroup->SetAttribute("name", *subgroup_it ); // subgroup name
      group->LinkEndChild( subgroup );
    }
           
  }
}

// ******************************************************************************************
// Generate XML for SRDF disabled collisions of robot link pairs
// ******************************************************************************************
void SRDFWriter::createDisabledCollisionsXML( TiXmlElement *root )
{
  // Convenience comments
  if( disabled_collisions_.size() ) // only show comments if there are corresponding elements
  {
    TiXmlComment *comment = new TiXmlComment();
    comment->SetValue( "DISABLE COLLISIONS: By default it is assumed that any link of the robot could potentially come into collision with any other link in the robot. This tag disables collision checking between a specified pair of links. " );  
    root->LinkEndChild( comment );  
  }

  for ( std::vector<srdf::Model::DisabledCollision>::const_iterator pair_it = disabled_collisions_.begin();
        pair_it != disabled_collisions_.end() ; ++pair_it)
  {
    // Create new element for each link pair
    TiXmlElement *link_pair = new TiXmlElement("disable_collisions");
    link_pair->SetAttribute("link1", pair_it->link1_ );
    link_pair->SetAttribute("link2", pair_it->link2_ );
    link_pair->SetAttribute("reason", pair_it->reason_ );

    root->LinkEndChild( link_pair );
  }
}

// ******************************************************************************************
// Generate XML for SRDF group states
// ******************************************************************************************
void SRDFWriter::createGroupStatesXML( TiXmlElement *root )
{
  // Convenience comments
  if( group_states_.size() ) // only show comments if there are corresponding elements
  {
    TiXmlComment *comment = new TiXmlComment();
    comment->SetValue( "GROUP STATES: Purpose: Define a named state for a particular group, in terms of joint values. This is useful to define states like 'folded arms'" );
    root->LinkEndChild( comment );  
  }

  for ( std::vector<srdf::Model::GroupState>::const_iterator state_it = group_states_.begin();
        state_it != group_states_.end() ; ++state_it)
  {
    // Create new element for each group state
    TiXmlElement *state = new TiXmlElement("group_state");
    state->SetAttribute("name", state_it->name_ );
    state->SetAttribute("group", state_it->group_ );
    root->LinkEndChild( state );

    // Add all joints
    for( std::map<std::string, std::vector<double> >::const_iterator value_it = state_it->joint_values_.begin();
         value_it != state_it->joint_values_.end(); ++value_it )
    {
      TiXmlElement *joint = new TiXmlElement("joint");
      joint->SetAttribute("name", value_it->first ); // joint name
      joint->SetDoubleAttribute("value", value_it->second[0] ); // joint value

      // TODO: use the vector to support multi-DOF joints
      state->LinkEndChild( joint );
    }
  }
}

// ******************************************************************************************
// Generate XML for SRDF end effectors
// ******************************************************************************************
void SRDFWriter::createEndEffectorsXML( TiXmlElement *root )
{
  // Convenience comments
  if( end_effectors_.size() ) // only show comments if there are corresponding elements
  {
    TiXmlComment *comment = new TiXmlComment();
    comment->SetValue( "END EFFECTOR: Purpose: Represent information about an end effector." );
    root->LinkEndChild( comment );  
  }

  for ( std::vector<srdf::Model::EndEffector>::const_iterator effector_it = end_effectors_.begin();
        effector_it != end_effectors_.end() ; ++effector_it)
  {
    // Create new element for each link pair
    TiXmlElement *effector = new TiXmlElement("end_effector");
    effector->SetAttribute("name", effector_it->name_ );
    effector->SetAttribute("parent_link", effector_it->parent_link_ );
    effector->SetAttribute("group", effector_it->component_group_ );
    if (!effector_it->parent_group_.empty())
      effector->SetAttribute("parent_group", effector_it->parent_group_ );
    root->LinkEndChild( effector );
  }
}

// ******************************************************************************************
// Generate XML for SRDF virtual joints
// ******************************************************************************************
void SRDFWriter::createVirtualJointsXML( TiXmlElement *root )
{
  // Convenience comments
  if( virtual_joints_.size() ) // only show comments if there are corresponding elements
  {
    TiXmlComment *comment = new TiXmlComment();
    comment->SetValue( "VIRTUAL JOINT: Purpose: this element defines a virtual joint between a robot link and an external frame of reference (considered fixed with respect to the robot)" );
    root->LinkEndChild( comment );  
  }

  for ( std::vector<srdf::Model::VirtualJoint>::const_iterator virtual_it = virtual_joints_.begin();
        virtual_it != virtual_joints_.end() ; ++virtual_it)
  {
    // Create new element for each link pair
    TiXmlElement *virtual_joint = new TiXmlElement("virtual_joint");
    virtual_joint->SetAttribute("name", virtual_it->name_ );
    virtual_joint->SetAttribute("type", virtual_it->type_ );
    virtual_joint->SetAttribute("parent_frame", virtual_it->parent_frame_ );
    virtual_joint->SetAttribute("child_link", virtual_it->child_link_ );

    root->LinkEndChild( virtual_joint );
  }
}

void SRDFWriter::createPassiveJointsXML( TiXmlElement *root )
{
  if ( passive_joints_.size() )
  {
    TiXmlComment *comment = new TiXmlComment();
    comment->SetValue( "PASSIVE JOINT: Purpose: this element is used to mark joints that are not actuated" );
    root->LinkEndChild( comment );  
  }
  for ( std::vector<srdf::Model::PassiveJoint>::const_iterator p_it = passive_joints_.begin();
        p_it != passive_joints_.end() ; ++p_it)
  {
    // Create new element for each link pair
    TiXmlElement *p_joint = new TiXmlElement("passive_joint");
    p_joint->SetAttribute("name", p_it->name_ );
    root->LinkEndChild( p_joint );
  }
}


}
