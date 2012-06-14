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

#include "start_screen_widget.h"


static const std::string ROBOT_DESCRIPTION="robot_description";
static const std::string ROBOT_DESCRIPTION_SEMANTICS="robot_description_semantics";

// ******************************************************************************************
// Start screen user interface for MoveIt Configuration Assistant
// ******************************************************************************************
StartScreenWidget::StartScreenWidget( QWidget* parent )
  :  QWidget( parent )
{
  // Basic widget container
  QVBoxLayout *layout = new QVBoxLayout( this );
  // Horizontal layout splitter 
  QHBoxLayout *hlayout = new QHBoxLayout( );
  // Left side of screen
  QVBoxLayout *left_layout = new QVBoxLayout( );
  // Right side of screen
  QVBoxLayout *right_layout = new QVBoxLayout( );

  // Top Label Area ------------------------------------------------

  // Page Title
  QLabel *page_title = new QLabel( this );
  page_title->setText("MoveIt Setup Assistant");
  this->setWindowTitle("MoveIt Setup Assistant"); // title of window
  QFont page_title_font( "Arial", 18, QFont::Bold );
  page_title->setFont(page_title_font);
  layout->addWidget( page_title);
  layout->setAlignment( page_title, Qt::AlignTop);
  
  // Page Instructions
  QLabel *page_instructions = new QLabel( this );
  page_instructions->setText("Welcome to the MoveIt Setup Assistant! These tools will assist you in creating "
                             "a planning configuration for your robot. Start by specifying the MoveIt configuration stack location, URDF file and SRDF file.");
  page_instructions->setWordWrap(true);
  page_instructions->setMargin(10);
  layout->addWidget( page_instructions );
  layout->setAlignment( page_instructions, Qt::AlignTop);
  

  // Path Box Area ----------------------------------------------------
  
  // Stack Path Dialog
  stack_path_ = new LoadPathWidget("MoveIt Configuration Stack Path", 
                                   "Specify the location for a new or existing MoveIt configuration stack for your desired robot. If you do not already have a stack created, this tool can create it for you - just provide the path to ROS workspace directory you would like the stack created. Inside this stack the necessary packages will be added/edited to run MoveIt. Example stack name: '~/ros/moveit_pr2'",
                                   true, this); // is directory
  left_layout->addWidget( stack_path_ );

  // URDF File Dialog
  urdf_file_ = new LoadPathWidget("URDF File",
                                  "Specify the location for an existing Unified Robot Description Format (URDF) file for your robot. This configuration assistant will not make changes to a URDF.", 
                                  false, true, this); // no directory, load only
  left_layout->addWidget( urdf_file_ );

  // SRDF File Dialog
  srdf_file_ = new LoadPathWidget("SRDF File",
                                  "Specify the location for a new or existing Semantic Robot Description Format (SRDF) file for your robot. This configuration assistant can create this file for you.",
                                  false, false, this); // no directory, save
  left_layout->addWidget( srdf_file_ );
  
  // Load settings box ---------------------------------------------

  btn_load_ = new QPushButton("Load Files", this);
  btn_load_->setMinimumWidth(200);
  btn_load_->setMinimumHeight(40);
  left_layout->addWidget( btn_load_ );  
  left_layout->setAlignment( btn_load_, Qt::AlignRight );
  connect( btn_load_, SIGNAL( clicked() ), this, SLOT( loadFiles() ) );
  
  // Right Image Area ----------------------------------------------
  QImage image;

  if(chdir(ros::package::getPath("moveit_configuration_tools").c_str()) != 0)
  {
    ROS_ERROR("FAILED TO CHANGE PACKAGE TO moveit_configuration_tools");
  }
  if(!image.load("./resources/MoveIt_Setup_Asst_Sm.png"))
  {
    ROS_ERROR("FAILED TO LOAD ./resources/wizard.png");
  }
  QLabel* imageLabel = new QLabel( this );
  imageLabel->setPixmap(QPixmap::fromImage(image));
  imageLabel->setMinimumHeight(493);  // size of image
  imageLabel->setMinimumWidth(450);
  right_layout->addWidget(imageLabel);
  right_layout->setAlignment(imageLabel, Qt::AlignRight | Qt::AlignCenter);


  // Final Layout Setup ---------------------------------------------
  // Alignment
  layout->setAlignment( Qt::AlignTop );
  hlayout->setAlignment( Qt::AlignTop );
  left_layout->setAlignment( Qt::AlignCenter );
  right_layout->setAlignment( Qt::AlignCenter );

  // Stretch
  left_layout->setSpacing( 30 );

  // Attach Layouts
  hlayout->addLayout( left_layout );
  hlayout->addLayout( right_layout );
  layout->addLayout( hlayout );

  this->setLayout(layout);
  
}

StartScreenWidget::~StartScreenWidget()
{

}

// ******************************************************************************************
// Load files to parameter server
// ******************************************************************************************
void StartScreenWidget::loadFiles()
{
  // check that URDF can be loaded
  std::cout << "LOADING FILE" << std::endl;

}

// ******************************************************************************************
// ******************************************************************************************
// Class for selecting files
// ******************************************************************************************
// ******************************************************************************************

// ******************************************************************************************
// Create the widget
// ******************************************************************************************
LoadPathWidget::LoadPathWidget( const std::string &title, const std::string &instructions, 
                                const bool dir_only, const bool load_only, QWidget* parent )
  : QFrame(parent), dir_only_(dir_only), load_only_(load_only)
{
  // Set frame graphics
  setFrameShape(QFrame::StyledPanel);
  setFrameShadow(QFrame::Raised);
  setLineWidth(1);
  setMidLineWidth(0);

  // Basic widget container
  QVBoxLayout *layout = new QVBoxLayout(this);
  setLayout(layout);

  // Horizontal layout splitter
  QHBoxLayout *hlayout = new QHBoxLayout();
    
  // Widget Title
  QLabel * widget_title = new QLabel(this);
  widget_title->setText( title.c_str() );
  QFont widget_title_font( "Arial", 12, QFont::Bold);
  widget_title->setFont(widget_title_font);
  layout->addWidget( widget_title);
  layout->setAlignment( widget_title, Qt::AlignTop);

  // Widget Instructions
  QLabel * widget_instructions = new QLabel(this);
  widget_instructions->setText( instructions.c_str() );
  widget_instructions->setWordWrap(true);
  layout->addWidget( widget_instructions);
  layout->setAlignment( widget_instructions, Qt::AlignTop);    

  // Line Edit
  path_box_ = new QLineEdit(this);
  hlayout->addWidget(path_box_);

  // Button
  button_ = new QPushButton(this);
  button_->setText("Browse");
  connect( button_, SIGNAL( clicked() ), this, SLOT( btn_file_dialog() ) );
  hlayout->addWidget(button_);

  // Add horizontal layer to verticle layer
  layout->addLayout(hlayout);
}

// ******************************************************************************************
// Load the file dialog
// ******************************************************************************************
void LoadPathWidget::btn_file_dialog()
{
  QString path;
  if( dir_only_ ) // only allow user to select a directory
  {
    path = QFileDialog::getExistingDirectory(this, "Open Stack Directory", path_box_->text(),
                                             QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
  }
  else // only allow user to select file
  {
    QString start_path;
    // smart load: open file dialog in location of stack directory
    if( path_box_->text() != "" )
    {
      start_path = path_box_->text();
    }
    else
    {
      // Temp pointers used for casting and accessing parent widget elements
      StartScreenWidget *my_parent = qobject_cast< StartScreenWidget* >( this->parentWidget() );
      LoadPathWidget *my_stack_path = qobject_cast< LoadPathWidget* >( my_parent->stack_path_ );
      LoadPathWidget *my_urdf_path = qobject_cast< LoadPathWidget* >( my_parent->urdf_file_ );

      // Check if the urdf file was already loaded
      if( my_urdf_path->path_box_->text() != "" ) // it has text
      {
        start_path = my_urdf_path->path_box_->text();
      }
      else
      {
        start_path = my_stack_path->path_box_->text();	
      }
    }
    if( load_only_ )
    {
      path = QFileDialog::getOpenFileName(this, "Open File", start_path, "");
    }
    else
    {
      path = QFileDialog::getSaveFileName(this, "Create/Load File", start_path, "" );
    }
  }
  
  // check they did not press cancel
  if (path != NULL)
    path_box_->setText( path );
}

