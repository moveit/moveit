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

#include "moveit_configuration_tools/widgets/start_screen_widget.h"

static const std::string ROBOT_DESCRIPTION="robot_description";
static const std::string ROBOT_DESCRIPTION_SEMANTICS="robot_description_semantics";

class LoadPathWidget : public QFrame
{
public:
  LoadPathWidget( const std::string &title, const std::string &instructions )
  {
    // Set frame graphics
    setFrameShape(QFrame::StyledPanel);
    setFrameShadow(QFrame::Raised);
    setLineWidth(1);
    setMidLineWidth(0);

    // Basic widget container
    QVBoxLayout *layout = new QVBoxLayout();
    this->setLayout(layout);

    // Horizontal layout splitter
    QHBoxLayout *hlayout = new QHBoxLayout();
    
    // Widget Title
    QLabel *widget_title = new QLabel();
    widget_title->setText( title.c_str() );
    QFont widget_title_font( "Arial", 12, QFont::Bold);
    widget_title->setFont(widget_title_font);
    layout->addWidget( widget_title);
    layout->setAlignment( widget_title, Qt::AlignTop);

    // Widget Instructions
    QLabel *widget_instructions = new QLabel();
    widget_instructions->setText( instructions.c_str() );
    widget_instructions->setWordWrap(true);
    layout->addWidget( widget_instructions);
    layout->setAlignment( widget_instructions, Qt::AlignTop);    

    // Line Edit
    QLineEdit *path_box = new QLineEdit();
    hlayout->addWidget(path_box);

    // Button
    QPushButton *button = new QPushButton();
    button->setText("Browse");
    //button->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    //connect(button, SIGNAL(clicked()), this, SLOT(generateCollisionTable()));
    hlayout->addWidget(button);

    // Add horizontal layer to verticle layer
    layout->addLayout(hlayout);
  }
};

// ******************************************************************************************
// Start screen user interface for MoveIt Configuration Assistant
// ******************************************************************************************
StartScreenWidget::StartScreenWidget()
{
  // Basic widget container
  QVBoxLayout *layout = new QVBoxLayout( );
  // Horizontal layout splitter
  QHBoxLayout *hlayout = new QHBoxLayout( );
  // Left side of screen
  QVBoxLayout *left_layout = new QVBoxLayout( );
  // Left side controls
  QVBoxLayout *left_controls_layout = new QVBoxLayout( );
  // Right side of screen
  QVBoxLayout *right_layout = new QVBoxLayout( );
  
  this->setLayout(layout);
  layout->addLayout( hlayout );
  hlayout->addLayout( left_layout );
  hlayout->addLayout( right_layout );
  left_layout->addStretch(3);

  // Top Label Area ------------------------------------------------

  // Page Title
  QLabel *page_title = new QLabel( );
  page_title->setText("MoveIt Configuration Assistant");
  QFont page_title_font( "Arial", 18, QFont::Bold);
  page_title->setFont(page_title_font);
  left_layout->addWidget( page_title);
  left_layout->setAlignment( page_title, Qt::AlignTop);

  // Page Instructions
  QLabel *page_instructions = new QLabel( );
  page_instructions->setText("Welcome to the MoveIt Configuration Assistant! This tool will assist you in creating "
                             "a planning configuration for your robot. Start by specifying the package and file of "
                             "Unified Robot Description Format (URDF) file of your robot.");
  page_instructions->setWordWrap(true);
  page_instructions->setMargin(10);
  left_layout->addWidget( page_instructions);
  left_layout->setAlignment( page_instructions, Qt::AlignTop);

  // Left Controls Layout
  left_layout->addLayout( left_controls_layout );

  // Stack Path Box Area
  LoadPathWidget *package_path = new LoadPathWidget("MoveIt Configuration Stack Path", 
                                                    "Specify the location for a new or existing MoveIt configuration stack for your desired robot. Inside this stack the necessary packages will be added/edited to run MoveIt. Example stack name: 'moveit_pr2'");
  left_controls_layout->addWidget( package_path );
  left_controls_layout->setAlignment( package_path, Qt::AlignTop);

  // URDF File Dialog
  LoadPathWidget *urdf_file = new LoadPathWidget("URDF File",
                                                 "Specify the location for an existing Unified Robot Description Format (URDF) file for your robot. This configuration assistant will not make changes to a URDF.");
  left_controls_layout->addWidget( urdf_file );
  left_controls_layout->setAlignment( urdf_file, Qt::AlignTop);

  // SRDF File Dialog
  LoadPathWidget *srdf_file = new LoadPathWidget("SRDF File",
                                                 "Specify the location for a new or existing Semantic Robot Description Format (SRDF) file for your robot. This configuration assistant can create this file for you.");
  left_controls_layout->addWidget( srdf_file );
  left_controls_layout->setAlignment( srdf_file, Qt::AlignTop);

  // Filler Layer
  //QSpacerItem *blank_stretch = new QSpacerItem(100,500);
  //left_controls_layout->addSpacerItem( blank_stretch );

  // Right Image Area ----------------------------------------------
  QImage* image = new QImage( );

  if(chdir(ros::package::getPath("moveit_configuration_tools").c_str()) != 0)
  {
    ROS_ERROR("FAILED TO CHANGE PACKAGE TO moveit_configuration_tools");
  }
  if(!image->load("./resources/MoveIt_Setup_Asst_Sm.png"))
  {
    ROS_ERROR("FAILED TO LOAD ./resources/wizard.png");
  }
  QLabel* imageLabel = new QLabel( ); //start_page_);
  imageLabel->setPixmap(QPixmap::fromImage(*image));
  imageLabel->setMinimumHeight(10);  
  imageLabel->setMinimumWidth(10);  // TODO
  right_layout->addWidget(imageLabel);
  right_layout->setAlignment(imageLabel, Qt::AlignRight | Qt::AlignTop);

  
  /*
  // Top Button Area -----------------------------------------------
  controls_box_ = new QGroupBox();
  layout_->addWidget( controls_box_ );
  QHBoxLayout *controls_box_layout = new QHBoxLayout( controls_box_ );
  controls_box_layout->setAlignment(Qt::AlignTop | Qt::AlignLeft);

  // Slider Label
  QLabel *density_label = new QLabel();
  density_label->setText("Self Collision Sampling Density:");
  controls_box_layout->addWidget(density_label);

  // Slider
  density_slider_ = new QSlider();
  density_slider_->setTickPosition(QSlider::TicksBelow);
  density_slider_->setMinimum( 0 ); 
  density_slider_->setMaximum( 99 );
  density_slider_->setSingleStep( 10 );
  density_slider_->setPageStep( 50 );
  density_slider_->setSliderPosition( 9 ); // 10,000 is default
  density_slider_->setTickInterval( 10 );
  density_slider_->setOrientation( Qt::Horizontal );
  controls_box_layout->addWidget(density_slider_);
  connect(density_slider_, SIGNAL(valueChanged(int)), this, SLOT(changeDensityLabel(int)));

  // Slider Value Label
  density_value_label_ = new QLabel();
  density_value_label_->setMinimumWidth(100);
  controls_box_layout->addWidget(density_value_label_);
  changeDensityLabel( density_slider_->value() ); // initialize label with value

  // Generate Button
  generate_button_ = new QPushButton();
  generate_button_->setText("Generate Default Collision Matrix");
  generate_button_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  connect(generate_button_, SIGNAL(clicked()), this, SLOT(generateCollisionTable()));
  controls_box_layout->addWidget(generate_button_);

  // Progress Bar Area ---------------------------------------------

  // Progress Label
  progress_label_ = new QLabel();
  progress_label_->setText("Generating Default Collision Matrix");
  progress_label_->hide();
  layout_->addWidget(progress_label_);

  // Progress Bar
  progress_bar_ = new QProgressBar();
  progress_bar_->setMaximum(100);
  progress_bar_->setMinimum(0);
  progress_bar_->hide(); // only show when computation begins
  layout_->addWidget(progress_bar_); //,Qt::AlignCenter);

  // Table Area --------------------------------------------  

  // Table
  collision_table_ = new QTableWidget();
  collision_table_->setColumnCount(4);
  collision_table_->setColumnWidth(0, 330);
  collision_table_->setColumnWidth(1, 330);
  collision_table_->setColumnWidth(2, 85);
  collision_table_->setColumnWidth(3, 180);
  collision_table_->setSortingEnabled(true);
  connect(collision_table_, SIGNAL(cellChanged(int,int)), this, SLOT(toggleCheckBox(int,int)));
  layout_->addWidget(collision_table_);

  // Table Headers
  header_list_ = new QStringList();
  header_list_->append("Link A");
  header_list_->append("Link B");
  header_list_->append("Disabled");
  header_list_->append("Reason To Disable");
  collision_table_->setHorizontalHeaderLabels(*header_list_);

  // Bottom Area ----------------------------------------
  controls_box_bottom_ = new QGroupBox();
  layout_->addWidget( controls_box_bottom_ );
  QHBoxLayout *controls_box_bottom_layout = new QHBoxLayout( controls_box_bottom_ );

  // Checkbox
  collision_checkbox_ = new QCheckBox();
  collision_checkbox_->setText("Show Non-Disabled Link Pairs");
  connect(collision_checkbox_, SIGNAL(toggled(bool)), this, SLOT(collisionCheckboxToggle()));
  controls_box_bottom_layout->addWidget(collision_checkbox_);
  controls_box_bottom_layout->setAlignment(collision_checkbox_, Qt::AlignLeft);

  // Save Button
  save_button_ = new QPushButton();
  save_button_->setText("Save to SRDF");
  save_button_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  save_button_->setMinimumWidth(300);
  connect(save_button_, SIGNAL(clicked()), this, SLOT(saveToSRDF()));
  controls_box_bottom_layout->addWidget(save_button_);
  controls_box_bottom_layout->setAlignment(save_button_, Qt::AlignRight);

  // Quit
  //quitButton_ = new QPushButton("Quit");
  //connect(quitButton_, SIGNAL(clicked()), this, SLOT(quit()));
  //layout_->addWidget(quitButton_);

  // Does user need to save before exiting?
  unsaved_changes_ = false;

  */

  setWindowTitle("Start Screen");
}

