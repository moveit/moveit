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

#include "moveit_configuration_tools/widgets/compute_default_collisions_widget.h"
#include <planning_scene_monitor/planning_scene_monitor.h>
#include <boost/unordered_map.hpp>
#include <boost/assign.hpp>

static const std::string ROBOT_DESCRIPTION="robot_description";

// Boost mapping of reasons for disabling a link pair to strings
const boost::unordered_map<moveit_configuration_tools::DisabledReason, const char*> longReasonsToString = 
  boost::assign::map_list_of
  ( moveit_configuration_tools::NEVER, "Never in Collision" )
  ( moveit_configuration_tools::DEFAULT, "Collision by Default" )
  ( moveit_configuration_tools::ADJACENT, "Adjacent Links" )
  ( moveit_configuration_tools::ALWAYS, "Always in Collision" )
  ( moveit_configuration_tools::USER, "User Chosen" )
  ( moveit_configuration_tools::NOT_DISABLED, "");

ComputeDefaultCollisionsWidget::ComputeDefaultCollisionsWidget()
{
  // Basic widget container
  layout_ = new QVBoxLayout;

  // Top Label Area ------------------------------------------------

  // Page Title
  QLabel *page_title = new QLabel();
  page_title->setText("Optimize Self-Collision Checking");
  QFont page_title_font( "Arial", 18, QFont::Bold);
  page_title->setFont(page_title_font);
  layout_->addWidget( page_title);

  // Page Instructions
  QLabel *page_instructions = new QLabel();
  page_instructions->setText("The Default Self-Collision Matrix Generator will search for pairs of links "
                             "on the robot that can safely be disabled from collision checking, decreasing motion planning processing time. "
                             "These pairs of links are disabled they are always in collision, never in collision, collision in the robot's default position and when the links are adjacent to each other on the kinematic chain. Sampling density specifies how many random robot positions to check for self collision. Higher densities require more computation time.");
  page_instructions->setWordWrap(true);
  layout_->addWidget( page_instructions);

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
  collision_table_->setColumnWidth(0, 320);
  collision_table_->setColumnWidth(1, 320);
  collision_table_->setColumnWidth(2, 175);
  collision_table_->setColumnWidth(4, 60);
  collision_table_->setSortingEnabled(true);
  layout_->addWidget(collision_table_);

  // Table Headers
  QStringList titleList;
  titleList.append("Link A");
  titleList.append("Link B");
  titleList.append("Reason Disabled");
  titleList.append("CC Disabled");
  collision_table_->setHorizontalHeaderLabels(titleList);

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

  setLayout(layout_);

  setWindowTitle("Default Collision Matrix");
}

void ComputeDefaultCollisionsWidget::generateCollisionTable()
{
  // Disable controls on form
  disableControls(true);

  unsigned int collision_progress = 0; // shared variable with thread
  progress_bar_->setValue(collision_progress);  

  QApplication::processEvents(); // allow the progress bar to be shown


  //generateCollisionTableThread( &collision_progress );

  // Create thread to do actual work
  boost::thread workerThread( boost::bind( &ComputeDefaultCollisionsWidget::generateCollisionTableThread, 
                                           this, &collision_progress ));

  // Check interval
  boost::posix_time::seconds check_interval(1);  

  // Continually loop until threaded computation is finished
  while( collision_progress < 100 )
  {
    // Set updated progress value.
    progress_bar_->setValue(collision_progress);

    // Allow GUI thread to do its stuff
    QApplication::processEvents(); 

    // 1 second sleep
    boost::this_thread::sleep(check_interval);  
  }

  ROS_INFO("I am outputting");
  // Wait for thread to finish
  workerThread.join();
  ROS_INFO("Now  I am not outputting");

  // Load the results into the GUI
  loadCollisionTable();

  // Hide the progress bar
  disableControls(false); // enable everything else
}

void ComputeDefaultCollisionsWidget::generateCollisionTableThread( unsigned int *collision_progress )
{
  ROS_INFO("Inner thread");

  unsigned int num_trials = density_slider_->value() * 1000 + 1000; // scale to trials amount

  const bool verbose = true; // Output benchmarking and statistics
  const bool include_never_colliding = true;

  // Load robot description
  planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION);

  // Find the default collision matrix - all links that are allowed to collide
  link_pairs = moveit_configuration_tools::computeDefaultCollisions(psm.getPlanningScene(), collision_progress, 
                                                                    include_never_colliding, num_trials, verbose);
  
  // Output results to an XML file
  //moveit_configuration_tools::outputDisabledCollisionsXML( link_pairs );
  
  // End the progress bar loop
  *collision_progress = 100;

  ROS_INFO_STREAM("Thread complete " << link_pairs.size());
}


void ComputeDefaultCollisionsWidget::loadCollisionTable()
{
  // Show Progress Bar
  progress_bar_->setValue(0);  

  QApplication::processEvents(); // allow the progress bar to be shown
  progress_label_->setText("Loading table...");

  // Setup Collision Table
  collision_table_->clearContents();
  int row = 0;

  // Check if there are no disabled collisions (unprobable?)
  if(link_pairs.size() == 0)
  {
    collision_table_->setRowCount(1);
    QTableWidgetItem* no_collide = new QTableWidgetItem("No Link Pairs Of This Kind");
    collision_table_->setItem(0, 0, no_collide);
  }
  else
  {
    // The table will be populated, so indicate it on the button
    generate_button_->setText("Regenerate Default Collision Matrix");
  }

  int progress_counter = 0;

  for ( moveit_configuration_tools::LinkPairMap::const_iterator pair_it = link_pairs.begin(); 
        pair_it != link_pairs.end(); 
        ++pair_it)
  {
    // Add link pair row if 1) it is disabled from collision checking or 2) the SHOW ALL LINK PAIRS checkbox is checked
    if( pair_it->second.disable_check || collision_checkbox_->isChecked() ) 
    {
      collision_table_->setRowCount( row + 1 ); 
      
      // Create row elements
      QTableWidgetItem* linkA = new QTableWidgetItem( pair_it->first.first.c_str() ); 
      linkA->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

      QTableWidgetItem* linkB = new QTableWidgetItem( pair_it->first.second.c_str() );
      linkB->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

      QTableWidgetItem* reason = new QTableWidgetItem( longReasonsToString.at( pair_it->second.reason ) );

      reason->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

      QCheckBox* enable_box = new QCheckBox(collision_table_);
      if( pair_it->second.disable_check ) // Checked means no collision checking
      {
        enable_box->setChecked(true);
      } else 
      {
        enable_box->setChecked(false);
      }
      //connect(enable_box, SIGNAL(toggled(bool)), this, SLOT(toggleCheckBox(bool)));

      // Insert row elements into collision table
      collision_table_->setItem( row, 0, linkA);
      collision_table_->setItem( row, 1, linkB);
      collision_table_->setItem( row, 2, reason);
      collision_table_->setCellWidget( row, 3, enable_box); 
      
      // Increment row count
      ++row;
    }
    
    ++progress_counter; // for calculating progress bar

    if( progress_counter % 200 == 0 )
    {
      // Update Progress Bar
      progress_bar_->setValue( progress_counter * 100 / link_pairs.size() );  
      QApplication::processEvents(); // allow the progress bar to be shown
    }

  }

}

void ComputeDefaultCollisionsWidget::quit()
{
  QMessageBox messageBox;
  messageBox.setWindowTitle("Default Collision Matrix");
  messageBox.setText("Do you really want to quit?");
  messageBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
  messageBox.setDefaultButton(QMessageBox::No);
  if(messageBox.exec() == QMessageBox::Yes)
    QApplication::quit();
}

void ComputeDefaultCollisionsWidget::changeDensityLabel(int value)
{
  density_value_label_->setText( QString::number( value*1000 + 1000).append(" samples") );
}

void ComputeDefaultCollisionsWidget::disableControls(bool disable)
{
  controls_box_->setDisabled( disable );
  collision_table_->setDisabled( disable );
  collision_checkbox_->setDisabled( disable );
  save_button_->setDisabled( disable );

  if( disable )
  {
    progress_bar_->show(); // only show when computation begins
    progress_label_->show();
  }
  else
  {
    progress_label_->hide();
    progress_bar_->hide();
  }

  QApplication::processEvents(); // allow the progress bar to be shown
}

void ComputeDefaultCollisionsWidget::collisionCheckboxToggle()
{
  // Show Progress bar
  disableControls( true );

  // Update link_pairs data structure
  copyTableChanges();

  // Now update collision table with updates
  loadCollisionTable();

  // Hide Progress bar
  disableControls( false );
}

void ComputeDefaultCollisionsWidget::copyTableChanges()
{
  progress_label_->setText("Scanning table...");

  int progress_counter = 0;

  // Scan the table for all checkbox changes and save to data structure
  for(int i = 0; i < collision_table_->rowCount(); i++)
  {
    QCheckBox* box = dynamic_cast<QCheckBox*> (collision_table_->cellWidget(i, 3));
    if(box != NULL)
    {
      std::pair<std::string, std::string> link_pair;
      link_pair.first = collision_table_->item(i, 0)->text().toStdString();
      link_pair.second = collision_table_->item(i, 1)->text().toStdString();

      ROS_INFO_STREAM( link_pair.first << " " << link_pair.second );

      // Check if the checbox state has changed from original value
      if( link_pairs[ link_pair ].disable_check != box->isChecked() )
      {
        ROS_INFO_STREAM( link_pair.first << " has changed " );
        
        // Save the change
        link_pairs[ link_pair ].disable_check = box->isChecked();

        // Set to USER if its disabled but originally it was enabled
        if( link_pairs[ link_pair ].disable_check == true && 
            link_pairs[ link_pair ].reason != moveit_configuration_tools::NOT_DISABLED )
        {
          link_pairs[ link_pair ].reason = moveit_configuration_tools::USER;
          ROS_INFO_STREAM( link_pair.first << " is now USER " );
        }
      }
    }
    else
    {
      ROS_INFO(" NOTHING ");
    }

    ++progress_counter; // for calculating progress bar

    if( progress_counter % 200 == 0 )
    {
      // Update Progress Bar
      progress_bar_->setValue( progress_counter * 100 / link_pairs.size() );  
      QApplication::processEvents(); // allow the progress bar to be shown
    }

  }

}

void ComputeDefaultCollisionsWidget::saveToSRDF()
{
  // Show Progress bar
  disableControls( true );

  // Update link_pairs data structure
  copyTableChanges();

  // Do the save
  moveit_configuration_tools::outputDisabledCollisionsXML( link_pairs );

  // Hide Progress bar
  disableControls( false );
}
