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
#include <moveit_configuration_tools/tools/compute_default_collisions.h>
#include <planning_scene_monitor/planning_scene_monitor.h>

static const std::string ROBOT_DESCRIPTION="robot_description";

ComputeDefaultCollisionsWidget::ComputeDefaultCollisionsWidget()
{
  // Basic widget container
  layout_ = new QVBoxLayout;

  // Button
  generateButton_ = new QPushButton();
  generateButton_->setText("Generate Default Collision Matrix");
  generateButton_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  connect(generateButton_, SIGNAL(clicked()), this, SLOT(generateCollisionTable()));
  layout_->addWidget(generateButton_);

  // Table
  collision_table_ = new QTableWidget();
  collision_table_->clear();
  collision_table_->setColumnCount(4);
  collision_table_->setColumnWidth(0, 300);
  collision_table_->setColumnWidth(1, 300);
  collision_table_->setColumnWidth(2, 200);
  collision_table_->setColumnWidth(4, 50);
  //collision_table_->setResizeMode( QHeaderView::Interactive );
  layout_->addWidget(collision_table_);

  // Table Headers
  QStringList titleList;
  titleList.append("Link A");
  titleList.append("Link B");
  titleList.append("Reason");
  titleList.append("CC Disabled");
  collision_table_->setHorizontalHeaderLabels(titleList);

  // Quit
  //quitButton_ = new QPushButton("Quit");
  //connect(quitButton_, SIGNAL(clicked()), this, SLOT(quit()));
  //layout_->addWidget(quitButton_);

  setLayout(layout_);

  setWindowTitle("Default Collision Matrix");
}

void ComputeDefaultCollisionsWidget::generateCollisionTable()
{
  ROS_INFO("Generating");

  //QProgressDialog* progress = new QProgressDialog("Fetching data...", "Cancel", 0, 100);
  //progress->setWindowModality(Qt::WindowModal);


  //QApplication a(argc, argv);
  //ProgressDialog w;
  //w.showMaximized();
 
 
  QVBoxLayout* layout = new QVBoxLayout;
  QWidget*  win = new QWidget;
 
  //The minimum and maximum is the number of steps in the operation for which this progress dialog shows progress.
  //for example here 0 and 100.
  QProgressDialog* progress = new QProgressDialog("Generating Default Collision Matrix...", "", 0, 100);
  progress->setWindowModality(Qt::WindowModal);
  progress->setCancelButton(0);
  progress->setWindowFlags(Qt::WindowTitleHint | Qt::WindowMinimizeButtonHint);

  layout->addWidget(progress,Qt::AlignCenter);
  win->setLayout(layout);
  
  unsigned int collision_progress = 0; // shared variable with thread
  progress->setValue(collision_progress);  
  win->show();
  QApplication::processEvents(); // allow the progress bar to be shown

  // Create data structure for resulting disabled links list
  std::map<std::string, std::set<std::string> > disabled_links;

  // Create thread to do actual work
  boost::thread workerThread( boost::bind( &ComputeDefaultCollisionsWidget::generateCollisionTableThread, 
                                           this, &collision_progress, &disabled_links )); 

  // Check interval
  boost::posix_time::seconds check_interval(1);  

  while( collision_progress < 100 )
  {
    ROS_INFO_STREAM("CHECKING " << collision_progress << "%");

    // Set updated progress value.
    progress->setValue(collision_progress);
    //win->show();

    // Allow GUI thread to do its stuff
    QApplication::processEvents(); 

    // 1 second sleep
    boost::this_thread::sleep(check_interval);  
  }

  // Wait for thread to finish
  workerThread.join();

  // Hide progress bar
  win->hide();

  ROS_INFO("Thread joined");

  // Insert disabled collisions into table
  //collision_table_->setRowCount((int)collision_pairs_.size()+(int)not_in_collision.size());
  if(disabled_links.size() == 0)
  {
    collision_table_->setRowCount(1);
    QTableWidgetItem* no_collide = new QTableWidgetItem("No Link Pairs Of This Kind");
    collision_table_->setItem(0, 0, no_collide);
  }


  int row;

  for (std::map<std::string, std::set<std::string> >::const_iterator it = disabled_links.begin() ; it != disabled_links.end() ; ++it)
  {    
    // disable all connected links to current link by looping through them
    for (std::set<std::string>::const_iterator link2_it = it->second.begin(); 
         link2_it != it->second.end(); 
         ++link2_it)
    {
      // Increment the row count (is this necessary?)
      row = collision_table_->rowCount();
      collision_table_->setRowCount( row + 1 ); 

      QTableWidgetItem* linkA = new QTableWidgetItem( it->first.c_str() ); 
      linkA->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

      QTableWidgetItem* linkB = new QTableWidgetItem( (*link2_it).c_str() );
      linkB->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

      QTableWidgetItem* reason = new QTableWidgetItem( "Dunno" );
      reason->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

      QCheckBox* enable_box = new QCheckBox(collision_table_);
      if(true) // Checked means no collision checking
      {
        enable_box->setChecked(true);
      } else 
      {
        enable_box->setChecked(false);
      }
      //connect(enable_box, SIGNAL(toggled(bool)), this, SLOT(tableChanged()));

      collision_table_->setItem( row, 0, linkA);
      collision_table_->setItem( row, 1, linkB);
      collision_table_->setItem( row, 2, reason);
      collision_table_->setCellWidget( row, 3, enable_box); 

    }
  }
  //collision_table_->resizeSection();

}

void ComputeDefaultCollisionsWidget::generateCollisionTableThread( unsigned int *collision_progress, 
                                                                   std::map<std::string, std::set<std::string> > *disabled_links )
{
  ROS_INFO("Inner thread");

  unsigned int num_trials = 1000;

  const bool verbose = false; // Output benchmarking and statistics
  const bool include_never_colliding = true;
  // Load robot description
  planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION);

  // Find the default collision matrix - all links that are allowed to collide
  *disabled_links = moveit_configuration_tools::computeDefaultCollisions(psm.getPlanningScene(), collision_progress, 
                                                                         include_never_colliding, num_trials, verbose);
  
  // Output results to an XML file
  //moveit_configuration_tools::outputDisabledCollisionsXML( disabled_links );
  
  *collision_progress = 100;

  ROS_INFO_STREAM("Thread complete " << disabled_links->size());
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
