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
 *   * Neither the name of Willow Garage nor the names of its
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

#include <QHBoxLayout>
#include <QMessageBox>
#include <QProgressDialog>
#include <QString>
#include <QFont>
#include <QApplication>
#include "default_collisions_widget.h"
#include "collision_matrix_model.h"
#include "collision_linear_model.h"
#include <ros/console.h>

namespace moveit_setup_assistant
{
// ******************************************************************************************
// User interface for editing the default collision matrix list in an SRDF
// ******************************************************************************************
DefaultCollisionsWidget::DefaultCollisionsWidget(QWidget *parent, MoveItConfigDataPtr config_data)
  : SetupScreenWidget(parent), config_data_(config_data), model_(NULL), selection_model_(NULL)
{
  // Basic widget container
  layout_ = new QVBoxLayout(this);

  // Top Label Area ------------------------------------------------
  HeaderWidget *header = new HeaderWidget(
      "Optimize Self-Collision Checking",
      "The Default Self-Collision Matrix Generator will search for pairs of links on the robot that can safely be "
      "disabled from collision checking, decreasing motion planning processing time. These pairs of links are disabled "
      "when they are always in collision, never in collision, in collision in the robot's default position or when the "
      "links are adjacent to each other on the kinematic chain. Sampling density specifies how many random robot "
      "positions to check for self collision. Higher densities require more computation time.",
      this);
  layout_->addWidget(header);

  // Top Button Area -----------------------------------------------
  controls_box_ = new QGroupBox(this);
  layout_->addWidget(controls_box_);
  QHBoxLayout *controls_box_layout = new QHBoxLayout(controls_box_);
  controls_box_layout->setAlignment(Qt::AlignTop | Qt::AlignLeft);

  // Slider Label
  QLabel *density_left_label = new QLabel(this);
  density_left_label->setText("Sampling Density: Low");
  controls_box_layout->addWidget(density_left_label);

  // Slider
  density_slider_ = new QSlider(this);
  density_slider_->setTickPosition(QSlider::TicksBelow);
  density_slider_->setMinimum(0);
  density_slider_->setMaximum(99);
  density_slider_->setSingleStep(10);
  density_slider_->setPageStep(50);
  density_slider_->setSliderPosition(9);  // 10,000 is default
  density_slider_->setTickInterval(10);
  density_slider_->setOrientation(Qt::Horizontal);
  controls_box_layout->addWidget(density_slider_);
  connect(density_slider_, SIGNAL(valueChanged(int)), this, SLOT(changeDensityLabel(int)));

  // Slider Right Label
  QLabel *density_right_label = new QLabel(this);
  density_right_label->setText("High   ");
  controls_box_layout->addWidget(density_right_label);

  // Slider Value Label
  density_value_label_ = new QLabel(this);
  density_value_label_->setMinimumWidth(50);
  controls_box_layout->addWidget(density_value_label_);
  changeDensityLabel(density_slider_->value());  // initialize label with value

  // Generate Button
  btn_generate_ = new QPushButton(this);
  btn_generate_->setText("&Generate Collision Matrix");
  btn_generate_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
  connect(btn_generate_, SIGNAL(clicked()), this, SLOT(generateCollisionTable()));
  layout_->addWidget(btn_generate_);
  layout_->setAlignment(btn_generate_, Qt::AlignRight);

  // Progress Bar Area ---------------------------------------------

  // Progress Label
  progress_label_ = new QLabel(this);
  progress_label_->setText("Generating Default Collision Matrix");
  progress_label_->hide();
  layout_->addWidget(progress_label_);

  // Progress Bar
  progress_bar_ = new QProgressBar(this);
  progress_bar_->setMaximum(100);
  progress_bar_->setMinimum(0);
  progress_bar_->hide();              // only show when computation begins
  layout_->addWidget(progress_bar_);  //,Qt::AlignCenter);

  // Table Area --------------------------------------------

  // Table
  collision_table_ = new QTableView(this);
  layout_->addWidget(collision_table_);

  // Bottom Area ----------------------------------------
  controls_box_bottom_ = new QGroupBox(this);
  layout_->addWidget(controls_box_bottom_);
  QHBoxLayout *controls_box_bottom_layout = new QHBoxLayout(controls_box_bottom_);

  // Checkbox
  collision_checkbox_ = new QCheckBox(this);
  collision_checkbox_->setText("Show Non-Disabled Link Pairs");
  connect(collision_checkbox_, SIGNAL(toggled(bool)), this, SLOT(collisionCheckboxToggle()));
  controls_box_bottom_layout->addWidget(collision_checkbox_);

  fraction_label_ = new QLabel(this);
  fraction_label_->setText("Min. collisions for \"always\"-colliding pairs:");

  controls_box_bottom_layout->addWidget(fraction_label_);

  fraction_spinbox_ = new QSpinBox(this);
  fraction_spinbox_->setRange(1, 100);
  fraction_spinbox_->setValue(95);
  fraction_spinbox_->setSuffix("%");
  controls_box_bottom_layout->addWidget(fraction_spinbox_);

  controls_box_bottom_layout->setAlignment(collision_checkbox_, Qt::AlignLeft);

  setLayout(layout_);
  setWindowTitle("Default Collision Matrix");
}

DefaultCollisionsWidget::~DefaultCollisionsWidget()
{
  delete model_;
}

// ******************************************************************************************
// Qt close event function for reminding user to save
// ******************************************************************************************
void DefaultCollisionsWidget::generateCollisionTable()
{
  // Confirm the user wants to overwrite the current disabled collisions
  if (!config_data_->srdf_->disabled_collisions_.empty())
  {
    if (QMessageBox::question(this, "Confirm Disabled Collision Overwrite", "Are you sure you want to overwrite the "
                                                                            "current default collisions matrix with a "
                                                                            "newly generated one?",
                              QMessageBox::Ok | QMessageBox::Cancel) == QMessageBox::Cancel)
    {
      return;  // abort
    }
  }
  QApplication::processEvents();  // allow the progress bar to be shown
  progress_label_->setText("Computing default collision matrix for robot model...");

  // Disable controls on form
  disableControls(true);

  // Create a progress variable that will be shared with the compute_default_collisions tool and its threads
  // NOTE: be sure not to delete this variable until the subprograms have finished using it. Because of the simple
  // use case of this variable (1 thread writes to it, the parent process reads it) it was decided a boost shared
  // pointer is not necessary.
  unsigned int collision_progress = 0;
  progress_bar_->setValue(collision_progress);

  QApplication::processEvents();  // allow the progress bar to be shown

  // Create thread to do actual work
  boost::thread workerThread(
      boost::bind(&DefaultCollisionsWidget::generateCollisionTableThread, this, &collision_progress));
  // Check interval
  boost::posix_time::seconds check_interval(1);

  // Continually loop until threaded computation is finished
  while (collision_progress < 100)
  {
    // Set updated progress value.
    progress_bar_->setValue(collision_progress);

    // Allow GUI thread to do its stuff
    QApplication::processEvents();

    // 1 second sleep
    boost::this_thread::sleep(check_interval);
    // usleep(1000 * 1000);
  }

  // Wait for thread to finish
  workerThread.join();

  // Load the results into the GUI
  loadCollisionTable();

  // Hide the progress bar
  disableControls(false);  // enable everything else

  config_data_->changes |= MoveItConfigData::COLLISIONS;
}

// ******************************************************************************************
// The thread that is called to allow the GUI to update. Calls an external function to do calcs
// ******************************************************************************************
void DefaultCollisionsWidget::generateCollisionTableThread(unsigned int *collision_progress)
{
  unsigned int num_trials = density_slider_->value() * 1000 + 1000;  // scale to trials amount
  double min_frac = (double)fraction_spinbox_->value() / 100.0;

  const bool verbose = true;  // Output benchmarking and statistics
  const bool include_never_colliding = true;

  // clear previously loaded collision matrix entries
  config_data_->getPlanningScene()->getAllowedCollisionMatrixNonConst().clear();

  // Find the default collision matrix - all links that are allowed to collide
  link_pairs_ = moveit_setup_assistant::computeDefaultCollisions(
      config_data_->getPlanningScene(), collision_progress, include_never_colliding, num_trials, min_frac, verbose);

  // Copy data changes to srdf_writer object
  linkPairsToSRDF();

  // End the progress bar loop
  *collision_progress = 100;

  ROS_INFO_STREAM("Thread complete " << link_pairs_.size());
}

// ******************************************************************************************
// Displays data in the link_pairs_ data structure into a QtTableWidget
// ******************************************************************************************
void DefaultCollisionsWidget::loadCollisionTable()
{
  CollisionMatrixModel *matrix_model = new CollisionMatrixModel(
      link_pairs_, config_data_->getPlanningScene()->getRobotModel()->getLinkModelNamesWithCollisionGeometry());
  CollisionLinearModel *linear_model = new CollisionLinearModel(matrix_model);
  collision_table_->setModel(linear_model);
  // delete old and remember new model
  delete model_;
  model_ = linear_model;

  // delete old and fetch new selection model
  delete selection_model_;
  selection_model_ = collision_table_->selectionModel();

  connect(selection_model_, SIGNAL(currentChanged(QModelIndex, QModelIndex)), this, SLOT(previewSelected(QModelIndex)));

  collision_table_->setSortingEnabled(true);
  collision_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  collision_table_->setSelectionMode(QAbstractItemView::SingleSelection);
  // collision_table_->setHorizontalHeader(new RotatedHeaderView(Qt::Horizontal));
  // collision_table_->setVerticalHeader(new RotatedHeaderView(Qt::Vertical));

  connect(model_, SIGNAL(dataChanged(QModelIndex, QModelIndex, QVector<int>)), this,
          SLOT(collisionsChanged(QModelIndex)));
}

void DefaultCollisionsWidget::collisionsChanged(const QModelIndex &index)
{
  selection_model_->setCurrentIndex(index, QItemSelectionModel::Select | QItemSelectionModel::Current |
                                               QItemSelectionModel::Rows);
}

// ******************************************************************************************
// GUI func for showing sampling density amount
// ******************************************************************************************
void DefaultCollisionsWidget::changeDensityLabel(int value)
{
  density_value_label_->setText(QString::number(value * 1000 + 1000));  //.append(" samples") );
}

// ******************************************************************************************
// Helper function to disable parts of GUI during computation
// ******************************************************************************************
void DefaultCollisionsWidget::disableControls(bool disable)
{
  controls_box_->setDisabled(disable);
  collision_table_->setDisabled(disable);
  collision_checkbox_->setDisabled(disable);

  if (disable)
  {
    progress_bar_->show();  // only show when computation begins
    progress_label_->show();
  }
  else
  {
    progress_label_->hide();
    progress_bar_->hide();
  }

  QApplication::processEvents();  // allow the progress bar to be shown
}

// ******************************************************************************************
// Changes the table to show or hide collisions that are not disabled (that have collision checking enabled)
// ******************************************************************************************
void DefaultCollisionsWidget::collisionCheckboxToggle()
{
  // Show Progress bar
  disableControls(true);

  // Now update collision table with updates
  loadCollisionTable();

  // Hide Progress bar
  disableControls(false);
}

// ******************************************************************************************
// Output Link Pairs to SRDF Format and update the collision matrix
// ******************************************************************************************
void DefaultCollisionsWidget::linkPairsToSRDF()
{
  // reset the data in the SRDF Writer class
  config_data_->srdf_->disabled_collisions_.clear();

  // Create temp disabled collision
  srdf::Model::DisabledCollision dc;

  // copy the data in this class's LinkPairMap datastructure to srdf::Model::DisabledCollision format
  for (moveit_setup_assistant::LinkPairMap::const_iterator pair_it = link_pairs_.begin(); pair_it != link_pairs_.end();
       ++pair_it)
  {
    // Only copy those that are actually disabled
    if (pair_it->second.disable_check)
    {
      dc.link1_ = pair_it->first.first;
      dc.link2_ = pair_it->first.second;
      dc.reason_ = moveit_setup_assistant::disabledReasonToString(pair_it->second.reason);
      config_data_->srdf_->disabled_collisions_.push_back(dc);
    }
  }

  // Update collision_matrix for robot pose's use
  config_data_->loadAllowedCollisionMatrix();
}

// ******************************************************************************************
// Load Link Pairs from SRDF Format
// ******************************************************************************************
void DefaultCollisionsWidget::linkPairsFromSRDF()
{
  // Clear all the previous data in the compute_default_collisions tool
  link_pairs_.clear();

  // Create new instance of planning scene using pointer
  planning_scene::PlanningScenePtr scene = config_data_->getPlanningScene()->diff();

  // Populate link_pairs_ list with every possible n choose 2 combination of links
  moveit_setup_assistant::computeLinkPairs(*scene, link_pairs_);

  // Create temp link pair data struct
  moveit_setup_assistant::LinkPairData link_pair_data;
  std::pair<std::string, std::string> link_pair;

  // Loop through all disabled collisions in SRDF and update the comprehensive list that has already been created
  for (std::vector<srdf::Model::DisabledCollision>::const_iterator collision_it =
           config_data_->srdf_->disabled_collisions_.begin();
       collision_it != config_data_->srdf_->disabled_collisions_.end(); ++collision_it)
  {
    // Set the link names
    link_pair.first = collision_it->link1_;
    link_pair.second = collision_it->link2_;

    // Set the link meta data
    link_pair_data.reason = moveit_setup_assistant::disabledReasonFromString(collision_it->reason_);
    link_pair_data.disable_check = true;  // disable checking the collision btw the 2 links

    // Insert into map
    link_pairs_[link_pair] = link_pair_data;
  }
}

// ******************************************************************************************
// Preview whatever element is selected
// ******************************************************************************************
void DefaultCollisionsWidget::previewSelected(const QModelIndex &index)
{
  // Unhighlight all links
  Q_EMIT unhighlightAll();

  if (!index.isValid())
    return;

  // Highlight link pair
  const QString &first_link = model_->data(model_->index(index.row(), 0), Qt::DisplayRole).toString();
  const QString &second_link = model_->data(model_->index(index.row(), 1), Qt::DisplayRole).toString();
  uint check_state = model_->data(model_->index(index.row(), 2), Qt::CheckStateRole).toUInt();

  QColor color = (check_state == Qt::Checked) ? QColor(0, 255, 0) : QColor(255, 0, 0);
  Q_EMIT highlightLink(first_link.toStdString(), color);
  Q_EMIT highlightLink(second_link.toStdString(), color);
}

// ******************************************************************************************
// Called when setup assistant navigation switches to this screen
// ******************************************************************************************
void DefaultCollisionsWidget::focusGiven()
{
  // Convert the SRDF data to LinkPairData format
  linkPairsFromSRDF();

  // Load the data to the table
  loadCollisionTable();

  // Enable the table
  disableControls(false);
}

}  // namespace
