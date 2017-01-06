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
#include <boost/unordered_map.hpp>
#include <boost/assign.hpp>
#include <ros/console.h>

namespace moveit_setup_assistant
{
/// Boost mapping of reasons for disabling a link pair to strings
const boost::unordered_map<moveit_setup_assistant::DisabledReason, const char*> longReasonsToString =
    boost::assign::map_list_of(moveit_setup_assistant::NEVER, "Never in Collision")(moveit_setup_assistant::DEFAULT,
                                                                                    "Collision by Default")(
        moveit_setup_assistant::ADJACENT, "Adjacent Links")(moveit_setup_assistant::ALWAYS, "Always in Collision")(
        moveit_setup_assistant::USER, "User Disabled")(moveit_setup_assistant::NOT_DISABLED, "");

/**
 * \brief Subclass QTableWidgetItem for checkboxes to allow custom sorting by implementing the < operator
 */
class CheckboxSortWidgetItem : public QTableWidgetItem
{
public:
  /**
   * \brief Override the standard comparision operator
   */
  bool operator<(const QTableWidgetItem& other) const
  {
    return checkState() < other.checkState();
  }
};

// ******************************************************************************************
// User interface for editing the default collision matrix list in an SRDF
// ******************************************************************************************
DefaultCollisionsWidget::DefaultCollisionsWidget(QWidget* parent, MoveItConfigDataPtr config_data)
  : SetupScreenWidget(parent), config_data_(config_data)
{
  // Basic widget container
  layout_ = new QVBoxLayout(this);

  // Top Label Area ------------------------------------------------
  HeaderWidget* header = new HeaderWidget(
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
  QHBoxLayout* controls_box_layout = new QHBoxLayout(controls_box_);
  controls_box_layout->setAlignment(Qt::AlignTop | Qt::AlignLeft);

  // Slider Label
  QLabel* density_left_label = new QLabel(this);
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
  QLabel* density_right_label = new QLabel(this);
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
  collision_table_ = new QTableWidget(this);
  collision_table_->setColumnCount(4);
  collision_table_->setSortingEnabled(true);
  collision_table_->setSelectionMode(QAbstractItemView::SingleSelection);
  collision_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  connect(collision_table_, SIGNAL(currentCellChanged(int, int, int, int)), this, SLOT(previewSelected(int)));
  connect(collision_table_, SIGNAL(cellChanged(int, int)), this, SLOT(toggleCheckBox(int, int)));
  layout_->addWidget(collision_table_);

  QStringList header_list;
  header_list.append("Link A");
  header_list.append("Link B");
  header_list.append("Disabled");
  header_list.append("Reason To Disable");
  collision_table_->setHorizontalHeaderLabels(header_list);

  // Resize headers
  collision_table_->resizeColumnToContents(0);
  collision_table_->resizeColumnToContents(1);
  collision_table_->resizeColumnToContents(2);
  collision_table_->resizeColumnToContents(3);

  // Bottom Area ----------------------------------------
  controls_box_bottom_ = new QGroupBox(this);
  layout_->addWidget(controls_box_bottom_);
  QHBoxLayout* controls_box_bottom_layout = new QHBoxLayout(controls_box_bottom_);

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
void DefaultCollisionsWidget::generateCollisionTableThread(unsigned int* collision_progress)
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
  int row = 0;
  int progress_counter = 0;

  // Show Progress Bar
  progress_bar_->setValue(0);

  QApplication::processEvents();  // allow the progress bar to be shown
  progress_label_->setText("Loading table...");

  // Setup Collision Table
  collision_table_->setUpdatesEnabled(false);  // prevent table from updating until we are completely done
  collision_table_->setDisabled(true);         // make sure we disable it so that the cellChanged event is not called
  collision_table_->clearContents();

  // Check if there are no disabled collisions (unprobable?)
  if (link_pairs_.empty())
  {
    collision_table_->setRowCount(1);
    QTableWidgetItem* no_collide = new QTableWidgetItem("No Link Pairs Of This Kind");
    collision_table_->setItem(0, 0, no_collide);
  }
  else
  {
    // The table will be populated, so indicate it on the button
    btn_generate_->setText("Regenerate Default Collision Matrix");
  }

  // Intially set the table to be worst-case scenario of every possible element pair
  collision_table_->setRowCount(link_pairs_.size());

  for (moveit_setup_assistant::LinkPairMap::const_iterator pair_it = link_pairs_.begin(); pair_it != link_pairs_.end();
       ++pair_it)
  {
    // Add link pair row if 1) it is disabled from collision checking or 2) the SHOW ALL LINK PAIRS checkbox is checked
    if (pair_it->second.disable_check || collision_checkbox_->isChecked())
    {
      // Create row elements
      QTableWidgetItem* linkA = new QTableWidgetItem(pair_it->first.first.c_str());
      linkA->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

      QTableWidgetItem* linkB = new QTableWidgetItem(pair_it->first.second.c_str());
      linkB->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

      CheckboxSortWidgetItem* disable_check = new CheckboxSortWidgetItem();
      disable_check->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable | Qt::ItemIsSelectable);
      if (pair_it->second.disable_check)  // Checked means no collision checking
        disable_check->setCheckState(Qt::Checked);
      else
        disable_check->setCheckState(Qt::Unchecked);

      QTableWidgetItem* reason = new QTableWidgetItem(longReasonsToString.at(pair_it->second.reason));
      reason->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

      // Insert row elements into collision table
      collision_table_->setItem(row, 0, linkA);
      collision_table_->setItem(row, 1, linkB);
      collision_table_->setItem(row, 2, disable_check);
      collision_table_->setItem(row, 3, reason);

      // Increment row count
      ++row;
    }

    ++progress_counter;  // for calculating progress bar

    if (progress_counter % 200 == 0)
    {
      // Update Progress Bar
      progress_bar_->setValue(progress_counter * 100 / link_pairs_.size());
      QApplication::processEvents();  // allow the progress bar to be shown
    }
  }

  // Reduce the table size to only the number of used rows.
  collision_table_->setRowCount(row);

  // Resize headers. The hiding is a hack so that it resizes correctly
  collision_table_->setVisible(false);
  collision_table_->resizeColumnToContents(0);
  collision_table_->resizeColumnToContents(1);
  collision_table_->resizeColumnToContents(2);
  collision_table_->resizeColumnToContents(3);
  collision_table_->setVisible(true);

  collision_table_->setUpdatesEnabled(true);  // prevent table from updating until we are completely done
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
// Called when user changes data in table, really just the checkbox
// ******************************************************************************************
void DefaultCollisionsWidget::toggleCheckBox(int row, int column)
{
  // Only accept cell changes if table is enabled, otherwise it is this program making changes
  // Also make sure the change is in the checkbox column
  if (!collision_table_->isEnabled() || column != 2)
    return;

  // Convert row to pair
  std::pair<std::string, std::string> link_pair;
  link_pair.first = collision_table_->item(row, 0)->text().toStdString();
  link_pair.second = collision_table_->item(row, 1)->text().toStdString();

  // Get the state of checkbox
  bool check_state = collision_table_->item(row, 2)->checkState();

  // Check if the checkbox state has changed from original value
  if (link_pairs_[link_pair].disable_check != check_state)
  {
    // Save the change
    link_pairs_[link_pair].disable_check = check_state;

    // Handle USER Reasons: 1) pair is disabled by user
    if (link_pairs_[link_pair].disable_check == true &&
        link_pairs_[link_pair].reason == moveit_setup_assistant::NOT_DISABLED)
    {
      link_pairs_[link_pair].reason = moveit_setup_assistant::USER;

      // Change Reason in Table
      collision_table_->item(row, 3)->setText(longReasonsToString.at(link_pairs_[link_pair].reason));
    }
    // Handle USER Reasons: 2) pair was disabled by user and now is enabled (not checked)
    else if (link_pairs_[link_pair].disable_check == false &&
             link_pairs_[link_pair].reason == moveit_setup_assistant::USER)
    {
      link_pairs_[link_pair].reason = moveit_setup_assistant::NOT_DISABLED;

      // Change Reason in Table
      collision_table_->item(row, 3)->setText("");
    }

    config_data_->changes |= MoveItConfigData::COLLISIONS;
  }

  // Copy data changes to srdf_writer object
  linkPairsToSRDF();

  previewSelected(row);
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
void DefaultCollisionsWidget::previewSelected(int row)
{
  // Unhighlight all links
  Q_EMIT unhighlightAll();

  // Highlight link
  QTableWidgetItem* first_link_item = collision_table_->item(row, 0);
  if (!first_link_item)
    return;  // nothing to highlight

  const QString& first_link = first_link_item->text();
  const QString& second_link = collision_table_->item(row, 1)->text();
  Qt::CheckState check_state = collision_table_->item(row, 2)->checkState();

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
