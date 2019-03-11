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

#ifndef MOVEIT_MOVEIT_SETUP_ASSISTANT_WIDGETS_DEFAULT_COLLISIONS_WIDGET__
#define MOVEIT_MOVEIT_SETUP_ASSISTANT_WIDGETS_DEFAULT_COLLISIONS_WIDGET__

#include <QLabel>
#include <QVBoxLayout>
#include <QTableView>
#include <QSlider>
#include <QPushButton>
#include <QGroupBox>
#include <QProgressBar>
#include <QCheckBox>
#include <QRadioButton>
#include <QSpinBox>
#include <QThread>
#include <QLineEdit>
#include <QAction>

#ifndef Q_MOC_RUN
#include <boost/thread/thread.hpp>
#include <boost/function/function_fwd.hpp>
#include <moveit/setup_assistant/tools/moveit_config_data.h>
#endif

#include "setup_screen_widget.h"  // a base class for screens in the setup assistant

namespace moveit_setup_assistant
{
class MonitorThread;

/**
 * \brief User interface for editing the default collision matrix list in an SRDF
 */
class DefaultCollisionsWidget : public SetupScreenWidget
{
  Q_OBJECT

public:
  enum ViewMode
  {
    MatrixMode = 0,
    LinearMode = 1
  };

  // ******************************************************************************************
  // Public Functions
  // ******************************************************************************************

  /**
   * \brief User interface for editing the default collision matrix list in an SRDF
   * \param urdf_file String srdf file location. It will create a new file or will edit an existing one
   */
  DefaultCollisionsWidget(QWidget* parent, const MoveItConfigDataPtr& config_data);
  ~DefaultCollisionsWidget() override;

  /**
   * \brief Output Link Pairs to SRDF Format
   */
  void linkPairsToSRDF();

  /**
   * \brief Load Link Pairs from SRDF Format
   */
  void linkPairsFromSRDF();

private Q_SLOTS:

  // ******************************************************************************************
  // Slot Event Functions
  // ******************************************************************************************

  /**
   * \brief start generating collision matrix in a worker thread
   */
  void startGeneratingCollisionTable();
  /**
   * \brief finish generating collision matrix after worker thread has finished
   */
  void finishGeneratingCollisionTable();

  /**
   * \brief GUI func for showing sampling density amount
   * \param value Sampling density
   */
  void changeDensityLabel(int value);

  /**
   * \brief Update view and data model for the link_pairs data structure
   */
  void loadCollisionTable();

  /**
   * \brief Change filter settings to show/hide enabled collisions
   */
  void checkedFilterChanged();

  /**
   * \brief Collision model changed
   */
  void collisionsChanged(const QModelIndex& index);

  /**
   * \brief Revert current changes to collision matrix
   */
  void revertChanges();

  /**
  * \brief Called when current row has changed
  */
  void previewSelectedMatrix(const QModelIndex& index);
  void previewSelectedLinear(const QModelIndex& index);

  /**
   * \brief Called when setup assistant navigation switches to this screen
   */
  void focusGiven() override;

  /**
   * \brief Called when setup assistant navigation switches away from this screen
   */
  bool focusLost() override;

  void showHeaderContextMenu(const QPoint& p);
  void hideSections();
  void hideOtherSections();
  void showSections();

private:
  // ******************************************************************************************
  // Qt Components
  // ******************************************************************************************
  QLabel* page_title_;
  QTableView* collision_table_;
  QAbstractItemModel* model_;
  QItemSelectionModel* selection_model_;
  QVBoxLayout* layout_;
  QLabel* density_value_label_;
  QSlider* density_slider_;
  QPushButton* btn_generate_;
  QGroupBox* controls_box_;
  QProgressBar* progress_bar_;
  QLabel* progress_label_;
  QLineEdit* link_name_filter_;
  QCheckBox* collision_checkbox_;
  QLabel* fraction_label_;
  QSpinBox* fraction_spinbox_;
  QPushButton* btn_revert_;
  QButtonGroup* view_mode_buttons_;

  QList<QAction*> header_actions_;    // context actions for header sections
  Qt::Orientations clicked_headers_;  // remember which header section activated context actions
  int clicked_section_;               // remember which header section activated context actions

  // ******************************************************************************************
  // Variables
  // ******************************************************************************************
  MonitorThread* worker_;

  /// main storage of link pair data
  moveit_setup_assistant::LinkPairMap link_pairs_;

  /// Contains all the configuration data for the setup assistant
  moveit_setup_assistant::MoveItConfigDataPtr config_data_;

  // ******************************************************************************************
  // Private Functions
  // ******************************************************************************************

  /**
   * \brief The thread that is called to allow the GUI to update. Calls an external function to do calcs
   * \param collision_progress A shared pointer between 3 threads to allow progress bar to update. See declaration
   * location for more details and warning.
   */
  void generateCollisionTable(unsigned int* collision_progress);

  /**
   * \brief Helper function to disable parts of GUI during computation
   * \param disable A command
   */
  void disableControls(bool disable);

  /**
   * \brief Allow toggling of all checkboxes in selection by filtering <space> keypresses
   */
  bool eventFilter(QObject* object, QEvent* event) override;

  /**
   * \brief Show header's sections in logicalIndexes and everything in between
   */
  void showSections(QHeaderView* header, const QList<int>& logicalIndexes);
  /**
   * \brief Toggle enabled status of selection
   */
  void toggleSelection(QItemSelection selection);
};

/**
 * \brief QThread to monitor progress of a boost::thread
 */
class MonitorThread : public QThread
{
  Q_OBJECT

public:
  MonitorThread(const boost::function<void(unsigned int*)>& f, QProgressBar* progress_bar = NULL);
  void run() override;
  void cancel()
  {
    canceled_ = true;
  }
  bool canceled() const
  {
    return canceled_;
  }

Q_SIGNALS:
  void progress(int);

private:
  boost::thread worker_;
  unsigned int progress_;
  bool canceled_;
};
}

#endif
