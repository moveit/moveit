/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, CITEC, Bielefeld University
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

/* Author: Robert Haschke */

#ifndef MOVEIT_ROS_MOVEIT_SETUP_ASSISTANT_WIDGETS_COLLISION_LINEAR_MODEL_H
#define MOVEIT_ROS_MOVEIT_SETUP_ASSISTANT_WIDGETS_COLLISION_LINEAR_MODEL_H

#include <QAbstractProxyModel>
#include <QSortFilterProxyModel>
#include <QVector>
#include <moveit/setup_assistant/tools/compute_default_collisions.h>

#include "collision_matrix_model.h"
class CollisionLinearModel : public QAbstractProxyModel
{
  Q_OBJECT

public:
  CollisionLinearModel(CollisionMatrixModel* src, QObject* parent = NULL);
  ~CollisionLinearModel();

  // reimplement to return the model index in the proxy model that to the sourceIndex from the source model
  QModelIndex mapFromSource(const QModelIndex& sourceIndex) const;
  // Reimplement this function to return the model index in the source model that corresponds to the proxyIndex in the
  // proxy model
  QModelIndex mapToSource(const QModelIndex& proxyIndex) const;

  int rowCount(const QModelIndex& parent) const;
  int columnCount(const QModelIndex& parent) const;

  QModelIndex index(int row, int column, const QModelIndex& parent = QModelIndex()) const;
  QModelIndex parent(const QModelIndex& child) const;
  QVariant data(const QModelIndex& index, int role) const;
  moveit_setup_assistant::DisabledReason reason(int row) const;

  bool setData(const QModelIndex& index, const QVariant& value, int role);
  void setEnabled(const QItemSelection& selection, bool value);

  Qt::ItemFlags flags(const QModelIndex& index) const;
  QVariant headerData(int section, Qt::Orientation orientation, int role) const;
};

/** proxy model to allow for sorting of CollisionLinearModel, considering sorting history */
class SortFilterProxyModel : public QSortFilterProxyModel
{
  Q_OBJECT

public:
  SortFilterProxyModel(QObject* parent = 0);
  QVariant headerData(int section, Qt::Orientation orientation, int role) const;
  void sort(int column, Qt::SortOrder order);
  void setShowAll(bool show_all);
  void setEnabled(const QItemSelection& selection, bool value);

protected:
  bool filterAcceptsRow(int source_row, const QModelIndex& source_parent) const;
  bool lessThan(const QModelIndex& src_left, const QModelIndex& src_right) const;

private Q_SLOTS:
  void initSorting();

private:
  bool show_all_;
  QVector<int> sort_columns_;  // sorting history
  QVector<int> sort_orders_;   // corresponding sort orders
};

#endif  // MOVEIT_ROS_MOVEIT_SETUP_ASSISTANT_WIDGETS_COLLISION_LINEAR_MODEL_H
