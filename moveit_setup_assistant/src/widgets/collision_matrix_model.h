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

#ifndef MOVEIT_ROS_MOVEIT_SETUP_ASSISTANT_WIDGETS_COLLISION_MATRIX_MODEL_
#define MOVEIT_ROS_MOVEIT_SETUP_ASSISTANT_WIDGETS_COLLISION_MATRIX_MODEL_

#include <QAbstractTableModel>
#include <moveit/setup_assistant/tools/compute_default_collisions.h>

class QItemSelection;

class CollisionMatrixModel : public QAbstractTableModel
{
  Q_OBJECT
public:
  CollisionMatrixModel(moveit_setup_assistant::LinkPairMap &pairs, const std::vector<std::string> &names,
                       QObject *parent = NULL);
  int rowCount(const QModelIndex &parent = QModelIndex()) const;
  int columnCount(const QModelIndex &parent = QModelIndex()) const;
  QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const;
  QVariant headerData(int section, Qt::Orientation orientation, int role) const;
  moveit_setup_assistant::DisabledReason reason(const QModelIndex &index) const;

  // for editing
  Qt::ItemFlags flags(const QModelIndex &index) const;
  bool setData(const QModelIndex &, const QVariant &value, int role);
  void setEnabled(const QItemSelection &selection, bool value);
  void setEnabled(const QModelIndexList &indexes, bool value);

public Q_SLOTS:
  void setFilterRegExp(const QString &filter);

private:
  moveit_setup_assistant::LinkPairMap::iterator item(const QModelIndex &index);
  moveit_setup_assistant::LinkPairMap::const_iterator item(const QModelIndex &index) const
  {
    return const_cast<CollisionMatrixModel *>(this)->item(index);
  }

private:
  moveit_setup_assistant::LinkPairMap &pairs;
  const std::vector<std::string> std_names;  // names of links
  QList<QString> q_names;                    // names of links
  QList<int> visual_to_index;                // map from visual index to actual index
};

#endif  // MOVEIT_ROS_MOVEIT_SETUP_ASSISTANT_WIDGETS_COLLISION_MATRIX_MODEL_
