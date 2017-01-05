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

#include "collision_matrix_model.h"
#include <boost/unordered_map.hpp>
#include <boost/assign.hpp>
#include <QVector>
#include <QBrush>
#include <QColor>
#include <QPalette>
#include <QApplication>
#include <QItemSelection>

using namespace moveit_setup_assistant;

/// Boost mapping of reasons for disabling a link pair to strings
static const boost::unordered_map<moveit_setup_assistant::DisabledReason, const char*> longReasonsToString =
    boost::assign::map_list_of  // clang-format off
    ( moveit_setup_assistant::NEVER, "Never in Collision" )
    ( moveit_setup_assistant::DEFAULT, "Collision by Default" )
    ( moveit_setup_assistant::ADJACENT, "Adjacent Links" )
    ( moveit_setup_assistant::ALWAYS, "Always in Collision" )
    ( moveit_setup_assistant::USER, "User Disabled" )
    ( moveit_setup_assistant::NOT_DISABLED, "");  // clang-format on

/// Boost mapping of reasons to a background color
static const boost::unordered_map<moveit_setup_assistant::DisabledReason, QVariant> longReasonsToBrush =
    boost::assign::map_list_of  // clang-format off
    ( moveit_setup_assistant::NEVER, QBrush(QColor("lightgreen")) )
    ( moveit_setup_assistant::DEFAULT, QBrush(QColor("lightpink")) )
    ( moveit_setup_assistant::ADJACENT, QBrush(QColor("powderblue")) )
    ( moveit_setup_assistant::ALWAYS, QBrush(QColor("tomato")) )
    ( moveit_setup_assistant::USER, QBrush(QColor("yellow")) )
    ( moveit_setup_assistant::NOT_DISABLED, QBrush());  // clang-format on

CollisionMatrixModel::CollisionMatrixModel(moveit_setup_assistant::LinkPairMap& pairs,
                                           const std::vector<std::string>& names, QObject* parent)
  : QAbstractTableModel(parent), pairs(pairs), std_names(names)
{
  int idx = 0;
  for (std::vector<std::string>::const_iterator it = names.begin(), end = names.end(); it != end; ++it, ++idx)
  {
    visual_to_index << idx;
    q_names << QString::fromStdString(*it);
  }
}

// return item in pairs map given a normalized index, use item(normalized(index))
moveit_setup_assistant::LinkPairMap::iterator CollisionMatrixModel::item(const QModelIndex& index)
{
  int r = visual_to_index[index.row()], c = visual_to_index[index.column()];
  if (r == c)
    return pairs.end();

  // setLinkPair() actually inserts the pair (A,B) where A < B
  if (std_names[r] >= std_names[c])
    std::swap(r, c);

  return pairs.find(std::make_pair(std_names[r], std_names[c]));
}

int CollisionMatrixModel::rowCount(const QModelIndex& /*parent*/) const
{
  return visual_to_index.size();
}

int CollisionMatrixModel::columnCount(const QModelIndex& /*parent*/) const
{
  return visual_to_index.size();
}

QVariant CollisionMatrixModel::data(const QModelIndex& index, int role) const
{
  if (index.isValid() && index.row() == index.column() && role == Qt::BackgroundRole)
    return QApplication::palette().window();

  moveit_setup_assistant::LinkPairMap::const_iterator item = this->item(index);
  if (item == pairs.end())
    return QVariant();

  switch (role)
  {
    case Qt::CheckStateRole:
      return item->second.disable_check ? Qt::Checked : Qt::Unchecked;
    case Qt::ToolTipRole:
      return longReasonsToString.at(item->second.reason);
    case Qt::BackgroundRole:
      return longReasonsToBrush.at(item->second.reason);
  }
  return QVariant();
}

moveit_setup_assistant::DisabledReason CollisionMatrixModel::reason(const QModelIndex& index) const
{
  moveit_setup_assistant::LinkPairMap::const_iterator item = this->item(index);
  if (item == pairs.end())
    return moveit_setup_assistant::NOT_DISABLED;
  return item->second.reason;
}

bool CollisionMatrixModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
  if (role == Qt::CheckStateRole)
  {
    moveit_setup_assistant::LinkPairMap::iterator item = this->item(index);
    if (item == pairs.end())
      return false;

    bool new_value = (value.toInt() == Qt::Checked);
    if (item->second.disable_check == new_value)
      return true;

    item->second.disable_check = new_value;

    // Handle USER Reasons: 1) pair is disabled by user
    if (item->second.disable_check == true && item->second.reason == moveit_setup_assistant::NOT_DISABLED)
      item->second.reason = moveit_setup_assistant::USER;

    // Handle USER Reasons: 2) pair was disabled by user and now is enabled (not checked)
    else if (item->second.disable_check == false && item->second.reason == moveit_setup_assistant::USER)
      item->second.reason = moveit_setup_assistant::NOT_DISABLED;

    QModelIndex mirror = this->index(index.column(), index.row());
    Q_EMIT dataChanged(index, index);
    Q_EMIT dataChanged(mirror, mirror);
    return true;
  }
  return false;  // reject all other changes
}

void CollisionMatrixModel::setEnabled(const QItemSelection& selection, bool value)
{
  // perform changes without signalling
  QItemSelection changes;
  blockSignals(true);
  for (const auto range : selection)
  {
    setEnabled(range.indexes(), value);

    const QModelIndex& top_left = range.topLeft();
    const QModelIndex& bottom_right = range.bottomRight();
    changes.select(top_left, bottom_right);
    changes.select(createIndex(top_left.column(), top_left.row()),
                   createIndex(bottom_right.column(), bottom_right.row()));
  }
  blockSignals(false);

  // emit changes
  for (const auto range : changes)
    Q_EMIT dataChanged(range.topLeft(), range.bottomRight());
}

void CollisionMatrixModel::setEnabled(const QModelIndexList& indexes, bool value)
{
  for (const auto idx : indexes)
    setData(idx, value ? Qt::Checked : Qt::Unchecked, Qt::CheckStateRole);
}

void CollisionMatrixModel::setFilterRegExp(const QString& filter)
{
  beginResetModel();
  QRegExp regexp(filter);
  visual_to_index.clear();
  for (int idx = 0, end = q_names.size(); idx != end; ++idx)
  {
    if (q_names[idx].contains(regexp))
      visual_to_index << idx;
  }
  endResetModel();
}

QVariant CollisionMatrixModel::headerData(int section, Qt::Orientation, int role) const
{
  if (role == Qt::DisplayRole)
    return q_names[visual_to_index[section]];
  return QVariant();
}

Qt::ItemFlags CollisionMatrixModel::flags(const QModelIndex& index) const
{
  if (!index.isValid())
    return 0;

  Qt::ItemFlags f = QAbstractTableModel::flags(index);
  if (index.row() != index.column())
    f |= Qt::ItemIsUserCheckable;
  return f;
}
