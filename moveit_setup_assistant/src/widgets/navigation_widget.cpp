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

#include "navigation_widget.h"
#include <QDebug>
#include <iostream>

namespace moveit_setup_assistant
{

// ******************************************************************************************
// CLASS
// ******************************************************************************************

NavigationWidget::NavigationWidget(QWidget *parent) :
  QListView(parent)
{
  setItemDelegate(new NavDelegate(this));
  setEditTriggers(QAbstractItemView::NoEditTriggers);
  
  //setAttribute(Qt::WA_MacShowFocusRect, false);

  // Set frame graphics
  setFrameShape(QFrame::StyledPanel);
  setFrameShadow(QFrame::Raised);
  setLineWidth(1);
  setMidLineWidth(0);

  // Hard code width and height
  setMaximumWidth(160);
  setMinimumWidth(160);
  setMinimumHeight(300);

  verticalScrollBar()->setPageStep(3);
  verticalScrollBar()->setSingleStep(1);

  model_ = new QStandardItemModel(this);
  setModel(model_);
}

void NavigationWidget::setNavs(const QList<QString> &navs)
{
  setModel(NULL);
  model_->clear();

  for(int i = 0; i < navs.size(); i ++)
  {
    QStandardItem *item = new QStandardItem();
    item->setData(QVariant::fromValue(navs.at(i)), Qt::DisplayRole);
    item->setFlags( Qt::NoItemFlags );
    model_->appendRow(item);
  }

  setModel(model_);
}

void NavigationWidget::setEnabled( const int &index, bool enabled )
{
  if( enabled )
    model_->item( index )->setFlags( Qt::ItemIsSelectable | Qt::ItemIsEditable | Qt::ItemIsDragEnabled | Qt::ItemIsDropEnabled | Qt::ItemIsEnabled );
  else
    model_->item( index )->setFlags( Qt::NoItemFlags );
}

void NavigationWidget::setSelected( const int &index )
{
  // First make sure item is enabled
  setEnabled( index, true );

  // Select one box from column 0, row index
  QModelIndex top = model_->index(index, 0, QModelIndex());
  QModelIndex bottom = model_->index(index, 0, QModelIndex());

  QItemSelection selection(top, bottom);
  selectionModel()->reset(); // set them all to deselected
  selectionModel()->select(selection, QItemSelectionModel::Select);
}


// ******************************************************************************************
// CLASS
// ******************************************************************************************

NavDelegate::NavDelegate(QObject *parent) :
  QStyledItemDelegate(parent)
{
}

QSize NavDelegate::sizeHint(const QStyleOptionViewItem &option, const QModelIndex &index) const
{
  return QSize(option.rect.width(), 45);
}

void NavDelegate::paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const
{
  const bool isSelected = option.state & QStyle::State_Selected;

  //NavScreen tp = index.data().value<NavScreen>();
  QString nav_name = index.data().value<QString>();

  painter->save();

  QLinearGradient backgroundGradient(QPoint(option.rect.x(), option.rect.y()), QPoint(option.rect.x(), option.rect.y()+option.rect.height()));
  if(isSelected) 
  {
    backgroundGradient.setColorAt(0, QColor(109, 164, 219));
    backgroundGradient.setColorAt(1, QColor(61, 138, 212));
    painter->fillRect(option.rect, QBrush(backgroundGradient));
  } 
  else 
  {
    backgroundGradient.setColorAt(0, QColor(245, 245, 245));
    backgroundGradient.setColorAt(1, QColor(240, 240, 240));
    painter->fillRect(option.rect, QBrush(backgroundGradient));
  }

  painter->setPen(QColor(225, 225, 225));
  if(isSelected)
  {
    painter->setPen(QColor(37, 105, 169));
    painter->drawLine(option.rect.bottomLeft(), option.rect.bottomRight());
    painter->setPen(Qt::transparent);
  }
  painter->drawLine(option.rect.topLeft(), option.rect.topRight());
  if(!isSelected)
  {
    painter->setPen(QColor(248, 248, 248));
    painter->drawLine(QPoint(option.rect.x(), option.rect.y()+1), QPoint(option.rect.x()+option.rect.width(), option.rect.y()+1));
  }

  QRect textRect(option.rect.x()+10, option.rect.y(), option.rect.width()-10, option.rect.height());

  QFont textFont(painter->font());
  textFont.setPixelSize(14); // Set font size
  textFont.setFamily("Arial"); //Helvetica Neue");
  painter->setFont(textFont);

  // Font color
  if(isSelected)
  {
    // Selected
    painter->setPen(QColor(229, 229, 229));
  }
  else if( index.flags().testFlag( Qt::NoItemFlags ) )
  {
    // Disabled font color if disabled
    painter->setPen(QColor(170, 170, 170)); // TODO: make this work
  }
  else
  {
    // Normal
    painter->setPen(QColor(69, 69, 69));
  }

  painter->drawText(textRect, Qt::AlignLeft|Qt::AlignVCenter, nav_name );

  painter->restore();
}


}
