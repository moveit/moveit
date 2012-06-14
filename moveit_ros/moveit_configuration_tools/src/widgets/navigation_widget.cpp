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
#include <iostream>

// ******************************************************************************************
// CLASS
// ******************************************************************************************

NavItem::NavItem()
{
}

NavItem::NavItem(const QString &name, QWidget *screen ) :
  name_(name), screen_(screen)
{

}

QString NavItem::name() const
{
  return name_;
}

QWidget * NavItem::screen()
{
  return screen_;
}

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

void NavigationWidget::setNavs(QList<NavItem> &navs)
{
  setModel(NULL);
  model_->clear();

  for(int i = 0; i < navs.size(); i ++)
  {
    QStandardItem *item = new QStandardItem();
    item->setData(QVariant::fromValue(navs.at(i)), Qt::DisplayRole);
    model_->appendRow(item);
  }

  setModel(model_);
}

void NavigationWidget::setSelected(const int &index)
{

  // Select one box from column 0, row index
  QModelIndex top = model_->index(index, 0, QModelIndex());
  QModelIndex bottom = model_->index(index, 0, QModelIndex());

  QItemSelection selection(top, bottom);
  selectionModel()->select(selection, QItemSelectionModel::Select);
}

/*void NavigationWidget::pressed( const QModelIndex & index )
{
  std::cout << "helloooo222" << std::endl;
  }*/

// ******************************************************************************************
// CLASS
// ******************************************************************************************

NavDelegate::NavDelegate(QObject *parent) :
  QStyledItemDelegate(parent)
{
}

void NavDelegate::paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const
{
  if(index.data().canConvert<NavItem>())
    paintNav(painter, option, index);
  else
    paintLetter(painter, option, index);
}

QSize NavDelegate::sizeHint(const QStyleOptionViewItem &option, const QModelIndex &index) const
{
  if(index.data().canConvert<NavItem>())
    return QSize(option.rect.width(), 45);
  else
    return QSize(option.rect.width(), 25);
}

void NavDelegate::paintNav(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const
{
  const bool isSelected = option.state & QStyle::State_Selected;
  //const bool isHovered = option.state & QStyle::State_MouseOver;

  NavItem tp = index.data().value<NavItem>();

  painter->save();

  QLinearGradient backgroundGradient(QPoint(option.rect.x(), option.rect.y()), QPoint(option.rect.x(), option.rect.y()+option.rect.height()));
  if(isSelected) 
  {
    //        painter->fillRect(option.rect, QBrush(QColor(49, 49, 49)));
    backgroundGradient.setColorAt(0, QColor(109, 164, 219));
    backgroundGradient.setColorAt(1, QColor(61, 138, 212));
    painter->fillRect(option.rect, QBrush(backgroundGradient));
    //        painter->fillRect(option.rect, QBrush(QColor(225, 225, 225)));
  } 
  else 
  {
    //        painter->fillRect(option.rect, QBrush(QColor(244, 244, 244)));
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
  //    QString text = index.model()->data(index, Qt::DisplayRole).toString();
  QRect textRect(option.rect.x()+10, option.rect.y(), option.rect.width()-10, option.rect.height());

  QFont textFont(painter->font());
  textFont.setPixelSize(14); // Set font size
  textFont.setFamily("Arial"); //Helvetica Neue");

  // Font color
  painter->setPen(QColor(69, 69, 69)); //69
  if(isSelected){
    painter->setPen(QColor(229, 229, 229));
  }


  painter->setFont(textFont);
  painter->drawText(textRect, Qt::AlignLeft|Qt::AlignVCenter, tp.name());

  painter->restore();
}

void NavDelegate::paintLetter(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const
{
  //const bool isSelected = option.state & QStyle::State_Selected;
  //const bool isHovered = option.state & QStyle::State_MouseOver;

  painter->save();

  QLinearGradient backgroundGradient(QPoint(option.rect.x(), option.rect.y()), 
                                     QPoint(option.rect.x(), option.rect.y()+option.rect.height()));
  //        painter->fillRect(option.rect, QBrush(QColor(225, 225, 225)));

  backgroundGradient.setColorAt(0, QColor(215, 215, 215));
  backgroundGradient.setColorAt(1, QColor(230, 230, 230));
  painter->fillRect(option.rect, QBrush(backgroundGradient));

  painter->setPen(QColor(213, 213, 213));
  painter->drawLine(option.rect.bottomLeft(), option.rect.bottomRight());

  QFont textFont(painter->font());
  textFont.setPixelSize(14); // font size
  textFont.setFamily("Arial"); //Helvetica Neue");
  painter->setFont(textFont);

  QRect textRect(option.rect.x()+10, option.rect.y(), option.rect.width()-10, option.rect.height());
  painter->setPen(QColor(39, 39, 39));
  painter->drawText(textRect, Qt::AlignLeft|Qt::AlignVCenter, index.model()->data(index).toString());

  painter->restore();
}
