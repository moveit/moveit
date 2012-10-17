/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Author: E. Gil Jones

#include <moveit_visualization_ros/attach_object_addition_dialog.h>

#include <QGroupBox>
#include <QGridLayout>
#include <QDialogButtonBox>
#include <QLabel>
#include <QPushButton>

namespace moveit_visualization_ros
{

AttachObjectAdditionDialog::AttachObjectAdditionDialog(QWidget* parent,
                                                       const planning_models::KinematicModelConstPtr& kmodel) :
  QDialog(parent)
{
  QVBoxLayout* layout = new QVBoxLayout(this);
  QGroupBox* panel = new QGroupBox(this);
  panel->setTitle("Link for attaching");
  
  QVBoxLayout* panel_layout = new QVBoxLayout(panel);
  attach_link_box_= new QComboBox(this);

  std::vector<std::string> link_names = kmodel->getLinkModelNames();
  for(unsigned int i = 0; i < link_names.size(); i++) {
    attach_link_box_->addItem(link_names[i].c_str());
  }
  panel_layout->addWidget(attach_link_box_);
  panel->setLayout(panel_layout);
  layout->addWidget(panel);

  QGroupBox* grid_panel = new QGroupBox(this);
  QGridLayout* grid = new QGridLayout(grid_panel);
  
  QLabel* all_label = new QLabel(this);
  all_label->setText("Links and groups");

  QLabel* touch_links = new QLabel(this);
  touch_links->setText("Touch links");

  grid->addWidget(all_label, 0, 0);
  grid->addWidget(touch_links, 0, 2);

  QPushButton* add_button = new QPushButton(this);
  add_button->setText("Add ->");
  
  connect(add_button, SIGNAL(clicked()), this, SLOT(addTouchLinkClicked()));

  QPushButton* remove_button = new QPushButton(this);
  remove_button->setText("Remove");

  connect(remove_button, SIGNAL(clicked()), this, SLOT(removeTouchLinkClicked()));

  grid->addWidget(add_button, 1, 1, Qt::AlignTop);  
  grid->addWidget(remove_button, 1, 1, Qt::AlignBottom);

  possible_touch_links_ = new QListWidget(this);
  possible_touch_links_->setSelectionMode(QAbstractItemView::ExtendedSelection);
  for(unsigned int i = 0; i < link_names.size(); i++) {
    possible_touch_links_->addItem(link_names[i].c_str());
  }
  possible_touch_links_->addItem("------Groups---------");
  std::vector<std::string> group_names = kmodel->getJointModelGroupNames();
  for(unsigned int i = 0; i < group_names.size(); i++) {
    possible_touch_links_->addItem(group_names[i].c_str());
  }
  added_touch_links_ = new QListWidget(this);
  added_touch_links_->setSelectionMode(QAbstractItemView::ExtendedSelection);

  grid->addWidget(possible_touch_links_, 1, 0);
  grid->addWidget(added_touch_links_, 1, 2);

  layout->addWidget(grid_panel);

  QDialogButtonBox* qdb = new QDialogButtonBox();
  QPushButton* cancel_button = new QPushButton("Cancel");
  QPushButton* attach_button = new QPushButton("Attach");
  qdb->addButton(cancel_button, QDialogButtonBox::RejectRole);
  qdb->addButton(attach_button, QDialogButtonBox::AcceptRole);
  connect(qdb, SIGNAL(accepted()), this, SLOT(accept()));
  connect(qdb, SIGNAL(rejected()), this, SLOT(reject()));

  layout->addWidget(qdb, Qt::AlignRight);

  setLayout(layout);
}

void AttachObjectAdditionDialog::attachObject(const std::string& name)
{
  object_to_attach_ = name;
  show();
}

void AttachObjectAdditionDialog::accept() 
{
  std::vector<std::string> touch_links;
  for(int i = 0; i < added_touch_links_->count(); i++) {
    touch_links.push_back(added_touch_links_->item(i)->text().toStdString());
  }
  attachCollisionObjectRequested(object_to_attach_, 
                                 attach_link_box_->currentText().toStdString(), 
                                 touch_links);
  hide();
}

void AttachObjectAdditionDialog::addTouchLinkClicked() 
{
  QList<QListWidgetItem *> l = possible_touch_links_->selectedItems();

  for(int i = 0; i < l.size(); i++) {
    bool found = false;
    for(int j = 0; j < added_touch_links_->count(); j++) {
      if(l[i]->text() == added_touch_links_->item(j)->text()) {
        found = true;
        break;
      }
    }
    if(!found && l[i]->text().toStdString() != std::string("------Groups---------")) {
      added_touch_links_->addItem(l[i]->text());
    }
  }
}

void AttachObjectAdditionDialog::removeTouchLinkClicked()
{
  qDeleteAll(added_touch_links_->selectedItems());
}

}
