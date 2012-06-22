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

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QPushButton>
#include <QGridLayout>
#include <QLineEdit>
#include <QMessageBox>
#include <QString>
#include "kinematic_chain_widget.h"

// ******************************************************************************************
// 
// ******************************************************************************************
KinematicChainWidget::KinematicChainWidget( QWidget *parent, moveit_setup_assistant::MoveItConfigDataPtr config_data )
  :  QWidget( parent ), config_data_( config_data )
{
  // Basic widget container
  QVBoxLayout *layout = new QVBoxLayout( );

  // Label ------------------------------------------------
  title_ = new QLabel( "", this ); // specify the title from the parent widget
  QFont group_title_font( "Arial", 12, QFont::Bold );
  title_->setFont(group_title_font);
  layout->addWidget( title_ );
  
  // Simple form -------------------------------------------
  /* QFormLayout *form_layout = new QFormLayout();
     form_layout->setContentsMargins( 0, 15, 0, 15 );

     // Name input
     name_input_ = new QLineEdit( this );
     name_input_->setMaximumWidth( 400 );
     form_layout->addRow( "Data Collection Name:", name_input_ );
  
     layout->addLayout( form_layout );
     layout->setAlignment( Qt::AlignTop );
  */

  
  // Create link tree ------------------------------------------------------
  link_tree_ = new QTreeWidget( this );
  link_tree_->setHeaderLabel( "Robot Links" );
  //connect(link_tree_, SIGNAL(itemSelectionChanged()), this, SLOT(showTreeLink()));
  layout->addWidget( link_tree_ );

  // Create Grid Layout for form --------------------------------------------
  QGridLayout *form_grid = new QGridLayout();
  form_grid->setContentsMargins( 20, 20, 20, 20 ); // left top right bottom
  
  // Row 1: Base Link
  QLabel *base_link_label = new QLabel( "Base Link", this );
  form_grid->addWidget( base_link_label, 0, 0, Qt::AlignRight );

  base_link_field_ = new QLineEdit( this );
  base_link_field_->setMinimumWidth( 200 );
  form_grid->addWidget( base_link_field_, 0, 1, Qt::AlignLeft );

  QPushButton *btn_base_link = new QPushButton( "Choose Selected", this );
  connect( btn_base_link, SIGNAL( clicked() ), this, SLOT( baseLinkTreeClick() ));
  form_grid->addWidget( btn_base_link, 0, 2, Qt::AlignLeft );

  // Add form grid layout
  layout->addLayout( form_grid );

  // Bottom Controls ---------------------------------------------------------
  QHBoxLayout *controls_layout = new QHBoxLayout();

  // Spacer
  QWidget *spacer = new QWidget( this );
  spacer->setSizePolicy( QSizePolicy::Expanding, QSizePolicy::Preferred );
  controls_layout->addWidget( spacer );

  // Save
  QPushButton *btn_save = new QPushButton( "&Save", this );
  btn_save->setMaximumWidth( 200 );
  connect( btn_save, SIGNAL(clicked()), this, SIGNAL( doneEditing() ) );
  controls_layout->addWidget( btn_save );
  controls_layout->setAlignment(btn_save, Qt::AlignRight);

  // Cancel
  QPushButton *btn_cancel = new QPushButton( "&Cancel", this );
  btn_cancel->setMaximumWidth( 200 );
  connect( btn_cancel, SIGNAL(clicked()), this, SIGNAL( cancelEditing() ) );
  controls_layout->addWidget( btn_cancel );
  controls_layout->setAlignment(btn_cancel, Qt::AlignRight);
  
  // Add layout
  layout->addLayout( controls_layout );

  // Finish Layout --------------------------------------------------
  this->setLayout(layout);
}

// ******************************************************************************************
// Load robot links into tree
// ******************************************************************************************
void KinematicChainWidget::setAvailable()
{
  //const KinematicModel::JointModel* rootJoint = config_data_->getKinematicModel->getRoot();
  //addLinktoTreeRecursive(rootJoint->getChildLinkModel(), NULL);

  /* link_tree_->expandToDepth(0);
  link_tree_->header()->setResizeMode(0, QHeaderView::ResizeToContents);
  link_tree_->header()->setStretchLastSection(false);*/
}

// ******************************************************************************************
//
// ******************************************************************************************
/*void KinematicChainWizardPage::addLinktoTreeRecursive(const KinematicModel::LinkModel* link,
                                                      const KinematicModel::LinkModel* parent)
{
  QTreeWidgetItem* toAdd = new QTreeWidgetItem(link_tree_);
  if(parent == NULL)
  {
    toAdd->setText(0, link->getName().c_str());
    link_tree_->addTopLevelItem(toAdd);
  }
  else
  {
    for(int i = 0; i < link_tree_->topLevelItemCount(); i++)
    {
      if(addLinkChildRecursive(link_tree_->topLevelItem(i), link, parent->getName()))
      {
        break;
      }
    }
  }
  for(size_t i = 0; i < link->getChildJointModels().size(); i++)
  {
    addLinktoTreeRecursive(link->getChildJointModels()[i]->getChildLinkModel(), link);
  }
}

// ******************************************************************************************
//
// ******************************************************************************************
bool KinematicChainWizardPage::addLinkChildRecursive(QTreeWidgetItem* parent,
                                                     const KinematicModel::LinkModel* link,
                                                     const string& parentName)
{
  if(parent->text(0).toStdString() == parentName)
  {
    QTreeWidgetItem* toAdd = new QTreeWidgetItem(parent);
    toAdd->setText(0, link->getName().c_str());
    parent->addChild(toAdd);
    return true;
  }
  else
  {
    for(int i = 0; i < parent->childCount(); i++)
    {
      if(addLinkChildRecursive(parent->child(i), link, parentName))
      {
        return true;
      }
    }
  }

  return false;
}
*/
// ******************************************************************************************
// Set the links
// ******************************************************************************************
void KinematicChainWidget::setSelected( const std::string &base_link, const std::string &tip_link )
{
  base_link_field_->setText( QString( base_link.c_str() ) );
  tip_link_field_->setText( QString( tip_link.c_str() ) );
}

// ******************************************************************************************
// Choose the base link
// ******************************************************************************************
void KinematicChainWidget::baseLinkTreeClick() {
  QTreeWidgetItem* item = link_tree_->currentItem();
  if(item != NULL)
  {
    base_link_field_->setText(item->text(0));
  }
}

// ******************************************************************************************
// Choose the tip link
// ******************************************************************************************
void KinematicChainWidget::tipLinkTreeClick()
{
  QTreeWidgetItem* item = link_tree_->currentItem();
  if(item != NULL)
  {
    tip_link_field_->setText(item->text(0));
  }
}
