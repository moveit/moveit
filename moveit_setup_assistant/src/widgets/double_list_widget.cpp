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
#include <QFormLayout>
#include <QLineEdit>
#include <QMessageBox>
#include <QString>
#include "double_list_widget.h"

namespace moveit_setup_assistant
{

// ******************************************************************************************
// 
// ******************************************************************************************
DoubleListWidget::DoubleListWidget( QWidget *parent, moveit_setup_assistant::MoveItConfigDataPtr config_data,
                                    QString long_name, QString short_name )
  :  QWidget( parent ), long_name_( long_name ), short_name_( short_name ), config_data_( config_data )
{
  // Basic widget container
  QVBoxLayout *layout = new QVBoxLayout( );

  // Label ------------------------------------------------
  title_ = new QLabel( "", this ); // specify the title from the parent widget
  QFont group_title_font( "Arial", 12, QFont::Bold );
  title_->setFont(group_title_font);
  layout->addWidget( title_ );
  
  // Double selection lists -------------------------------
  QHBoxLayout *hlayout = new QHBoxLayout();
  
  // Left column -------------------------------------------
  QVBoxLayout *column1 = new QVBoxLayout();

  // Label
  QLabel *column1_label = new QLabel( QString("Available ").append( short_name_ ).append( 's' ), this );
  column1->addWidget( column1_label );

  // Table
  data_table_ = new QTableWidget( this );
  data_table_->setColumnCount(1);
  data_table_->setSortingEnabled(true);
  column1->addWidget( data_table_ );

  // Table headers
  QStringList data_header_list;
  data_header_list.append( QString( " Names" ).prepend( short_name_ ));
  data_table_->setHorizontalHeaderLabels( data_header_list );

  // Add layouts
  hlayout->addLayout( column1 );

  // Center column ------------------------------------------
  QVBoxLayout *column2 = new QVBoxLayout();
  column2->setSizeConstraint( QLayout::SetFixedSize ); // constraint it
  
  // Right Arrow Button
  QPushButton *btn_right = new QPushButton( ">", this);
  btn_right->setMaximumSize(25, 80);
  connect( btn_right, SIGNAL( clicked() ), this, SLOT( selectDataButtonClicked() ) );
  column2->addWidget( btn_right );
  
  // Left Arrow Button
  QPushButton *btn_left = new QPushButton( "<", this);
  btn_left->setMaximumSize(25, 80);
  connect( btn_left, SIGNAL( clicked() ), this, SLOT( deselectDataButtonClicked() ) );
  column2->addWidget( btn_left );

  // Add layouts
  hlayout->addLayout( column2 );

  // Right column -------------------------------------------
  QVBoxLayout *column3 = new QVBoxLayout();

  // Label
  QLabel *column3_label = new QLabel( QString("Selected ").append( short_name_ ).append( "s" ), this );
  column3->addWidget( column3_label );

  // Table
  selected_data_table_ = new QTableWidget( this );
  selected_data_table_->setColumnCount(1);
  selected_data_table_->setSortingEnabled(true);
  column3->addWidget( selected_data_table_ );

  // Table Headers (use same)
  selected_data_table_->setHorizontalHeaderLabels( data_header_list );

  // Add layouts
  hlayout->addLayout( column3 );

  // End Double Selection List ---------------------------------
  layout->addLayout( hlayout );
  

  // Button controls -------------------------------------------
  QHBoxLayout *controls_layout = new QHBoxLayout();
  controls_layout->setContentsMargins( 0, 25, 0, 15 );

  // Spacer
  QWidget *spacer = new QWidget( this );
  spacer->setSizePolicy( QSizePolicy::Expanding, QSizePolicy::Preferred );
  controls_layout->addWidget( spacer );

  // Save
  QPushButton *btn_save = new QPushButton( "&Save", this );
  //btn_save->setMaximumWidth( 200 );
  connect( btn_save, SIGNAL(clicked()), this, SIGNAL( doneEditing() ) );
  controls_layout->addWidget( btn_save );
  controls_layout->setAlignment(btn_save, Qt::AlignRight);

  // Cancel
  QPushButton *btn_cancel = new QPushButton( "&Cancel", this );
  //btn_cancel->setMaximumWidth( 200 );
  connect( btn_cancel, SIGNAL(clicked()), this, SIGNAL( cancelEditing() ) );
  controls_layout->addWidget( btn_cancel );
  controls_layout->setAlignment(btn_cancel, Qt::AlignRight);
  
  // Add layout
  layout->addLayout( controls_layout );

  // Finish Layout --------------------------------------------------
  this->setLayout(layout);
}

// ******************************************************************************************
// Set the left box
// ******************************************************************************************
void DoubleListWidget::setAvailable( const std::vector<std::string> &items )
{
  setTable( items, data_table_ );

  // Resize both tables
  data_table_->resizeColumnToContents(0);
  selected_data_table_->setColumnWidth( 0, data_table_->columnWidth( 0 ) );
}

// ******************************************************************************************
// Set the right box
// ******************************************************************************************
void DoubleListWidget::setSelected( const std::vector<std::string> &items )
{
  setTable( items, selected_data_table_ );
}

// ******************************************************************************************
// Convenience function for reusing set table code
// ******************************************************************************************
void DoubleListWidget::setTable( const std::vector<std::string> &items, QTableWidget *table )
{
  // Disable Table
  table->setUpdatesEnabled(false); // prevent table from updating until we are completely done
  table->setDisabled(true); // make sure we disable it so that the cellChanged event is not called
  table->clearContents();

  // Set size of datatable
  table->setRowCount( items.size() ); 

  // Loop through every joint
  int row = 0;
  for( std::vector<std::string>::const_iterator data_it = items.begin(); data_it != items.end(); ++data_it )
  {
    // Create row elements
    QTableWidgetItem* data_name = new QTableWidgetItem( data_it->c_str() );
    data_name->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

    // Add to table
    table->setItem( row, 0, data_name );
    
    // Increment counter
    ++row;
  }

  // Reenable
  table->setUpdatesEnabled(true); // prevent table from updating until we are completely done
  table->setDisabled(false); // make sure we disable it so that the cellChanged event is not called
}

// ******************************************************************************************
// Move selected data right
// ******************************************************************************************
void DoubleListWidget::selectDataButtonClicked()
{
  // Get list of all selected items
  QList<QTableWidgetItem*> selected = data_table_->selectedItems();

  // Loop through all selected items
  for(int i = 0; i < selected.size(); i++)
  {
    std::string name = selected[i]->text().toStdString();
    bool alreadyExists = false;
    int rowToAdd = 0;

    // Check if this selected joint is already in the selected joint table
    for(int r = 0; r < selected_data_table_->rowCount(); r++)
    {
      QTableWidgetItem* item = selected_data_table_->item(r, 0);

      if(item->text().toStdString() == name)
      {
        alreadyExists = true;
        break;
      }
      rowToAdd = r + 1;
    }

    // This joint needs to be added to the selected joint table
    if(!alreadyExists)
    {
      selected_data_table_->setRowCount(selected_data_table_->rowCount() + 1);
      QTableWidgetItem* newItem = new QTableWidgetItem(name.c_str());
      newItem->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
      selected_data_table_->setItem(rowToAdd, 0, newItem);
    }
  }

  //TODO?
  /*
    if(selected_data_table_->rowCount() > 0) {
    first_data_field_->setText("has");
    }
  */

}

// ******************************************************************************************
// Move selected data left
// ******************************************************************************************
void DoubleListWidget::deselectDataButtonClicked()
{
  // Get list of joints to be removed from selected list
  QList<QTableWidgetItem*> deselected = selected_data_table_->selectedItems();

  // loop through deselect list and remove
  for(int i = 0; i < deselected.size(); i++)
  {
    selected_data_table_->removeRow(deselected[i]->row());
  }

  // TODO?
  /*
    if(selected_data_table_->rowCount() == 0) {
    first_data_field_->clear();
    }
  */
}


} //namespace moveit_setup_assistant


