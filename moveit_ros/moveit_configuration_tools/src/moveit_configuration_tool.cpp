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

// Author: Dave Coleman

//#include <moveit_configuration_tools/moveit_configuration_tool.h>
#include <QApplication>
#include <QPushButton>
#include <QTableWidget>
//#include <qt4/QtGui/qradiobutton.h>
//#include <qt4/QtGui/qdialogbuttonbox.h>
//#include <qt4/QtCore/qtextstream.h>
//#include <QHeaderView>
//#include <boost/thread.hpp>

using namespace std;


int main(int argv, char **args)
{
  QApplication app(argv, args);

  //parent_ = parent;
  std::string title = "Link Pairs";
  std::string subtitle  = "The following links are always in collision over the sample space.\nClick on a link name to visualize the collision in Rviz";

  //setTitle(title.c_str());

  QVBoxLayout* layout = new QVBoxLayout();
  //setSubTitle(subtitle.c_str());

  QPushButton* generateButton = new QPushButton();
  generateButton->setText("Generate List (May take a minute)");
  layout->addWidget(generateButton);

  //connect(generateButton, SIGNAL(clicked()), this, SLOT(quit())); //SLOT(generateCollisionTable()));
  generateButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);

  QTableWidget* collision_table_ = new QTableWidget();
  layout->addWidget(collision_table_);

  QPushButton* toggle_selected = new QPushButton();
  toggle_selected->setText("Toggle Selected");
  layout->addWidget(toggle_selected);
  //connect(toggle_selected, SIGNAL(clicked()), this, SLOT(toggleTable()));
  toggle_selected->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);

  //  connect(collision_table_, SIGNAL(cellClicked(int, int)), this, SLOT(tableClicked()));


  ComputeDefaultCollisionMatrixWidget cdcm;
  cdcm.show();

  //QWidget window;
  //window.setLayout(layout);
  //window.show();

  return app.exec();
}
