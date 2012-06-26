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

#ifndef _ATTACH_OBJECT_ADDITION_DIALOG_H_
#define _ATTACH_OBJECT_ADDITION_DIALOG_H_

#include <QDialog>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSpinBox>
#include <QComboBox>
#include <QListWidget>
#include <planning_models/kinematic_model.h>

namespace moveit_visualization_ros
{

class AttachObjectAdditionDialog: public QDialog
{
  Q_OBJECT

  public:
  
  AttachObjectAdditionDialog(QWidget* parent,
                             const planning_models::KinematicModelConstPtr& kmodel);
public Q_SLOTS:

  virtual void accept();
  void attachObject(const std::string& name);
  void addTouchLinkClicked();
  void removeTouchLinkClicked();


Q_SIGNALS:

  void attachCollisionObjectRequested(const std::string& name,
                                      const std::string& link_name,
                                      const std::vector<std::string>& touch_links);
protected:

  std::string object_to_attach_;

  QComboBox* attach_link_box_;
  QListWidget* possible_touch_links_;
  QListWidget* added_touch_links_;

};

}
#endif
