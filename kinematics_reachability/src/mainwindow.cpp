#include <ros/ros.h>
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <tf/transform_datatypes.h>
#include <iostream>
#include "angles/angles.h"


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui_(new Ui::MainWindow)
{
  ui_->setupUi(this);

  //Hide tool offset on startup
  ui_->edit_text_offset_x->hide();
  ui_->edit_text_offset_y->hide();
  ui_->edit_text_offset_z->hide();
  ui_->label_offset_x->hide();
  ui_->label_offset_y->hide();
  ui_->label_offset_z->hide();
  ui_->edit_text_offset_roll->hide();
  ui_->edit_text_offset_pitch->hide();
  ui_->edit_text_offset_yaw->hide();
  ui_->label_offset_roll->hide();
  ui_->label_offset_pitch->hide();
  ui_->label_offset_yaw->hide();
  ui_->progress_bar->hide();


  //Set default text in name fields
  ui_->frame_id_label->setText("");
  ui_->name_label->setText("");

  ros::NodeHandle node_handle("~");
  std::string group_name, frame_id;
  node_handle.param<std::string>("group", group_name, std::string());
  node_handle.param<std::string>("frame_id", frame_id, std::string());

  ROS_INFO("Group name: %s",group_name.c_str());
  ROS_INFO("Frame id: %s",frame_id.c_str());
  
  //Set from ROS parameters
  ui_->name_label->setText(group_name.c_str());
  ui_->frame_id_label->setText(frame_id.c_str());

  if(!reachability_solver_.initialize())
    ROS_ERROR("Could not initialize reachability solver");
  workspace_.group_name = group_name;
  workspace_.header.frame_id = frame_id;    

  workspace_subscriber_ = node_handle.subscribe("workspace_recorded", 1, &MainWindow::bagCallback, this);
  progress_subscriber_ = node_handle.subscribe("planner_progress", 1, &MainWindow::updateProgressBar, this);
  ros::spinOnce();
  ui_->progress_bar->setMinimum(0);
  ui_->progress_bar->setValue(0);
  ui_->progress_bar->reset();
}

MainWindow::~MainWindow()
{
  delete ui_;
}

void MainWindow::addRow()
{
  QString roll = ui_->edit_text_roll->text();
  QString pitch = ui_->edit_text_pitch->text();
  QString yaw = ui_->edit_text_yaw->text();

  MainWindow::addRow(roll, pitch, yaw);

  geometry_msgs::Quaternion quaternion;
  quaternion = tf::createQuaternionMsgFromRollPitchYaw(angles::from_degrees(roll.toDouble()),angles::from_degrees(pitch.toDouble()),angles::from_degrees(yaw.toDouble()));
  workspace_.orientations.push_back(quaternion);

}

void MainWindow::addRow(QString roll, QString pitch, QString yaw)
{
  ui_->table_widget->setColumnCount(3);
  ui_->table_widget->setHorizontalHeaderLabels(QString("Roll;Pitch;Yaw").split(";"));
  ui_->table_widget->insertRow(ui_->table_widget->rowCount());

  //QString arm_roll = ui_->edit_text_roll->text();
  //QString arm_pitch = ui_->edit_text_pitch->text();
  //QString arm_yaw = ui_->edit_text_yaw->text();

  ui_->table_widget->setItem(ui_->table_widget->rowCount() -1, 0, new QTableWidgetItem(roll));
  ui_->table_widget->setItem(ui_->table_widget->rowCount() -1, 1, new QTableWidgetItem(pitch));
  ui_->table_widget->setItem(ui_->table_widget->rowCount() -1, 2, new QTableWidgetItem(yaw));

  ui_->edit_text_roll->clear();
  ui_->edit_text_pitch->clear();
  ui_->edit_text_yaw->clear();


    
}

void MainWindow::compute()
{

  ui_->progress_bar->setVisible(true);
  ui_->compute_button->hide();
  workspace_.points.clear();  
  MainWindow::setBoundaries(workspace_);
  //    MainWindow::close();
  /**** WORKSPACE PARAMETERS - These are the parameters you need to change to specify a different 
        region in the workspace for which reachability is to be computed****/
  while(!reachability_solver_.isActive())
  {
    sleep(1.0);
    ROS_INFO("Waiting for planning scene to be set");
  }          
  reachability_solver_.computeWorkspace(workspace_, true);
  reachability_solver_.visualize(workspace_,"solutions");
  reachability_solver_.animateWorkspace(workspace_);
  reachability_solver_.publishWorkspace(workspace_);
  ROS_INFO("Success");  
  ui_->progress_bar->setMaximum(100);
  ui_->progress_bar->setValue(100);
  ui_->progress_bar->hide();
  ui_->compute_button->setVisible(true);
  //    ros::waitForShutdown();  
}

void MainWindow::showOffset(bool checked)
{
  ui_->edit_text_offset_x->setVisible(checked);
  ui_->edit_text_offset_y->setVisible(checked);
  ui_->edit_text_offset_z->setVisible(checked);
  ui_->label_offset_x->setVisible(checked);
  ui_->label_offset_y->setVisible(checked);
  ui_->label_offset_z->setVisible(checked);
  ui_->edit_text_offset_roll->setVisible(checked);
  ui_->edit_text_offset_pitch->setVisible(checked);
  ui_->edit_text_offset_yaw->setVisible(checked);
  ui_->label_offset_roll->setVisible(checked);
  ui_->label_offset_pitch->setVisible(checked);
  ui_->label_offset_yaw->setVisible(checked);
}

void MainWindow::visualiseWorkspace()
{
  MainWindow::setBoundaries(workspace_);
  reachability_solver_.visualizeWorkspaceSamples(workspace_);
  ROS_INFO("Samples visualised.");
}

void MainWindow::setBoundaries(kinematics_reachability::WorkspacePoints &workspace)
{
  double origin_x = ui_->edit_text_origin_x->text().toDouble();
  double origin_y = ui_->edit_text_origin_y->text().toDouble();
  double origin_z = ui_->edit_text_origin_z->text().toDouble();

  double min_corner_x = origin_x - (ui_->edit_text_size_x->text().toDouble()/2.0);
  double min_corner_y = origin_y - (ui_->edit_text_size_y->text().toDouble()/2.0);
  double min_corner_z = origin_z - (ui_->edit_text_size_z->text().toDouble()/2.0);

  double max_corner_x = origin_x + (ui_->edit_text_size_x->text().toDouble()/2.0);
  double max_corner_y = origin_y + (ui_->edit_text_size_y->text().toDouble()/2.0);
  double max_corner_z = origin_z + (ui_->edit_text_size_z->text().toDouble()/2.0);

  workspace.parameters.min_corner.x = min_corner_x;
  workspace.parameters.min_corner.y = min_corner_y;
  workspace.parameters.min_corner.z = min_corner_z;

  workspace.parameters.max_corner.x = max_corner_x;
  workspace.parameters.max_corner.y = max_corner_y;
  workspace.parameters.max_corner.z = max_corner_z;

  double resolution = ui_->edit_text_resolution->text().toDouble();
  workspace.position_resolution = resolution;

  geometry_msgs::Pose tool_frame_offset;
  double offset_roll = 0;
  double offset_pitch = 0;
  double offset_yaw = 0;

  double offset_x = 0; 
  double offset_y = 0;
  double offset_z = 0;

  bool checked = ui_->tool_offset_enabled->isChecked();
  if (checked)
  {
    offset_roll = angles::from_degrees(ui_->edit_text_offset_roll->text().toDouble());
    offset_pitch = angles::from_degrees(ui_->edit_text_offset_pitch->text().toDouble());
    offset_yaw = angles::from_degrees(ui_->edit_text_offset_yaw->text().toDouble());

    offset_x = ui_->edit_text_offset_x->text().toDouble(); 
    offset_y = ui_->edit_text_offset_y->text().toDouble();
    offset_z = ui_->edit_text_offset_z->text().toDouble();
  }
  tool_frame_offset.orientation = tf::createQuaternionMsgFromRollPitchYaw(offset_roll,offset_pitch,offset_yaw);
  tool_frame_offset.position.x = offset_x;
  tool_frame_offset.position.y = offset_y;
  tool_frame_offset.position.z = offset_z;
  workspace.tool_frame_offset = tool_frame_offset;    
}

void MainWindow::setUI(const kinematics_reachability::WorkspacePoints &workspace)
{

  for (int i=0; i < ui_->table_widget->rowCount(); i++)
    ui_->table_widget->removeRow(i);

  for(int n=0; n<workspace_.orientations.size(); n++)
  {
    double roll, pitch, yaw;
    tf::Quaternion q;
    tf::quaternionMsgToTF(workspace.orientations[n], q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    QString q_roll = QString::number(angles::to_degrees(roll));
    QString q_pitch = QString::number(angles::to_degrees(pitch));
    QString q_yaw = QString::number(angles::to_degrees(yaw));

    MainWindow::addRow(q_roll, q_pitch, q_yaw);
  }

  double resolution = workspace.position_resolution;
  ui_->edit_text_resolution->setText(QString::number(resolution));

  MainWindow::loadBoundaries(workspace);

  double offset_roll, offset_pitch, offset_yaw;

  tf::Quaternion q;
  tf::quaternionMsgToTF(workspace.tool_frame_offset.orientation, q);
  tf::Matrix3x3(q).getRPY(offset_roll, offset_pitch, offset_yaw);

  double offset_x = workspace.tool_frame_offset.position.x;
  double offset_y = workspace.tool_frame_offset.position.y;
  double offset_z = workspace.tool_frame_offset.position.z;

  

  if(!((offset_x && offset_y && offset_z && offset_roll && offset_pitch && offset_yaw) == 0.0))
  {

    ui_->tool_offset_enabled->setChecked(true);
    MainWindow::showOffset(true);

    QString q_offset_roll = QString::number(angles::to_degrees(offset_roll));
    QString q_offset_pitch = QString::number(angles::to_degrees(offset_pitch));
    QString q_offset_yaw = QString::number(angles::to_degrees(offset_yaw));

    ui_->edit_text_offset_roll->setText(q_offset_roll);
    ui_->edit_text_offset_pitch->setText(q_offset_pitch);
    ui_->edit_text_offset_yaw->setText(q_offset_yaw);

    QString q_offset_x = QString::number(offset_x);
    QString q_offset_y = QString::number(offset_y);
    QString q_offset_z = QString::number(offset_z);

    ui_->edit_text_offset_x->setText(q_offset_x);
    ui_->edit_text_offset_y->setText(q_offset_y);
    ui_->edit_text_offset_z->setText(q_offset_z);    
       
  }

}

void MainWindow::bagCallback(const kinematics_reachability::WorkspacePointsConstPtr &msg)
{
  workspace_ = *msg;
  setUI(workspace_);  
  reachability_solver_.visualizeWorkspaceSamples(workspace_);
  reachability_solver_.visualize(workspace_,"bag");
  reachability_solver_.animateWorkspace(workspace_);
  ROS_INFO("Samples visualised.");
}

void MainWindow::updateProgressBar(const kinematics_reachability::ProgressConstPtr &msg)
{
  ui_->progress_bar->setMaximum(msg->total);
  ui_->progress_bar->setValue(msg->current); 
}

void MainWindow::loadBoundaries(kinematics_reachability::WorkspacePoints workspace)
{

  double min_corner_x = workspace.parameters.min_corner.x;
  double min_corner_y = workspace.parameters.min_corner.y;
  double min_corner_z = workspace.parameters.min_corner.z; 

  double max_corner_x = workspace.parameters.max_corner.x;
  double max_corner_y = workspace.parameters.max_corner.y;
  double max_corner_z = workspace.parameters.max_corner.z;

  ui_->edit_text_size_x->setText(QString::number(max_corner_x - min_corner_x));
  ui_->edit_text_size_y->setText(QString::number(max_corner_y - min_corner_y));
  ui_->edit_text_size_z->setText(QString::number(max_corner_z - min_corner_z));  

  ui_->edit_text_origin_x->setText(QString::number(max_corner_x - (max_corner_x - min_corner_x)/2.0));
  ui_->edit_text_origin_y->setText(QString::number(max_corner_y - (max_corner_y - min_corner_y)/2.0));
  ui_->edit_text_origin_z->setText(QString::number(max_corner_z - (max_corner_z - min_corner_z)/2.0));
}

