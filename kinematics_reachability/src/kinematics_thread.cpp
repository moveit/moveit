#include <QMainWindow>
#include <kinematics_reachability/kinematics_reachability.h>
#include <ros/ros.h>
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <tf/transform_datatypes.h>
#include <iostream>
#include "angles/angles.h"
#include "kinematics_thread.h"



/*class KinematicsThread : public QThread
{

public:
  void run();
Q_SIGNALS:
  void currentProgressSignal(int current);
  void maxProgresSignal(int total);  
  void setNameLabel(QString group_name, QString frame_id);
  void setMinimumProgress(int minimum_progress);
  //void resetProgressBar();
  void setUISignal(double *orientations, double resolution, double min_x, double min_y, double min_z, double max_x, double max_y, double max_z, double offset_x, double offset_y, double offset_z, double offset_roll, double offset_pitch, double offset_yaw);
  //void addRow(double, double, double);
  void doneComputing();

private Q_SLOTS:
  void addOrientation(QString roll, QString pitch, QString yaw);
  void computeKinematics(double min_x, double min_y, double min_z, double max_x, double max_y, double max_z, double resolution, double offset_roll, double offset_pitch, double offset_yaw, double offset_x, double offset_y, double offset_z);
  void visualise(double min_x, double min_y, double min_z, double max_x, double max_y, double max_z, double resolution, double offset_roll, double offset_pitch, double offset_yaw, double offset_x, double offset_y, double offset_z);

private:
  kinematics_reachability::WorkspacePoints workspace_;
  kinematics_reachability::KinematicsReachability reachability_solver_;
  ros::Subscriber workspace_subscriber_;
  ros::Subscriber progress_subscriber_;

  void setBoundaries(kinematics_reachability::WorkspacePoints &workspace, double min_x, double min_y, double min_z, double max_x, double max_y, double max_z, double resolution, double offset_roll, double offset_pitch, double offset_yaw, double offset_x, double offset_y, double offset_z);
  void ThreadKinematics::bagCallback(const kinematics_reachability::WorkspacePointsConstPtr &msg);
  void ThreadKinematics::updateProgressBar(const kinematics_reachability::ProgressConstPtr &msg);
  void KinematicsThread::setUI(const kinematics_reachability::WorkspacePoints &workspace);

};
*/

namespace kinematics_thread
{

KinematicsThread::KinematicsThread()
{
}

void KinematicsThread::initialise()
{

  //Set default text in name fields
  //ui_->frame_id_label->setText("");
  //ui_->name_label->setText("");

  ros::NodeHandle node_handle("~");
  std::string group_name, frame_id;
  node_handle.param<std::string>("group", group_name, std::string());
  node_handle.param<std::string>("frame_id", frame_id, std::string());

  ROS_INFO("Group name: %s",group_name.c_str());
  ROS_INFO("Frame id: %s",frame_id.c_str());
  
  //Set from ROS parameters
  //ui_->name_label->setText(group_name.c_str());
  //ui_->frame_id_label->setText(frame_id.c_str());

  QString q_name = QString::fromUtf8(group_name.data(), group_name.size());
  QString q_frame_id = QString::fromUtf8(frame_id.data(), frame_id.size());

  //QString q_name = "blah";
  //QString q_frame_id = "blah blah";
  Q_EMIT setFrameIdLabel(q_frame_id);
  Q_EMIT setNameLabel(q_name);
  
  if(!reachability_solver_.initialize())
    ROS_ERROR("Could not initialize reachability solver");
  workspace_.group_name = group_name;
  workspace_.header.frame_id = frame_id;    

  workspace_subscriber_ = node_handle.subscribe("workspace_recorded", 1, &KinematicsThread::bagCallback, this);
  progress_subscriber_ = node_handle.subscribe("planner_progress", 1, &KinematicsThread::updateProgressBar, this);
  ros::spinOnce();
  //ui_->progress_bar->setMinimum(0);
  //ui_->progress_bar->setValue(0);
  //ui_->progress_bar->reset();
  //Q_EMIT setMinimumProgress(0);
  //Q_EMIT resetProgressBar();

}

KinematicsThread::~KinematicsThread()
{
}

void KinematicsThread::addOrientation(QString roll, QString pitch, QString yaw)
{
  //QString roll = ui_->edit_text_roll->text();
  //QString pitch = ui_->edit_text_pitch->text();
  //QString yaw = ui_->edit_text_yaw->text();

  //MainWindow::addRow(roll, pitch, yaw);
  //Q_EMIT addRow(roll, pitch, yaw);

  geometry_msgs::Quaternion quaternion;
  quaternion = tf::createQuaternionMsgFromRollPitchYaw(angles::from_degrees(roll.toDouble()),angles::from_degrees(pitch.toDouble()),angles::from_degrees(yaw.toDouble()));
  workspace_.orientations.push_back(quaternion);

}

/*void MainWindow::addRow(QString roll, QString pitch, QString yaw)
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
*/
void KinematicsThread::computeKinematics(double min_x, double min_y, double min_z, double max_x, double max_y, double max_z, double resolution, double offset_roll, double offset_pitch, double offset_yaw, double offset_x, double offset_y, double offset_z)
{
  //QObject::connect(MainWindow::MainWindow, SIGNAL(max_progress_signal(int)), ui_->progress_bar, SLOT(setMaximum(int)));
  //QObject::connect(MainWindow::MainWindow, SIGNAL(current_progress_signal(int)), ui_->progress_bar, SLOT(setValue(int)));

  //QMetaObject::connectSlotsByName(MainWindow::MainWindow);

  //ui_->progress_bar->setVisible(true);
  //ui_->compute_button->hide();

  //Q_EMIT showProgressBar(true);
  //Q_EMIT hideComputeButton();

  workspace_.points.clear();  
  KinematicsThread::setBoundaries(workspace_, min_x, min_y, min_z, max_x, max_y, max_z, resolution, offset_roll, offset_pitch, offset_yaw, offset_x, offset_y, offset_z);
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
  Q_EMIT doneComputing();
  //ui_->progress_bar->setMaximum(100);
  //ui_->progress_bar->setValue(100);
  //ui_->progress_bar->hide();
  
  //    ros::waitForShutdown();  
}

/*void MainWindow::showOffset(bool checked)
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
*/

void KinematicsThread::visualise(double min_x, double min_y, double min_z, double max_x, double max_y, double max_z, double resolution, double offset_roll, double offset_pitch, double offset_yaw, double offset_x, double offset_y, double offset_z)
{
  KinematicsThread::setBoundaries(workspace_, min_x, min_y, min_z, max_x, max_y, max_z, resolution, offset_roll, offset_pitch, offset_yaw, offset_x, offset_y, offset_z);
  reachability_solver_.visualizeWorkspaceSamples(workspace_);
  ROS_INFO("Samples visualised.");
}

void KinematicsThread::setBoundaries(kinematics_reachability::WorkspacePoints &workspace, double min_x, double min_y, double min_z, double max_x, double max_y, double max_z, double resolution, double offset_roll, double offset_pitch, double offset_yaw, double offset_x, double offset_y, double offset_z)
{
  /*double origin_x = ui_->edit_text_origin_x->text().toDouble();
  double origin_y = ui_->edit_text_origin_y->text().toDouble();
  double origin_z = ui_->edit_text_origin_z->text().toDouble();

  double min_corner_x = origin_x - (ui_->edit_text_size_x->text().toDouble()/2.0);
  double min_corner_y = origin_y - (ui_->edit_text_size_y->text().toDouble()/2.0);
  double min_corner_z = origin_z - (ui_->edit_text_size_z->text().toDouble()/2.0);

  double max_corner_x = origin_x + (ui_->edit_text_size_x->text().toDouble()/2.0);
  double max_corner_y = origin_y + (ui_->edit_text_size_y->text().toDouble()/2.0);
  double max_corner_z = origin_z + (ui_->edit_text_size_z->text().toDouble()/2.0);
  */

  
  

  workspace.parameters.min_corner.x = min_x;
  workspace.parameters.min_corner.y = min_y;
  workspace.parameters.min_corner.z = min_z;

  workspace.parameters.max_corner.x = max_x;
  workspace.parameters.max_corner.y = max_y;
  workspace.parameters.max_corner.z = max_z;

  //double resolution = ui_->edit_text_resolution->text().toDouble();
  workspace.position_resolution = resolution;

  geometry_msgs::Pose tool_frame_offset;
  //double offset_roll_rad = 0;
  //double offset_pitch_rad = 0;
  //double offset_yaw_rad = 0;

  /*double offset_x = 0; 
  double offset_y = 0;
  double offset_z = 0;
  */
  //bool checked = ui_->tool_offset_enabled->isChecked();
  //if (checked)
  //{
  double offset_roll_rad = angles::from_degrees(offset_roll);
  double offset_pitch_rad = angles::from_degrees(offset_pitch);
  double offset_yaw_rad = angles::from_degrees(offset_yaw);
  //double offset_x = ui_->edit_text_offset_x->text().toDouble(); 
  //double offset_y = ui_->edit_text_offset_y->text().toDouble();
  //double offset_z = ui_->edit_text_offset_z->text().toDouble();
  //}
  tool_frame_offset.orientation = tf::createQuaternionMsgFromRollPitchYaw(offset_roll_rad,offset_pitch_rad,offset_yaw_rad);
  tool_frame_offset.position.x = offset_x;
  tool_frame_offset.position.y = offset_y;
  tool_frame_offset.position.z = offset_z;
  workspace.tool_frame_offset = tool_frame_offset;    
}

void KinematicsThread::setUI(const kinematics_reachability::WorkspacePoints &workspace)
{


  //for (int i=0; i < ui_->table_widget->rowCount(); i++)
  //  ui_->table_widget->removeRow(i);

  //int size = workspace.orientations.size();

  /*for(int n=0; n<workspace_.orientations.size(); n++)
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
  */
  //double resolution = workspace.position_resolution;
  //ui_->edit_text_resolution->setText(QString::number(resolution));

//  KinematicsThread::loadBoundaries(workspace);

  double offset_roll, offset_pitch, offset_yaw;

  tf::Quaternion q;
  tf::quaternionMsgToTF(workspace.tool_frame_offset.orientation, q);
  tf::Matrix3x3(q).getRPY(offset_roll, offset_pitch, offset_yaw);

  /*double offset_x = workspace.tool_frame_offset.position.x;
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
       
  }*/

  //Q_EMIT setUISignal(workspace.orientations, size, workspace.position_resolution, workspace.parameters.min_corner.x, workspace.parameters.min_corner.y, workspace.parameters.min_corner.z, workspace.parameters.max_corner.x, workspace.parameters.max_corner.y, workspace.parameters.max_corner.z, workspace.tool_frame_offset.position.x, workspace.tool_frame_offset.position.y, workspace.tool_frame_offset.position.z, offset_roll, offset_pitch, offset_yaw); 
  Q_EMIT setUISignal(workspace);
}

void KinematicsThread::bagCallback(const kinematics_reachability::WorkspacePointsConstPtr &msg)
{
  workspace_ = *msg;
  setUI(workspace_);  
  reachability_solver_.visualizeWorkspaceSamples(workspace_);
  reachability_solver_.visualize(workspace_,"bag");
  reachability_solver_.animateWorkspace(workspace_);
  ROS_INFO("Samples visualised.");
}

void KinematicsThread::updateProgressBar(const kinematics_reachability::ProgressConstPtr &msg)
{

  Q_EMIT maxProgressSignal(msg->total);
  Q_EMIT currentProgressSignal(msg->current);

  //QApplication::processEvents();
  //ui_->progress_bar->setMaximum(msg->total);
  //ui_->progress_bar->setValue(msg->current); 
}
}
/*void MainWindow::loadBoundaries(kinematics_reachability::WorkspacePoints workspace)
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



 void KinematicsThread::run()
 {
    
      doSomeProcessing(data.nextItem())
      emit percentageCompleted(computePercentageCompleted());
 }*/
