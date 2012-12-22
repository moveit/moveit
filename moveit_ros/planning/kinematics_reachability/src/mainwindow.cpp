#include <ros/ros.h>
#include <moveit/kinematics_reachability/mainwindow.h>
//#include <moveit/kinematics_reachability/ui_mainwindow.h>
#include <ui_mainwindow.h>
#include <tf/transform_datatypes.h>
#include <iostream>
#include "angles/angles.h"
#include <moveit/kinematics_reachability/kinematics_thread.h>
#include <QThread>
#include <qapplication.h>
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui_(new Ui::MainWindow)
{
  ui_->setupUi(this);

  qRegisterMetaType<moveit_ros_planning::WorkspacePoints>("moveit_ros_planning::WorkspacePoints");

  kinematics_thread_.moveToThread(&thread_);
  thread_.start();

  QObject::connect(&kinematics_thread_,SIGNAL(currentProgressSignal(int)),ui_->progress_bar,SLOT(setValue(int)));
  QObject::connect(&kinematics_thread_,SIGNAL(maxProgressSignal(int)),ui_->progress_bar,SLOT(setMaximum(int)));
  QObject::connect(&kinematics_thread_,SIGNAL(setFrameIdLabel(QString)),ui_->frame_id_label,SLOT(setText(QString)));
  QObject::connect(&kinematics_thread_,SIGNAL(setNameLabel(QString)),ui_->name_label,SLOT(setText(QString)));
  QObject::connect(&kinematics_thread_,SIGNAL(setUISignal(const moveit_ros_planning::WorkspacePoints&)),this,SLOT(setUIFromWorkspace(moveit_ros_planning::WorkspacePoints)));
  QObject::connect(&kinematics_thread_,SIGNAL(doneComputing()),this,SLOT(Success()));
  QObject::connect(&kinematics_thread_,SIGNAL(sendWorkspace(const moveit_ros_planning::WorkspacePoints&)),this,SLOT(receiveWorkspace(const moveit_ros_planning::WorkspacePoints&)));
  //  QObject::connect(this,SIGNAL(addRowSignal(QString, QString, QString)),&kinematics_thread_,SLOT(addOrientation(QString, QString, QString)));
  QObject::connect(this,SIGNAL(startComputation(const moveit_ros_planning::WorkspacePoints&)),&kinematics_thread_,SLOT(computeKinematics(const moveit_ros_planning::WorkspacePoints&)));
  QObject::connect(this,SIGNAL(startVisualisation(const moveit_ros_planning::WorkspacePoints&)),&kinematics_thread_,SLOT(visualise(const moveit_ros_planning::WorkspacePoints&)));
  QObject::connect(this,SIGNAL(startFKComputation(const moveit_ros_planning::WorkspacePointkinematics_reachability::WorkspacePoints&, double)),&kinematics_thread_,SLOT(computeFK(const moveit_ros_planning::WorkspacePoints&, double)));
  //QObject::connect(this,SIGNAL(cancelComputationIK()),&kinematics_thread_,SLOT(stopSolver()));

  kinematics_thread_.initialise();


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

  //  Q_EMIT addRowSignal(roll, pitch, yaw);

  MainWindow::addRow(roll, pitch, yaw);

}

void MainWindow::addRow(QString roll, QString pitch, QString yaw)
{
  ui_->table_widget->setColumnCount(3);
  ui_->table_widget->setHorizontalHeaderLabels(QString("Roll;Pitch;Yaw").split(";"));
  ui_->table_widget->insertRow(ui_->table_widget->rowCount());

  ui_->table_widget->setItem(ui_->table_widget->rowCount() -1, 0, new QTableWidgetItem(roll));
  ui_->table_widget->setItem(ui_->table_widget->rowCount() -1, 1, new QTableWidgetItem(pitch));
  ui_->table_widget->setItem(ui_->table_widget->rowCount() -1, 2, new QTableWidgetItem(yaw));

  ui_->edit_text_roll->clear();
  ui_->edit_text_pitch->clear();
  ui_->edit_text_yaw->clear();

  geometry_msgs::Quaternion quaternion;
  quaternion = tf::createQuaternionMsgFromRollPitchYaw(angles::from_degrees(roll.toDouble()),angles::from_degrees(pitch.toDouble()),angles::from_degrees(yaw.toDouble()));
  workspace_.orientations.push_back(quaternion);
}

void MainWindow::compute()
{
  ui_->compute_button->hide();
  ui_->progress_bar->setVisible(true);
 
  MainWindow::setWorkspace();
  //Q_EMIT startComputation(min_x, min_y, min_z, max_x, max_y, max_z, resolution, offset_roll, offset_pitch, offset_yaw, offset_x, offset_y, offset_z);
  kinematics_thread_.enableSolver();
  Q_EMIT startComputation(workspace_);
}

void MainWindow::Success()
{
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
  MainWindow::setWorkspace();
  //Q_EMIT startVisualisation(min_x, min_y, min_z, max_x, max_y, max_z, resolution, offset_roll, offset_pitch, offset_yaw, offset_x, offset_y, offset_z);
  Q_EMIT startVisualisation(workspace_);
}

void MainWindow::setUIFromWorkspace(const moveit_ros_planning::WorkspacePoints &workspace)
{

  workspace_.orientations.clear();

  int row_count = ui_->table_widget->rowCount();

  for (int i=0; i < row_count; i++)
    ui_->table_widget->removeRow(0);

  for(int n=0; n<workspace.orientations.size(); n++)
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

  MainWindow::loadBoundaries(workspace.parameters.min_corner.x, workspace.parameters.min_corner.y, workspace.parameters.min_corner.z, workspace.parameters.max_corner.x, workspace.parameters.max_corner.y, workspace.parameters.max_corner.z);

  double offset_roll, offset_pitch, offset_yaw;

  tf::Quaternion q;
  tf::quaternionMsgToTF(workspace.tool_frame_offset.orientation, q);
  tf::Matrix3x3(q).getRPY(offset_roll, offset_pitch, offset_yaw);

  double offset_x = workspace.tool_frame_offset.position.x;
  double offset_y = workspace.tool_frame_offset.position.y;
  double offset_z = workspace.tool_frame_offset.position.z;



  //  if(!((offset_x && offset_y && offset_z && offset_roll && offset_pitch && offset_yaw) == 0.0))
  if(offset_x != 0.0 || offset_y != 0.0 || offset_z != 0.0 || offset_roll !=0.0 || offset_pitch != 0.0 || offset_yaw != 0.0)
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



void MainWindow::loadBoundaries(double min_x, double min_y, double min_z, double max_x, double max_y, double max_z)
{
  ui_->edit_text_size_x->setText(QString::number(max_x - min_x));
  ui_->edit_text_size_y->setText(QString::number(max_y - min_y));
  ui_->edit_text_size_z->setText(QString::number(max_z - min_z));  

  ui_->edit_text_origin_x->setText(QString::number(max_x - (max_x - min_x)/2.0));
  ui_->edit_text_origin_y->setText(QString::number(max_y - (max_y - min_y)/2.0));
  ui_->edit_text_origin_z->setText(QString::number(max_z - (max_z - min_z)/2.0));
}


void MainWindow::setWorkspace()
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

  workspace_.parameters.min_corner.x = min_corner_x;
  workspace_.parameters.min_corner.y = min_corner_y;
  workspace_.parameters.min_corner.z = min_corner_z;

  workspace_.parameters.max_corner.x = max_corner_x;
  workspace_.parameters.max_corner.y = max_corner_y;
  workspace_.parameters.max_corner.z = max_corner_z;

  double resolution = ui_->edit_text_resolution->text().toDouble();
  workspace_.position_resolution = resolution;

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
  workspace_.tool_frame_offset = tool_frame_offset;
}

void MainWindow::computeFK()
{
 
  MainWindow::setWorkspace();

  double timeout = ui_->edit_text_timeout->text().toDouble();


  Q_EMIT startFKComputation(workspace_, timeout);
}

void MainWindow::receiveWorkspace(const moveit_ros_planning::WorkspacePoints& workspace)
{
  workspace_ = workspace;

}

void MainWindow::cancelComputation()
{

  //thread_.terminate();
  //thread_.start();
  //kinematics_thread_.initialise();
  ui_->progress_bar->setMaximum(100);
  ui_->progress_bar->setValue(0);
  ui_->progress_bar->hide();
  ui_->compute_button->setVisible(true);

  //Q_EMIT cancelComputationIK();
  kinematics_thread_.stopSolver();
  //QApplication::processEvents(); 
}
