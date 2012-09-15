#include <ros/ros.h>
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <tf/transform_datatypes.h>
#include <iostream>
#include "angles/angles.h"


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //Hide tool offset on startup
    ui->edit_text_offset_x->hide();
    ui->edit_text_offset_y->hide();
    ui->edit_text_offset_z->hide();
    ui->label_offset_x->hide();
    ui->label_offset_y->hide();
    ui->label_offset_z->hide();
    ui->edit_text_offset_roll->hide();
    ui->edit_text_offset_pitch->hide();
    ui->edit_text_offset_yaw->hide();
    ui->label_offset_roll->hide();
    ui->label_offset_pitch->hide();
    ui->label_offset_yaw->hide();

    //Set default text in name fields
    ui->frame_id_label->setText("");
    ui->name_label->setText("");
    
    ros::AsyncSpinner spinner(2); 
    spinner.start();

    ros::NodeHandle node_handle("~");
    std::string group_name, frame_id;
    node_handle.param<std::string>("group", group_name, std::string());
    node_handle.param<std::string>("frame_id", frame_id, std::string());

    ROS_INFO("Group name: %s",group_name.c_str());
    ROS_INFO("Frame id: %s",frame_id.c_str());
   
    //Set from ROS parameters
    ui->name_label->setText(group_name.c_str());
    ui->frame_id_label->setText(frame_id.c_str());

    if(!reachability_solver_.initialize())
      ROS_ERROR("Could not initialize reachability solver");
    workspace_.group_name = group_name;
    workspace_.header.frame_id = frame_id;    
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::addRow()
{
    ui->table_widget->setColumnCount(3);
    ui->table_widget->setHorizontalHeaderLabels(QString("Roll;Pitch;Yaw").split(";"));
    ui->table_widget->insertRow(ui->table_widget->rowCount());

    QString arm_roll = ui->edit_text_roll->text();
    QString arm_pitch = ui->edit_text_pitch->text();
    QString arm_yaw = ui->edit_text_yaw->text();

    ui->table_widget->setItem(ui->table_widget->rowCount() -1, 0, new QTableWidgetItem(arm_roll));
    ui->table_widget->setItem(ui->table_widget->rowCount() -1, 1, new QTableWidgetItem(arm_pitch));
    ui->table_widget->setItem(ui->table_widget->rowCount() -1, 2, new QTableWidgetItem(arm_yaw));

    ui->edit_text_roll->clear();
    ui->edit_text_pitch->clear();
    ui->edit_text_yaw->clear();

    geometry_msgs::Quaternion quaternion;
    quaternion = tf::createQuaternionMsgFromRollPitchYaw(angles::from_degrees(arm_roll.toDouble()),angles::from_degrees(arm_pitch.toDouble()),angles::from_degrees(arm_yaw.toDouble()));
    workspace_.orientations.push_back(quaternion);
    
}

void MainWindow::compute()
{
  MainWindow::setBoundaries(workspace_);
  workspace_.points.clear();  
  //    MainWindow::close();
  /**** WORKSPACE PARAMETERS - These are the parameters you need to change to specify a different 
        region in the workspace for which reachability is to be computed****/
  while(!reachability_solver_.isActive())
  {
    sleep(1.0);
    ROS_INFO("Waiting for planning scene to be set");
  }          
  reachability_solver_.computeWorkspace(workspace_, true);
  reachability_solver_.visualize(workspace_,"full");
  reachability_solver_.animateWorkspace(workspace_);
  ROS_INFO("Success");  
  reachability_solver_.publishWorkspace(workspace_);
  //    ros::waitForShutdown();  
}

void MainWindow::showOffset(bool checked)
{
    ui->edit_text_offset_x->setVisible(checked);
    ui->edit_text_offset_y->setVisible(checked);
    ui->edit_text_offset_z->setVisible(checked);
    ui->label_offset_x->setVisible(checked);
    ui->label_offset_y->setVisible(checked);
    ui->label_offset_z->setVisible(checked);
    ui->edit_text_offset_roll->setVisible(checked);
    ui->edit_text_offset_pitch->setVisible(checked);
    ui->edit_text_offset_yaw->setVisible(checked);
    ui->label_offset_roll->setVisible(checked);
    ui->label_offset_pitch->setVisible(checked);
    ui->label_offset_yaw->setVisible(checked);
}

void MainWindow::visualiseWorkspace()
{
  /*    kinematics_reachability::WorkspacePoints sample_workspace;
    sample_workspace.group_name = "arm";

    double resolution = ui->edit_text_resolution->text().toDouble();

    sample_workspace.position_resolution = resolution;
    sample_workspace.header.frame_id = "arm_base_link";

    MainWindow::setBoundaries(sample_workspace);

    geometry_msgs::Quaternion quaternion;
    quaternion.w = 1.0;
    sample_workspace.orientations.push_back(quaternion);

    while(!reachability_solver_.isActive())
    {
        sleep(1.0);
        ROS_INFO("Waiting for planning scene to be set");
    }
*/
    MainWindow::setBoundaries(workspace_);
    reachability_solver_.visualizeWorkspaceSamples(workspace_);
    ROS_INFO("Samples visualised.");
}

void MainWindow::setBoundaries(kinematics_reachability::WorkspacePoints &w)
{
    double origin_x = ui->edit_text_origin_x->text().toDouble();
    double origin_y = ui->edit_text_origin_y->text().toDouble();
    double origin_z = ui->edit_text_origin_z->text().toDouble();

    double min_corner_x = origin_x - (ui->edit_text_size_x->text().toDouble()/2.0);
    double min_corner_y = origin_y - (ui->edit_text_size_y->text().toDouble()/2.0);
    double min_corner_z = origin_z - (ui->edit_text_size_z->text().toDouble()/2.0);

    double max_corner_x = origin_x + (ui->edit_text_size_x->text().toDouble()/2.0);
    double max_corner_y = origin_y + (ui->edit_text_size_y->text().toDouble()/2.0);
    double max_corner_z = origin_z + (ui->edit_text_size_z->text().toDouble()/2.0);

    w.parameters.min_corner.x = min_corner_x;
    w.parameters.min_corner.y = min_corner_y;
    w.parameters.min_corner.z = min_corner_z;

    w.parameters.max_corner.x = max_corner_x;
    w.parameters.max_corner.y = max_corner_y;
    w.parameters.max_corner.z = max_corner_z;

    double resolution = ui->edit_text_resolution->text().toDouble();
    w.position_resolution = resolution;

    geometry_msgs::Pose tool_frame_offset;
    double offset_roll = 0;
    double offset_pitch = 0;
    double offset_yaw = 0;

    double offset_x = 0; 
    double offset_y = 0;
    double offset_z = 0;

    bool checked = ui->tool_offset_enabled->isChecked();
    if (checked)
    {
      offset_roll = angles::from_degrees(ui->edit_text_offset_roll->text().toDouble());
      offset_pitch = angles::from_degrees(ui->edit_text_offset_pitch->text().toDouble());
      offset_yaw = angles::from_degrees(ui->edit_text_offset_yaw->text().toDouble());

      offset_x = ui->edit_text_offset_x->text().toDouble(); 
      offset_y = ui->edit_text_offset_y->text().toDouble();
      offset_z = ui->edit_text_offset_z->text().toDouble();
    }
    tool_frame_offset.orientation = tf::createQuaternionMsgFromRollPitchYaw(offset_roll,offset_pitch,offset_yaw);
    tool_frame_offset.position.x = offset_x;
    tool_frame_offset.position.y = offset_y;
    tool_frame_offset.position.z = offset_z;
    w.tool_frame_offset = tool_frame_offset;    
}
