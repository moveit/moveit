#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <iostream>
#include "angles/angles.h"
#include <kinematics_reachability/kinematics_reachability.h>


kinematics_reachability::WorkspacePoints workspace;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //Hide tool offset on startup
    ui->lineEdit_10->hide();
    ui->lineEdit_11->hide();
    ui->lineEdit_12->hide();
    ui->label_12->hide();
    ui->label_13->hide();
    ui->label_14->hide();
    ui->lineEdit_13->hide();
    ui->lineEdit_14->hide();
    ui->lineEdit_15->hide();
    ui->label_16->hide();
    ui->label_17->hide();
    ui->label_18->hide();

    //Set default text in name fields
    ui->lineEdit->setText("");
    ui->lineEdit_2->setText("");
    ui->lineEdit_3->setText("");
    
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::add_row()
{
    ui->tableWidget->setColumnCount(3);
    ui->tableWidget->setHorizontalHeaderLabels(QString("Roll;Pitch;Yaw").split(";"));
    ui->tableWidget->insertRow(ui->tableWidget->rowCount());

    QString arm_roll = ui->lineEdit_7->text();
    QString arm_pitch = ui->lineEdit_8->text();
    QString arm_yaw = ui->lineEdit_9->text();

    ui->tableWidget->setItem(ui->tableWidget->rowCount() -1, 0, new QTableWidgetItem(arm_roll));
    ui->tableWidget->setItem(ui->tableWidget->rowCount() -1, 1, new QTableWidgetItem(arm_pitch));
    ui->tableWidget->setItem(ui->tableWidget->rowCount() -1, 2, new QTableWidgetItem(arm_yaw));

    ui->lineEdit_7->clear();
    ui->lineEdit_8->clear();
    ui->lineEdit_9->clear();

    geometry_msgs::Quaternion quaternion;
    quaternion = tf::createQuaternionMsgFromRollPitchYaw(angles::from_degrees(arm_roll.toDouble()),angles::from_degrees(arm_pitch.toDouble()),angles::from_degrees(arm_yaw.toDouble()));
    workspace.orientations.push_back(quaternion);
    
}

void MainWindow::compute()
{

    geometry_msgs::Pose tool_frame_offset;

    double offset_roll = 0;
    double offset_pitch = 0;
    double offset_yaw = 0;

    double offset_x = 0; 
    double offset_y = 0;
    double offset_z = 0;
    
    double resolution = ui->lineEdit_6->text().toDouble();
 
    double origin_x = ui->lineEdit_4->text().toDouble();
    double origin_y = ui->lineEdit_5->text().toDouble();
    double origin_z = ui->lineEdit_16->text().toDouble();

    double min_corner_x = origin_x - (ui->lineEdit_17->text().toDouble()/2.0);
    double min_corner_y = origin_y - (ui->lineEdit_18->text().toDouble()/2.0);
    double min_corner_z = origin_z - (ui->lineEdit_19->text().toDouble()/2.0);

    double max_corner_x = origin_x + (ui->lineEdit_17->text().toDouble()/2.0);
    double max_corner_y = origin_y + (ui->lineEdit_18->text().toDouble()/2.0);
    double max_corner_z = origin_z + (ui->lineEdit_19->text().toDouble()/2.0);

    bool checked = ui->checkBox->isChecked();

    if (checked)
    {

        offset_roll = angles::from_degrees(ui->lineEdit_13->text().toDouble());
        offset_pitch = angles::from_degrees(ui->lineEdit_14->text().toDouble());
        offset_yaw = angles::from_degrees(ui->lineEdit_15->text().toDouble());

        offset_x = ui->lineEdit_10->text().toDouble(); 
        offset_y = ui->lineEdit_11->text().toDouble();
        offset_z = ui->lineEdit_12->text().toDouble();
    }

    //Get values for names
    /*
    std::string name = ui->lineEdit->text().toStdString();
    std::string root_name = ui->lineEdit_2->text().toStdString();
    std::string tip_name = ui->lineEdit_3->text().toStdString();
    */

    MainWindow::close();    

    ros::AsyncSpinner spinner(2); 
    spinner.start();

    ros::NodeHandle node_handle("~");
    std::string group_name, root_name;
    node_handle.param<std::string>("group", group_name, std::string());
    node_handle.param<std::string>(group_name+"/root_name", root_name, std::string());
    ros::NodeHandle root_handle;

    /**** WORKSPACE PARAMETERS - These are the parameters you need to change to specify a different 
    region in the workspace for which reachability is to be computed****/
    kinematics_reachability::KinematicsReachability reachability_solver;
    if(!reachability_solver.initialize())
        return;

    workspace.group_name = "arm";

    workspace.position_resolution = resolution;
    workspace.header.frame_id = "arm_base_link";

    workspace.parameters.min_corner.x = min_corner_x;
    workspace.parameters.min_corner.y = min_corner_y;
    workspace.parameters.min_corner.z = min_corner_z;

    workspace.parameters.max_corner.x = max_corner_x;
    workspace.parameters.max_corner.y = max_corner_y;
    workspace.parameters.max_corner.z = max_corner_z;

    //SET OF ORIENTATIONS TO TEST FOR REACHABILITY

    //geometry_msgs::Quaternion quaternion;
    //quaternion.w = 1.0;
    //workspace.orientations.push_back(quaternion);  

    /*  quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0,M_PI/2.0,-M_PI/2.0);
    workspace.orientations.push_back(quaternion);

    quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0,M_PI/2.0,M_PI/2.0);
    workspace.orientations.push_back(quaternion);

    quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0,M_PI/2.0,0.0);
    workspace.orientations.push_back(quaternion);

    quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0,-M_PI/2.0,0.0);
    workspace.orientations.push_back(quaternion);
    */
    /*
    // The octants
    quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0,M_PI/4.0,M_PI/4.0);
    workspace.orientations.push_back(quaternion);

    quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0,-M_PI/4.0,M_PI/4.0);
    workspace.orientations.push_back(quaternion);

    quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0,-M_PI/4.0,-M_PI/4.0);
    workspace.orientations.push_back(quaternion);

    quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0,M_PI/4.0,-M_PI/4.0);
    workspace.orientations.push_back(quaternion);
    */
    while(!reachability_solver.isActive())
    {
        sleep(1.0);
        ROS_INFO("Waiting for planning scene to be set");
    }
      
 
    tool_frame_offset.orientation = tf::createQuaternionMsgFromRollPitchYaw(offset_roll,offset_pitch,offset_yaw);
    tool_frame_offset.position.x = offset_x;
    tool_frame_offset.position.y = offset_y;
    tool_frame_offset.position.z = offset_z;

    reachability_solver.computeWorkspace(workspace, tool_frame_offset, true);
    
    reachability_solver.visualize(workspace,"full");

    reachability_solver.animateWorkspace(workspace);

    reachability_solver.visualizeWithArrows(workspace,"full_arrows");
    //  aw.visualize(workspace,"RPY(0,0,0)",zero_orientation);
    ROS_INFO("Success");

    reachability_solver.publishWorkspace(workspace);

    ros::waitForShutdown();

    return;
}

void MainWindow::show_offset(bool checked)
{
    ui->lineEdit_10->setVisible(checked);
    ui->lineEdit_11->setVisible(checked);
    ui->lineEdit_12->setVisible(checked);
    ui->label_12->setVisible(checked);
    ui->label_13->setVisible(checked);
    ui->label_14->setVisible(checked);
    ui->lineEdit_13->setVisible(checked);
    ui->lineEdit_14->setVisible(checked);
    ui->lineEdit_15->setVisible(checked);
    ui->label_16->setVisible(checked);
    ui->label_17->setVisible(checked);
    ui->label_18->setVisible(checked);
}



void MainWindow::visualise_workspace()
{
    kinematics_reachability::KinematicsReachability reachability_solver;
    if(!reachability_solver.initialize())
        return;
    kinematics_reachability::WorkspacePoints sample_workspace;

    sample_workspace.group_name = "arm";

    double resolution = ui->lineEdit_6->text().toDouble();

    sample_workspace.position_resolution = resolution;
    sample_workspace.header.frame_id = "arm_base_link";

    double origin_x = ui->lineEdit_4->text().toDouble();
    double origin_y = ui->lineEdit_5->text().toDouble();
    double origin_z = ui->lineEdit_16->text().toDouble();

    double min_corner_x = origin_x - (ui->lineEdit_17->text().toDouble()/2.0);
    double min_corner_y = origin_y - (ui->lineEdit_18->text().toDouble()/2.0);
    double min_corner_z = origin_z - (ui->lineEdit_19->text().toDouble()/2.0);

    double max_corner_x = origin_x + (ui->lineEdit_17->text().toDouble()/2.0);
    double max_corner_y = origin_y + (ui->lineEdit_18->text().toDouble()/2.0);
    double max_corner_z = origin_z + (ui->lineEdit_19->text().toDouble()/2.0);

    sample_workspace.parameters.min_corner.x = min_corner_x;
    sample_workspace.parameters.min_corner.y = min_corner_y;
    sample_workspace.parameters.min_corner.z = min_corner_z;

    sample_workspace.parameters.max_corner.x = max_corner_x;
    sample_workspace.parameters.max_corner.y = max_corner_y;
    sample_workspace.parameters.max_corner.z = max_corner_z;

    geometry_msgs::Quaternion quaternion;
    quaternion.w = 1.0;
    sample_workspace.orientations.push_back(quaternion);

    while(!reachability_solver.isActive())
    {
        sleep(1.0);
        ROS_INFO("Waiting for planning scene to be set");
    }

    reachability_solver.visualizeWorkspaceSamples(sample_workspace, "samples");
}
