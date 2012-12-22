#ifndef KINEMATICSTHREAD_H
#define KINEMATICSTHREAD_H

#include <moveit/kinematics_reachability/kinematics_reachability.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <iostream>
#include "angles/angles.h"
#include <QThread>
#include <QMainWindow>

namespace kinematics_thread
{
class KinematicsThread : public QObject
{

Q_OBJECT

public:
  explicit KinematicsThread();
  ~KinematicsThread();
  void initialise();
  void stopSolver();
  void enableSolver();

Q_SIGNALS:
  void currentProgressSignal(int current);
  void maxProgressSignal(int total);
  void setFrameIdLabel(QString frame_id);
  void setNameLabel(QString name);
  void setUISignal(const moveit_ros_planning::WorkspacePoints &workspace);
  void doneComputing();
  void sendWorkspace(const moveit_ros_planning::WorkspacePoints &workspace);

private Q_SLOTS:
  void addOrientation(QString roll, QString pitch, QString yaw);
  //void computeKinematics(double min_x, double min_y, double min_z, double max_x, double max_y, double max_z, double resolution, double offset_roll, double offset_pitch, double offset_yaw, double offset_x, double offset_y, double offset_z);
  //void visualise(double min_x, double min_y, double min_z, double max_x, double max_y, double max_z, double resolution, double offset_roll, double offset_pitch, double offset_yaw, double offset_x, double offset_y, double offset_z);
  void computeKinematics(const moveit_ros_planning::WorkspacePoints& workspace);
  void visualise(const moveit_ros_planning::WorkspacePoints& workspace);
  void computeFK(const moveit_ros_planning::WorkspacePoints& workspace, double timeout);
  //void stopSolver();

private:
  moveit_ros_planning::WorkspacePoints workspace_;
  kinematics_reachability::KinematicsReachability * reachability_solver_;
  ros::Subscriber workspace_subscriber_;
  ros::Subscriber progress_subscriber_;

  void setBoundaries(moveit_ros_planning::WorkspacePoints &workspace, double min_x, double min_y, double min_z, double max_x, double max_y, double max_z, double resolution, double offset_roll, double offset_pitch, double offset_yaw, double offset_x, double offset_y, double offset_z);
  void bagCallback(const moveit_ros_planning::WorkspacePointsConstPtr &msg);
  void updateProgressBar(const moveit_ros_planning::ProgressConstPtr &msg);
  void setUI(const moveit_ros_planning::WorkspacePoints &workspace);

};
}
#endif //KINEMATICSTHREAD_H
