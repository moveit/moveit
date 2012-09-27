
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <kinematics_reachability/kinematics_reachability.h>
#include "kinematics_thread.h"
#include <tf/transform_datatypes.h>
#include <QThread>

//class kinematics_thread::Kinematics_Thread;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
  explicit MainWindow(QWidget *parent = 0);
  ~MainWindow();
               
private Q_SLOTS:
  void addRow();
  void compute();
  void showOffset(bool checked);
  void visualiseWorkspace();
  void Success();
  void setUIFromWorkspace(const kinematics_reachability::WorkspacePoints &workspace);
  void computeFK();
  void receiveWorkspace(const kinematics_reachability::WorkspacePoints &workspace);
  void cancelComputation();

Q_SIGNALS:
  void addRowSignal(QString roll, QString pitch, QString yaw);
  //void startComputation(double min_x, double min_y, double min_z, double max_x, double max_y, double max_z, double resolution, double offset_roll, double offset_pitch, double offset_yaw, double offset_x, double offset_y, double offset_z);
  //void startVisualisation(double min_x, double min_y, double min_z, double max_x, double max_y, double max_z, double resolution, double offset_roll, double offset_pitch, double offset_yaw, double offset_x, double offset_y, double offset_z);
  void startComputation(const kinematics_reachability::WorkspacePoints& workspace);
  void startVisualisation(const kinematics_reachability::WorkspacePoints& workspace);
  void startFKComputation(const kinematics_reachability::WorkspacePoints& workspace, double timeout);
  void cancelComputationIK();

private:
  Ui::MainWindow *ui_;

  kinematics_thread::KinematicsThread kinematics_thread_;
  kinematics_reachability::WorkspacePoints workspace_;
  kinematics_reachability::KinematicsReachability reachability_solver_;
  QThread thread_;
  //ros::Subscriber workspace_subscriber_;
  //ros::Subscriber progress_subscriber_;
  /*double origin_x_;
  double origin_y_; 
  double origin_z_; 

  double min_x_;
  double min_y_;
  double min_z_;

  double max_x_;
  double max_y_;
  double max_z_;

  double resolution_; 

  double offset_roll_ = 0;
  double offset_pitch_ = 0;
  double offset_yaw_ = 0;

  double offset_x_ = 0;
  double offset_y_ = 0;
  double offset_z_ = 0;
  */
  void setWorkspace();
  void addRow(QString roll, QString pitch, QString yaw);
  void loadBoundaries(double min_x, double min_y, double min_z, double max_x, double max_y, double max_z);
  
};

#endif // MAINWINDOW_H
