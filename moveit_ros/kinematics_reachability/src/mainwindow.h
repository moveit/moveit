
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <kinematics_reachability/kinematics_reachability.h>
#include "kinematics_thread.h"
#include <tf/transform_datatypes.h>

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

Q_SIGNALS:
  void addRowSignal(QString roll, QString pitch, QString yaw);
  void startComputation(double min_x, double min_y, double min_z, double max_x, double max_y, double max_z, double resolution, double offset_roll, double offset_pitch, double offset_yaw, double offset_x, double offset_y, double offset_z);
  void startVisualisation(double min_x, double min_y, double min_z, double max_x, double max_y, double max_z, double resolution, double offset_roll, double offset_pitch, double offset_yaw, double offset_x, double offset_y, double offset_z);

private:
  Ui::MainWindow *ui_;

  kinematics_thread::KinematicsThread kinematics_thread_;
  //kinematics_reachability::WorkspacePoints workspace_;
  kinematics_reachability::KinematicsReachability reachability_solver_;
  //ros::Subscriber workspace_subscriber_;
  //ros::Subscriber progress_subscriber_;


  void addRow(QString roll, QString pitch, QString yaw);
  void loadBoundaries(double min_x, double min_y, double min_z, double max_x, double max_y, double max_z);
  
};

#endif // MAINWINDOW_H
