
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <moveit/kinematics_reachability/kinematics_reachability.h>
#include <moveit/kinematics_reachability/kinematics_thread.h>
#include <tf/transform_datatypes.h>
#include <QThread>
#include <QMainWindow>
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
  void setUIFromWorkspace(const moveit_ros_planning::WorkspacePoints &workspace);
  void computeFK();
  void receiveWorkspace(const moveit_ros_planning::WorkspacePoints &workspace);
  void cancelComputation();

Q_SIGNALS:
  void addRowSignal(QString roll, QString pitch, QString yaw);
  void startComputation(const moveit_ros_planning::WorkspacePoints& workspace);
  void startVisualisation(const moveit_ros_planning::WorkspacePoints& workspace);
  void startFKComputation(const moveit_ros_planning::WorkspacePoints& workspace, double timeout);
  void cancelComputationIK();

private:
  Ui::MainWindow *ui_;

  kinematics_thread::KinematicsThread kinematics_thread_;
  moveit_ros_planning::WorkspacePoints workspace_;
  QThread thread_;

  void setWorkspace();
  void addRow(QString roll, QString pitch, QString yaw);
  void loadBoundaries(double min_x, double min_y, double min_z, double max_x, double max_y, double max_z);
  
};

#endif // MAINWINDOW_H
