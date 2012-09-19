
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <kinematics_reachability/kinematics_reachability.h>

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
  
private:
  Ui::MainWindow *ui_;
  kinematics_reachability::WorkspacePoints workspace_;
  kinematics_reachability::KinematicsReachability reachability_solver_;
  ros::Subscriber workspace_subscriber_;
  ros::Subscriber progress_subscriber_;


  void setBoundaries(kinematics_reachability::WorkspacePoints &workspace);
  void setUI(const kinematics_reachability::WorkspacePoints &workspace);
  void bagCallback(const kinematics_reachability::WorkspacePointsConstPtr &msg);
  void updateProgressBar(const kinematics_reachability::ProgressConstPtr &msg);
  void addRow(QString roll, QString pitch, QString yaw);
  void loadBoundaries(kinematics_reachability::WorkspacePoints workspace);
};

#endif // MAINWINDOW_H
