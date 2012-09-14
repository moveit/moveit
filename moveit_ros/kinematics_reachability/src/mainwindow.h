
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <kinematics_reachability/WorkspacePoints.h>

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
    void add_row();
    void compute();
    void show_offset(bool checked);
    void visualise_workspace();

private:
    Ui::MainWindow *ui;
  kinematics_reachability::WorkspacePoints workspace;

};

#endif // MAINWINDOW_H
