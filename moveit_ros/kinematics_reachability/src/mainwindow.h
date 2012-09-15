
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
    void addRow();
    void compute();
    void showOffset(bool checked);
    void visualiseWorkspace();

private:
    Ui::MainWindow *ui;
    kinematics_reachability::WorkspacePoints workspace;
    void setBoundaries(kinematics_reachability::WorkspacePoints &w);
};

#endif // MAINWINDOW_H
