
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

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
};

#endif // MAINWINDOW_H
