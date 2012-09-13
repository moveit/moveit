#include <QtGui/QApplication>
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <ros/ros.h>
int main(int argc, char *argv[])
{


    ros::init(argc, argv, "arm_workspace_tests"); 
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    int result = a.exec();
 
    return result;
}

