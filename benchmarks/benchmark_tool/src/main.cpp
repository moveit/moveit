#include <ros/ros.h>
#include <QApplication>
#include <main_window.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "benchmark_tool");

  QApplication app(argc, argv);
  benchmark_tool::MainWindow w(argc, argv);
  w.show();

  return app.exec();
}

