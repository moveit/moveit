#include <ros/ros.h>
#include <benchmark_processing_thread.h>

BenchmarkProcessingThread::BenchmarkProcessingThread(const moveit_benchmarks::BenchmarkExecution &be, const moveit_benchmarks::BenchmarkType &bt, QObject *parent)
: QThread(parent), be_(be), bt_(bt)
{
  setTerminationEnabled();
}

BenchmarkProcessingThread::~BenchmarkProcessingThread()
{
}

void BenchmarkProcessingThread::startAndShow()
{
  this->start();

  progress_dialog_.reset(new QProgressDialog("Running benchmark","Cancel",0,0));
  connect( this, SIGNAL( finished() ), progress_dialog_.get(), SLOT( cancel() ));
  progress_dialog_->setMinimum(0);
  progress_dialog_->setMaximum(0);
  progress_dialog_->exec();
}

void BenchmarkProcessingThread::run()
{
  ROS_INFO("Thread running");
  be_.runAllBenchmarks(bt_);
}
