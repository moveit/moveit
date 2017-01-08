#ifndef BENCHMARK_PROCESSING_THREAD_H
#define BENCHMARK_PROCESSING_THREAD_H

#include <QtCore/QThread>
#include <QProgressDialog>

#ifndef Q_MOC_RUN
#include <moveit/benchmarks/benchmark_execution.h>
#endif

class BenchmarkProcessingThread : public QThread
{
  Q_OBJECT

public:
  BenchmarkProcessingThread(const moveit_benchmarks::BenchmarkExecution& be, const moveit_benchmarks::BenchmarkType& bt,
                            QObject* parent = 0);
  ~BenchmarkProcessingThread();

  void startAndShow();

private:
  boost::shared_ptr<QProgressDialog> progress_dialog_;

  moveit_benchmarks::BenchmarkExecution be_;
  moveit_benchmarks::BenchmarkType bt_;

protected:
  void run();
};

#endif  // BENCHMARK_PROCESSING_THREAD_H
