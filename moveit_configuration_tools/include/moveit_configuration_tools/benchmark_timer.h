//#ifndef BENCHMARK_TIMER_H
//#define BENCHMARK_TIMER_H

#include <map>
#include <string>
#include <cstdlib>
#include <sys/time.h>
#include <sys/resource.h>
#include <iostream>
#include <stdio.h>
//#include <time.h>

// A simple class to help with benchmarking
class BenchmarkTimer
{
public:
  BenchmarkTimer();
  void start(std::string key);
  void end(std::string key);
  void printTimes();
  double getSimpleTime();
	
private:
  std::map<std::string, double> totalTimes;
  std::map<std::string, double> intermediateTimes;
};

//#endif
