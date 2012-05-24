#include "moveit_configuration_tools/benchmark_timer.h"
#include <map>
#include <string>

using namespace std;

BenchmarkTimer::BenchmarkTimer()
{
  totalTimes = map<string, double>();
  intermediateTimes = map<string, double>();
}

double BenchmarkTimer::getSimpleTime(){
  struct timeval t;
  struct timezone tzp;
  gettimeofday(&t, &tzp);
  return t.tv_sec + t.tv_usec*1e-6;
}

void BenchmarkTimer::start(string key)
{
  intermediateTimes[key] = getSimpleTime();
}

void BenchmarkTimer::end(string key)
{
  totalTimes[key] += (getSimpleTime() - intermediateTimes[key]);
}

void BenchmarkTimer::printTimes(){
  map<string, double>::iterator it;
	
  cout << endl;
  cout << "-------------------------------------------------------------------------------" << endl;
  cout << "Timing metrics:   " << endl;

  double total_time = totalTimes["Total"];

  for(it=totalTimes.begin(); it != totalTimes.end(); it++)
  {
    printf("%6.2f%% : %s\n", it->second/total_time*100, it->first.c_str());		
  }
	
  cout << endl << "Copy to Spreadsheet:" << endl;
  for(it=totalTimes.begin(); it != totalTimes.end(); it++)
  {
    printf("%.2lf\t", it->second);
  }
}


