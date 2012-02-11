#ifndef MY_TIMER_H
#define MY_TIMER_H

#ifdef WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif //WIN32

namespace ParabolicRamp {

#ifdef WIN32
typedef DWORD TimerCounterType;
#else
#ifdef  _POSIX_MONOTONIC_CLOCK
typedef timespec TimerCounterType;
#else
typedef timeval TimerCounterType;
#endif //_POSIX_MONOTONIC_CLOCK
#endif //WIN32

class Timer
{
 public:
  Timer();
  void Reset();

  // Returns elapsed time in milliseconds,seconds respectively
  long long ElapsedTicks();
  double ElapsedTime();

  // Doesn't refresh the current time
  long long LastElapsedTicks() const;
  double LastElapsedTime() const;

 private:
  TimerCounterType start;
  TimerCounterType current;
};

} //namespace ParabolicRamp

#endif
