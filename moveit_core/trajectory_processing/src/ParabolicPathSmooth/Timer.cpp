#include "trajectory_processing/ParabolicPathSmooth/Timer.h"
#include <stdlib.h>
using namespace ParabolicRamp;

#ifdef WIN32
#define GETCURRENTTIME(x) x=timeGetTime()
#else
#ifdef  _POSIX_MONOTONIC_CLOCK
#define GETCURRENTTIME(x) clock_gettime(CLOCK_MONOTONIC,&x)
#else
#define GETCURRENTTIME(x) gettimeofday(&x,NULL)
#endif //_POSIX_MONOTONIC_CLOCK
#endif //WIN32

// Sadly, timersub isn't defined in Solaris. :(
// So we use this instead. (added by Ryan)

#if defined (__SVR4) && defined (__sun)
#include "timersub.h"
#endif

Timer::Timer()
{
  Reset();
}

void Timer::Reset()
{
  GETCURRENTTIME(start);
  current=start;
}

long long Timer::ElapsedTicks()
{
  GETCURRENTTIME(current);
  return LastElapsedTicks();
}

long long Timer::LastElapsedTicks() const
{
#ifdef WIN32
  return current-start;
#else
#ifdef  _POSIX_MONOTONIC_CLOCK
  long long ticks = (current.tv_sec-start.tv_sec)*1000 + (current.tv_nsec-start.tv_nsec)/1000000;
  return ticks;
#else
  timeval delta;
  timersub(&current,&start,&delta);
  long long ticks = delta.tv_sec*1000 + delta.tv_usec/1000;
  return ticks;
#endif //_POSIX_MONOTONIC_CLOCK
#endif //WIN32
}
    
double Timer::ElapsedTime()
{
  GETCURRENTTIME(current);
  return LastElapsedTime();
}

double Timer::LastElapsedTime() const
{
#ifdef WIN32
  return double(current-start)/1000.0;
#else
#ifdef  _POSIX_MONOTONIC_CLOCK
  double secs=double(current.tv_sec-start.tv_sec);
  secs += double(current.tv_nsec-start.tv_nsec)/1000000000.0;
  return secs;
#else
  timeval delta;
  timersub(&current,&start,&delta);
  double secs=double(delta.tv_sec);
  secs += double(delta.tv_usec)/1000000.0;
  return secs;
#endif //_POSIX_MONOTONIC_CLOCK
#endif //WIN32
}

/*
clock_t Timer::ElapsedTicks()
{
  current = clock();
  return (current-start);
}

double Timer::ElapsedTime()
{
  current = clock();
  return double(current-start)/CLOCKS_PER_SEC;
}

clock_t Timer::LastElapsedTicks() const
{
  return current-start;
}

double Timer::LastElapsedTime() const
{
  return double(current-start)/CLOCKS_PER_SEC;
}
*/
