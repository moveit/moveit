#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <sys/time.h>
#include <time.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_utils/clock.hpp>

namespace util
{
static long unsigned int startTime = 0;

/*
 * Starts the clock!  Call this once at the beginning of the program.
 * Calling again will reset the clock to 0;  but doing so is not
 * thread-safe if other threads may be calling GetClock(); (which
 * is thread-safe since it only reads values and calls thread-safe
 * functions in the kernel).
 */
// time in units of seconds since some time in the past
void StartClock()
{
  // determine start time
  struct timeval startTimeStruct;
  gettimeofday(&startTimeStruct, nullptr);
  startTime = static_cast<unsigned long>(startTimeStruct.tv_sec * 1e6l + startTimeStruct.tv_usec);
}

/*
 * Returns the current time since the call to StartClock();
 */
double GetClock()
{
  struct timeval startTimeStruct;
  unsigned long int curTime;
  gettimeofday(&startTimeStruct, nullptr);
  curTime = static_cast<unsigned long>(startTimeStruct.tv_sec * 1e6l + startTimeStruct.tv_usec);
  return (1e-6) * static_cast<double>(curTime - startTime);
}
}
