#ifndef PARABOLIC_RAMP_CONFIG_H
#define PARABOLIC_RAMP_CONFIG_H

#include <assert.h>
#include "ros/assert.h"
#include "rosconsole/macros_generated.h"

///assertion function
#define PARABOLIC_RAMP_ASSERT(x) assert(x)

///print an error
#define PARABOLIC_RAMP_PERROR(...) ROS_ERROR(__VA_ARGS__)

///print a notification
#define PARABOLIC_RAMP_PLOG(...) ROS_INFO(__VA_ARGS__)

namespace ParabolicRamp {

  ///tolerance for time equality
  const static Real EpsilonT = 1e-6;

  ///tolerance for position equality
  const static Real EpsilonX = 1e-5;

  ///tolerance for velocity equality
  const static Real EpsilonV = 1e-5;

  ///tolerance for acceleration equality
  const static Real EpsilonA = 1e-6;

  ///self validity check level:
  ///- 0 no checking
  ///- 1 moderate checking 
  ///- 2 full checking
  const static int gValidityCheckLevel = 2;

  ///verbosity level:
  ///- 0 all messages off
  ///- 1 brief messages
  ///- 2 detailed messages
  const static int gVerbose = 2;

  ///whether or not to pause on serious errors
  const static bool gErrorGetchar = true;

  ///whether or not errors are logged to disk
  const static bool gErrorSave = true;

} //namespace ParabolicRamp

#endif
