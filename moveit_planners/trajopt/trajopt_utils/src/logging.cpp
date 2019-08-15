#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <cstdlib>
#include <iostream>
#include <string>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_utils/logging.hpp>

namespace util
{
LogLevel gLogLevel;

int LoggingInit()
{
  const char* VALID_THRESH_VALUES = "FATAL ERROR WARN INFO DEBUG TRACE";

  char* lvlc = getenv("TRAJOPT_LOG_THRESH");
  std::string lvlstr;
  if (lvlc == nullptr)
  {
    std::printf("You can set logging level with TRAJOPT_LOG_THRESH. Valid values: "
                "%s. Defaulting to ERROR\n",
                VALID_THRESH_VALUES);
#ifdef NDEBUG
    lvlstr = "ERROR";
#else
    lvlstr = "DEBUG";
#endif
  }
  else
    lvlstr = std::string(lvlc);
  if (lvlstr == "FATAL")
    gLogLevel = LevelFatal;
  else if (lvlstr == "ERROR")
    gLogLevel = LevelError;
  else if (lvlstr == "WARN")
    gLogLevel = LevelWarn;
  else if (lvlstr == "INFO")
    gLogLevel = LevelInfo;
  else if (lvlstr == "DEBUG")
    gLogLevel = LevelDebug;
  else if (lvlstr == "TRACE")
    gLogLevel = LevelTrace;
  else
  {
    std::printf("Invalid value for environment variable TRAJOPT_LOG_THRESH: %s\n", lvlstr.c_str());
    std::printf("Valid values: %s\n", VALID_THRESH_VALUES);
    abort();
  }
  return 1;
}
int this_is_a_hack_but_rhs_executes_on_library_load = LoggingInit();
}
