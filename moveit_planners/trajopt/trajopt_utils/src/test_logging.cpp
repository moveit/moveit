#include <trajopt_ros/utils/logging.hpp>

int main()
{
  LOG_FATAL("fatal");
  LOG_ERROR("error");
  LOG_WARN("warn");
  LOG_INFO("info");
  LOG_DEBUG("debug");
  LOG_TRACE("trace");
  printf("hi\n");
}
