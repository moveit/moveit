#pragma once
#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <cmath>
TRAJOPT_IGNORE_WARNINGS_POP

namespace util
{
float randf() { return (float)rand() / (float)RAND_MAX; }
}
