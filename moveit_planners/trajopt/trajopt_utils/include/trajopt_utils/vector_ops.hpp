#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <vector>
TRAJOPT_IGNORE_WARNINGS_POP

namespace util
{
std::vector<int> arange(int n)
{
  std::vector<int> out(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i)
    out[static_cast<size_t>(i)] = i;
  return out;
}

inline bool doubleEquals(double x, double y, double eps = 1E-5) { return std::abs(x - y) < eps; }
}
