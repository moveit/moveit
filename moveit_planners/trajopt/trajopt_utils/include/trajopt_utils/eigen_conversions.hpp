#pragma once
#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <vector>
TRAJOPT_IGNORE_WARNINGS_POP

namespace util
{
inline std::vector<double> toDblVec(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>& x)
{
  return std::vector<double>(x.data(), x.data() + x.size());
}
inline Eigen::VectorXd toVectorXd(const std::vector<double>& x)
{
  return Eigen::Map<const Eigen::VectorXd>(x.data(), static_cast<long int>(x.size()));
}
}
