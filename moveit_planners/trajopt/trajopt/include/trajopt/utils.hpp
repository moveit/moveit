#pragma once
#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <unordered_map>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt/typedefs.hpp>

#include <iostream>

namespace trajopt
{
template <typename Key, typename Value>
using AlignedUnorderedMap = std::unordered_map<Key, Value, std::hash<Key>, std::equal_to<Key>,
                                               Eigen::aligned_allocator<std::pair<const Key, Value>>>;

/**
Extract trajectory array from solution vector x using indices in array vars
*/
TrajArray TRAJOPT_API getTraj(const DblVec& x, const VarArray& vars);
TrajArray TRAJOPT_API getTraj(const DblVec& x, const AffArray& arr);

inline DblVec trajToDblVec(const TrajArray& x)
{
  return DblVec(x.data(), x.data() + x.rows() * x.cols());
}

/**
 * @brief Appends b to a of type VectorXd
 */
inline Eigen::VectorXd concat(const Eigen::VectorXd& a, const Eigen::VectorXd& b)
{
  Eigen::VectorXd out(a.size() + b.size());
  out.topRows(a.size()) = a;
  out.middleRows(a.size(), b.size()) = b;
  return out;
}

/**
 * @brief Appends b to a of type T
 */
template <typename T>
std::vector<T> concat(const std::vector<T>& a, const std::vector<T>& b)
{
  std::vector<T> out;
  std::vector<int> x(a.size() + b.size());
  out.insert(out.end(), a.begin(), a.end());
  out.insert(out.end(), b.begin(), b.end());
  return out;
}

template <typename T>
std::vector<T> singleton(const T& x)
{
  return std::vector<T>(1, x);
}

void TRAJOPT_API AddVarArrays(sco::OptProb& prob, int rows, const std::vector<int>& cols,
                              const std::vector<std::string>& name_prefix, const std::vector<VarArray*>& newvars);

void TRAJOPT_API AddVarArray(sco::OptProb& prob, int rows, int cols, const std::string& name_prefix, VarArray& newvars);

/** @brief Store Safety Margin Data for a given timestep */
struct SafetyMarginData
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SafetyMarginData(const double& default_safety_margin, const double& default_safety_margin_coeff)
    : default_safety_margin_data_(default_safety_margin, default_safety_margin_coeff)
    , max_safety_margin_(default_safety_margin)
  {
  }

  /**
   * @brief Set the safety margin for a given contact pair
   *
   * The order of the object names does not matter, that is handled internal to
   * the class.
   *
   * @param obj1 The first object name
   * @param obj2 The Second object name
   * @param safety_margins contacts with distance < safety_margin are penalized
   * @param safety_margin_coeffs A safety margin coefficient vector where each
   * element corresponds to a given timestep.
   */
  void SetPairSafetyMarginData(const std::string& obj1, const std::string& obj2, const double& safety_margin,
                               const double& safety_margin_coeff)
  {
    Eigen::Vector2d data(safety_margin, safety_margin_coeff);

    pair_lookup_table_[obj1 + obj2] = data;
    pair_lookup_table_[obj2 + obj1] = data;

    if (safety_margin > max_safety_margin_)
    {
      max_safety_margin_ = safety_margin;
    }
  }

  const Eigen::Vector2d& getPairSafetyMarginData(const std::string& obj1, const std::string& obj2) const
  {
    const std::string& key = obj1 + obj2;
    auto it = pair_lookup_table_.find(key);

    if (it != pair_lookup_table_.end())
    {
      return it->second;
    }
    else
    {
      return default_safety_margin_data_;
    }
  }

  const double& getMaxSafetyMargin() const
  {
    return max_safety_margin_;
  }

private:
  /// The coeff used during optimization
  /// safety margin: contacts with distance < dist_pen are penalized
  /// Stores [dist_pen, coeff]
  Eigen::Vector2d default_safety_margin_data_;

  /// This use when requesting collision data because you can only provide a
  /// single contact distance threshold.
  double max_safety_margin_;

  /// A map of link pair to contact distance setting [dist_pen, coeff]
  AlignedUnorderedMap<std::string, Eigen::Vector2d> pair_lookup_table_;
};
typedef std::shared_ptr<SafetyMarginData> SafetyMarginDataPtr;
typedef std::shared_ptr<const SafetyMarginData> SafetyMarginDataConstPtr;

/**
 * @brief This is a utility function for creating the Safety Margin data vector
 * @param num_elements The number of objects to create
 * @param default_safety_margin Default safety margin
 * @param default_safety_margin_coeff Default safety margin coeff
 * @return A vector of Safety Margin Data
 */
inline std::vector<SafetyMarginDataPtr> createSafetyMarginDataVector(int num_elements,
                                                                     const double& default_safety_margin,
                                                                     const double& default_safety_margin_coeff)
{
  std::vector<SafetyMarginDataPtr> info;
  info.reserve(static_cast<size_t>(num_elements));
  for (auto i = 0; i < num_elements; ++i)
  {
    info.push_back(SafetyMarginDataPtr(new SafetyMarginData(default_safety_margin, default_safety_margin_coeff)));
  }
  return info;
}

/**
 * Print a vector
 * */
template <typename K>
void printVector(std::string str, std::vector<K> v)
{
  std::stringstream ss;
  ss << str << " ";
  for (std::size_t i = 0; i < v.size(); ++i)
      ss << v[i] << " ";

  std::cout << ss.str() << std::endl;
}

}
