#pragma once

#include <moveit_msgs/MoveItErrorCodes.h>

namespace moveit
{
namespace core
{
/**
 * @brief a wrapper around moveit_msgs::MoveItErrorCodes to make it easier to return an error code message from a function
 */
class MoveItErrorCode : public moveit_msgs::MoveItErrorCodes
{
public:
  MoveItErrorCode()
  {
    val = 0;
  }
  MoveItErrorCode(int code)
  {
    val = code;
  }
  MoveItErrorCode(const moveit_msgs::MoveItErrorCodes& code)
  {
    val = code.val;
  }
  explicit operator bool() const
  {
    return val == moveit_msgs::MoveItErrorCodes::SUCCESS;
  }
  bool operator==(const int c) const
  {
    return val == c;
  }
  bool operator!=(const int c) const
  {
    return val != c;
  }
};

}  // namespace core
}  // namespace moveit
