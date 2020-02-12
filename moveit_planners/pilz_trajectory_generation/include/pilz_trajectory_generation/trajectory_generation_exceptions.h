/*
 * Copyright (c) 2019 Pilz GmbH & Co. KG
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef TRAJECTORY_GENERATION_EXCEPTIONS_H
#define TRAJECTORY_GENERATION_EXCEPTIONS_H

#include <stdexcept>
#include <string>

#include <moveit_msgs/MoveItErrorCodes.h>

namespace pilz_trajectory_generation
{

/**
 * @brief Exception storing an moveit_msgs::MoveItErrorCodes value.
 */
class MoveItErrorCodeException : public std::runtime_error
{
public:
  MoveItErrorCodeException(const std::string& msg);

protected:
  MoveItErrorCodeException(const MoveItErrorCodeException&) = default;
  MoveItErrorCodeException(MoveItErrorCodeException&&) = default;
  ~MoveItErrorCodeException() = default;

  MoveItErrorCodeException& operator=(const MoveItErrorCodeException&) = default;
  MoveItErrorCodeException& operator=(MoveItErrorCodeException&&) = default;

public:
  virtual const moveit_msgs::MoveItErrorCodes::_val_type& getErrorCode() const = 0;
};

template <moveit_msgs::MoveItErrorCodes::_val_type ERROR_CODE = moveit_msgs::MoveItErrorCodes::FAILURE>
class TemplatedMoveItErrorCodeException : public MoveItErrorCodeException
{
public:
  TemplatedMoveItErrorCodeException(const std::string& msg);
  TemplatedMoveItErrorCodeException(const std::string& msg,
                                    const moveit_msgs::MoveItErrorCodes::_val_type& error_code);

public:
  virtual const moveit_msgs::MoveItErrorCodes::_val_type& getErrorCode() const override;

private:
  const moveit_msgs::MoveItErrorCodes::_val_type error_code_ {ERROR_CODE};
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
inline MoveItErrorCodeException::MoveItErrorCodeException(const std::string& msg)
  : std::runtime_error(msg)
{}

template <moveit_msgs::MoveItErrorCodes::_val_type ERROR_CODE>
inline TemplatedMoveItErrorCodeException<ERROR_CODE>::TemplatedMoveItErrorCodeException(const std::string& msg)
  : MoveItErrorCodeException(msg)
{}

template <moveit_msgs::MoveItErrorCodes::_val_type ERROR_CODE>
inline TemplatedMoveItErrorCodeException<ERROR_CODE>::TemplatedMoveItErrorCodeException(const std::string& msg,
                                                                                        const moveit_msgs::MoveItErrorCodes::_val_type& error_code)
  : MoveItErrorCodeException(msg)
  , error_code_(error_code)
{}

template <moveit_msgs::MoveItErrorCodes::_val_type ERROR_CODE>
inline const moveit_msgs::MoveItErrorCodes::_val_type& TemplatedMoveItErrorCodeException<ERROR_CODE>::getErrorCode() const
{
  return error_code_;
}

/*
 * @brief Macro to automatically generate a derived class of
 * the MoveItErrorCodeException class.
 */
#define CREATE_MOVEIT_ERROR_CODE_EXCEPTION(EXCEPTION_CLASS_NAME, ERROR_CODE) \
class EXCEPTION_CLASS_NAME : public TemplatedMoveItErrorCodeException<ERROR_CODE> \
{ \
public: \
  EXCEPTION_CLASS_NAME(const std::string& msg) \
    : TemplatedMoveItErrorCodeException(msg) \
  {} \
\
  EXCEPTION_CLASS_NAME(const std::string& msg, const moveit_msgs::MoveItErrorCodes::_val_type& error_code) \
    : TemplatedMoveItErrorCodeException(msg, error_code) \
  {} \
}

}

#endif // TRAJECTORY_GENERATION_EXCEPTIONS_H
