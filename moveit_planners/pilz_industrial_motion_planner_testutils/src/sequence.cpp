/*
 * Copyright (c) 2019 Pilz GmbH & Co. KG
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "pilz_industrial_motion_planner_testutils/sequence.h"

#include <algorithm>
#include <boost/variant.hpp>

namespace pilz_industrial_motion_planner_testutils
{
/**
 * @brief Visitor transforming the stored command into a MotionPlanRequest.
 */
class ToReqVisitor : public boost::static_visitor<planning_interface::MotionPlanRequest>
{
public:
  template <typename T>
  planning_interface::MotionPlanRequest operator()(T& cmd) const
  {
    return cmd.toRequest();
  }
};

/**
 * @brief Visitor returning not the specific command type but the base type.
 */
class ToBaseVisitor : public boost::static_visitor<MotionCmd&>
{
public:
  template <typename T>
  MotionCmd& operator()(T& cmd) const
  {
    return cmd;
  }
};

moveit_msgs::MotionSequenceRequest Sequence::toRequest() const
{
  moveit_msgs::MotionSequenceRequest req;

  std::vector<std::string> group_names;
  for (const auto& cmd : cmds_)
  {
    moveit_msgs::MotionSequenceItem item;
    item.req = boost::apply_visitor(ToReqVisitor(), cmd.first);

    if (std::find(group_names.begin(), group_names.end(), item.req.group_name) != group_names.end())
    {
      // Remove start state because only the first request of a group
      // is allowed to have a start state in a sequence.
      item.req.start_state = moveit_msgs::RobotState();
    }
    else
    {
      group_names.emplace_back(item.req.group_name);
    }

    item.blend_radius = cmd.second;
    req.items.push_back(item);
  }
  return req;
}

void Sequence::erase(const size_t start, const size_t end)
{
  const size_t orig_n{ size() };
  if (start > orig_n || end > orig_n)
  {
    std::string msg;
    msg.append("Parameter start=").append(std::to_string(start));
    msg.append(" and end=").append(std::to_string(end));
    msg.append(" must not be greater then the number of #commands=");
    msg.append(std::to_string(size()));
    throw std::invalid_argument(msg);
  }
  cmds_.erase(cmds_.begin() + start, cmds_.begin() + end);
  if (end == orig_n)
  {
    // Make sure last radius is set zero
    cmds_.at(size() - 1).second = 0.;
  }
}

MotionCmd& Sequence::getCmd(const size_t index_cmd)
{
  return boost::apply_visitor(ToBaseVisitor(), cmds_.at(index_cmd).first);
}

}  // namespace pilz_industrial_motion_planner_testutils
