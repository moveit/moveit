/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#include <moveit/collision_detection/collision_robot.h>
#include <limits>

static inline bool validateScale(double scale)
{
  if (scale < std::numeric_limits<double>::epsilon())
  {
    CONSOLE_BRIDGE_logError("Scale must be positive");
    return false;
  }
  if (scale > std::numeric_limits<double>::max())
  {
    CONSOLE_BRIDGE_logError("Scale must be finite");
    return false;
  }
  return true;
}

static inline bool validatePadding(double padding)
{
  if (padding < 0.0)
  {
    CONSOLE_BRIDGE_logError("Padding cannot be negative");
    return false;
  }
  if (padding > std::numeric_limits<double>::max())
  {
    CONSOLE_BRIDGE_logError("Padding must be finite");
    return false;
  }
  return true;
}

<<<<<<< HEAD
namespace collision_detection
{
CollisionRobot::CollisionRobot(const robot_model::RobotModelConstPtr& model,  // NOLINT
                               double padding, double scale)
=======
collision_detection::CollisionRobot::CollisionRobot(const robot_model::RobotModelConstPtr& model, double padding,
                                                    double scale)
>>>>>>> upstream/indigo-devel
  : robot_model_(model)
{
  if (!validateScale(scale))
    scale = 1.0;
  if (!validatePadding(padding))
    padding = 0.0;

  const std::vector<const robot_model::LinkModel*>& links = robot_model_->getLinkModelsWithCollisionGeometry();
<<<<<<< HEAD
  for (auto link : links)
=======
  for (std::size_t i = 0; i < links.size(); ++i)
>>>>>>> upstream/indigo-devel
  {
    link_padding_[link->getName()] = padding;
    link_scale_[link->getName()] = scale;
  }
}

<<<<<<< HEAD
CollisionRobot::CollisionRobot(const CollisionRobot& other) : robot_model_(other.robot_model_)
=======
collision_detection::CollisionRobot::CollisionRobot(const CollisionRobot& other) : robot_model_(other.robot_model_)
>>>>>>> upstream/indigo-devel
{
  link_padding_ = other.link_padding_;
  link_scale_ = other.link_scale_;
}

void CollisionRobot::setPadding(double padding)
{
  if (!validatePadding(padding))
    return;
  std::vector<std::string> u;
  const std::vector<const robot_model::LinkModel*>& links = robot_model_->getLinkModelsWithCollisionGeometry();
<<<<<<< HEAD
  for (auto link : links)
=======
  for (std::size_t i = 0; i < links.size(); ++i)
>>>>>>> upstream/indigo-devel
  {
    if (getLinkPadding(link->getName()) != padding)
      u.push_back(link->getName());
    link_padding_[link->getName()] = padding;
  }
  if (!u.empty())
    updatedPaddingOrScaling(u);
}

void CollisionRobot::setScale(double scale)
{
  if (!validateScale(scale))
    return;
  std::vector<std::string> u;
  const std::vector<const robot_model::LinkModel*>& links = robot_model_->getLinkModelsWithCollisionGeometry();
<<<<<<< HEAD
  for (auto link : links)
=======
  for (std::size_t i = 0; i < links.size(); ++i)
>>>>>>> upstream/indigo-devel
  {
    if (getLinkScale(link->getName()) != scale)
      u.push_back(link->getName());
    link_scale_[link->getName()] = scale;
  }
  if (!u.empty())
    updatedPaddingOrScaling(u);
}

<<<<<<< HEAD
void CollisionRobot::setLinkPadding(const std::string& link_name, double padding)
=======
void collision_detection::CollisionRobot::setLinkPadding(const std::string& link_name, double padding)
>>>>>>> upstream/indigo-devel
{
  bool update = getLinkPadding(link_name) != padding;
  link_padding_[link_name] = padding;
  if (update)
  {
    std::vector<std::string> u(1, link_name);
    updatedPaddingOrScaling(u);
  }
}

<<<<<<< HEAD
double CollisionRobot::getLinkPadding(const std::string& link_name) const
=======
double collision_detection::CollisionRobot::getLinkPadding(const std::string& link_name) const
>>>>>>> upstream/indigo-devel
{
  auto it = link_padding_.find(link_name);
  if (it != link_padding_.end())
    return it->second;
  else
    return 0.0;
}

<<<<<<< HEAD
void CollisionRobot::setLinkPadding(const std::map<std::string, double>& padding)
{
  std::vector<std::string> u;
  for (const auto& link_pad_pair : padding)
=======
void collision_detection::CollisionRobot::setLinkPadding(const std::map<std::string, double>& padding)
{
  std::vector<std::string> u;
  for (std::map<std::string, double>::const_iterator it = padding.begin(); it != padding.end(); ++it)
>>>>>>> upstream/indigo-devel
  {
    bool update = getLinkPadding(link_pad_pair.first) != link_pad_pair.second;
    link_padding_[link_pad_pair.first] = link_pad_pair.second;
    if (update)
      u.push_back(link_pad_pair.first);
  }
  if (!u.empty())
    updatedPaddingOrScaling(u);
}

<<<<<<< HEAD
const std::map<std::string, double>& CollisionRobot::getLinkPadding() const
=======
const std::map<std::string, double>& collision_detection::CollisionRobot::getLinkPadding() const
>>>>>>> upstream/indigo-devel
{
  return link_padding_;
}

<<<<<<< HEAD
void CollisionRobot::setLinkScale(const std::string& link_name, double scale)
=======
void collision_detection::CollisionRobot::setLinkScale(const std::string& link_name, double scale)
>>>>>>> upstream/indigo-devel
{
  bool update = getLinkScale(link_name) != scale;
  link_scale_[link_name] = scale;
  if (update)
  {
    std::vector<std::string> u(1, link_name);
    updatedPaddingOrScaling(u);
  }
}

<<<<<<< HEAD
double CollisionRobot::getLinkScale(const std::string& link_name) const
=======
double collision_detection::CollisionRobot::getLinkScale(const std::string& link_name) const
>>>>>>> upstream/indigo-devel
{
  auto it = link_scale_.find(link_name);
  if (it != link_scale_.end())
    return it->second;
  else
    return 1.0;
}

<<<<<<< HEAD
void CollisionRobot::setLinkScale(const std::map<std::string, double>& scale)
{
  std::vector<std::string> u;
  for (const auto& link_scale_pair : scale)
=======
void collision_detection::CollisionRobot::setLinkScale(const std::map<std::string, double>& scale)
{
  std::vector<std::string> u;
  for (std::map<std::string, double>::const_iterator it = scale.begin(); it != scale.end(); ++it)
>>>>>>> upstream/indigo-devel
  {
    bool update = getLinkScale(link_scale_pair.first) != link_scale_pair.second;
    link_scale_[link_scale_pair.first] = link_scale_pair.second;
    if (update)
      u.push_back(link_scale_pair.first);
  }
  if (!u.empty())
    updatedPaddingOrScaling(u);
}

<<<<<<< HEAD
const std::map<std::string, double>& CollisionRobot::getLinkScale() const
=======
const std::map<std::string, double>& collision_detection::CollisionRobot::getLinkScale() const
>>>>>>> upstream/indigo-devel
{
  return link_scale_;
}

<<<<<<< HEAD
void CollisionRobot::setPadding(const std::vector<moveit_msgs::LinkPadding>& padding)
{
  std::vector<std::string> u;
  for (const auto& p : padding)
=======
void collision_detection::CollisionRobot::setPadding(const std::vector<moveit_msgs::LinkPadding>& padding)
{
  std::vector<std::string> u;
  for (std::size_t i = 0; i < padding.size(); ++i)
>>>>>>> upstream/indigo-devel
  {
    bool update = getLinkPadding(p.link_name) != p.padding;
    link_padding_[p.link_name] = p.padding;
    if (update)
      u.push_back(p.link_name);
  }
  if (!u.empty())
    updatedPaddingOrScaling(u);
}

<<<<<<< HEAD
void CollisionRobot::setScale(const std::vector<moveit_msgs::LinkScale>& scale)
{
  std::vector<std::string> u;
  for (const auto& s : scale)
=======
void collision_detection::CollisionRobot::setScale(const std::vector<moveit_msgs::LinkScale>& scale)
{
  std::vector<std::string> u;
  for (std::size_t i = 0; i < scale.size(); ++i)
>>>>>>> upstream/indigo-devel
  {
    bool update = getLinkScale(s.link_name) != s.scale;
    link_scale_[s.link_name] = s.scale;
    if (update)
      u.push_back(s.link_name);
  }
  if (!u.empty())
    updatedPaddingOrScaling(u);
}

<<<<<<< HEAD
void CollisionRobot::getPadding(std::vector<moveit_msgs::LinkPadding>& padding) const
{
  padding.clear();
  for (const auto& lp : link_padding_)
=======
void collision_detection::CollisionRobot::getPadding(std::vector<moveit_msgs::LinkPadding>& padding) const
{
  padding.clear();
  for (std::map<std::string, double>::const_iterator it = link_padding_.begin(); it != link_padding_.end(); ++it)
>>>>>>> upstream/indigo-devel
  {
    moveit_msgs::LinkPadding lp_msg;
    lp_msg.link_name = lp.first;
    lp_msg.padding = lp.second;
    padding.push_back(lp_msg);
  }
}

<<<<<<< HEAD
void CollisionRobot::getScale(std::vector<moveit_msgs::LinkScale>& scale) const
{
  scale.clear();
  for (const auto& ls : link_scale_)
=======
void collision_detection::CollisionRobot::getScale(std::vector<moveit_msgs::LinkScale>& scale) const
{
  scale.clear();
  for (std::map<std::string, double>::const_iterator it = link_scale_.begin(); it != link_scale_.end(); ++it)
>>>>>>> upstream/indigo-devel
  {
    moveit_msgs::LinkScale ls_msg;
    ls_msg.link_name = ls.first;
    ls_msg.scale = ls.second;
    scale.push_back(ls_msg);
  }
}

<<<<<<< HEAD
void CollisionRobot::updatedPaddingOrScaling(const std::vector<std::string>& links)
=======
void collision_detection::CollisionRobot::updatedPaddingOrScaling(const std::vector<std::string>& links)
>>>>>>> upstream/indigo-devel
{
}

}  // end of namespace collision_detection