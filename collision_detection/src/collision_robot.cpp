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
 *   * Neither the name of the Willow Garage nor the names of its
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

/* Author Ioan Sucan */

#include "collision_detection/collision_robot.h"
#include <limits>
#include <ros/console.h>

static inline bool validateScale(double scale)
{
    if (scale < std::numeric_limits<double>::epsilon())
    {
        ROS_ERROR("Scale must be positive");
        return false;
    }
    if (scale > std::numeric_limits<double>::max())
    {
        ROS_ERROR("Scale must be finite");
        return false;
    }
    return true;
}

static inline bool validatePadding(double padding)
{
    if (padding < 0.0)
    {
        ROS_ERROR("Padding cannot be negative");
        return false;
    }
    if (padding > std::numeric_limits<double>::max())
    {
        ROS_ERROR("Padding must be finite");
        return false;
    }
    return true;
}

collision_detection::CollisionRobot::CollisionRobot(const planning_models::KinematicModelConstPtr &kmodel, double padding, double scale) : kmodel_(kmodel)
{
    if (!validateScale(scale))
        scale = 1.0;
    if (!validatePadding(padding))
        padding = 0.0;

    const std::vector<planning_models::KinematicModel::LinkModel*>& links = kmodel_->getLinkModels();
    for (std::size_t i = 0 ; i < links.size() ; ++i)
        if (links[i] && links[i]->getShape())
        {
            link_padding_[links[i]->getName()] = padding;
            link_scale_[links[i]->getName()] = scale;
        }
}

void collision_detection::CollisionRobot::setPadding(double padding)
{
    if (!validatePadding(padding))
        return;
    std::vector<std::string> u;
    const std::vector<planning_models::KinematicModel::LinkModel*>& links = kmodel_->getLinkModels();
    for (std::size_t i = 0 ; i < links.size() ; ++i)
        if (links[i] && links[i]->getShape())
        {
            link_padding_[links[i]->getName()] = padding;
            u.push_back(links[i]->getName());
        }
    updatedPaddingOrScaling(u);
}

void collision_detection::CollisionRobot::setScale(double scale)
{
    if (!validateScale(scale))
        return;
    std::vector<std::string> u;
    const std::vector<planning_models::KinematicModel::LinkModel*>& links = kmodel_->getLinkModels();
    for (std::size_t i = 0 ; i < links.size() ; ++i)
        if (links[i] && links[i]->getShape())
        {
            link_scale_[links[i]->getName()] = scale;
            u.push_back(links[i]->getName());
        }
    updatedPaddingOrScaling(u);
}

void collision_detection::CollisionRobot::setLinkPadding(const std::string &link_name, double padding)
{
    link_padding_[link_name] = padding;
    std::vector<std::string> u(1, link_name);
    updatedPaddingOrScaling(u);
}

double collision_detection::CollisionRobot::getLinkPadding(const std::string &link_name) const
{
    std::map<std::string, double>::const_iterator it = link_padding_.find(link_name);
    if (it != link_padding_.end())
        return it->second;
    else
        return 0.0;
}

void collision_detection::CollisionRobot::setLinkPadding(const std::map<std::string, double> &padding)
{
    std::vector<std::string> u;
    for (std::map<std::string, double>::const_iterator it = padding.begin() ; it != padding.end() ; ++it)
    {
        link_padding_[it->first] = it->second;
        u.push_back(it->first);
    }
    updatedPaddingOrScaling(u);
}

const std::map<std::string, double> &collision_detection::CollisionRobot::getLinkPadding(void) const
{
    return link_padding_;
}

void collision_detection::CollisionRobot::setLinkScale(const std::string &link_name, double scale)
{
    link_scale_[link_name] = scale;
    std::vector<std::string> u(1, link_name);
    updatedPaddingOrScaling(u);
}

double collision_detection::CollisionRobot::getLinkScale(const std::string &link_name) const
{
    std::map<std::string, double>::const_iterator it = link_scale_.find(link_name);
    if (it != link_scale_.end())
        return it->second;
    else
        return 1.0;
}

void collision_detection::CollisionRobot::setLinkScale(const std::map<std::string, double> &scale)
{
    std::vector<std::string> u;
    for (std::map<std::string, double>::const_iterator it = scale.begin() ; it != scale.end() ; ++it)
    {
        link_scale_[it->first] = it->second;
        u.push_back(it->first);
    }
    updatedPaddingOrScaling(u);
}

const std::map<std::string, double> &collision_detection::CollisionRobot::getLinkScale(void) const
{
    return link_scale_;
}

void collision_detection::CollisionRobot::setPadding(const std::vector<moveit_msgs::LinkPadding> &padding)
{
    std::vector<std::string> u;
    for (std::size_t i = 0 ; i < padding.size() ; ++i)
    {
        link_padding_[padding[i].link_name] = padding[i].padding;
        u.push_back(padding[i].link_name);
    }
    updatedPaddingOrScaling(u);
}

void collision_detection::CollisionRobot::getPadding(std::vector<moveit_msgs::LinkPadding> &padding) const
{
    padding.clear();
    for (std::map<std::string, double>::const_iterator it = link_padding_.begin() ; it != link_padding_.end() ; ++it)
    {
        moveit_msgs::LinkPadding lp;
        lp.link_name = it->first;
        lp.padding = it->second;
        padding.push_back(lp);
    }
}

void collision_detection::CollisionRobot::updatedPaddingOrScaling(const std::vector<std::string> &links)
{
}
