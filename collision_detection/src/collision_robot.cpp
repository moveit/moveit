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

/** \author Ioan Sucan */

#include "collision_detection/collision_robot.h"

void collision_detection::CollisionRobot::setLinkPadding(const std::string &link_name, double padding)
{
    link_padding_[link_name] = padding;
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
    link_padding_ = padding;	    
}

const std::map<std::string, double> &collision_detection::CollisionRobot::getLinkPadding(void) const
{
    return link_padding_;
}

void collision_detection::CollisionRobot::setLinkScale(const std::string &link_name, double scale)
{
    link_scale_[link_name] = scale;
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
    link_scale_ = scale;	    
}

const std::map<std::string, double> &collision_detection::CollisionRobot::getLinkScale(void) const
{
    return link_scale_;
}
