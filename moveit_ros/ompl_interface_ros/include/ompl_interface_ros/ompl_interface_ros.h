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

/* Author: Ioan Sucan, Sachin Chitta */

#include <ompl_interface/ompl_interface.h>
#include <planning_scene_ros/planning_scene_ros.h>

namespace ompl_interface_ros
{
    class IKLoader;

    class OMPLInterfaceROS : public ompl_interface::OMPLInterface
    {
    public:

        OMPLInterfaceROS(const std::string &robot_description);

        bool computePlan(moveit_msgs::GetMotionPlan::Request &req, moveit_msgs::GetMotionPlan::Response &res);

        void status(void);

    protected:

        std::vector<std::string> getAdditionalConfigGroupNames(void);
        void configureIKSolvers(void);
        bool configurePlanners(void);

        ros::NodeHandle                       nh_;
        ros::ServiceServer                    plan_service_;
        planning_scene_ros::PlanningSceneROS *planning_scene_;
        planning_scene::PlanningScenePtr      planning_scene_ptr_;

    private:
        boost::shared_ptr<IKLoader>           ik_loader_;
    };

}
