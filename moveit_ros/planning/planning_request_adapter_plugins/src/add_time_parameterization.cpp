/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
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

/* Author: Ioan Sucan */

#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <class_loader/class_loader.h>
#include <ros/console.h>

/*
#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>
*/

namespace default_planner_request_adapters
{

class AddTimeParameterization : public planning_request_adapter::PlanningRequestAdapter
{
public:
  
  AddTimeParameterization() : planning_request_adapter::PlanningRequestAdapter()
  {
  }
  /*
  void reflexxesComputeTime(robot_trajectory::RobotTrajectory &traj) const
  {
    
    boost::scoped_ptr<ReflexxesAPI> RML;
    boost::scoped_ptr<RMLPositionInputParameters> IP;
    boost::scoped_ptr<RMLPositionOutputParameters> OP;
    RMLPositionFlags Flags;
    
    // ********************************************************************
    // Creating all relevant objects of the Reflexxes Motion Library	
    
    RML.reset(new ReflexxesAPI(7, 0.01));
    IP.reset(new RMLPositionInputParameters(7));
    OP.reset(new RMLPositionOutputParameters(7));
    
    std::vector<double> v;
    traj.getFirstWayPoint().getJointStateGroup(traj.getGroup()->getName())->getVariableValues(v);
    
    for (int i = 0; i < 7 ; ++i)
    {
      IP->CurrentPositionVector->VecData		[i]	=	   v[i]; // rad
      IP->CurrentVelocityVector->VecData		[i]	=	   0.0		; // rad/s
      IP->CurrentAccelerationVector->VecData	        [i]	=	   0.0		; // not needed for type 2
      IP->MaxVelocityVector->VecData			[i]	=	   1.5		; // rad/s
      IP->MaxAccelerationVector->VecData		[i]	=	   1.0		; // 
      IP->MaxJerkVector->VecData			[i]	=	   0.1		; // not needed for type 2
      IP->TargetPositionVector->VecData		        [i]	=	   v[i]; // first waypoint
      IP->TargetVelocityVector->VecData		        [i]	=	   0.0		; // velocity at first waypoint
      IP->SelectionVector->VecData			[i]	=	  true		;
    }	
    bool error = false;
    for (std::size_t i = 1 ; !error && i < traj.getWayPointCount() ; ++i)
    {
      int ResultValue = 0;
      double totalt = 0.0;
      
      while (ResultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED)
      {
        
        // ****************************************************************									
        // Wait for the next timer tick
        // (not implemented in this example in order to keep it simple)
        // ****************************************************************	
        
        // Calling the Reflexxes OTG algorithm
        ResultValue	=	RML->RMLPosition(*IP,	OP.get(),	Flags);
        
        if (ResultValue < 0)
        {
          printf("An error occurred (%d).\n", ResultValue	);
          error = true;
          break;
        }
        
        totalt += 0.01;
        
        
        // ****************************************************************									
        // Here, the new state of motion, that is
        //
        // - OP->NewPositionVector		
        // - OP->NewVelocityVector		
        // - OP->NewAccelerationVector
        //
        // can be used as input values for lower level controllers. In the 
        // most simple case, a position controller in actuator space is 
        // used, but the computed state can be applied to many other 
        // controllers (e.g., Cartesian impedance controllers, 
        // operational space controllers).
        // ****************************************************************
        
        // ****************************************************************
        // Feed the output values of the current control cycle back to 
        // input values of the next control cycle
        
        *IP->CurrentPositionVector		=	*OP->NewPositionVector		;
        *IP->CurrentVelocityVector		=	*OP->NewVelocityVector		;
        *IP->CurrentAccelerationVector	        =	*OP->NewAccelerationVector	;
      }
      std::cout << "T" << i << " :" << totalt << std::endl;
      
      traj.setWayPointDurationFromPrevious(i, totalt);
      traj.getWayPoint(i).getJointStateGroup(traj.getGroup()->getName())->getVariableValues(v);
      for (int k = 0 ; k < 7 ; ++k)
        IP->TargetPositionVector->VecData[k] = v[k];      
    }
  }
  */
  virtual std::string getDescription() const { return "Add Time Parameterization"; }
  
  virtual bool adaptAndPlan(const PlannerFn &planner,
                            const planning_scene::PlanningSceneConstPtr& planning_scene,
                            const planning_interface::MotionPlanRequest &req, 
                            planning_interface::MotionPlanResponse &res,
                            std::vector<std::size_t> &added_path_index) const
  { 
    bool result = planner(planning_scene, req, res);
    if (result && res.trajectory_)
    {  
      ROS_DEBUG("Running '%s'", getDescription().c_str());
      time_param_.computeTimeStamps(*res.trajectory_, req.start_state);
    }
    
    return result;
  }   
  
private:
  
  trajectory_processing::IterativeParabolicTimeParameterization time_param_;
};

}

CLASS_LOADER_REGISTER_CLASS(default_planner_request_adapters::AddTimeParameterization,
                            planning_request_adapter::PlanningRequestAdapter);
