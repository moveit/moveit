//Software License Agreement (BSD License)

//Copyright (c) 2008, Willow Garage, Inc.
//All rights reserved.

//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions
//are met:

// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of Willow Garage, Inc. nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.

//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//POSSIBILITY OF SUCH DAMAGE.

#include "pr2_arm_kinematics_utils.h"
#include "pr2_arm_kinematics_constants.h"

namespace pr2_arm_kinematics
{
  static const double IK_DEFAULT_TIMEOUT = 10.0;

bool getKDLChain(const urdf::ModelInterface& model, const std::string &root_name, const std::string &tip_name, KDL::Chain &kdl_chain)
  {
    // create robot chain from root to tip
    KDL::Tree tree;
    if (!kdl_parser::treeFromUrdfModel(model, tree))
    {
      ROS_ERROR("Could not initialize tree object");
      return false;
    }
    if (!tree.getChain(root_name, tip_name, kdl_chain))
    {
      ROS_ERROR_STREAM("Could not initialize chain object for base " << root_name << " tip " << tip_name);
      return false;
    }
    return true;
  }


  bool getKDLChain(const std::string &xml_string, const std::string &root_name, const std::string &tip_name, KDL::Chain &kdl_chain)
  {
    // create robot chain from root to tip
    KDL::Tree tree;
    if (!kdl_parser::treeFromString(xml_string, tree))
    {
      ROS_ERROR("Could not initialize tree object");
      return false;
    }
    if (!tree.getChain(root_name, tip_name, kdl_chain))
    {
      ROS_ERROR_STREAM("Could not initialize chain object for base " << root_name << " tip " << tip_name);
      return false;
    }
    return true;
  }

  bool getKDLTree(const std::string &xml_string, const std::string &root_name, const std::string &tip_name, KDL::Tree &kdl_tree)
  {
    // create robot chain from root to tip
    if (!kdl_parser::treeFromString(xml_string, kdl_tree))
    {
      ROS_ERROR("Could not initialize tree object");
      return false;
    }
    return true;
  }


  Eigen::Matrix4f KDLToEigenMatrix(const KDL::Frame &p)
  {
    Eigen::Matrix4f b = Eigen::Matrix4f::Identity();
    for(int i=0; i < 3; i++)
    {
      for(int j=0; j<3; j++)
      {
        b(i,j) = p.M(i,j);
      }
      b(i,3) = p.p(i);
    }
    return b;
  }

  double computeEuclideanDistance(const std::vector<double> &array_1, const KDL::JntArray &array_2)
  {
    double distance = 0.0;
    for(int i=0; i< (int) array_1.size(); i++)
    {
      distance += (array_1[i] - array_2(i))*(array_1[i] - array_2(i));
    }
    return sqrt(distance);
  }


  double distance(const urdf::Pose &transform)
  {
    return sqrt(transform.position.x*transform.position.x+transform.position.y*transform.position.y+transform.position.z*transform.position.z);
  }


  bool solveQuadratic(const double &a, const double &b, const double &c, double *x1, double *x2)
  {
    double discriminant = b*b-4*a*c;
    if(fabs(a) < IK_EPS)
    {
      *x1 = -c/b;
      *x2 = *x1;
      return true;
    }
    //ROS_DEBUG("Discriminant: %f\n",discriminant);
    if (discriminant >= 0)
    {      
      *x1 = (-b + sqrt(discriminant))/(2*a); 
      *x2 = (-b - sqrt(discriminant))/(2*a);
      return true;
    } 
    else if(fabs(discriminant) < IK_EPS)
    {
      *x1 = -b/(2*a);
      *x2 = -b/(2*a);
      return true;
    }
    else
    {
      *x1 = -b/(2*a);
      *x2 = -b/(2*a);
      return false;
    }
  }

  Eigen::Matrix4f matrixInverse(const Eigen::Matrix4f &g)
  {
    Eigen::Matrix4f result = g;
    Eigen::Matrix3f Rt = Eigen::Matrix3f::Identity();

    Eigen::Vector3f p = Eigen::Vector3f::Zero(3);
    Eigen::Vector3f pinv = Eigen::Vector3f::Zero(3);

    Rt(0,0) = g(0,0);
    Rt(1,1) = g(1,1);
    Rt(2,2) = g(2,2);

    Rt(0,1) = g(1,0);
    Rt(1,0) = g(0,1);

    Rt(0,2) = g(2,0);
    Rt(2,0) = g(0,2);

    Rt(1,2) = g(2,1);
    Rt(2,1) = g(1,2);

    p(0) = g(0,3);
    p(1) = g(1,3);
    p(2) = g(2,3);

    pinv = -Rt*p;

    result(0,0) = g(0,0);
    result(1,1) = g(1,1);
    result(2,2) = g(2,2);

    result(0,1) = g(1,0);
    result(1,0) = g(0,1);

    result(0,2) = g(2,0);
    result(2,0) = g(0,2);

    result(1,2) = g(2,1);
    result(2,1) = g(1,2);

    result(0,3) = pinv(0);
    result(1,3) = pinv(1);
    result(2,3) = pinv(2);
  
    return result;
  }


  bool solveCosineEqn(const double &a, const double &b, const double &c, double &soln1, double &soln2)
  {
    double theta1 = atan2(b,a);
    double denom  = sqrt(a*a+b*b);

    if(fabs(denom) < IK_EPS) // should never happen, wouldn't make sense but make sure it is checked nonetheless
    {
#ifdef DEBUG
      std::cout << "denom: " << denom << std::endl;
#endif
      return false;
    }
    double rhs_ratio = c/denom;
    if(rhs_ratio < -1 || rhs_ratio > 1)
    {
#ifdef DEBUG
      std::cout << "rhs_ratio: " << rhs_ratio << std::endl;
#endif
      return false;
    }
    double acos_term = acos(rhs_ratio);
    soln1 = theta1 + acos_term;
    soln2 = theta1 - acos_term;

    return true;
  }


bool checkJointNames(const std::vector<std::string> &joint_names,
                       const moveit_msgs::KinematicSolverInfo &chain_info)
  {    
    for(unsigned int i=0; i < chain_info.joint_names.size(); i++)
    {
      int index = -1;
      for(unsigned int j=0; j < joint_names.size(); j++)
      {
        if(chain_info.joint_names[i] == joint_names[j])
        {
          index = j;
          break;
        }
      }
      if(index < 0)
      {
        ROS_ERROR("Joint state does not contain joint state for %s.",chain_info.joint_names[i].c_str());
        return false;
      }
    }
    return true;
  }

  bool checkLinkNames(const std::vector<std::string> &link_names,
                      const moveit_msgs::KinematicSolverInfo &chain_info)
  {
    if(link_names.empty())
      return false;
    for(unsigned int i=0; i < link_names.size(); i++)
    {
      if(!checkLinkName(link_names[i],chain_info))
        return false;
    }
    return true;   
  }

  bool checkLinkName(const std::string &link_name,
                   const moveit_msgs::KinematicSolverInfo &chain_info)
  {
    for(unsigned int i=0; i < chain_info.link_names.size(); i++)
    {
      if(link_name == chain_info.link_names[i])
        return true;
    }
    return false;   
  }

  bool checkRobotState(moveit_msgs::RobotState &robot_state,
                     const moveit_msgs::KinematicSolverInfo &chain_info)
  {
    if((int) robot_state.joint_state.position.size() != (int) robot_state.joint_state.name.size())
    {
      ROS_ERROR("Number of joints in robot_state.joint_state does not match number of positions in robot_state.joint_state");
      return false;
    }    
    if(!checkJointNames(robot_state.joint_state.name,chain_info))
    {
      ROS_ERROR("Robot state must contain joint state for every joint in the kinematic chain");
      return false;
    }
    return true;
  }

  bool checkFKService(moveit_msgs::GetPositionFK::Request &request, 
                      moveit_msgs::GetPositionFK::Response &response, 
                      const moveit_msgs::KinematicSolverInfo &chain_info)
  {
    if(!checkLinkNames(request.fk_link_names,chain_info))
    {
      ROS_ERROR("Link name in service request does not match links that kinematics can provide solutions for.");
      response.error_code.val = response.error_code.INVALID_LINK_NAME;
      return false;
    }
    if(!checkRobotState(request.robot_state,chain_info))
    {
      response.error_code.val = response.error_code.INVALID_ROBOT_STATE;
      return false;
    }
    return true;
  }

  int getJointIndex(const std::string &name,
                  const moveit_msgs::KinematicSolverInfo &chain_info)
  {
    for(unsigned int i=0; i < chain_info.joint_names.size(); i++)
    {
      if(chain_info.joint_names[i] == name)
      {
          return i;
      }
    }
    return -1;
  }

  void getKDLChainInfo(const KDL::Chain &chain,
                       moveit_msgs::KinematicSolverInfo &chain_info)
  {
    int i=0; // segment number
    while(i < (int)chain.getNrOfSegments())
    {
      chain_info.link_names.push_back(chain.getSegment(i).getName());
      i++;
    }
  }

  int getKDLSegmentIndex(const KDL::Chain &chain, 
                         const std::string &name)
  {
    int i=0; // segment number
    while(i < (int)chain.getNrOfSegments())
    {
      if(chain.getSegment(i).getName() == name)
      {
        return i+1;
      }
      i++;
    }
    return -1;   
  }


}
