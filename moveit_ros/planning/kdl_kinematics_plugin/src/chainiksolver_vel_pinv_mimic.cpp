// Copyright  (C)  2007  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>

// Version: 1.0
// Author: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Maintainer: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// URL: http://www.orocos.org/kdl

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

// Modified to account for "mimic" joints, i.e. joints whose motion has a 
// linear relationship to that of another joint. 
// Copyright  (C)  2013  Sachin Chitta, Willow Garage

#include <moveit/kdl_kinematics_plugin/chainiksolver_vel_pinv_mimic.hpp>
#include <ros/console.h>

namespace KDL
{
ChainIkSolverVel_pinv_mimic::ChainIkSolverVel_pinv_mimic(const Chain& _chain, int _num_mimic_joints, bool _position_ik, double _eps, int _maxiter):
  position_ik(_position_ik),
  chain(_chain),
  jnt2jac(chain),
  jac(chain.getNrOfJoints()),
  jac_reduced(chain.getNrOfJoints()-_num_mimic_joints),
  svd(jac_reduced),
  U_translate(MatrixXd::Zero(3,chain.getNrOfJoints()-_num_mimic_joints)),
  S_translate(VectorXd::Zero(chain.getNrOfJoints()-_num_mimic_joints)),
  V_translate(MatrixXd::Zero(chain.getNrOfJoints()-_num_mimic_joints,chain.getNrOfJoints()-_num_mimic_joints)),
  tmp_translate(VectorXd::Zero(chain.getNrOfJoints())),
  U(6,JntArray(chain.getNrOfJoints()-_num_mimic_joints)),
  S(chain.getNrOfJoints()-_num_mimic_joints),
  V(chain.getNrOfJoints()-_num_mimic_joints,JntArray(chain.getNrOfJoints()-_num_mimic_joints)),
  tmp(chain.getNrOfJoints()-_num_mimic_joints),
  eps(_eps),
  maxiter(_maxiter),
  qdot_out_reduced(chain.getNrOfJoints()-_num_mimic_joints),
  num_mimic_joints(_num_mimic_joints)
{
  mimic_joints_.resize(chain.getNrOfJoints());
  for(std::size_t i=0; i < mimic_joints_.size(); ++i)
    mimic_joints_[i].reset(i);    
}

ChainIkSolverVel_pinv_mimic::~ChainIkSolverVel_pinv_mimic()
{
}

bool ChainIkSolverVel_pinv_mimic::setMimicJoints(const std::vector<kdl_kinematics_plugin::JointMimic> & mimic_joints)
{
  if(mimic_joints.size() != chain.getNrOfJoints())
    return false;
  
  for(std::size_t i=0; i < mimic_joints.size(); ++i)
  {
    if(mimic_joints[i].map_index >= chain.getNrOfJoints())
      return false;
  }
  mimic_joints_ = mimic_joints;
  return true;
}

bool ChainIkSolverVel_pinv_mimic::jacToJacReduced(const Jacobian &jac, Jacobian &jac_reduced)
{
  jac_reduced.data.setZero();  
  for(std::size_t i=0; i < chain.getNrOfJoints(); ++i)
  {
    Twist vel1 = jac_reduced.getColumn(mimic_joints_[i].map_index);
    Twist vel2 = jac.getColumn(i);
    Twist result = vel1 + (mimic_joints_[i].multiplier*vel2);
    jac_reduced.setColumn(mimic_joints_[i].map_index,result);    
  }
  return true;
}

int ChainIkSolverVel_pinv_mimic::CartToJnt(const JntArray& q_in, const Twist& v_in, JntArray& qdot_out)
{
  //Let the ChainJntToJacSolver calculate the jacobian "jac" for
  //the current joint positions "q_in". This will include the mimic joints
  jnt2jac.JntToJac(q_in,jac);
  
  //Now compute the actual jacobian that involves only the active DOFs
  jacToJacReduced(jac,jac_reduced);
  //Do a singular value decomposition of "jac" with maximum
  //iterations "maxiter", put the results in "U", "S" and "V"
  //jac = U*S*Vt

  int ret;  
  if(!position_ik)
    ret = svd.calculate(jac_reduced,U,S,V,maxiter);
  else
    ret = svd_eigen_HH(jac_reduced.data.topLeftCorner(3,chain.getNrOfJoints()-num_mimic_joints),U_translate,S_translate,V_translate,tmp_translate,maxiter);
 
  double sum;
  unsigned int i,j;
  
  // We have to calculate qdot_out = jac_pinv*v_in
  // Using the svd decomposition this becomes(jac_pinv=V*S_pinv*Ut):
  // qdot_out = V*S_pinv*Ut*v_in

  unsigned int columns, rows;
  if(!position_ik)
    rows = jac_reduced.rows();
  else
    rows = 3;
  
  //first we calculate Ut*v_in
  for (i=0;i<jac_reduced.columns();i++) {
    sum = 0.0;
    for (j=0;j<rows;j++) {
      if(!position_ik)
        sum+= U[j](i)*v_in(j);
      else
        sum+= U_translate(j,i)*v_in(j);
    }
    //If the singular value is too small (<eps), don't invert it but
    //set the inverted singular value to zero (truncated svd)
    if(!position_ik)
      tmp(i) = sum*(fabs(S(i))<eps?0.0:1.0/S(i));
    else
      tmp(i) = sum*(fabs(S_translate(i))<eps?0.0:1.0/S_translate(i));
  }
  //tmp is now: tmp=S_pinv*Ut*v_in, we still have to premultiply
  //it with V to get qdot_out
  for (i=0;i<jac_reduced.columns();i++) {
    sum = 0.0;
    for (j=0;j<jac_reduced.columns();j++) {
      if(!position_ik)
        sum+=V[i](j)*tmp(j);
      else
        sum+=V_translate(i,j)*tmp(j);        
    }
    //Put the result in qdot_out
    qdot_out_reduced(i)=sum;
  }
  ROS_DEBUG_STREAM("Solution:");
  for(i=0; i < chain.getNrOfJoints(); ++i)
  {
    qdot_out(i) = qdot_out_reduced(mimic_joints_[i].map_index) * mimic_joints_[i].multiplier;
  }
  //return the return value of the svd decomposition
  return ret;
}
}
