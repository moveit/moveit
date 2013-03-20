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

namespace KDL
{
ChainIkSolverVel_pinv_mimic::ChainIkSolverVel_pinv_mimic(const Chain& _chain, int _num_mimic_joints, double _eps,int _maxiter):
  chain(_chain),
  jnt2jac(chain),
  jac(chain.getNrOfJoints()),
  jac_mimic(chain.getNrOfJoints()-_num_mimic_joints),
  svd(jac_mimic),
  U(6,JntArray(chain.getNrOfJoints()-_num_mimic_joints)),
  S(chain.getNrOfJoints()-_num_mimic_joints),
  V(chain.getNrOfJoints()-_num_mimic_joints,JntArray(chain.getNrOfJoints()-_num_mimic_joints)),
  tmp(chain.getNrOfJoints()-_num_mimic_joints),
  eps(_eps),
  maxiter(_maxiter),
  qdot_out_mimic(chain.getNrOfJoints()-_num_mimic_joints)
{
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

bool ChainIkSolverVel_pinv_mimic::jacToJacMimic(const Jacobian &jac, Jacobian &jac_mimic)
{
  jac_mimic.data.setZero();  
  for(std::size_t i=0; i < chain.getNrOfJoints(); ++i)
  {
    Twist vel1 = jac_mimic.getColumn(mimic_joints_[i].map_index);
    Twist vel2 = jac.getColumn(i);
    Twist result = vel1 + (mimic_joints_[i].multiplier*vel2);
    jac_mimic.setColumn(mimic_joints_[i].map_index,result);    
  }
  return true;
}

int ChainIkSolverVel_pinv_mimic::CartToJnt(const JntArray& q_in, const Twist& v_in, JntArray& qdot_out)
{
  //Let the ChainJntToJacSolver calculate the jacobian "jac" for
  //the current joint positions "q_in" 
  jnt2jac.JntToJac(q_in,jac);
  
  jacToJacMimic(jac,jac_mimic);
  //Do a singular value decomposition of "jac" with maximum
  //iterations "maxiter", put the results in "U", "S" and "V"
  //jac = U*S*Vt
  int ret = svd.calculate(jac_mimic,U,S,V,maxiter);
  
  double sum;
  unsigned int i,j;
  
  // We have to calculate qdot_out = jac_pinv*v_in
  // Using the svd decomposition this becomes(jac_pinv=V*S_pinv*Ut):
  // qdot_out = V*S_pinv*Ut*v_in
  
  //first we calculate Ut*v_in
  for (i=0;i<jac_mimic.columns();i++) {
    sum = 0.0;
    for (j=0;j<jac_mimic.rows();j++) {
      sum+= U[j](i)*v_in(j);
    }
    //If the singular value is too small (<eps), don't invert it but
    //set the inverted singular value to zero (truncated svd)
    tmp(i) = sum*(fabs(S(i))<eps?0.0:1.0/S(i));
  }
  //tmp is now: tmp=S_pinv*Ut*v_in, we still have to premultiply
  //it with V to get qdot_out
  for (i=0;i<jac_mimic.columns();i++) {
    sum = 0.0;
    for (j=0;j<jac_mimic.columns();j++) {
      sum+=V[i](j)*tmp(j);
    }
    //Put the result in qdot_out
    qdot_out_mimic(i)=sum;
  }
  for(i=0; i < chain.getNrOfJoints(); ++i)
  {
    qdot_out(i) = qdot_out_mimic(mimic_joints_[i].map_index) * mimic_joints_[i].multiplier;    
  }
  //return the return value of the svd decomposition
  return ret;
}
}
