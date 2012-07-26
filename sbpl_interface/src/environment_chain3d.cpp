/*
 * Copyright (c) 2010, Maxim Likhachev
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Pennsylvania nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/** \author Benjamin Cohen */

#include <sbpl_interface/environment_chain3d.h>
#include <collision_detection/collision_common.h>
#include <planning_models/conversions.h>
#include <boost/timer.hpp>


static const unsigned int DEBUG_OVER = 1;

namespace sbpl_interface
{

EnvironmentChain3D::EnvironmentChain3D(const planning_scene::PlanningSceneConstPtr& planning_scene) : 
  planning_scene_(planning_scene),
  bfs_(NULL),
  state_(planning_scene->getCurrentState()),
  planning_data_(StateID2IndexMapping), 
  goal_constraint_set_(planning_scene->getKinematicModel(),
                       planning_scene->getTransforms()),
  closest_to_goal_(DBL_MAX)
{
  coll_checks_ = 0;
}


EnvironmentChain3D::~EnvironmentChain3D()
{
}

/////////////////////////////////////////////////////////////////////////////
//                      SBPL Planner Interface
/////////////////////////////////////////////////////////////////////////////

bool EnvironmentChain3D::InitializeMDPCfg(MDPConfig *MDPCfg)
{
  if(planning_data_.goal_hash_entry_ &&
     planning_data_.start_hash_entry_) {
    MDPCfg->goalstateid = planning_data_.goal_hash_entry_->stateID;
    MDPCfg->startstateid = planning_data_.start_hash_entry_->stateID;
    return true;
  }
  return false;
}

bool EnvironmentChain3D::InitializeEnv(const char* sEnvFile)
{
  ROS_INFO("[env] InitializeEnv is not implemented right now.");
  return true;
}

int EnvironmentChain3D::GetFromToHeuristic(int FromStateID, int ToStateID)
{
  //std::cerr << "Getting heuristic" << std::endl;
  return getEndEffectorHeuristic(FromStateID,ToStateID);
}

int EnvironmentChain3D::GetGoalHeuristic(int stateID)
{
  if(planning_data_.state_ID_to_coord_table_.size() < DEBUG_OVER) {
    std::cerr << "Getting heur distance from " << stateID << " to " 
              << planning_data_.goal_hash_entry_->stateID << " " 
              << GetFromToHeuristic(stateID, planning_data_.goal_hash_entry_->stateID) << std::endl;
  }
  return GetFromToHeuristic(stateID, planning_data_.goal_hash_entry_->stateID);
}

int EnvironmentChain3D::GetStartHeuristic(int stateID)
{
  return GetFromToHeuristic(stateID, planning_data_.start_hash_entry_->stateID);
}

int EnvironmentChain3D::SizeofCreatedEnv()
{
  return planning_data_.state_ID_to_coord_table_.size();
}

void EnvironmentChain3D::PrintState(int stateID, bool bVerbose, FILE* fOut /*=NULL*/)
{
  // if(fOut == NULL)
  //   fOut = stdout;

  // EnvChain3DHashEntry* HashEntry = EnvChain.StateID2CoordTable[stateID];

  // bool bGoal = false;
  // if(stateID == EnvChain.goalHashEntry->stateID)
  //   bGoal = true;

  // if(stateID == EnvChain.goalHashEntry->stateID && bVerbose)
  // {
  //   //ROS_DEBUG_NAMED(fOut, "the state is a goal state\n");
  //   bGoal = true;
  // }

  // printJointArray(fOut, HashEntry, bGoal, bVerbose);
}

void EnvironmentChain3D::PrintEnv_Config(FILE* fOut)
{
  std::cerr <<("ERROR in EnvChain... function: PrintEnv_Config is undefined\n");
  throw new SBPL_Exception();
}

void EnvironmentChain3D::GetSuccs(int source_state_ID, 
                                  std::vector<int>* succ_idv, 
                                  std::vector<int>* cost_v)
{
  //std::cerr << "Calling get succ state " << source_state_ID << std::endl;

  succ_idv->clear();
  cost_v->clear();

  //From environment_robarm3d.cpp -- //goal state should be absorbing
  if(source_state_ID == planning_data_.goal_hash_entry_->stateID) {
    std::cerr << "Think we have goal" << std::endl;
    return;
  }

  if(source_state_ID > (int)planning_data_.state_ID_to_coord_table_.size()-1) {
    ROS_WARN_STREAM("source id too large");
    std::cerr << "Source id too large" << std::endl;
    return;
  }

  if(planning_data_.state_ID_to_coord_table_.size() < DEBUG_OVER) {
    std::cerr << "Expanding " << source_state_ID << std::endl;
  }

  EnvChain3DHashEntry* hash_entry = planning_data_.state_ID_to_coord_table_[source_state_ID];

  std::vector<double> source_joint_angles = hash_entry->angles;
  //convertCoordToJointAngles(hash_entry->coord, source_joint_angles);

  std::vector<int> succ_coord;
  std::vector<double> succ_joint_angles;

  // for(unsigned int i = 0; i < source_joint_angles.size(); i++) {
  //   std::cerr << "Source " << i << " " << source_joint_angles[i] << std::endl;
  // }

  for(unsigned int i = 0; i < possible_actions_.size(); i++) {
    possible_actions_[i]->generateSuccessorState(source_joint_angles, succ_joint_angles);

    double dist = 0.0;
    for(unsigned int j = 0; j < planning_data_.goal_hash_entry_->angles.size(); j++) {
      dist += fabs(planning_data_.goal_hash_entry_->angles[j]-succ_joint_angles[j]);
      //std::cerr << "Succ " << i << " " << j << " " << succ_joint_angles[j] << std::endl;
    }
    if(dist < closest_to_goal_) {
      //std::cerr << "Got dist " << dist << std::endl;
      closest_to_goal_ = dist;
    }

    convertJointAnglesToCoord(succ_joint_angles, succ_coord);
    
    joint_state_group_->setStateValues(succ_joint_angles);

    if(!joint_state_group_->satisfiesBounds()) {
      //std::cerr << "Successor doesn't satisfy bounds" << std::endl;
      continue;
    }
    ros::WallTime before_coll = ros::WallTime::now();
    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;
    req.group_name = planning_group_;
    hy_world_->checkCollisionDistanceField(req, 
                                           res, 
                                           *hy_robot_->getCollisionRobotDistanceField().get(),
                                           state_,
                                           gsr_);
    coll_checks_++;
    //std::cerr << "Elapsed " << t.elapsed() << std::endl;
    ros::WallDuration dur(ros::WallTime::now()-before_coll);
    //std::cerr << dur.toSec() << std::endl;
    //ROS_DEBUG_STREAM("Time " << ros::WallTime::now()-before_coll);
    total_coll_check_time_ += dur;
    if(res.collision) {
      //std::cerr << "Successor in collision" << std::endl;
      continue;
    }

    Eigen::Affine3d pose = tip_link_state_->getGlobalLinkTransform();
    
    int xyz[3];
    if(!getGridXYZInt(pose, xyz)) {
      std::cerr << "Can't get successor x y z" << std::endl;
      continue;
    }

    kinematic_constraints::ConstraintEvaluationResult con_res = goal_constraint_set_.decide(state_);
    EnvChain3DHashEntry* succ_hash_entry;
    if(con_res.satisfied) {
      succ_hash_entry = planning_data_.goal_hash_entry_;
    } else {
      succ_hash_entry = planning_data_.getHashEntry(succ_coord, i);
    }
    if(!succ_hash_entry) {
      succ_hash_entry = planning_data_.addHashEntry(succ_coord, succ_joint_angles, xyz, i);
    } else {
      if(planning_data_.state_ID_to_coord_table_.size() < DEBUG_OVER) {
        std::cerr << "Already have hash entry " << succ_hash_entry->stateID << std::endl;
      }
    }

    //std::cerr << "Adding hash entry" << std::endl;
    if(planning_data_.state_ID_to_coord_table_.size() < DEBUG_OVER) {
      std::cerr << std::endl;
      std::cerr << "Adding " << succ_hash_entry->stateID << std::endl;
      for(unsigned int j = 0; j < planning_data_.goal_hash_entry_->angles.size(); j++) {
        std::cerr << "Succ " << j << " " << succ_joint_angles[j] << std::endl;
      }
    }
    succ_idv->push_back(succ_hash_entry->stateID);
    cost_v->push_back(calculateCost(hash_entry, succ_hash_entry));
  }
}

void EnvironmentChain3D::GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* cost_v)
{
  std::cerr <<("ERROR in EnvChain... function: GetPreds is undefined\n");
  throw new SBPL_Exception();
}

bool EnvironmentChain3D::AreEquivalent(int StateID1, int StateID2)
{
  std::cerr <<("ERROR in EnvChain... function: AreEquivalent is undefined\n");
  throw new SBPL_Exception();
}

void EnvironmentChain3D::SetAllActionsandAllOutcomes(CMDPSTATE* state)
{
  std::cerr <<("ERROR in EnvChain..function: SetAllActionsandOutcomes is undefined\n");
  throw new SBPL_Exception();
}

void EnvironmentChain3D::SetAllPreds(CMDPSTATE* state)
{
  std::cerr <<("ERROR in EnvChain... function: SetAllPreds is undefined\n");
  throw new SBPL_Exception();
}

/////////////////////////////////////////////////////////////////////////////
//                      End of SBPL Planner Interface
/////////////////////////////////////////////////////////////////////////////

bool EnvironmentChain3D::setupForMotionPlan(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                            const moveit_msgs::GetMotionPlan::Request &mreq)
{
  planning_scene_ = planning_scene;
  
  planning_group_ = mreq.motion_plan_request.group_name;
  
  setMotionPrimitives(planning_group_);
  
  hy_world_ = dynamic_cast<const collision_detection::CollisionWorldHybrid*>(planning_scene->getCollisionWorld().get());
  if(!hy_world_) {
    ROS_WARN_STREAM("Could not initialize hybrid collision world from planning scene");
    return false;
  }
  
  hy_robot_ = dynamic_cast<const collision_detection::CollisionRobotHybrid*>(planning_scene->getCollisionRobot().get());
  if(!hy_robot_) {
    ROS_WARN_STREAM("Could not initialize hybrid collision robot from planning scene");
    return false;
  }

  state_ = planning_scene->getCurrentState();
  planning_models::robotStateToKinematicState(*planning_scene->getTransforms(), mreq.motion_plan_request.start_state, state_);
  joint_state_group_ = state_.getJointStateGroup(planning_group_);
  tip_link_state_ = state_.getLinkState(joint_state_group_->getJointModelGroup()->getLinkModelNames().back());

  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  req.group_name = planning_group_;
  hy_world_->checkCollisionDistanceField(req, 
                                         res, 
                                         *hy_robot_->getCollisionRobotDistanceField().get(),
                                         state_,
                                         planning_scene_->getAllowedCollisionMatrix(),
                                         gsr_);

  angle_discretization_ = gsr_->dfce_->distance_field_->getResolution();
  // bfs_ = new BFS_3D(gsr_->dfce_->distance_field_->getXNumCells(),
  //                   gsr_->dfce_->distance_field_->getYNumCells(),
  //                   gsr_->dfce_->distance_field_->getZNumCells());

  // boost::shared_ptr<const distance_field::DistanceField> world_distance_field = hy_world_->getCollisionWorldDistanceField()->getDistanceField();
  // if(world_distance_field->getXNumCells() != gsr_->dfce_->distance_field_->getXNumCells() ||
  //    world_distance_field->getYNumCells() != gsr_->dfce_->distance_field_->getYNumCells() ||
  //    world_distance_field->getZNumCells() != gsr_->dfce_->distance_field_->getZNumCells()) {
  //   ROS_WARN_STREAM("Size mismatch between world and self distance fields");
  //   std::cerr << "Size mismatch between world and self distance fields" << std::endl;
  //   return false;
  // }
  // std::cerr << "BFS dimensions are "
  //           << world_distance_field->getXNumCells() << " " 
  //           << world_distance_field->getYNumCells() << " "
  //           << world_distance_field->getZNumCells() << std::endl;
  // unsigned int wall_count = 0;
  // for(int i = 0; i < gsr_->dfce_->distance_field_->getXNumCells()-2; i++) {
  //   for(int j = 0; j < gsr_->dfce_->distance_field_->getYNumCells()-2; j++) {
  //     for(int k = 0; k < gsr_->dfce_->distance_field_->getZNumCells()-2; k++) {
  //       if(gsr_->dfce_->distance_field_->getDistanceFromCell(i+1, j+1, k+1) == 0.0 ||
  //          world_distance_field->getDistanceFromCell(i+1, j+1, k+1) == 0.0) {
  //         bfs_->setWall(i+1, j+1, k+1);
  //         wall_count++;
  //       } 
  //     }
  //   }
  // }
  // std::cerr << "Wall cells are " << wall_count << " of " << 
  //   world_distance_field->getXNumCells()*world_distance_field->getYNumCells()*world_distance_field->getZNumCells() << std::endl;

  //setting start position
  std::vector<double> start_joint_values;
  joint_state_group_->getGroupStateValues(start_joint_values);
  std::vector<int> start_coords;
  convertJointAnglesToCoord(start_joint_values, start_coords);
  Eigen::Affine3d start_pose = tip_link_state_->getGlobalLinkTransform();

  int start_xyz[3];
  if(!getGridXYZInt(start_pose, start_xyz)) {
    std::cerr << "Bad start pose" << std::endl;
    return false;
  }
  planning_data_.start_hash_entry_ = planning_data_.addHashEntry(start_coords,
                                                                 start_joint_values,
                                                                 start_xyz,
                                                                 0);

  //setting goal position
  planning_models::KinematicState goal_state(state_);
  std::map<std::string, double> goal_vals;
  for(unsigned int i = 0; i < mreq.motion_plan_request.goal_constraints[0].joint_constraints.size(); i++) {
    goal_vals[mreq.motion_plan_request.goal_constraints[0].joint_constraints[i].joint_name] = mreq.motion_plan_request.goal_constraints[0].joint_constraints[i].position;
  }
  goal_state.setStateValues(goal_vals);
  std::vector<double> goal_joint_values;
  planning_models::KinematicState::JointStateGroup* goal_joint_state_group = goal_state.getJointStateGroup(planning_group_);
  goal_joint_state_group->getGroupStateValues(goal_joint_values);
  std::vector<int> goal_coords;
  convertJointAnglesToCoord(goal_joint_values, goal_coords);
  goal_pose_ = goal_state.getLinkState(tip_link_state_->getName())->getGlobalLinkTransform();
  int goal_xyz[3];
  if(!getGridXYZInt(goal_pose_, goal_xyz)) {
    std::cerr << "Bad goal pose" << std::endl;
    return false;
  }
  std::vector<std::string> goal_dofs = goal_joint_state_group->getJointModelGroup()->getActiveDOFNames();
  for(unsigned int i = 0; i < goal_dofs.size(); i++) {
    ROS_INFO_STREAM("Goal " << goal_dofs[i] << " pos " << goal_joint_values[i]);
  }
  std::cerr << "Running bfs with goal " << goal_xyz[0] << " " <<  goal_xyz[1] << " " << goal_xyz[2] << std::endl;
  //bfs_->run(goal_xyz[0], goal_xyz[1], goal_xyz[2]);
  goal_constraint_set_.clear();
  goal_constraint_set_.add(mreq.motion_plan_request.goal_constraints[0]);
  planning_data_.goal_hash_entry_ = planning_data_.addHashEntry(goal_coords,
                                                                goal_joint_values,
                                                                goal_xyz,
                                                                0);
  return true;
}

void EnvironmentChain3D::setMotionPrimitives(const std::string& group_name) {
  possible_actions_.clear();
  unsigned int var_count = planning_scene_->getKinematicModel()->getJointModelGroup(group_name)->getVariableCount();
  for(unsigned int i = 0; i < var_count; i++) {
    boost::shared_ptr<SingleJointMotionPrimitive> sing_pos(new SingleJointMotionPrimitive(i, .1));
    boost::shared_ptr<SingleJointMotionPrimitive> sing_neg(new SingleJointMotionPrimitive(i, -.1));
    possible_actions_.push_back(sing_pos);
    possible_actions_.push_back(sing_neg);
  }
}

int EnvironmentChain3D::calculateCost(EnvChain3DHashEntry* HashEntry1, EnvChain3DHashEntry* HashEntry2)
{
  //if(prms_.use_uniform_cost_)
  return 1000.0;
  //return .05;//prms_.cost_multiplier_;
  //  else
  //   {
  //     // Max's suggestion is to just put a high cost on being close to
  //     // obstacles but don't provide some sort of gradient 
  //     if(int(HashEntry2->dist) < 7) // in cells
  //       return prms_.cost_multiplier_ * prms_.range1_cost_;
  //     else if(int(HashEntry2->dist) < 12)
  //       return prms_.cost_multiplier_ * prms_.range2_cost_;
  //     else if(int(HashEntry2->dist) < 17)
  //       return prms_.cost_multiplier_ * prms_.range3_cost_;
  //     else
  //       return prms_.cost_multiplier_;
  //   }
  // }
}

// double EnvironmentChain3D::getEpsilon()
// {
//   return 10.0;
//   //printf("%0.3f\n",prms_.epsilon_);fflush(stdout);
//   //return prms_.epsilon_;
// }

// int EnvironmentChain3D::getEdgeCost(int FromStateID, int ToStateID)
// {
// #if DEBUG
//   if(FromStateID >= (int)EnvChain.StateID2CoordTable.size() 
//       || ToStateID >= (int)EnvChain.StateID2CoordTable.size())
//   {
//     std::cerr <<("ERROR in EnvChain... function: stateID illegal\n");
//     throw new SBPL_Exception();
//   }
// #endif

//   //get X, Y for the state
//   EnvChain3DHashEntry* FromHashEntry = EnvChain.StateID2CoordTable[FromStateID];
//   EnvChain3DHashEntry* ToHashEntry = EnvChain.StateID2CoordTable[ToStateID];

//   return cost(FromHashEntry, ToHashEntry, false);
// }

int EnvironmentChain3D::getBFSCostToGoal(int x, int y, int z) const
{
  //std::cerr << "Getting cost for " << x << " " << y << " " << z << std::endl;
  //TODO - deal with cost_per_cell
  return  bfs_->getDistance(x,y,z);
  //* .05;//prms_.cost_per_cell_;
}

int EnvironmentChain3D::getEndEffectorHeuristic(int from_stateID, int to_stateID)
{
  EnvChain3DHashEntry* from_hash_entry = planning_data_.state_ID_to_coord_table_[from_stateID];
  EnvChain3DHashEntry* to_hash_entry = planning_data_.state_ID_to_coord_table_[to_stateID];
  double dist = 0.0;
  for(unsigned int i = 0; i < from_hash_entry->angles.size(); i++) {
    dist += fabs(from_hash_entry->angles[i]-to_hash_entry->angles[i]);
  }
  return dist*1000;
  //return getBFSCostToGoal(from_hash_entry->xyz[0], from_hash_entry->xyz[1], from_hash_entry->xyz[2]);
  //else
  //{
  // double x, y, z;
  // if(!gsr_->dfce_->distance_field_->gridToWorld(from_hash_entry->xyz[0],
  //                                               from_hash_entry->xyz[1],
  //                                               from_hash_entry->xyz[2],
  //                                               x,y,z)) {
  //   std::cerr << "problem" << std::endl;
  //   return 1000000;
  // }
  // return getEuclideanDistance(x, y, z, goal_pose_.translation().x(), goal_pose_.translation().y(), goal_pose_.translation().z())*1000.0;
  // heur =  getEuclideanDistance(x, y, z, env_chain_config_.goal.xyz[0],env_chain_config_.goal.xyz[1], env_chain_config_.goal.xyz[2]) * prms_.cost_per_meter_;
  //}
}

bool EnvironmentChain3D::getGridXYZInt(const Eigen::Affine3d& pose,
                                       int(&xyz)[3]) const
{
  if(!gsr_ || !gsr_->dfce_->distance_field_) {
    ROS_WARN_STREAM("No distance field cache entry available");
    return false;
  }
  if(!gsr_->dfce_->distance_field_->worldToGrid(pose.translation().x(), 
                                                pose.translation().y(),
                                                pose.translation().z(),
                                                xyz[0],
                                                xyz[1],
                                                xyz[2])) {
    ROS_WARN_STREAM("Pose out of bounds");
    return false;
  }
  return true;
}

bool EnvironmentChain3D::populateTrajectoryFromStateIDSequence(const std::vector<int>& state_ids,
                                                               trajectory_msgs::JointTrajectory& traj) const
{
  traj.joint_names = joint_state_group_->getJointModelGroup()->getActiveDOFNames();
  std::vector<std::vector<double> > angle_vector;
  if(!planning_data_.convertFromStateIDsToAngles(state_ids,
                                                 angle_vector)) {
    return false;
  }
  traj.points.resize(angle_vector.size());
  for(unsigned int i = 0; i < angle_vector.size(); i++) {
    traj.points[i].positions = angle_vector[i];
  }
  for(unsigned int i = 0; i < traj.points.back().positions.size(); i++) {
    ROS_INFO_STREAM("Last " << i << " " << traj.points.back().positions[i]);
  }
  return true;
}

}

