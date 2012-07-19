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

#include <sbpl_arm_planner/environment_robarm3d.h>

#define DEG2RAD(d) ((d)*(M_PI/180.0))
#define RAD2DEG(r) ((r)*(180.0/M_PI))

#define DEBUG_SEARCH 0

//Statistics
bool near_goal = false;
clock_t starttime;
double time_to_goal_region;

using namespace std;

namespace sbpl_interface
{

EnvironmentChain3D::EnvironmentChain3D() : bfs_(NULL)
{
  EnvChain.Coord2StateIDHashTable = NULL;
  env_chain_config_.bInitialized = false;
  env_chain_config_.solved_by_ik = 0;
  env_chain_config_.solved_by_os = 0;
  env_chain_config_.num_calls_to_ik = 0;
  env_chain_config_.num_ik_invalid_path = 0; 
  env_chain_config_.num_invalid_ik_solutions = 0;
  env_chain_config_.num_no_ik_solutions = 0;
  env_chain_config_.num_ik_invalid_joint_limits = 0;
  env_chain_config_.ik_solution.resize(7,0);
  save_expanded_states = true;
  using_short_mprims_ = false;

  prms_.environment_type_ = "jointspace";

  getHeuristic_ = &EnvironmentChain3D::getEndEffectorHeuristic;
}

EnvironmentChain3D::~EnvironmentChain3D()
{
  if(bfs_ != NULL)
    delete bfs_;
}

void EnvironmentChain3D::initializeForGroup(const std::string& group) 
{
  const planning_models::KinematicModel::JointModelGroup* jmg = pscene_->getKinematicModel()->getJointModelGroup(group); 
  
}

/////////////////////////////////////////////////////////////////////////////
//                      SBPL Planner Interface
/////////////////////////////////////////////////////////////////////////////

bool EnvironmentChain3D::InitializeMDPCfg(MDPConfig *MDPCfg)
{
  //initialize MDPCfg with the start and goal ids	
  MDPCfg->goalstateid = EnvChain.goalHashEntry->stateID;
  MDPCfg->startstateid = EnvChain.startHashEntry->stateID;
  return true;
}

bool EnvironmentChain3D::InitializeEnv(const char* sEnvFile)
{
  ROS_INFO("[env] InitializeEnv is not implemented right now.");
  return true;
}

int EnvironmentChain3D::GetFromToHeuristic(int FromStateID, int ToStateID)
{
  return (*this.*getHeuristic_)(FromStateID,ToStateID);
}

int EnvironmentChain3D::GetGoalHeuristic(int stateID)
{
#if DEBUG_HEUR
  if(stateID >= (int)EnvChain.StateID2CoordTable.size())
  {
    SBPL_ERROR("ERROR in EnvChain... function: stateID illegal");
    return -1;
  }
#endif

  return GetFromToHeuristic(stateID, EnvChain.goalHashEntry->stateID);
}

int EnvironmentChain3D::GetStartHeuristic(int stateID)
{
#if DEBUG_HEUR
  if(stateID >= (int)EnvChain.StateID2CoordTable.size())
  {
    SBPL_ERROR("ERROR in EnvChain... function: stateID illegal\n");
    return -1;
  }
#endif

  return GetFromToHeuristic(stateID, EnvChain.startHashEntry->stateID);
}

int EnvironmentChain3D::SizeofCreatedEnv()
{
  return EnvChain.StateID2CoordTable.size();
}

void EnvironmentChain3D::PrintState(int stateID, bool bVerbose, FILE* fOut /*=NULL*/)
{
#if DEBUG_HEUR
  if(stateID >= (int)EnvChain.StateID2CoordTable.size())
  {
    SBPL_ERROR("ERROR in EnvChain... function: stateID illegal (2)\n");
    throw new SBPL_Exception();
  }
#endif

  if(fOut == NULL)
    fOut = stdout;

  EnvChain3DHashEntry* HashEntry = EnvChain.StateID2CoordTable[stateID];

  bool bGoal = false;
  if(stateID == EnvChain.goalHashEntry->stateID)
    bGoal = true;

  if(stateID == EnvChain.goalHashEntry->stateID && bVerbose)
  {
    //ROS_DEBUG_NAMED(fOut, "the state is a goal state\n");
    bGoal = true;
  }

  printJointArray(fOut, HashEntry, bGoal, bVerbose);
}

void EnvironmentChain3D::PrintEnv_Config(FILE* fOut)
{
  SBPL_ERROR("ERROR in EnvChain... function: PrintEnv_Config is undefined\n");
  throw new SBPL_Exception();
}

void EnvironmentChain3D::GetSuccs(int source_state_ID, 
                                  std::vector<int>* succ_idv, 
                                  std::vector<int>* cost_v)
{
  unsigned char dist=0;
  int path_length=0, nchecks=0;

  //to support two sets of succesor actions
  int actions_i_min = 0;
  int actions_i_max = prms_.num_mprims_;

  //clear the successor array
  succ_idv->clear();
  cost_v->clear();

  //goal state should be absorbing
  if(source_state_ID == env_chain_planning_instance_.planning_data.goal_hash_entry->stateID)
    return;

  //get X, Y, Z for the state
  EnvChain3DHashEntry* hash_entry = EnvChain.StateID2CoordTable[SourceStateID];

  std::vector<int> succ_coord = hash_entry->coord;

  //used for interpolated collision check
  std::vector source_angles;
  coordToAngles(succ_coord, source_angles);

  //check if cell is close to enough to goal to use higher resolution actions
  //int dist_to_goal = getBFSCostToGoal(hash_entry->xyz[0],hash_entry->xyz[1],hash_entry->xyz[2]);
  
  //iterate through successors of state s (possible actions)
  for (unsigned int i = actions_i_min; i < actions_i_max; i++)
  {
    stats_.updateExpand();
    stats_.total_num_solver_used_[solver_types::NONE]++;

    //add the motion primitive to the current joint configuration
    for(a = 0; a < arm_->num_joints_; ++a)
    {
      if((hash_entry->coord[a] + int(prms_.mprims_[i][a])) < 0)
        succ_coord[a] = ((env_chain_config_.coord_vals[a] + hash_entry->coord[a] + int(prms_.mprims_[i][a])) % env_chain_config_.coord_vals[a]);
      else
        succ_coord[a] = ((int)(hash_entry->coord[a] + int(prms_.mprims_[i][a])) % env_chain_config_.coord_vals[a]);
    }

    //get the successor
    bool succ_is_goal = false;
    final_mp_cost = 0;

    //get continous angles
    std::vector<double> angles;
    coordToAngles(succ_coord, angles);

    env_chain_planning_instance_->joint_state_group_->setStateValues(angles);

    if(!env_chain_planning_instance_->joint_state_group_->satisfiesBounds())  {
      continue;
    }

    //check for collisions
    stats_.nchecks_per_solver_per_succ_[solver_types::NONE]++;
    collision_detection::CollisionRequest req;
    collision_detection::CollisionResponse res;
    hy_world_->checkCollisionDistanceField(req, 
                                           res, 
                                           *hy_robot_->getCollisionRobotDistanceField().get(),
                                           env_chain_planning_instance_->state_,
                                           env_chain_planning_instance_->gsr_);
    if(res.collision) {
      continue;
    }
    
    // check for collision along interpolated path between sourcestate and succ
    nchecks = 0;
//     if(!cspace_->checkPathForCollision(source_angles, angles, prms_.verbose_, path_length, nchecks, dist))
//     {
// //#if DEBUG_SEARCH
// //      if(prms_.verbose_)
//         ROS_DEBUG_NAMED(prms_.expands_log_, " succ: %2d  dist: %2d is in collision along interpolated path. (path_length: %d)", i, int(dist), path_length);
// //#endif
//       stats_.nchecks_per_solver_per_succ_[solver_types::NONE] += nchecks;
//       continue;
//     }
//     stats_.nchecks_per_solver_per_succ_[solver_types::NONE] += nchecks;

    //get end effector position 

    Eigen::Affine3d pose = tip_link_state_->getGlobalLinkTransform();

    int xyz[3];
    getGridXYZInt(pose, xyz);

    bool succ_is_goal = false;
    int final_mp_cost;
    //check if this state meets the goal criteria
    if(isGoalPosition(pose,
                      env_chain_planning_instance_->planning_goal_pos_, 
                      angles, 
                      final_mp_cost))
    {
      succ_is_goal = true;
      env_chain_planning_instance_->planning_data_->goal_hash_entry_->coord = succ_coord;
      env_chain_planning_instance_->planning_data_->goal_hash_entry_->xyz = xyz;
      env_chain_planning_instance_->planning_data_->goal_hash_entry_->action = i;
      //EnvChain.goalHashEntry->dist = dist;

      SBPL_DEBUG("[GetSuccs] Goal successor is generated. SourceStateID: %d (distance to nearest obstacle: %0.2fm)",SourceStateID,  (double)dist*grid_->getResolution());
    }

    //check if hash entry already exists, if not then create one

    EnvChain3DHashEntry* out_hash_entry = getHashEntry(succ_coord, i, succ_is_goal);
    if(out_hash_entry == NULL) {
      out_hash_entry = createHashEntry(succ_coord, xyz, i);
      //OutHashEntry->dist = dist;

      /*
      if(getBFSCostToGoal(endeff[0],endeff[1],endeff[2]) > 100000)
        ROS_INFO("xyz: %2d %2d %2d  dist_to_goal: %2d %2d %2d  heur: %2d", endeff[0], endeff[1], endeff[2], EnvChain.goalHashEntry->xyz[0]-endeff[0], EnvChain.goalHashEntry->xyz[1]-endeff[1], EnvChain.goalHashEntry->xyz[2]-endeff[2], getBFSCostToGoal(endeff[0], endeff[1], endeff[2]));
      */
#if DEBUG_SEARCH
  if(prms_.verbose_)
      ROS_DEBUG_NAMED(prms_.expands_log_, "%5i: action: %2d dist: %2d edge_distance_cost: %5d heur: %2d endeff: %3d %3d %3d", OutHashEntry->stateID, i, int(OutHashEntry->dist), cost(HashEntry,OutHashEntry,bSuccisGoal), GetFromToHeuristic(OutHashEntry->stateID, EnvChain.goalHashEntry->stateID), OutHashEntry->xyz[0],OutHashEntry->xyz[1],OutHashEntry->xyz[2]);
#endif
    }

    //put successor on successor list with the proper cost
    succ_idv->push_back(out_hash_entry->stateID);
    cost_v->push_back(cost(hash_entry,out_hash_entry,succ_is_goal) + final_mp_cost);
  }

  stats_.updateExpand();
  //stats_.printSingleExpandSummary();
  //stats_.printChecksPerSolverPerSingleExpand();
  stats_.logChecksForExpand();
  if(save_expanded_states)
    expanded_states.push_back(SourceStateID);
}

void EnvironmentChain3D::GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* cost_v)
{
  SBPL_ERROR("ERROR in EnvChain... function: GetPreds is undefined\n");
  throw new SBPL_Exception();
}

bool EnvironmentChain3D::AreEquivalent(int StateID1, int StateID2)
{
  SBPL_ERROR("ERROR in EnvChain... function: AreEquivalent is undefined\n");
  throw new SBPL_Exception();
}

void EnvironmentChain3D::SetAllActionsandAllOutcomes(CMDPSTATE* state)
{
  SBPL_ERROR("ERROR in EnvChain..function: SetAllActionsandOutcomes is undefined\n");
  throw new SBPL_Exception();
}

void EnvironmentChain3D::SetAllPreds(CMDPSTATE* state)
{
  SBPL_ERROR("ERROR in EnvChain... function: SetAllPreds is undefined\n");
  throw new SBPL_Exception();
}

/////////////////////////////////////////////////////////////////////////////
//                      End of SBPL Planner Interface
/////////////////////////////////////////////////////////////////////////////

// void EnvironmentChain3D::printHashTableHist()
// {
//   int s0=0, s1=0, s50=0, s100=0, s200=0, s300=0, slarge=0;

//   for(int  j = 0; j < EnvChain.HashTableSize; j++)
//   {
//     if((int)EnvChain.Coord2StateIDHashTable[j].size() == 0)
//       s0++;
//     else if((int)EnvChain.Coord2StateIDHashTable[j].size() < 50)
//       s1++;
//     else if((int)EnvChain.Coord2StateIDHashTable[j].size() < 100)
//       s50++;
//     else if((int)EnvChain.Coord2StateIDHashTable[j].size() < 200)
//       s100++;
//     else if((int)EnvChain.Coord2StateIDHashTable[j].size() < 300)
//       s200++;
//     else if((int)EnvChain.Coord2StateIDHashTable[j].size() < 400)
//       s300++;
//     else
//       slarge++;
//   }
//   SBPL_DEBUG("hash table histogram: 0:%d, <50:%d, <100:%d, <200:%d, <300:%d, <400:%d >400:%d",
//       s0,s1, s50, s100, s200,s300,slarge);
// }

EnvChain3DHashEntry* EnvironmentChain3D::getHashEntry(const std::vector<int> &coord, int action, bool bIsGoal)
{
  //if it is goal
  if(bIsGoal)
    return EnvChain.goalHashEntry;

  int binid = getHashBin(coord);

#if DEBUG
  if ((int)EnvChain.Coord2StateIDHashTable[binid].size() > 500)
  {
    SBPL_WARN("WARNING: Hash table has a bin %d (coord0=%d) of size %d", 
        binid, coord[0], int(EnvChain.Coord2StateIDHashTable[binid].size()));
    printHashTableHist();
  }
#endif

  //iterate over the states in the bin and select the perfect match
  for(int ind = 0; ind < (int)EnvChain.Coord2StateIDHashTable[binid].size(); ind++)
  {
    int j = 0;

    for(j = 0; j < int(coord.size()); j++)
    {
      if(EnvChain.Coord2StateIDHashTable[binid][ind]->coord[j] != coord[j]) 
        break;
    }

    if (j == int(coord.size()))
      return EnvChain.Coord2StateIDHashTable[binid][ind];
  }

  return NULL;
}

EnvChain3DHashEntry* EnvironmentChain3D::createHashEntry(const std::vector<int> &coord, int endeff[3], int action)
{
  int i;
  EnvChain3DHashEntry* HashEntry = new EnvChain3DHashEntry_t;

  HashEntry->coord = coord;

  memcpy(HashEntry->xyz, endeff, 3*sizeof(int));

  HashEntry->action = action;

  // assign a stateID to HashEntry to be used 
  HashEntry->stateID = EnvChain.StateID2CoordTable.size();

  //insert into the tables
  EnvChain.StateID2CoordTable.push_back(HashEntry);

  //get the hash table bin
  i = getHashBin(HashEntry->coord);

  //insert the entry into the bin
  EnvChain.Coord2StateIDHashTable[i].push_back(HashEntry);

  //insert into and initialize the mappings
  int* entry = new int [NUMOFINDICES_STATEID2IND];
  StateID2IndexMapping.push_back(entry);
  for(i = 0; i < NUMOFINDICES_STATEID2IND; i++)
  {
    StateID2IndexMapping[HashEntry->stateID][i] = -1;
  }

  if(HashEntry->stateID != (int)StateID2IndexMapping.size()-1)
  {
    SBPL_ERROR("ERROR in Env... function: last state has incorrect stateID");
    throw new SBPL_Exception();
  }

  return HashEntry;
}

void EnvironmentChain3D::initHeuristics()
{
  int dimX,dimY,dimZ;
  grid_->getGridSize(dimX, dimY, dimZ);
  bfs_ = new BFS_3D(dimX, dimY, dimZ);
  ROS_DEBUG("[env] BFS is initialized.");
}

void EnvironmentChain3D::discretizeAngles()
{
  for(int i = 0; i < arm_->num_joints_; i++)
  {
    env_chain_config_.coord_delta[i] = (2.0*M_PI) / prms_.angle_delta_;
    env_chain_config_.coord_vals[i] = prms_.angle_delta_;
  }
}

int EnvironmentChain3D::cost(EnvChain3DHashEntry* HashEntry1, EnvChain3DHashEntry* HashEntry2, bool bState2IsGoal)
{
  //if(prms_.use_uniform_cost_)
  return prms_.cost_multiplier_;
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

bool EnvironmentChain3D::initEnvConfig()
{
  int endeff[3] = {0};
  std::vector<int> coord(arm_->num_joints_,0);

  //these probably don't have to be done
  env_chain_config_.ik_solution.resize(arm_->num_joints_,0);

  env_chain_config_.coord_delta.resize(arm_->num_joints_);
  env_chain_config_.coord_vals.resize(arm_->num_joints_);

  discretizeAngles();

  //initialize the map from Coord to StateID
  EnvChain.HashTableSize = 32*1024; //should be power of two
  EnvChain.Coord2StateIDHashTable = new std::vector<EnvChain3DHashEntry*>[EnvChain.HashTableSize];

  //initialize the map from StateID to Coord
  EnvChain.StateID2CoordTable.clear();

  //create an empty start state
  EnvChain.startHashEntry = createHashEntry(coord, endeff, 0);

  //create the goal state
  EnvChain.goalHashEntry = createHashEntry(coord, endeff, 0);

  return true;
}

bool EnvironmentChain3D::initEnvironment(std::string arm_description_filename, std::string mprims_filename, std::string urdf, std::string srdf)
{
  FILE* mprims_fp=NULL;
  FILE* arm_fp=NULL;

  //initialize the arm planner parameters
  prms_.initFromParamServer();

  //parse motion primitives file
  if((mprims_fp=fopen(mprims_filename.c_str(),"r")) == NULL)
  {
    SBPL_ERROR("Failed to open motion primitive file.");
    return false;
  }
  if(!prms_.initMotionPrims(mprims_fp))
  {
    SBPL_ERROR("Failed to parse motion primitive file.");
    fclose(mprims_fp);
    return false;
  }
  fclose(mprims_fp);

  //initialize the arm model
  if((arm_fp=fopen(arm_description_filename.c_str(),"r")) == NULL)
  {
    SBPL_ERROR("Failed to open arm description file.");
    return false;
  }
  if(!initArmModel(arm_fp,urdf))
  {
    SBPL_ERROR("Failed to initialize arm model.");
    fclose(arm_fp);
    return false;
  }
  fclose(arm_fp);

  //initialize the environment & planning variables  
  if(!initGeneral())
  {
    SBPL_ERROR("Failed to initialize environment.");
    return false;
  }

  //set 'Environment is Initialized' flag
  env_chain_config_.bInitialized = true;

  ROS_INFO("[env] Environment has been initialized.");
  
  //for statistics purposes
  starttime = clock();
  return true;
}

bool EnvironmentChain3D::initGeneral()
{
  //create the occupancy grid
  grid_ = new OccupancyGrid(prms_.sizeX_,prms_.sizeY_,prms_.sizeZ_, prms_.resolution_,prms_.originX_,prms_.originY_,prms_.originZ_);

  //create the collision space
  cspace_ = new SBPLCollisionSpace(grid_);

  cspace_->setPlanningJoints(prms_.planning_joints_);

  cspace_->setPadding(0.01);

  if(!cspace_->init(prms_.group_name_))
    return false;

  //create the rpysolver
  rpysolver_ = new RPYSolver(arm_, cspace_);

#if DEBUG_SEARCH
  arm_->setDebugFile(std::string("sbpl"));
  cspace_->setDebugFile(stdout);
#endif

  //initialize Environment
  if(!initEnvConfig())
    return false;

  //compute the cost per cell to be used by heuristic
  computeCostPerCell();

  //initialize BFS
  initHeuristics();
  
  if(prms_.verbose_)
  {
    arm_->printArmDescription(std::string("sbpl_arm"));
    prms_.printParams(std::string("sbpl_arm"));
    prms_.printMotionPrims(std::string("sbpl_arm"));
  }

  //set heuristic function pointer
  getHeuristic_ = &sbpl_arm_planner::EnvironmentChain3D::getEndEffectorHeuristic;

  prefinal_joint_config.resize(7);
  final_joint_config.resize(7);
 
  return true;
}

double EnvironmentChain3D::getEpsilon()
{
  printf("%0.3f\n",prms_.epsilon_);fflush(stdout);
  return prms_.epsilon_;
}

bool EnvironmentChain3D::isGoalPosition(const std::vector<double> &pose, const GoalPos &goal, std::vector<double> jnt_angles, int &cost)
{
    if(goal.is_6dof_goal)
    {
      //check if position constraint is satisfied  
      if(fabs(pose[0]-goal.xyz[0]) <= goal.xyz_tolerance[0] && 
          fabs(pose[1]-goal.xyz[1]) <= goal.xyz_tolerance[1] && 
          fabs(pose[2]-goal.xyz[2]) <= goal.xyz_tolerance[2])
      {
        //log the amount of time required for the search to get close to the goal
        if(!near_goal)
        {
          time_to_goal_region = (clock() - starttime) / (double)CLOCKS_PER_SEC;
          near_goal = true;
          printf("Search is at %0.2f %0.2f %0.2f, within %0.3fm of the goal (%0.2f %0.2f %0.2f) after %.4f sec. (after %d expansions)\n", pose[0],pose[1],pose[2],goal.xyz_tolerance[0], goal.xyz[0], goal.xyz[1], goal.xyz[2], time_to_goal_region,(int)expanded_states.size());
          env_chain_config_.num_expands_to_position_constraint = expanded_states.size();
        }
        
        if(prms_.use_orientation_solver_)
        {
          //try to reach orientation constraint with orientation solver 
          if(isGoalStateWithOrientationSolver(goal,jnt_angles))
          {
            env_chain_config_.solved_by_os++;

            //compute cost of the motion
            cost = getActionCost(jnt_angles,final_joint_config,0);
            return true;
          }
        } 
      }

      if(prms_.use_ik_)
      {
        //try to reach orientation constraint with IK
        if(isGoalStateWithIK(pose,goal,jnt_angles))
        {
          env_chain_config_.solved_by_ik++;

          //compute cost of the motion
          cost = getActionCost(jnt_angles,final_joint_config,0);
          return true;
        }
      }
    }
    else
    {
      //check if position constraint is satisfied  
      if(fabs(pose[0]-goal.xyz[0]) <= goal.xyz_tolerance[0] && 
         fabs(pose[1]-goal.xyz[1]) <= goal.xyz_tolerance[1] && 
         fabs(pose[2]-goal.xyz[2]) <= goal.xyz_tolerance[2])
        return true;
    }

  return false;
}

bool EnvironmentChain3D::isGoalStateWithIK(const std::vector<double> &pose, const GoalPos &goal, std::vector<double> jnt_angles)
{
  //check distance to goal, if within range then run IK
  if(prms_.use_bfs_heuristic_)
  {
    int endeff[3];
    grid_->worldToGrid(pose[0],pose[1],pose[2],endeff[0],endeff[1],endeff[2]);
    int endeff_short[3]={endeff[0],endeff[1],endeff[2]};

    if(getBFSCostToGoal(endeff_short[0],endeff_short[1],endeff_short[2]) > prms_.solve_for_ik_thresh_)
      return false;
  }

  env_chain_config_.ik_solution=jnt_angles;

  std::vector<double> goal_pose(7,0);  //changed from 6
  unsigned char dist = 0;

  goal_pose[0] = goal.xyz[0];
  goal_pose[1] = goal.xyz[1];
  goal_pose[2] = goal.xyz[2];
  goal_pose[3] = goal.q[0];
  goal_pose[4] = goal.q[1];
  goal_pose[5] = goal.q[2];
  goal_pose[6] = goal.q[3];

  env_chain_config_.num_calls_to_ik++;

  //generate an IK solution
  if(!arm_->computeIK(goal_pose, jnt_angles, env_chain_config_.ik_solution))
  {
    env_chain_config_.num_no_ik_solutions++;
    return false;
  }
  stats_.total_num_solver_used_[solver_types::IK_SEARCH]++;

  //check joint limits
  if(!arm_->checkJointLimits(env_chain_config_.ik_solution, false))
  {
    env_chain_config_.num_ik_invalid_joint_limits++;
    return false;
  }

  //check for collisions
  stats_.nchecks_per_solver_per_succ_[solver_types::IK_SEARCH]++;
  if(!cspace_->checkCollision(env_chain_config_.ik_solution, prms_.verbose_, false, dist))
  {
    env_chain_config_.num_invalid_ik_solutions++;
    return false;
  }

  std::vector<std::vector<double> > path(2, std::vector<double>(arm_->num_joints_,0));
  for(unsigned int i = 0; i < path[0].size(); ++i)
  {
    path[0][i] = jnt_angles[i];
    path[1][i] = env_chain_config_.ik_solution[i];
  }

  //check for collisions along the path
  int path_length=0, nchecks=0;
  if(!cspace_->checkPathForCollision(jnt_angles, env_chain_config_.ik_solution, prms_.verbose_, path_length, nchecks, dist))
  {
    stats_.nchecks_per_solver_per_succ_[solver_types::IK_SEARCH] += nchecks;
    env_chain_config_.num_ik_invalid_path++;
    return false;
  }
  stats_.nchecks_per_solver_per_succ_[solver_types::IK_SEARCH] += nchecks;

  SBPL_DEBUG("[isGoalStateWithIK] The path to the IK solution for the goal is valid.");

  //added to be compatible with OP - 7/5/2010
  prefinal_joint_config = jnt_angles;
  final_joint_config = env_chain_config_.ik_solution;

  return true;
}

int EnvironmentChain3D::getActionCost(const std::vector<double> &from_config, const std::vector<double> &to_config, int dist)
{
  int num_prims = 0, cost = 0;
  double diff = 0, max_diff = 0;

  if(from_config.size() != to_config.size())
    return -1;

/* NOTE: Not including forearm roll OR wrist roll movement to calculate mprim cost */

  for(size_t i = 0; i < 6; i++)
  {
    if(i == 4)
      continue;

    diff = fabs(angles::shortest_angular_distance(from_config[i], to_config[i]));
    if(max_diff < diff)
      max_diff = diff;
  }

  num_prims = max_diff / prms_.max_mprim_offset_ + 0.5;
  cost = num_prims * prms_.cost_multiplier_;

#if DEBUG_SEARCH
  ROS_DEBUG_NAMED("search", "from: %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f", angles::normalize_angle(from_config[0]),angles::normalize_angle(from_config[1]),angles::normalize_angle(from_config[2]),angles::normalize_angle(from_config[3]),angles::normalize_angle(from_config[4]),angles::normalize_angle(from_config[5]),angles::normalize_angle(from_config[6]));
  ROS_DEBUG_NAMED("search", "  to: %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f diff: %0.2f num_prims: %d cost: %d (mprim_size: %0.3f)", to_config[0],to_config[1],to_config[2],to_config[3],to_config[4],to_config[5],to_config[6], max_diff, num_prims, cost, prms_.max_mprim_offset_);
#endif

  return cost;
}

int EnvironmentChain3D::getEdgeCost(int FromStateID, int ToStateID)
{
#if DEBUG
  if(FromStateID >= (int)EnvChain.StateID2CoordTable.size() 
      || ToStateID >= (int)EnvChain.StateID2CoordTable.size())
  {
    SBPL_ERROR("ERROR in EnvChain... function: stateID illegal\n");
    throw new SBPL_Exception();
  }
#endif

  //get X, Y for the state
  EnvChain3DHashEntry* FromHashEntry = EnvChain.StateID2CoordTable[FromStateID];
  EnvChain3DHashEntry* ToHashEntry = EnvChain.StateID2CoordTable[ToStateID];

  return cost(FromHashEntry, ToHashEntry, false);
}

bool EnvironmentChain3D::isGoalStateWithOrientationSolver(const GoalPos &goal, std::vector<double> jnt_angles)
{
  //return rpysolver_->isOrientationFeasible(goal.rpy, jnt_angles, prefinal_joint_config, final_joint_config, stats_.nchecks_per_solver_per_succ_[solver_types::ORIENTATION_SOLVER]);
  unsigned char dist = 100;
  int path_length, nchecks = 0;
  std::vector<std::vector<double> > path; 
  
  if(!rpysolver_->isOrientationFeasible(goal.rpy, jnt_angles, path))
    return false;
  stats_.total_num_solver_used_[solver_types::ORIENTATION_SOLVER]++;
 
  // check motion for collisions 
  if(!cspace_->checkPathForCollision(jnt_angles, path[0], prms_.verbose_, path_length, nchecks, dist))
  {
    stats_.nchecks_per_solver_per_succ_[solver_types::ORIENTATION_SOLVER] += nchecks;
    return false;
  }
  stats_.nchecks_per_solver_per_succ_[solver_types::ORIENTATION_SOLVER] += nchecks;

  prefinal_joint_config = jnt_angles;
  final_joint_config = path[0];
  return true;
}

bool EnvironmentChain3D::setStartConfiguration(const std::vector<double> angles)
{
  unsigned char dist = 255;
  int x,y,z;
  std::vector<double> pose(6,0);

  if(int(angles.size()) < arm_->num_joints_)
    return false;

  /*
  // temporary hack - should be returned to the new cspace
  std::vector<std::vector<double> > cuboids = arm_->getCollisionCuboids();
  ROS_INFO("[env] received %d cuboids.",int(cuboids.size()));
  for(size_t i = 0; i < cuboids.size(); i++)
  {
    if(cuboids[i].size() == 6)
      grid_->addCollisionCuboid(cuboids[i][0],cuboids[i][1],cuboids[i][2],cuboids[i][3],cuboids[i][4],cuboids[i][5]);
    else
      ROS_INFO("[env] Self-collision cuboid #%d has an incomplete description.", int(i));
  }
  */

  //get joint positions of starting configuration
  if(!arm_->getPlanningJointPose(angles, pose))
    SBPL_WARN("Unable to compute forward kinematics for initial robot state. Attempting to plan anyway.");

  //check joint limits of starting configuration but plan anyway
  if(!arm_->checkJointLimits(angles, true))
    SBPL_WARN("Starting configuration violates the joint limits. Attempting to plan anyway.");

  //check if the start configuration is in collision but plan anyway
  if(!cspace_->checkCollision(angles, true, true, dist))
  {
    SBPL_WARN("[env] The starting configuration is in collision. Attempting to plan anyway. (distance to nearest obstacle %0.2fm)", double(dist)*grid_->getResolution());
    if(cspace_->collision_spheres_.size() > 0)
    {
      ROS_WARN("[env] Spheres in Collision:");
      for(size_t i = 0; i < cspace_->collision_spheres_.size(); ++i)
        ROS_WARN("[env] [%d] xyz: %0.3f %0.3f %0.3f  rad: %0.3f", int(i), cspace_->collision_spheres_[i].v.x(),cspace_->collision_spheres_[i].v.y(),cspace_->collision_spheres_[i].v.z(),cspace_->collision_spheres_[i].radius);
    }
    if(cspace_->attached_collision_spheres_.size() > 0)
    {
      ROS_WARN("[env] Attached Spheres in Collision:");
      for(size_t i = 0; i < cspace_->attached_collision_spheres_.size(); ++i)
        ROS_WARN("[env] [%d] xyz: %0.3f %0.3f %0.3f  rad: %0.3f", int(i), cspace_->attached_collision_spheres_[i].v.x(),cspace_->attached_collision_spheres_[i].v.y(),cspace_->attached_collision_spheres_[i].v.z(),cspace_->attached_collision_spheres_[i].radius);
    }
  }

  //get arm position in environment
  anglesToCoord(angles, EnvChain.startHashEntry->coord);

  grid_->worldToGrid(pose[0],pose[1],pose[2],x,y,z);
  EnvChain.startHashEntry->xyz[0] = (int)x;
  EnvChain.startHashEntry->xyz[1] = (int)y;
  EnvChain.startHashEntry->xyz[2] = (int)z;
  return true;
}

bool EnvironmentChain3D::setGoalPosition(const std::vector <std::vector<double> > &goals, const std::vector<std::vector<double> > &tolerances)
{
  //goals: {{x1,y1,z1,r1,p1,y1,is_6dof},{x2,y2,z2,r2,p2,y2,is_6dof}...}

  if(!env_chain_config_.bInitialized)
  {
    SBPL_ERROR("Cannot set goal position because environment is not initialized.");
    return false;
  }

  if(goals.empty())
  {
    SBPL_ERROR("[setGoalPosition] No goal constraint set.");
    return false;
  }
/*
  // temporary hack - should be returned to the new cspace
  std::vector<std::vector<double> > cuboids = arm_->getCollisionCuboids();
  ROS_INFO("[env] received %d cuboids",int(cuboids.size()));

  for(size_t i = 0; i < cuboids.size(); i++)
  {
    if(cuboids[i].size() == 6)
      grid_->addCollisionCuboid(cuboids[i][0],cuboids[i][1],cuboids[i][2],cuboids[i][3],cuboids[i][4],cuboids[i][5]);
    else
      ROS_INFO("[env] Self-collision cuboid #%d has an incomplete description.\n", i);
  }
  grid_->visualize();
*/
  // Check if an IK solution exists for the goal pose before we do the search
  // we plan even if there is no solution
  std::vector<double> pose(7,0);
  std::vector<double> jnt_angles(7,0);
  pose = goals[0];
  //unsigned char dist_ik;

  /*
  if(!arm_->computeIK(pose, jnt_angles, env_chain_config_.ik_solution))
    SBPL_INFO("[env] No valid IK solution for the goal pose.");
  else
  {
    if(!cspace_->checkCollision(env_chain_config_.ik_solution, false, false, dist_ik))
      SBPL_INFO("[env] An IK solution for the goal pose was found but it's in collision. Valid solutions may exist.");
    else
      SBPL_INFO("[env] A valid IK solution for the goal pose was found.");
  }
  */

  env_chain_config_.goal.xyz[0] = goals[0][0];
  env_chain_config_.goal.xyz[1] = goals[0][1];
  env_chain_config_.goal.xyz[2] = goals[0][2];
  env_chain_config_.goal.rpy[0] = goals[0][3];
  env_chain_config_.goal.rpy[1] = goals[0][4];
  env_chain_config_.goal.rpy[2] = goals[0][5];

  env_chain_config_.goal.xyz_disc_tolerance = tolerances[0][0] / grid_->getResolution();

  env_chain_config_.goal.xyz_tolerance[0] = tolerances[0][0];
  env_chain_config_.goal.xyz_tolerance[1] = tolerances[0][1];
  env_chain_config_.goal.xyz_tolerance[2] = tolerances[0][2];

  env_chain_config_.goal.rpy_tolerance[0] = tolerances[0][3];
  env_chain_config_.goal.rpy_tolerance[1] = tolerances[0][4];
  env_chain_config_.goal.rpy_tolerance[2] = tolerances[0][5];
  //TODO: Read in fangle for the goal here
  env_chain_config_.goal.is_6dof_goal = goals[0][6];
  prms_.use_6d_pose_goal_ = goals[0][6];

  env_chain_config_.goal.q[0] = goals[0][7];
  env_chain_config_.goal.q[1] = goals[0][8];
  env_chain_config_.goal.q[2] = goals[0][9];
  env_chain_config_.goal.q[3] = goals[0][10];

  if(!prms_.use_6d_pose_goal_)
    SBPL_DEBUG("[env] Goal position constraint set. No goal orientation constraint requested.\n");

  //convert goal position from meters to cells
  grid_->worldToGrid(goals[0][0], goals[0][1], goals[0][2], env_chain_config_.goal.xyz_disc[0], env_chain_config_.goal.xyz_disc[1], env_chain_config_.goal.xyz_disc[2]);

    //TODO: Check if goal is on an invalid cell

  // set goal hash entry
  EnvChain.goalHashEntry->xyz[0] = env_chain_config_.goal.xyz_disc[0];
  EnvChain.goalHashEntry->xyz[1] = env_chain_config_.goal.xyz_disc[1];
  EnvChain.goalHashEntry->xyz[2] = env_chain_config_.goal.xyz_disc[2]; 
  EnvChain.goalHashEntry->action = 0;

  for(int i = 0; i < arm_->num_joints_; i++)
    EnvChain.goalHashEntry->coord[i] = 0;


#if DEBUG_SEARCH
  if(prms_.verbose_)
  {
    ROS_DEBUG_NAMED("search", "\n-----------------------------------------------------------------------------------");
    ROS_DEBUG_NAMED("search", "time: %f", clock() / (double)CLOCKS_PER_SEC);
    ROS_DEBUG_NAMED("search", "A new goal has been set.");
    ROS_DEBUG_NAMED("search", "grid: %u %u %u (cells)  xyz: %.2f %.2f %.2f (meters)  (tol: %.3f) rpy: %1.2f %1.2f %1.2f (radians) (tol: %.3f)",
        env_chain_config_.goal.xyz_disc[0], env_chain_config_.goal.xyz_disc[1],env_chain_config_.goal.xyz_disc[2],
        env_chain_config_.goal.xyz[0],env_chain_config_.goal.xyz[1],env_chain_config_.goal.xyz[2],tolerances[0][0],
        env_chain_config_.goal.rpy[0],env_chain_config_.goal.rpy[1],env_chain_config_.goal.rpy[2],tolerances[0][1]);

    ROS_DEBUG_NAMED("search", "-----------------------------------------------------------------------------------\n");
  }
#endif

  // push obstacles into bfs grid
  ros::WallTime start = ros::WallTime::now();
  distance_field::PropagationDistanceField* df = grid_->getDistanceFieldPtr();
  int dimX, dimY, dimZ;
  grid_->getGridSize(dimX, dimY, dimZ);

  int walls=0;
  for (int z = 0; z < dimZ - 2; z++)
    for (int y = 0; y < dimY - 2; y++)
      for (int x = 0; x < dimX - 2; x++)
        if (df->getDistanceFromCell(x,y,z) <= arm_->getLinkRadius(2))
        {
          bfs_->setWall(x + 1, y + 1, z + 1);
          walls++;
        }
  
  double set_walls_time = (ros::WallTime::now() - start).toSec();
  ROS_INFO("[env] %0.5fsec to set walls in new bfs. (%d walls (%0.3f percent))", set_walls_time, walls, double(walls)/double(dimX*dimY*dimZ));
  start = ros::WallTime::now();
  bfs_->run(EnvChain.goalHashEntry->xyz[0], EnvChain.goalHashEntry->xyz[1], EnvChain.goalHashEntry->xyz[2]);
  ROS_INFO("[env] COST TO GOAL OF START STATE: %d", getBFSCostToGoal(EnvChain.startHashEntry->xyz[0], EnvChain.startHashEntry->xyz[1], EnvChain.startHashEntry->xyz[2]));
  ROS_INFO("[env] Time required to compute at least enough of the BFS to reach the start state: %0.4fsec", (ros::WallTime::now() - start).toSec());

  stats_.resetSolverCounters();
  stats_.resetAllCheckCounters();
  clearStats();
  return true;
}

void EnvironmentChain3D::clearStats()
{
  //a flag used for debugging only
  near_goal = false;

  expanded_states.clear();
  env_chain_config_.num_expands_to_position_constraint = 0;

  //start the 'planning time' clock
  starttime = clock();
}

void EnvironmentChain3D::StateID2Angles(int stateID, std::vector<double> &angles)
{
  EnvChain3DHashEntry* HashEntry = EnvChain.StateID2CoordTable[stateID];

  if(stateID == EnvChain.goalHashEntry->stateID)
  {
    coordToAngles(EnvChain.goalHashEntry->coord, angles);
    
    angles.resize(14);
    for(unsigned int i = 0; i < (unsigned int)angles.size(); i++)
    {
      if(i < 7)
        angles[i] = prefinal_joint_config[i];
      else
        angles[i] = final_joint_config[i-7];
    }
  }
  else
    coordToAngles(HashEntry->coord, angles);

  for (size_t i = 0; i < angles.size(); i++)
  {
    if(angles[i] >= M_PI)
      angles[i] = -2.0*M_PI + angles[i];
  }
}

void EnvironmentChain3D::printJointArray(FILE* fOut, EnvChain3DHashEntry* HashEntry, bool bGoal, bool bVerbose)
{
  int i;
  std::vector<double> angles(arm_->num_joints_,0);

  if(bGoal)
    coordToAngles(EnvChain.goalHashEntry->coord, angles);
  else
    coordToAngles(HashEntry->coord, angles);

  if(bVerbose)
    fprintf(fOut, "angles: ");

  for(i = 0; i < int(angles.size()); i++)
  {
    if(i > 0)
      fprintf(fOut, "%-.3f ", angles[i]-angles[i-1]);
    else
      fprintf(fOut, "%-.3f ", angles[i]);
  }
  fprintf(fOut, "\n");
  /*
  if(bVerbose)
    ROS_DEBUG_NAMED(fOut, "  xyz: %-2d %-2d %-2d  rpy: %2.2f %2.2f %2.2f  action: %d", HashEntry->xyz[0],HashEntry->xyz[1],HashEntry->xyz[2], HashEntry->rpy[0], HashEntry->rpy[1], HashEntry->rpy[2], HashEntry->action);
  else
    ROS_DEBUG_NAMED(fOut, "%-2d %-2d %-2d %-2d", HashEntry->xyz[0],HashEntry->xyz[1],HashEntry->xyz[2],HashEntry->action);
  */
}

std::vector<int> EnvironmentChain3D::debugExpandedStates()
{
  return expanded_states;
}

void EnvironmentChain3D::getExpandedStates(std::vector<std::vector<double> >* ara_states)
{
  std::vector<double> angles(arm_->num_joints_,0);
  std::vector<double>state(7,0);

  for(unsigned int i = 0; i < expanded_states.size(); ++i)
  {
    StateID2Angles(expanded_states[i],angles);
    arm_->getPlanningJointPose(angles,state);
    state[6] = EnvChain.StateID2CoordTable[expanded_states[i]]->heur;
    ara_states->push_back(state);
  }
}

void EnvironmentChain3D::computeCostPerCell()
{
  int largest_action=0;
  double gridcell_size, eucl_dist, max_dist = 0;
  std::vector<double> pose(6,0), start_pose(6,0), angles(arm_->num_joints_,0), start_angles(arm_->num_joints_,0);

  gridcell_size = grid_->getResolution();

  //starting at zeroed angles, find end effector position after each action
  arm_->getPlanningJointPose(start_angles, start_pose);

  //iterate through all possible actions and find the one with the minimum cost per cell
  for (int i = 0; i < prms_.num_mprims_; i++)
  {
    for(int j = 0; j < arm_->num_joints_; j++)
      angles[j] = DEG2RAD(prms_.mprims_[i][j]);

    //starting at zeroed angles, find end effector position after each action
    if(!arm_->getPlanningJointPose(angles, pose))
    {
      SBPL_WARN("[env] Failed to compute cost per cell because forward kinematics is returning an error.");
      return;
    }

    eucl_dist = sqrt((start_pose[0]-pose[0])*(start_pose[0]-pose[0]) +
        (start_pose[1]-pose[1])*(start_pose[1]-pose[1]) +
        (start_pose[2]-pose[2])*(start_pose[2]-pose[2]));

    if (eucl_dist > max_dist)
    {
      max_dist = eucl_dist;
      largest_action = i;
    }
  }

  prms_.setCellCost(int(prms_.cost_multiplier_ / (max_dist/gridcell_size)));

  prms_.cost_per_meter_ = int(prms_.cost_per_cell_ / gridcell_size);

  SBPL_INFO("[env] max_dist_traveled_per_smp: %0.3fm  cost per cell: %d  cost per meter: %d  (type: jointspace)", max_dist, prms_.cost_per_cell_,prms_.cost_per_meter_);
}

int EnvironmentChain3D::getBFSCostToGoal(int x, int y, int z) const
{
  return  bfs_->getDistance(x,y,z) * prms_.cost_per_cell_;
}

std::vector<std::vector<double> > EnvironmentChain3D::getShortestPath()
{
  std::vector<int> start(3,0);
  std::vector<double> waypoint(3,0);
  std::vector<std::vector<int> > path;
  std::vector<std::vector<double> > dpath; 

  //compute shortest path to goal considering obstacles
  if(prms_.use_bfs_heuristic_)
  {
    start[0] = EnvChain.startHashEntry->xyz[0];
    start[1] = EnvChain.startHashEntry->xyz[1];
    start[2] = EnvChain.startHashEntry->xyz[2];
    /*
    if(!bfs_->getShortestPath(start, path))
    {
      ROS_WARN("Unable to retrieve shortest path.");
      return dpath;
    }
    */
    for(int i=0; i < (int)path.size(); ++i)
    {
      grid_->gridToWorld(path[i][0], path[i][1], path[i][2], waypoint[0], waypoint[1], waypoint[2]);
      dpath.push_back(waypoint);
    }
  }
  //compute a straight line path to goal
  else
  {
    getBresenhamPath(EnvChain.startHashEntry->xyz,env_chain_config_.goal.xyz_disc,&path);
  }

  for(int i=0; i < int(path.size()); ++i)
  {
    grid_->gridToWorld(path[i][0], path[i][1], path[i][2], waypoint[0], waypoint[1], waypoint[2]);
    dpath.push_back(waypoint);
  }

  return dpath;
}

void EnvironmentChain3D::getBresenhamPath(const int a[],const int b[], std::vector<std::vector<int> > *path)
{
  bresenham3d_param_t params;
  std::vector<int> nXYZ(3,0);

  //iterate through the points on the segment
  get_bresenham3d_parameters(a[0], a[1], a[2], b[0], b[1], b[2], &params);
  do {
    get_current_point3d(&params, &(nXYZ[0]), &(nXYZ[1]), &(nXYZ[2]));

    path->push_back(nXYZ);
  } while (get_next_point3d(&params));

  SBPL_DEBUG("[getBresenhamPath] Path has %d waypoints.",int(path->size()));
}

void EnvironmentChain3D::setKinematicsToPlanningTransform(KDL::Frame f, std::string &name)
{
  arm_->setRefFrameTransform(f, name);
}

std::string EnvironmentChain3D::getKinematicsRootFrameName()
{
  std::string name;
  arm_->getArmChainRootLinkName(name);
  return name;
}

int EnvironmentChain3D::getEndEffectorHeuristic(int FromStateID, int ToStateID)
{
  int heur = 0;
  EnvChain3DHashEntry* FromHashEntry = EnvChain.StateID2CoordTable[FromStateID];

  //get distance heuristic
  if(prms_.use_bfs_heuristic_)
    heur = getBFSCostToGoal(FromHashEntry->xyz[0], FromHashEntry->xyz[1], FromHashEntry->xyz[2]);
  else
  {
    double x, y, z;
    grid_->gridToWorld(FromHashEntry->xyz[0],FromHashEntry->xyz[1],FromHashEntry->xyz[2], x, y, z);
    heur =  getEuclideanDistance(x, y, z, env_chain_config_.goal.xyz[0],env_chain_config_.goal.xyz[1], env_chain_config_.goal.xyz[2]) * prms_.cost_per_meter_;
  }
  // storing heuristic for debugging
  FromHashEntry->heur = heur;

  return heur;
}

void EnvironmentChain3D::getCollisionCuboids(std::vector<std::string> &cube_frames, std::vector<std::vector<double> > &cubes)
{
  arm_->getCollisionCuboids(cube_frames, cubes);
}

void EnvironmentChain3D::printEnvironmentStats()
{
  rpysolver_->printStats();
  //ROS_INFO("Calls to IK: %d   No Solutions: %d  Invalid Joint Limits: %d   Invalid Solutions: %d   Invalid Paths: %d", env_chain_config_.num_calls_to_ik, env_chain_config_.num_no_ik_solutions, env_chain_config_.num_ik_invalid_joint_limits,env_chain_config_.num_invalid_ik_solutions, env_chain_config_.num_ik_invalid_path);

  stats_.updateAverageChecksPerExpand();
  stats_.printTotalChecksPerSolverSummary();
  stats_.printTotalSolverUsedSummary();
  stats_.printStatsToFile(prms_.environment_type_);
} 

void EnvironmentChain3D::setStat(std::string field, double value)
{
  stats_.setStat(field, value);
}

}

