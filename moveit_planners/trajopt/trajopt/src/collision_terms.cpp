#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <boost/functional/hash.hpp>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt/collision_terms.hpp>
#include <trajopt/utils.hpp>
#include <trajopt_sco/expr_ops.hpp>
#include <trajopt_sco/expr_vec_ops.hpp>
#include <trajopt_sco/modeling_utils.hpp>
#include <trajopt_sco/sco_common.hpp>
#include <trajopt_utils/eigen_conversions.hpp>
#include <trajopt_utils/logging.hpp>
#include <trajopt_utils/stl_to_string.hpp>

namespace trajopt
{

// converts a vector of contacts to a vector of distances
void CollisionsToDistances(const std::vector<collision_detection::Contact>& dist_results, DblVec& dists)
{
  dists.clear();
  dists.reserve(dist_results.size());

  // how to get distance from MoveIt to feed dists
  // for (auto i = 0u; i < dist_results.size(); ++i)
  //   dists.push_back(dist_results[i].distance);

  for (auto i = 0u; i < dist_results.size(); ++i)
    dists.push_back(dist_results[i].depth);
}

// directly related to equation 16 in the TrajOpt paper (the version I have)
// tesseract::ContactResultVector& dist_results
void CollisionsToDistanceExpressions(const std::vector<collision_detection::Contact>& dist_results,
                                     planning_scene::PlanningSceneConstPtr planning_scene,
                                     const sco::VarVector& vars,
                                     const DblVec& x,
                                     sco::AffExprVector& exprs,
                                     bool isTimestep1)
{
  // we are trying to fill up exprs which should be the signed distance expression ???
  // typedef std::vector<AffExpr> AffExprVector
  // AffExpr is struct in solver_interface.h:
  //        constant, coeffs, vars

  // typedef std::vector<Var> VarVector
  // typedef std:vector<double> DblVec

  Eigen::VectorXd dofvals = sco::getVec(x, vars); // joint values
  // const std::vector<std::string>& link_names = manip->getLinkNames();
  const std::vector<std::string>& link_names = planning_scene->getRobotModel()->getLinkModelNames();

  // All collision data is in world corrdinate system. This provides the
  // transfrom for converting data between world frame and manipulator
  // frame.
  // tesseract::EnvStateConstPtr state = env->getState();
  // Eigen::Isometry3d change_base = state->transforms.at(manip->getBaseLinkName());
  // assert(change_base.isApprox(env->getState(manip->getJointNames(), dofvals)->transforms.at(manip->getBaseLinkName())));

  moveit::core::RobotState& robot_state = planning_scene->getCurrentState();
  const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);
  std::vector<std::string> group_joint_names = joint_model_group->getActiveJointModelNames();
  int group_dof = group_joint_names.size();

  exprs.clear();
  exprs.reserve(dist_results.size());
  for (auto i = 0u; i < dist_results.size(); ++i)
  {
    // => for each ContactResult in the total ContactResultVector that contains all the contacts of each pair and all the pairs
    // const tesseract::ContactResult& res = dist_results[i];
    const collision_detection::Contact& res = dist_results[i];

    const LinkModel* link_1(res.body_name_1); // link_names[0] 
    const LinkModel* link_2(res.body_name_2); // link_names[1] 

    // ContactResult in the original trajopt has two bodies and a distance
    // In MoveIt, we have Contact (corresponding to ContactResult in tesseract) type
    // which has depth (penetration between bodies). I am going to use this depth 
    // sco::AffExpr dist(res.distance);
    sco::AffExpr dist(res.depth); // depth could be positive or negative, if I use bullet
    // is that a problem here?

    Eigen::VectorXd dist_grad_a, dist_grad_b;
    // => find the name of linkA in collision
    std::vector<std::string>::const_iterator itA = std::find(link_names.begin(), link_names.end(), res.link_names[0]);
    if (itA != link_names.end())
    {
      // => create a jacobian matrix
      Eigen::MatrixXd jac;
      jac.resize(6, group_dof);

   /* Compute the Jacobian with reference to a particular point on a given link, for a specified group.
   *  - group: The group to compute the Jacobian for
   *  - link_name: The name of the link
   *  - reference_point_position: The reference point position (with respect to the link specified in link_name)
   *  - jacobian: The resultant jacobian
   *  - use_quaternion_representation Flag indicating if the Jacobian should use a quaternion representation
   * (default is false)
   *  - return True if jacobian was successfully computed, false otherwise
   */
      bool succeed = robot_state.getJacobian(
                                      joint_model_group,
                                      link_1, 
                                      res.nearest_points[0],
                                      jac);
      
   /** tesseract:
   *  Calculated jacobian at a link given joint angles
   *  jac: jacobian Output jacobian for a given link
   *  change_base: The transform from the base frame of the manipulator to the desired frame.
   *  joint_angles: Input vector of joint angles
   *  link_name: Name of link to calculate jacobian
   *  state: The state of the environment
   *  link_point: Point in the base frame for which to calculate the jacobian about
   *  output is a bool: True if calculation successful, False if anything is wrong (including uninitialized BasicKin)
   */
      // manip->calcJacobian(jac, 
      //                     change_base,
      //                     dofvals, 
      //                     res.link_names[0],
      //                     *state,
      //                     res.nearest_points[0]);

      dist_grad_a = -res.normal.transpose() * jac.topRows(3);
      // this means: [normal . jac(1:3,1), normal . jac(1:3,2), normal . jac(1:3,3)]
      // in the paper: nT * J_PA 

      sco::exprInc(dist, sco::varDot(dist_grad_a, vars)); 
      // varDot: creates an AffExpr with constant = 0, vars = second arg and coeffs = first arg
      // exprInc: adds the second arg to the first one
      
      sco::exprInc(dist, -dist_grad_a.dot(dofvals));
      // -dist_grad_a (dot_product) dofvals
    }

    std::vector<std::string>::const_iterator itB = std::find(link_names.begin(), link_names.end(), res.link_names[1]);
    if (itB != link_names.end())
    {
      Eigen::MatrixXd jac;
      jac.resize(6, group_dof);

    // how do I calculate cc_nearest_points, Contact in MoveIt does not have such a thing
    bool succeed = robot_state.getJacobian(
                  joint_model_group,
                  link_2, 
                  (isTimestep1 && (res.cc_type == trajopt::ContinouseCollisionType::CCType_Between)) ?
                   res.cc_nearest_points[1] :
                   res.nearest_points[1], 
                  jac);
   
     
      dist_grad_b = res.normal.transpose() * jac.topRows(3);
      sco::exprInc(dist, sco::varDot(dist_grad_b, vars));
      sco::exprInc(dist, -dist_grad_b.dot(dofvals));
    }
    // DebugPrintInfo(res, dist_grad_a, dist_grad_b, dofvals, i == 0);

    if (itA != link_names.end() || itB != link_names.end())
    {
      exprs.push_back(dist);
    }
  }
}

void CollisionsToDistanceExpressions(const tesseract::ContactResultVector& dist_results,
                                     planning_scene::PlanningSceneConstPtr planning_scene,
                                     const sco::VarVector& vars0,
                                     const sco::VarVector& vars1,
                                     const DblVec& x,
                                     sco::AffExprVector& exprs)
{
  sco::AffExprVector exprs0, exprs1;
  CollisionsToDistanceExpressions(dist_results, planning_scene, vars0, x, exprs0, false);
  CollisionsToDistanceExpressions(dist_results, planning_scene, vars1, x, exprs1, true);

  exprs.resize(exprs0.size());
  for (std::size_t i = 0; i < exprs0.size(); ++i)
  {
    assert(dist_results[i].cc_time >= 0.0 && dist_results[i].cc_time <= 1.0);
    sco::exprScale(exprs0[i], (1 - dist_results[i].cc_time));
    sco::exprScale(exprs1[i], dist_results[i].cc_time);
    exprs[i] = sco::AffExpr(0);
    sco::exprInc(exprs[i], exprs0[i]);
    sco::exprInc(exprs[i], exprs1[i]);
    sco::cleanupAff(exprs[i]);
  }
}

inline size_t hash(const DblVec& x) { return boost::hash_range(x.begin(), x.end()); }
void CollisionEvaluator::GetCollisionsCached(const DblVec& x, std::vector<collision_detection::Contact>& dist_results)
void CollisionEvaluator::GetCollisionsCached(const DblVec& x, std::vector<collision_detection::Contact>& dist_results)
{
  size_t key = hash(sco::getDblVec(x, GetVars()));
  // tesseract::ContactResultVector* it = m_cache.get(key);
  std::vector<collision_detection::Contact>* it = m_cache.get(key);
  
  if (it != nullptr)
  {
    LOG_DEBUG("using cached collision check\n");
    dist_results = *it;
  }
  else
  {
    LOG_DEBUG("not using cached collision check\n");
    CalcCollisions(x, dist_results);
    m_cache.put(key, dist_results);
  }
}

SingleTimestepCollisionEvaluator::SingleTimestepCollisionEvaluator(planning_scene::PlanningSceneConstPtr planning_scene,
                                                                   SafetyMarginDataConstPtr safety_margin_data,
                                                                   const sco::VarVector& vars)
  : CollisionEvaluator(planning_scene, safety_margin_data), m_vars(vars)
{
  contact_manager_ = env_->getDiscreteContactManager();
  contact_manager_->setActiveCollisionObjects(manip_->getLinkNames());
  contact_manager_->setContactDistanceThreshold(safety_margin_data_->getMaxSafetyMargin() +
                                               0.04);  // The original implementation added a margin of 0.04;
}

// So, each pair has a vector of contacts, and this functions puts all of these vectors of all pairs to one big vector called dist_results
//void SingleTimestepCollisionEvaluator::CalcCollisions(const DblVec& x, tesseract::ContactResultVector& dist_results)
void SingleTimestepCollisionEvaluator::CalcCollisions(const DblVec& x, std::vector<collision_detection::Contact>& dist_results)
{
  // tesseract::ContactResultMap contacts;
  // tesseract::EnvStatePtr state = env_->getState(manip_->getJointNames(), sco::getVec(x, m_vars));
  // ===> getVec is defined as:
  // Eigen::VectorXd getVec(const DblVec& x, const VarVector& vars) and
  // DblVec is vector<double>
  // VarVec is vector<Var>
  // Var has VarRep (look in solver_interface) member which has properties like name and index 
  // getVec extracts the index of VarRep of each Var of a VarVector and finds the element in DblVec with the 
  // same index and creates the output as VectorXd. 
  // Finally, getState(joint_names, joint_values)
  // So, input x of this function, CalcCollisions, is represting the joint values 
  // joint_values = sco::getVec(x, m_vars);

  // So, I guess I need to set the robot state based on these joint values
  Eigen::VectorXd joint_values = sco::getVec(x, m_vars);
  std::vector<double> joint_values_DblVec = std::vector<double>(joint_values.data(), joint_values.data() + joint_values.size());
  // TODO: set the state of the robot based on this joing values:
  planning_scene->...

  // for (const auto& link_name : manip_->getLinkNames())
  //   contact_manager_->setCollisionObjectsTransform(link_name, state->transforms[link_name]);
  // ====>
  // setCollisionObjectTransform sets the transform of a collision object.
  // contact manager has collision objects and has functions to remove/add them or 
  // set their transforms. This is basically adding all the robot links as collision objects
  // We already have this in planning_scene, calling fcl or bullet would consider all the robot links as collision object? 

  //contact_manager_->contactTest(contacts, tesseract::ContactTestTypes::ALL);
  // ====> ContactTestTypes::ALL = > Return all contacts for a pair of objects

  // tesseract::moveContactResultsMapToContactResultsVector(contacts, dist_results);
  // ====> moveContactResultsMapToContactResultsVector moves the elements of ContactResultVector
  // of each ContactResultMap in contacts to one ContactResultVector called dist_results

  // ===> for MoveIt
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  planning_scene->checkCollision(collision_request, collision_result);
  // ====> checkCollision will check for both self-collisions and for collisions with the environment

  // tesseract:
  // ContactResultMap => AlignedMap<std::pair<std::string, std::string>, ContactResultVector>
  // ContactResultVector: is an array of ContactResult msg
  // ContactResult: 
  //            distance
  //            array of two (link_names )
  // MoveIt:
  // CollisionResult:
  //          distance (min distance among all collision pairs) (double)
  //          contact_count (number of contacts) 
  //          ContactMap
  //          
  // ContactMap => std::map<std::pair<std::string, std::string>, std::vector<Contact> > 
  // Contact:
  //          pos (contact position)
  //          normal (normal unit vector at contact)
  //          depth (penetration between bodies)
  //          body_name_1
  //          body_name_2
  // 

  
  // Fill dist_result with all the Contacts of all the collision pairs. Each pair has a vector of contacts
  // and we have many pairs. Here we are putting all of these contact to one big vector. Not sure why that is ???
  // This happens in moveContactResultsMapToContactResultsVector
  for (it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it)
  {
      ROS_INFO("Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
      // std::move(it->second.begin(), it->second.end(), dist_results)      
      dist_results.insert(dist_results.end(), it->second.begin(), it->second.end())
  }

}

// so, given a set of joint values called x, this function converts the vector of contacts to the vector of distances
void SingleTimestepCollisionEvaluator::CalcDists(const DblVec& x, DblVec& dists)
{
  std::vector<collision_detection::Contact> dist_results;
  GetCollisionsCached(x, dist_results); 
  CollisionsToDistances(dist_results, dists);
}

void SingleTimestepCollisionEvaluator::CalcDistExpressions(const DblVec& x, sco::AffExprVector& exprs)
{
  std::vector<collision_detection::Contact> dist_results;
  GetCollisionsCached(x, dist_results);
  CollisionsToDistanceExpressions(dist_results, env_, manip_, m_vars, x, exprs, false);

  LOG_DEBUG("%ld distance expressions\n", exprs.size());
}


////////////////////////////////////////

CastCollisionEvaluator::CastCollisionEvaluator(planning_scene::PlanningSceneConstPtr planning_scene,
                                               SafetyMarginDataConstPtr safety_margin_data,
                                               const sco::VarVector& vars0,
                                               const sco::VarVector& vars1)
  : CollisionEvaluator(planning_scene, safety_margin_data), m_vars0(vars0), m_vars1(vars1)
{
  contact_manager_ = env_->getContinuousContactManager();
  contact_manager_->setActiveCollisionObjects(manip_->getLinkNames());
  contact_manager_->setContactDistanceThreshold(safety_margin_data_->getMaxSafetyMargin() +
                                                0.04);  // The original implementation added a margin of 0.04;
}

// similar to CalcCollisions from SingleTimestepCollisionEvaluator except we need two states because it is the 
// swept volume?
void CastCollisionEvaluator::CalcCollisions(const DblVec& x, std::vector<collision_detection::Contact>& dist_results& dist_results)
{
  // tesseract::ContactResultMap contacts;
  // tesseract::EnvStatePtr state0 = env_->getState(manip_->getJointNames(), sco::getVec(x, m_vars0));
  // tesseract::EnvStatePtr state1 = env_->getState(manip_->getJointNames(), sco::getVec(x, m_vars1));
  // ===> so far they create two states based on two sets of joint values

  // for (const auto& link_name : manip_->getLinkNames())
  //   contact_manager_->setCollisionObjectsTransform(link_name, state0->transforms[link_name], state1->transforms[link_name]);
  // ===>>> in here, they add two collision transforms for each link

  // ===>>> in the folloinwg, considering two collision transforms for each link, they calculate all the contacts among them
  // contact_manager_->contactTest(contacts, tesseract::ContactTestTypes::ALL);
  // tesseract::moveContactResultsMapToContactResultsVector(contacts, dist_results);

  // =====>>> So, I need to do it for two states as well.
  Eigen::VectorXd joint_values_state_1 = sco::getVec(x, m_vars0);
  std::vector<double> joint_values_DblVec_1 = std::vector<double>(joint_values_state_1.data(), joint_values_state_1.data() + joint_values_state_1.size());  
  collision_detection::CollisionRequest collision_request_1;
  collision_detection::CollisionResult collision_result_1;
  planning_scene->checkCollision(collision_request_1, collision_result_1);
  // TODO: set the state of the robot based on this joing values:
  planning_scene->...
  for (it = collision_result_1.contacts.begin(); it != collision_result_1.contacts.end(); ++it)
  {
      ROS_INFO("Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
      dist_results.insert(dist_results.end(), it->second.begin(), it->second.end())
  }

  Eigen::VectorXd joint_values_state_2 = sco::getVec(x, m_vars1);
  std::vector<double> joint_values_DblVec_2 = std::vector<double>(joint_values_state_2.data(), joint_values_state_2.data() + joint_values_state_2.size());
  collision_detection::CollisionRequest collision_request_2;
  collision_detection::CollisionResult collision_result_2;
  planning_scene->checkCollision(collision_request_2, collision_result_2);
  // TODO: set the state of the robot based on this joing values:
  planning_scene->...
  for (it = collision_result_2.contacts.begin(); it != collision_result_2.contacts.end(); ++it)
  {
      ROS_INFO("Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
      dist_results.insert(dist_results.end(), it->second.begin(), it->second.end())
  }
}

void CastCollisionEvaluator::CalcDistExpressions(const DblVec& x, sco::AffExprVector& exprs)
{
  std::vector<collision_detection::Contact> dist_results;
  GetCollisionsCached(x, dist_results);
  CollisionsToDistanceExpressions(dist_results, env_, manip_, m_vars0, m_vars1, x, exprs);
}
void CastCollisionEvaluator::CalcDists(const DblVec& x, DblVec& dists)
{
  std::vector<collision_detection::Contact> dist_results;
  GetCollisionsCached(x, dist_results);
  CollisionsToDistances(dist_results, dists);
}

//////////////////////////////////////////

CollisionCost::CollisionCost(planning_scene::PlanningSceneConstPtr planning_scene,
                             SafetyMarginDataConstPtr safety_margin_data,
                             const sco::VarVector& vars)
  : Cost("collision"), m_calc(new SingleTimestepCollisionEvaluator(planning_scene, safety_margin_data, vars))
{
}

CollisionCost::CollisionCost(planning_scene::PlanningSceneConstPtr planning_scene,
                             SafetyMarginDataConstPtr safety_margin_data,
                             const sco::VarVector& vars0,
                             const sco::VarVector& vars1)
  : Cost("cast_collision"), m_calc(new CastCollisionEvaluator(planning_scene, safety_margin_data, vars0, vars1))
{
}

sco::ConvexObjectivePtr CollisionCost::convex(const sco::DblVec& x, sco::Model* model)
{
  sco::ConvexObjectivePtr out(new sco::ConvexObjective(model));
  sco::AffExprVector exprs;
  m_calc->CalcDistExpressions(x, exprs);

  std::vector<collision_detection::Contact> dist_results;
  // m_calc->GetCollisionsCached(x, dist_results);
  m_calc->CalcCollisions(x, dist_results);
  for (std::size_t i = 0; i < exprs.size(); ++i)
  {
    //??? link_name from dist_results in MoveIt 
    const Eigen::Vector2d& data = m_calc->getSafetyMarginData()->getPairSafetyMarginData(dist_results[i].body_name_1,
                                                                                         dist_results[i].body_name_2);

    sco::AffExpr viol = sco::exprSub(sco::AffExpr(data[0]), exprs[i]);
    out->addHinge(viol, data[1]);
  }
  return out;
}

double CollisionCost::value(const sco::DblVec& x)
{
  DblVec dists;
  m_calc->CalcDists(x, dists);

  std::vector<collision_detection::Contact> dist_results;
  m_calc->GetCollisionsCached(x, dist_results);
  double out = 0;
  for (std::size_t i = 0; i < dists.size(); ++i)
  {
    //??? link_name from dist_results in MoveIt 
    const Eigen::Vector2d& data = m_calc->getSafetyMarginData()->getPairSafetyMarginData(dist_results[i].body_name_1,
                                                                                         dist_results[i].body_name_2);
    out += sco::pospart(data[0] - dists[i]) * data[1];
  }
  return out;
}

// ALMOST EXACTLY COPIED FROM CollisionCost

CollisionConstraint::CollisionConstraint(planning_scene::PlanningSceneConstPtr planning_scene,
                                         SafetyMarginDataConstPtr safety_margin_data,
                                         const sco::VarVector& vars)
  : m_calc(new SingleTimestepCollisionEvaluator(planning_scene, safety_margin_data, vars))
{
  name_ = "collision";
}

CollisionConstraint::CollisionConstraint(planning_scene::PlanningSceneConstPtr planning_scene,
                                         SafetyMarginDataConstPtr safety_margin_data,
                                         const sco::VarVector& vars0,
                                         const sco::VarVector& vars1)
  : m_calc(new CastCollisionEvaluator(planning_scene, safety_margin_data, vars0, vars1))
{
  name_ = "collision";
}

sco::ConvexConstraintsPtr CollisionConstraint::convex(const sco::DblVec& x, sco::Model* model)
{
  sco::ConvexConstraintsPtr out(new sco::ConvexConstraints(model));
  sco::AffExprVector exprs;
  m_calc->CalcDistExpressions(x, exprs);

  std::vector<collision_detection::Contact> dist_results;
  m_calc->GetCollisionsCached(x, dist_results);
  for (std::size_t i = 0; i < exprs.size(); ++i)
  {
    const Eigen::Vector2d& data = m_calc->getSafetyMarginData()->getPairSafetyMarginData(dist_results[i].body_name_1,
                                                                                         dist_results[i].body_name_1);

    sco::AffExpr viol = sco::exprSub(sco::AffExpr(data[0]), exprs[i]);
    out->addIneqCnt(sco::exprMult(viol, data[1]));
  }
  return out;
}

DblVec CollisionConstraint::value(const sco::DblVec& x)
{
  DblVec dists;
  m_calc->CalcDists(x, dists);

  std::vector<collision_detection::Contact> dist_results;
  // m_calc->GetCollisionsCached(x, dist_results);
  m_calc->CalcCollisions(x, dist_results);
  DblVec out(dists.size());
  for (std::size_t i = 0; i < dists.size(); ++i)
  {
    const Eigen::Vector2d& data = m_calc->getSafetyMarginData()->getPairSafetyMarginData(dist_results[i].link_names[0],
                                                                                         dist_results[i].link_names[1]);

    out[i] = sco::pospart(data[0] - dists[i]) * data[1];
  }
  return out;
}
}
