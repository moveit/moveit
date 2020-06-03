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

void CollisionsToDistances(const std::vector<collision_detection::Contact>& dist_results, DblVec& dists)
{
  dists.clear();
  dists.reserve(dist_results.size());

  // how to get distance from MoveIt to feed dists
  for (auto i = 0u; i < dist_results.size(); ++i)
    dists.push_back(dist_results[i].distance);
}

// directly related to equation 16 in the TrajOpt paper (the version I have)
void CollisionsToDistanceExpressions(const tesseract::ContactResultVector& dist_results,
                                     const tesseract::BasicEnvConstPtr env,
                                     const tesseract::BasicKinConstPtr manip,
                                     const sco::VarVector& vars,
                                     const DblVec& x,
                                     sco::AffExprVector& exprs,
                                     bool isTimestep1)
{
  // we are trying to fill up exprs which should be the signed distance
  // typedef std::vector<Var> VarVector

  Eigen::VectorXd dofvals = sco::getVec(x, vars);
  const std::vector<std::string>& link_names = manip->getLinkNames();

  // All collision data is in world corrdinate system. This provides the
  // transfrom for converting data between world frame and manipulator
  // frame.
  tesseract::EnvStateConstPtr state = env->getState();
  Eigen::Isometry3d change_base = state->transforms.at(manip->getBaseLinkName());
  assert(change_base.isApprox(env->getState(manip->getJointNames(), dofvals)->transforms.at(manip->getBaseLinkName())));

  exprs.clear();
  exprs.reserve(dist_results.size());
  for (auto i = 0u; i < dist_results.size(); ++i)
  {
    // => for each ContactResult in ContactResultVector
    const tesseract::ContactResult& res = dist_results[i];

    // => get the distance between the bodies (ContactResult.distance)
    sco::AffExpr dist(res.distance);

    Eigen::VectorXd dist_grad_a, dist_grad_b;
    // => find the name of linkA in collision
    std::vector<std::string>::const_iterator itA = std::find(link_names.begin(), link_names.end(), res.link_names[0]);
    if (itA != link_names.end())
    {
      // => create a jacobian matrix
      Eigen::MatrixXd jac;
      jac.resize(6, manip->numJoints());
      // ??? calculate jacobian matrix in MoveIt for linkA
      manip->calcJacobian(jac, change_base, dofvals, res.link_names[0], *state, res.nearest_points[0]);
      dist_grad_a = -res.normal.transpose() * jac.topRows(3);
      sco::exprInc(dist, sco::varDot(dist_grad_a, vars));
      sco::exprInc(dist, -dist_grad_a.dot(dofvals));
    }

    std::vector<std::string>::const_iterator itB = std::find(link_names.begin(), link_names.end(), res.link_names[1]);
    if (itB != link_names.end())
    {
      Eigen::MatrixXd jac;
      jac.resize(6, manip->numJoints());
      manip->calcJacobian(jac,
                          change_base,
                          dofvals,
                          res.link_names[1],
                          *state,
                          (isTimestep1 && (res.cc_type == tesseract::ContinouseCollisionType::CCType_Between)) ?
                              res.cc_nearest_points[1] :
                              res.nearest_points[1]);
      dist_grad_b = res.normal.transpose() * jac.topRows(3);
      sco::exprInc(dist, sco::varDot(dist_grad_b, vars));
      sco::exprInc(dist, -dist_grad_b.dot(dofvals));
    }

    if (itA != link_names.end() || itB != link_names.end())
    {
      exprs.push_back(dist);
    }
  }
}

void CollisionsToDistanceExpressions(const tesseract::ContactResultVector& dist_results,
                                     const tesseract::BasicEnvConstPtr env,
                                     const tesseract::BasicKinConstPtr manip,
                                     const sco::VarVector& vars0,
                                     const sco::VarVector& vars1,
                                     const DblVec& x,
                                     sco::AffExprVector& exprs)
{
  sco::AffExprVector exprs0, exprs1;
  CollisionsToDistanceExpressions(dist_results, env, manip, vars0, x, exprs0, false);
  CollisionsToDistanceExpressions(dist_results, env, manip, vars1, x, exprs1, true);

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

// inline size_t hash(const DblVec& x) { return boost::hash_range(x.begin(), x.end()); }
// // void CollisionEvaluator::GetCollisionsCached(const DblVec& x, tesseract::ContactResultVector& dist_results)
// void CollisionEvaluator::GetCollisionsCached(const DblVec& x, std::vector<collision_detection::Contact>& dist_results)
// {
//   size_t key = hash(sco::getDblVec(x, GetVars()));
//   tesseract::ContactResultVector* it = m_cache.get(key);
//   if (it != nullptr)
//   {
//     LOG_DEBUG("using cached collision check\n");
//     dist_results = *it;
//   }
//   else
//   {
//     LOG_DEBUG("not using cached collision check\n");
//     CalcCollisions(x, dist_results);
//     m_cache.put(key, dist_results);
//   }
// }

SingleTimestepCollisionEvaluator::SingleTimestepCollisionEvaluator(planning_scene::PlanningSceneConstPtr planning_scene,
                                                                   SafetyMarginDataConstPtr safety_margin_data,
                                                                   const sco::VarVector& vars)
  : CollisionEvaluator(planning_scene, manip, env, safety_margin_data), m_vars(vars)
{
  // contact_manager_ = env_->getDiscreteContactManager();
  // contact_manager_->setActiveCollisionObjects(manip_->getLinkNames());
  // contact_manager_->setContactDistanceThreshold(safety_margin_data_->getMaxSafetyMargin() +
                                                0.04);  // The original implementation added a margin of 0.04;
}

//void SingleTimestepCollisionEvaluator::CalcCollisions(const DblVec& x, tesseract::ContactResultVector& dist_results)
void SingleTimestepCollisionEvaluator::CalcCollisions(const DblVec& x, std::vector<collision_detection::Contact>& dist_results)
{
  // tesseract::ContactResultMap contacts;
  // tesseract::EnvStatePtr state = env_->getState(manip_->getJointNames(), sco::getVec(x, m_vars));

  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  planning_scene->checkCollision(collision_request, collision_result);

  // for (const auto& link_name : manip_->getLinkNames())
  //   contact_manager_->setCollisionObjectsTransform(link_name, state->transforms[link_name]);

  //contact_manager_->contactTest(contacts, tesseract::ContactTestTypes::ALL);
  // tesseract::moveContactResultsMapToContactResultsVector(contacts, dist_results);
  // this moveContactResultsMapToContactResultsVector moves the elements of ContactResultVector
  // of each ContactResultMap in contacts to one ContactResultVector called dist_results

  // moveContactResultsMapToContactResultsVector does the following:
  // it gets the ContactResultVector of contacts and move them to dist_result
  
  // tesseract
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
  
  // Fill dist_result with all the Contacts of all the collision pairs
  for (it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it)
  {
      ROS_INFO("Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
      // std::move(it->second.begin(), it->second.end(), dist_results)      
      dist_results.insert(dist_results.end(), it->second.begin(), it->second.end())
  }

}

void SingleTimestepCollisionEvaluator::CalcDists(const DblVec& x, DblVec& dists)
{
  std::vector<collision_detection::Contact> dist_results;
  // GetCollisionsCached(x, dist_results);
  CalcCollisions(x, dist_results);
  CollisionsToDistances(dist_results, dists);
}

void SingleTimestepCollisionEvaluator::CalcDistExpressions(const DblVec& x, sco::AffExprVector& exprs)
{
  std::vector<collision_detection::Contact> dist_results;
  // GetCollisionsCached(x, dist_results);
  CalcCollisions(x, dist_results);
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
  // contact_manager_ = env_->getContinuousContactManager();
  // contact_manager_->setActiveCollisionObjects(manip_->getLinkNames());
  // contact_manager_->setContactDistanceThreshold(safety_margin_data_->getMaxSafetyMargin() +
                                                0.04);  // The original implementation added a margin of 0.04;
}

void CastCollisionEvaluator::CalcCollisions(const DblVec& x, std::vector<collision_detection::Contact>& dist_results& dist_results)
{
  // tesseract::ContactResultMap contacts;
  // tesseract::EnvStatePtr state0 = env_->getState(manip_->getJointNames(), sco::getVec(x, m_vars0));
  // tesseract::EnvStatePtr state1 = env_->getState(manip_->getJointNames(), sco::getVec(x, m_vars1));
  // for (const auto& link_name : manip_->getLinkNames())
  //   contact_manager_->setCollisionObjectsTransform(
  //       link_name, state0->transforms[link_name], state1->transforms[link_name]);

  // contact_manager_->contactTest(contacts, tesseract::ContactTestTypes::ALL);
  // tesseract::moveContactResultsMapToContactResultsVector(contacts, dist_results);

  // similar to CalcCollisions from SingleTimestep...
}
void CastCollisionEvaluator::CalcDistExpressions(const DblVec& x, sco::AffExprVector& exprs)
{
  std::vector<collision_detection::Contact> dist_results;
  // GetCollisionsCached(x, dist_results);
  CalcCollisions(x, dist_results);
  CollisionsToDistanceExpressions(dist_results, env_, manip_, m_vars0, m_vars1, x, exprs);
}
void CastCollisionEvaluator::CalcDists(const DblVec& x, DblVec& dists)
{
  std::vector<collision_detection::Contact> dist_results;
  // GetCollisionsCached(x, dist_results);
  CalcCollisions(x, dist_results);
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
    const Eigen::Vector2d& data = m_calc->getSafetyMarginData()->getPairSafetyMarginData(dist_results[i].link_names[0],
                                                                                         dist_results[i].link_names[1]);

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
  // m_calc->GetCollisionsCached(x, dist_results);
  m_calc->CalcCollisions(x, dist_results);
  double out = 0;
  for (std::size_t i = 0; i < dists.size(); ++i)
  {
    //??? link_name from dist_results in MoveIt 
    const Eigen::Vector2d& data = m_calc->getSafetyMarginData()->getPairSafetyMarginData(dist_results[i].link_names[0],
                                                                                         dist_results[i].link_names[1]);
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
  // m_calc->GetCollisionsCached(x, dist_results);
  m_calc->CalcCollisions(x, dist_results);
  for (std::size_t i = 0; i < exprs.size(); ++i)
  {
    const Eigen::Vector2d& data = m_calc->getSafetyMarginData()->getPairSafetyMarginData(dist_results[i].link_names[0],
                                                                                         dist_results[i].link_names[1]);

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
