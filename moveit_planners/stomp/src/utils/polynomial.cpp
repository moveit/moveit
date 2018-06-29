/**
 * @file polynomial.cpp
 * @brief TODO
 *
 * @author Jorge Nicho
 * @date Jan 6, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stomp_moveit/utils/polynomial.h>
#include <ros/console.h>

/**
 * @namespace stomp_moveit
 */
namespace stomp_moveit
{

/**
 * @namespace utils
 */
namespace utils
{

/**
 * @namespace smoothing
 */
namespace polynomial
{


PolyFitResults polyFit(const PolyFitRequest &request)
{

// Unconstrained Least Squares
// |p|=|A*A'|^-1 * |A*y|
//
// Constrained Least Squares
//  |p| - | 2*A*A', C |^-1 * | 2*A*b |
//  |z| - |     C', 0 |      |     d |
//
//  Variable Description:
//    x_t         (A) - Is the Vandermonde matrix of all (constrained and unconstrained) domain values
//    a_t         (C) - Is the Vandermonde matrix of the constrained domain values
//    y           (b) - An array of the values to perform the fit on
//    yf          (d) - An array of the values corresponding to the constrained domain values
//    poly_params (p) - An array of the polynomial coefficients solved for.

  PolyFitResults result;

  int num_r = request.d + 1;
  Eigen::MatrixXd x_t(num_r, request.xy.cols());
  if (!request.hasConstraints())
  {
    for (auto r = 0; r < num_r; r++)
      x_t.row(r) = request.xy.row(0).array().pow(r);

    Eigen::MatrixXd mat = x_t * x_t.transpose();
    Eigen::VectorXd vec = x_t * request.xy.row(1);

    result.p = mat.lu().solve(vec).head(num_r);
  }
  else
  {
    int num_constraints = request.xyfp.cols() + request.xyfs.cols();

    Eigen::MatrixXd a_t(num_r, num_constraints);
    Eigen::MatrixXd a_tp(num_r, request.xyfp.cols());
    Eigen::MatrixXd a_ts(num_r, request.xyfs.cols());

    for (auto r = 0; r < num_r; r++)
    {
      x_t.row(r) = request.xy.row(0).array().pow(r);

      if (request.hasPositionConstraints())
        a_tp.row(r) = request.xyfp.row(0).array().pow(r);

      if (request.hasSlopeConstraints())
        if (r == 0)
          a_ts.row(r) = request.xyfs.row(0).array() * 0.0;
        else
          a_ts.row(r) = request.xyfs.row(0).array().pow(r - 1) / r;

    }

    Eigen::MatrixXd top(num_r, num_r + num_constraints);
    Eigen::MatrixXd bot(num_constraints, num_r + num_constraints);
    Eigen::MatrixXd mat(num_r + num_constraints, num_r + num_constraints);
    Eigen::VectorXd vec(num_r + num_constraints);

    if (request.hasPositionConstraints() && request.hasSlopeConstraints())
    {
      a_t << a_tp, a_ts;
      vec << 2*x_t*request.xy.row(1).transpose(), request.xyfp.row(1).transpose(), request.xyfs.row(1).transpose();
    }
    else if (request.hasPositionConstraints())
    {
      a_t << a_tp;
      vec << 2*x_t*request.xy.row(1).transpose(), request.xyfp.row(1).transpose();
    }
    else if (request.hasSlopeConstraints())
    {
      a_t << a_ts;
      vec << 2*x_t*request.xy.row(1).transpose(), request.xyfs.row(1).transpose();
    }

    top << 2*x_t*x_t.transpose(), a_t;
    bot << a_t.transpose(), Eigen::MatrixXd::Zero(num_constraints, num_constraints);
    mat << top,
           bot;

    result.p = mat.lu().solve(vec).head(num_r);
  }

  if (request.output_domain.size() == 0)
  {
    result.x = request.xy.row(0);
    result.y = x_t.transpose() * result.p;
    result.successful = true;
    return result;
  }
  else
  {
    result.x = request.output_domain;
    fillVandermondeMatrix(result.x, request.d, x_t);
    result.y = x_t.transpose() * result.p;
    result.successful = true;
    return result;
  }
}

void fillVandermondeMatrix(const Eigen::ArrayXd &domain_vals, const int &order, Eigen::MatrixXd &v)
{
  v = Eigen::MatrixXd::Ones(order+1, domain_vals.size());
  for(auto p = 1u; p <=  order; p++)
    v.row(p) = domain_vals.pow(p);
}

void generateMinimumDomainValues(const std::vector<const  moveit::core::JointModel *> joint_models,
                                 const Eigen::MatrixXd &parameters,Eigen::VectorXd &domain_values)
{
  Eigen::VectorXd distance(parameters.rows());
  Eigen::VectorXd velocity(parameters.rows());
  Eigen::VectorXd t(parameters.rows());
  Eigen::VectorXd domain_dist(parameters.cols());
  double max_t = 0;

  domain_values.resize(parameters.cols());
  for(auto r = 0; r < parameters.rows(); r++)
  {
    moveit::core::VariableBounds bound = joint_models[r]->getVariableBounds()[0];
    velocity(r) = bound.max_velocity_;
    distance(r) = 0.0;
    domain_dist(0) = 0.0;
    for(auto c = 1; c < parameters.cols(); c++)
    {
      distance(r) += std::abs((parameters(r, c)) - (parameters(r, c - 1)));
      domain_dist(c) = distance(r);
    }

    t(r) = distance(r)/velocity(r);
    if (t(r) > max_t)
    {
      max_t = t(r);
      domain_values = domain_dist/velocity(r);
    }
  }
}

bool applyPolynomialSmoothing(moveit::core::RobotModelConstPtr robot_model, const std::string& group_name, Eigen::MatrixXd& parameters,
                              int poly_order, double joint_limit_margin)
{
  using namespace Eigen;
  using namespace moveit::core;

  const std::vector<const JointModel*> &joint_models = robot_model->getJointModelGroup(group_name)->getActiveJointModels();
  PolyFitRequest request;
  int num_timesteps = parameters.cols();

  VectorXd domain_vals;
  domain_vals.setLinSpaced(num_timesteps,0,1);

  request.d = poly_order;
  request.xy.resize(2, num_timesteps);
  request.xy.row(0) = domain_vals;
  for(auto r = 0; r < parameters.rows(); r++)
  {
    request.xy.row(1) = parameters.row(r);
    request.xyfp.resize(2, 2);
    request.xyfp(0, 0) = request.xy(0, 0);
    request.xyfp(1, 0) = request.xy(1, 0);
    request.xyfp(0, 1) = request.xy(0, num_timesteps - 1);
    request.xyfp(1, 1) = request.xy(1, num_timesteps - 1);

    PolyFitResults results;
    results = polyFit(request);
    for(auto i = 0; i < results.y.size(); ++i)
      joint_models[r]->enforcePositionBounds(&results.y[i]);

    //  Now check if joint trajectory is within joint limits
    bool finished;
    double min = results.y.minCoeff();
    double max = results.y.maxCoeff();
    finished = joint_models[r]->satisfiesPositionBounds(&min, joint_limit_margin) &&
               joint_models[r]->satisfiesPositionBounds(&max, joint_limit_margin);

    if ((results.p.array() != results.p.array()).any())
    {
      ROS_ERROR("Smoother, joint %s polynomial fit failed!", joint_models[r]->getName().c_str());
      return false;
    }

    if(!finished)
    {
      ROS_ERROR("Smoother, joint %s not within limits, Min: %f, Max: %f", joint_models[r]->getName().c_str(), min, max);
      return false;
    }

    parameters.row(r) = results.y;
  }

  return true;
}

} // end of namespace smoothing
} // end of namespace utils
} // end of namespace stomp_moveit




