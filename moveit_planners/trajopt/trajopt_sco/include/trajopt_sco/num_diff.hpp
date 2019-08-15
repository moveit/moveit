#pragma once
#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Dense>
#include <functional>
#include <memory>
TRAJOPT_IGNORE_WARNINGS_POP

/*
 * Numerical derivatives
 */

namespace sco
{
class ScalarOfVector;
class VectorOfVector;
class MatrixOfVector;
typedef std::shared_ptr<ScalarOfVector> ScalarOfVectorPtr;
typedef std::shared_ptr<VectorOfVector> VectorOfVectorPtr;
typedef std::shared_ptr<MatrixOfVector> MatrixOfVectorPtr;

class ScalarOfVector
{
public:
  virtual double operator()(const Eigen::VectorXd& x) const = 0;
  double call(const Eigen::VectorXd& x) const { return operator()(x); }
  virtual ~ScalarOfVector() {}
  typedef std::function<double(Eigen::VectorXd)> func;
  static ScalarOfVectorPtr construct(const func&);
  //  typedef VectorXd (*c_func)(const VectorXd&);
  //  static ScalarOfVectorPtr construct(const c_func&);
};
class VectorOfVector
{
public:
  virtual Eigen::VectorXd operator()(const Eigen::VectorXd& x) const = 0;
  Eigen::VectorXd call(const Eigen::VectorXd& x) const { return operator()(x); }
  virtual ~VectorOfVector() {}
  typedef std::function<Eigen::VectorXd(Eigen::VectorXd)> func;
  static VectorOfVectorPtr construct(const func&);
  //  typedef VectorXd (*c_func)(const VectorXd&);
  //  static VectorOfVectorPtr construct(const c_func&);
};
class MatrixOfVector
{
public:
  virtual Eigen::MatrixXd operator()(const Eigen::VectorXd& x) const = 0;
  Eigen::MatrixXd call(const Eigen::VectorXd& x) const { return operator()(x); }
  virtual ~MatrixOfVector() {}
  typedef std::function<Eigen::MatrixXd(Eigen::VectorXd)> func;
  static MatrixOfVectorPtr construct(const func&);
  //  typedef VectorMatrixXd (*c_func)(const VectorXd&);
  //  static MatrixOfVectorPtr construct(const c_func&);
};

Eigen::VectorXd calcForwardNumGrad(const ScalarOfVector& f, const Eigen::VectorXd& x, double epsilon);
Eigen::MatrixXd calcForwardNumJac(const VectorOfVector& f, const Eigen::VectorXd& x, double epsilon);
void calcGradAndDiagHess(const ScalarOfVector& f,
                         const Eigen::VectorXd& x,
                         double epsilon,
                         double& y,
                         Eigen::VectorXd& grad,
                         Eigen::VectorXd& hess);
void calcGradHess(ScalarOfVectorPtr f,
                  const Eigen::VectorXd& x,
                  double epsilon,
                  double& y,
                  Eigen::VectorXd& grad,
                  Eigen::MatrixXd& hess);
VectorOfVectorPtr forwardNumGrad(ScalarOfVectorPtr f, double epsilon);
MatrixOfVectorPtr forwardNumJac(VectorOfVectorPtr f, double epsilon);
}
