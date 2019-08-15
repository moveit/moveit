#include <trajopt_sco/num_diff.hpp>

namespace sco
{
ScalarOfVectorPtr ScalarOfVector::construct(const func& f)
{
  struct F : public ScalarOfVector
  {
    func f;
    F(const func& _f) : f(_f) {}
    double operator()(const Eigen::VectorXd& x) const override { return f(x); }
  };
  ScalarOfVector* sov = new F(f);  // to avoid erroneous clang warning
  return ScalarOfVectorPtr(sov);
}
VectorOfVectorPtr VectorOfVector::construct(const func& f)
{
  struct F : public VectorOfVector
  {
    func f;
    F(const func& _f) : f(_f) {}
    Eigen::VectorXd operator()(const Eigen::VectorXd& x) const override { return f(x); }
  };
  VectorOfVector* vov = new F(f);  // to avoid erroneous clang warning
  return VectorOfVectorPtr(vov);
}

Eigen::VectorXd calcForwardNumGrad(const ScalarOfVector& f, const Eigen::VectorXd& x, double epsilon)
{
  Eigen::VectorXd out(x.size());
  Eigen::VectorXd xpert = x;
  double y = f(x);
  for (int i = 0; i < x.size(); ++i)
  {
    xpert(i) = x(i) + epsilon;
    double ypert = f(xpert);
    out(i) = (ypert - y) / epsilon;
    xpert(i) = x(i);
  }
  return out;
}
Eigen::MatrixXd calcForwardNumJac(const VectorOfVector& f, const Eigen::VectorXd& x, double epsilon)
{
  Eigen::VectorXd y = f(x);
  Eigen::MatrixXd out(y.size(), x.size());
  Eigen::VectorXd xpert = x;
  for (int i = 0; i < x.size(); ++i)
  {
    xpert(i) = x(i) + epsilon;
    Eigen::VectorXd ypert = f(xpert);
    out.col(i) = (ypert - y) / epsilon;
    xpert(i) = x(i);
  }
  return out;
}

void calcGradAndDiagHess(const ScalarOfVector& f,
                         const Eigen::VectorXd& x,
                         double epsilon,
                         double& y,
                         Eigen::VectorXd& grad,
                         Eigen::VectorXd& hess)
{
  y = f(x);
  grad.resize(x.size());
  hess.resize(x.size());
  Eigen::VectorXd xpert = x;
  for (int i = 0; i < x.size(); ++i)
  {
    xpert(i) = x(i) + epsilon / 2;
    double yplus = f(xpert);
    xpert(i) = x(i) - epsilon / 2;
    double yminus = f(xpert);
    grad(i) = (yplus - yminus) / epsilon;
    hess(i) = (yplus + yminus - 2 * y) / (epsilon * epsilon / 4);
    xpert(i) = x(i);
  }
}

void calcGradHess(ScalarOfVectorPtr f,
                  const Eigen::VectorXd& x,
                  double epsilon,
                  double& y,
                  Eigen::VectorXd& grad,
                  Eigen::MatrixXd& hess)
{
  y = f->call(x);
  VectorOfVectorPtr grad_func = forwardNumGrad(f, epsilon);
  grad = grad_func->call(x);
  hess = calcForwardNumJac(*grad_func, x, epsilon);
  hess = (hess + hess.transpose()) / 2;
}

struct ForwardNumGrad : public VectorOfVector
{
  ScalarOfVectorPtr f_;
  double epsilon_;
  ForwardNumGrad(ScalarOfVectorPtr f, double epsilon) : f_(f), epsilon_(epsilon) {}
  Eigen::VectorXd operator()(const Eigen::VectorXd& x) const override { return calcForwardNumGrad(*f_, x, epsilon_); }
};

struct ForwardNumJac : public MatrixOfVector
{
  VectorOfVectorPtr f_;
  double epsilon_;
  ForwardNumJac(VectorOfVectorPtr f, double epsilon) : f_(f), epsilon_(epsilon) {}
  Eigen::MatrixXd operator()(const Eigen::VectorXd& x) const override { return calcForwardNumJac(*f_, x, epsilon_); }
};

VectorOfVectorPtr forwardNumGrad(ScalarOfVectorPtr f, double epsilon)
{
  return VectorOfVectorPtr(new ForwardNumGrad(f, epsilon));
}
MatrixOfVectorPtr forwardNumJac(VectorOfVectorPtr f, double epsilon)
{
  return MatrixOfVectorPtr(new ForwardNumJac(f, epsilon));
}
}
