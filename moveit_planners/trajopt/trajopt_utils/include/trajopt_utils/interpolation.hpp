#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
TRAJOPT_IGNORE_WARNINGS_POP

namespace util
{
template <typename VectorT>
Eigen::VectorXi searchsorted(const VectorT& x, const VectorT& y)
{
  // y(i-1) <= x(out(i)) < y(i)
  int nX = x.size();
  int nY = y.size();

  Eigen::VectorXi out(nX);
  int iY = 0;
  for (int iX = 0; iX < nX; iX++)
  {
    while (iY < nY && x[iX] > y[iY])
      iY++;
    out(iX) = iY;
  }
  return out;
}

template <typename MatrixT, typename VectorT>
MatrixT interp2d(const VectorT& xNew, const VectorT& xOld, const MatrixT& yOld)
{
  int nNew = xNew.size();
  int nOld = xOld.size();
  MatrixT yNew(nNew, yOld.cols());
  Eigen::VectorXi new2old = searchsorted(xNew, xOld);
  for (int iNew = 0; iNew < nNew; iNew++)
  {
    int iOldAbove = new2old(iNew);
    if (iOldAbove == 0)
      yNew.row(iNew) = yOld.row(0);
    else if (iOldAbove == nOld)
      yNew.row(iNew) = yOld.row(nOld - 1);
    else
    {
      double t = (xNew(iNew) - xOld(iOldAbove - 1)) / (xOld(iOldAbove) - xOld(iOldAbove - 1));
      yNew.row(iNew) = yOld.row(iOldAbove - 1) * (1 - t) + yOld.row(iOldAbove) * t;
    }
  }
  return yNew;
}
}
