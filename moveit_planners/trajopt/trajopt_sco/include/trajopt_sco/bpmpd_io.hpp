#pragma once
#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <string>
#include <vector>
#include <assert.h>
#include <iostream>
#include <stdlib.h>
#include <unistd.h>
TRAJOPT_IGNORE_WARNINGS_POP

namespace bpmpd_io
{
enum SerMode
{
  DESER,
  SER
};

template <typename T>
void ser(int fp, T& x, SerMode mode)
{
  switch (mode)
  {
    case SER:
    {
      T xcopy = x;
      long n = write(fp, &xcopy, sizeof(T));
      assert(n == sizeof(T));
      break;
    }
    case DESER:
    {
      long n = read(fp, &x, sizeof(T));
      assert(n == sizeof(T));
      break;
    }
  }
}

template <typename T>
void ser(int fp, std::vector<T>& x, SerMode mode)
{
  unsigned long size = x.size();
  ser(fp, size, mode);
  switch (mode)
  {
    case SER:
    {
      long n = write(fp, x.data(), sizeof(T) * size);
      assert(static_cast<unsigned long>(n) == sizeof(T) * size);
      break;
    }
    case DESER:
    {
      x.resize(size);
      long n = read(fp, x.data(), sizeof(T) * size);
      assert(static_cast<unsigned>(n) == sizeof(T) * size);
      break;
    }
  }
}

struct bpmpd_input
{
  int m, n, nz, qn, qnz;
  std::vector<int> acolcnt, acolidx;
  std::vector<double> acolnzs;
  std::vector<int> qcolcnt, qcolidx;
  std::vector<double> qcolnzs;
  std::vector<double> rhs, obj, lbound, ubound;

  bpmpd_input() {}
  bpmpd_input(int m,
              int n,
              int nz,
              int qn,
              int qnz,
              const std::vector<int>& acolcnt,
              const std::vector<int>& acolidx,
              const std::vector<double>& acolnzs,
              const std::vector<int>& qcolcnt,
              const std::vector<int>& qcolidx,
              const std::vector<double>& qcolnzs,
              const std::vector<double>& rhs,
              const std::vector<double>& obj,
              const std::vector<double>& lbound,
              const std::vector<double>& ubound)
    : m(m)
    , n(n)
    , nz(nz)
    , qn(qn)
    , qnz(qnz)
    , acolcnt(acolcnt)
    , acolidx(acolidx)
    , acolnzs(acolnzs)
    , qcolcnt(qcolcnt)
    , qcolidx(qcolidx)
    , qcolnzs(qcolnzs)
    , rhs(rhs)
    , obj(obj)
    , lbound(lbound)
    , ubound(ubound)
  {
  }
};

const char EXIT_CHAR = 123;
const char CHECK_CHAR = 111;

void ser(int fp, bpmpd_input& bi, SerMode mode)
{
  char scorrect = 'z', s = (mode == SER) ? scorrect : 0;
  ser(fp, s, mode);
  if (s == EXIT_CHAR)
  {
    exit(0);
  }

  ser(fp, bi.m, mode);
  ser(fp, bi.n, mode);
  ser(fp, bi.nz, mode);
  ser(fp, bi.qn, mode);
  ser(fp, bi.qnz, mode);
  ser(fp, bi.acolcnt, mode);
  ser(fp, bi.acolidx, mode);
  ser(fp, bi.acolnzs, mode);
  ser(fp, bi.qcolcnt, mode);
  ser(fp, bi.qcolidx, mode);
  ser(fp, bi.qcolnzs, mode);
  ser(fp, bi.rhs, mode);
  ser(fp, bi.obj, mode);
  ser(fp, bi.lbound, mode);
  ser(fp, bi.ubound, mode);
}

struct bpmpd_output
{
  std::vector<double> primal, dual;
  std::vector<int> status;
  int code;
  double opt;
  bpmpd_output() {}
  bpmpd_output(const std::vector<double>& primal,
               const std::vector<double>& dual,
               const std::vector<int>& status,
               int code,
               double opt)
    : primal(primal), dual(dual), status(status), code(code), opt(opt)
  {
  }
};

void ser(int fp, bpmpd_output& bo, SerMode mode)
{
  char scorrect = CHECK_CHAR, s = (mode == SER) ? scorrect : 0;
  ser(fp, s, mode);
  if (s == EXIT_CHAR)
  {
    exit(0);
  }
  ser(fp, bo.primal, mode);
  ser(fp, bo.dual, mode);
  ser(fp, bo.status, mode);
  ser(fp, bo.code, mode);
  ser(fp, bo.opt, mode);
}
}
