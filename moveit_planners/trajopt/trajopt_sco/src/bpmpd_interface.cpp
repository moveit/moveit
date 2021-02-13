#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <cmath>
#include <fstream>
#include <signal.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sco/bpmpd_interface.hpp>
#include <trajopt_sco/bpmpd_io.hpp>
#include <trajopt_utils/logging.hpp>
#include <trajopt_utils/stl_to_string.hpp>

/**
This is a readme file for the linux DLL version of BPMPD version 2.21
The DLL solves linear and convex quadratic problems.


The objective function is assumed to be in the form:

                1
   min    c'x + - x'Qx
                2


The interface (Fortran-like) for BPMPD:

    subroutine bpmpd(m,n,nz,qn,qnz,
               acolcnt,acolidx,acolnzs,
               qcolcnt,qcolidx,qcolnzs,
               rhs,obj,lbound,ubound,primal,dual,status,
               big,code,opt,memsiz)

integer*4 m,n,nz,qn,qnz,acolcnt(n),acolidx(nz),qcolcnt(n),qcolidx(qnz),
          status(n+m),code,
real*8    acolnzs(nz),qcolnzs(qnz),rhs(m),obj(n),lbound(n+m),ubound(n+m),
          primal(n+m),dual(n+m),big,opt


or C-like:

void bpmpd(int *m, int *n, int *nz, int *qn, int *qnz, int *acolcnt,
          int *acolidx, double *acolnzs, int *qcolcnt, int *qcolidx,
          double *qcolnzs, double *rhs, double *obj, double *lbound,
          double *ubound, double *primal, double *dual, int *status,
          double *big, int *code, double *opt, int *memsiz)

where

  memsiz : Number of bytes to be allocated by bpmpd. If memsiz <= 0
           bpmpd determines the allocatable memory itself. As output
           memsiz contains the memory in bytes which was used during
           the process.

       m : number of rows in the constraint matrix
       n : number of columns in the constraint matrix
      nz : number of nonzeros in the constraint matrix
      qn : number of quadratic variables
     qnz : number of nonzeros in the lower triangular of Q (with diagonals)

 acolcnt : number of nonzeros in each column of A
 acolidx : A matrix index values
 acolnzs : A matrix nonzero values
           It is supposed that the columns are placed continuousely from
           the first positions of the parallel arrays acolidx-acolnzs.

 qcolcnt : number of nonzeros in each column of the lower triangular part of Q
 qcolidx : Q matrix index values
 qcolnzs : Q matrix nonzero values
           It is supposed that the the lower triangular part (with diagonal)
           of Q is provided and its columns are placed continuousely from
           the first positions of the parallel arrays qcolidx-qcolnzs.

     rhs : right hand side
     obj : objective function
  lbound :    1 ..   n lower bounds of the variables
            n+1 .. n+m lower bounds on rows (slacks)
  ubound : upper bounds (like lbound)

  primal : optimal primal values
  dual   : optimal dual values
  status : basic/nonbasic status estimation of variables (1/0)

     big : Represents infinity (recommended big = 1.0d+30 )
    code : return code
           if code < 0 --> not enought memory
           if code = 1 --> solver stopped at feasible point (suboptimal
solution)
           if code = 2 --> optimal solution found
           if code = 3 --> problem dual infeasible
           if code = 4 --> problem primal infeasible

    opt : final primal objective value

The DLL will read the parameter file called bpmpd.par (if presented) and
write a log file called bpmpd.log.


  As example the LP problem :

                min x1 + x1*x1 + 2*x1*x2 + 2*x2*x2 + x4*x4

              1x1  + 2x2 + 0x3 -4x4 >=   0
              3x1  + 0x2 - 2x3 -1x4 <= 100
       30 >=  1x1  + 3x2 + 3x3 -2x4 >=  10

                  x1,x2,x3 >=0
                        x1 <= 20

  Then the input parameters:

  m=3, n=4, nz=10, qn=3, qnz=4

  ubound :  (20, big, big,  big,    big,    0,  20)
  lbound :  ( 0,   0,   0, -big,      0, -big,   0)
  rhs    :  ( 0, 100,  10)
  obj    :  ( 1,   0,   0,    0)

acolcnt  :  ( 3,         2,      2,      3)
acolidx  :  ( 1, 2, 3,   1, 3,   2, 3,   1, 2, 3)
acolnzs  :  ( 1, 3, 1,   2, 3,  -2, 3,  -4,-1,-2)

qcolcnt  :  ( 2,         1,      0,      1)
qcolidx  :  ( 1,2,       2,              4)
qcolnzs  :  ( 2,2,       4,              2)

                                      1        1  1  0         2  2  0
(Note : 2*x1*x2 = x1*x2+x2*x1   thus  - * Q =  1  2  0 and Q = 2  4  0 )
                                      2        0  0  1         0  0  2

NOTE:
The subroutine solves minimization problems, therefore you
have to change the sign of the objective, if you have a maximization
problem.

NOTE:
The DLL routine will create a log file called "bpmpd.log" at every call.
The old bpmpd.log file will be overwritten.

NOTE:
A sample driver solving the above problem is included as well as the
logfile it generates.
  **/

// extern "C" {
//   extern void bpmpd(int *, int *, int *, int *, int *, int *, int *,
//          double *, int *, int *, double *, double *, double *, double *,
//          double *, double *, double *, int *, double *, int *, double *, int
//          *);
// }

namespace sco
{
static double BPMPD_BIG = 1e+30;

ModelPtr createBPMPDModel()
{
  ModelPtr out(new BPMPDModel());
  return out;
}

#define READ 0
#define WRITE 1

pid_t popen2(const char* command, int* infp, int* outfp)
{
  int p_stdin[2], p_stdout[2];
  pid_t pid;

  if (pipe(p_stdin) != 0 || pipe(p_stdout) != 0)
    return -1;

  pid = fork();

  if (pid < 0)
  {
    assert(0);
    return pid;
  }
  else if (pid == 0)
  {
    close(p_stdin[WRITE]);
    dup2(p_stdin[READ], READ);
    close(p_stdout[READ]);
    dup2(p_stdout[WRITE], WRITE);

    execl("/bin/sh", "sh", "-c", command, nullptr);
    perror("execl");
    exit(1);
  }

  if (infp == nullptr)
    close(p_stdin[WRITE]);
  else
    *infp = p_stdin[WRITE];

  if (outfp == nullptr)
    close(p_stdout[READ]);
  else
    *outfp = p_stdout[READ];

  return pid;
}

static pid_t gPID = 0;
static int gPipeIn = 0;
static int gPipeOut = 0;

void fexit()
{
  char text[1] = { bpmpd_io::EXIT_CHAR };
  long n = write(gPipeIn, text, 1);
  ALWAYS_ASSERT(n == 1);
}

BPMPDModel::BPMPDModel() : m_pipeIn(0), m_pipeOut(0)
{
  if (gPID == 0)
  {
    atexit(fexit);
    gPID = popen2(BPMPD_CALLER, &gPipeIn, &gPipeOut);
  }
}

BPMPDModel::~BPMPDModel()
{
  // char text[1] = {123};
  // write(gPipeIn, text, 1);
  // // kill(m_pid, SIGKILL); // TODO: WHY DOES THIS KILL THE PARENT PROCESS?!
  // close(m_pipeIn);
  // close(m_pipeOut);
  // m_pipeIn=0;
  // m_pipeOut=0;
}

Var BPMPDModel::addVar(const std::string& name)
{
  m_vars.push_back(new VarRep(static_cast<int>(m_vars.size()), name, this));
  m_lbs.push_back(-BPMPD_BIG);
  m_ubs.push_back(BPMPD_BIG);
  return m_vars.back();
}
Cnt BPMPDModel::addEqCnt(const AffExpr& expr, const std::string& /*name*/)
{
  m_cnts.push_back(new CntRep(static_cast<int>(m_cnts.size()), this));
  m_cntExprs.push_back(expr);
  m_cntTypes.push_back(EQ);
  return m_cnts.back();
}
Cnt BPMPDModel::addIneqCnt(const AffExpr& expr, const std::string& /*name*/)
{
  m_cnts.push_back(new CntRep(static_cast<int>(m_cnts.size()), this));
  m_cntExprs.push_back(expr);
  m_cntTypes.push_back(INEQ);
  return m_cnts.back();
}
Cnt BPMPDModel::addIneqCnt(const QuadExpr&, const std::string& /*name*/)
{
  assert(0 && "NOT IMPLEMENTED");
  return nullptr;
}
void BPMPDModel::removeVars(const VarVector& vars)
{
  IntVec inds = vars2inds(vars);
  for (unsigned i = 0; i < vars.size(); ++i)
    vars[i].var_rep->removed = true;
}

void BPMPDModel::removeCnts(const CntVector& cnts)
{
  IntVec inds = cnts2inds(cnts);
  for (unsigned i = 0; i < cnts.size(); ++i)
    cnts[i].cnt_rep->removed = true;
}

void BPMPDModel::update()
{
  {
    size_t inew = 0;
    for (unsigned iold = 0; iold < m_vars.size(); ++iold)
    {
      const Var& var = m_vars[iold];
      if (!var.var_rep->removed)
      {
        m_vars[inew] = var;
        m_lbs[inew] = m_lbs[iold];
        m_ubs[inew] = m_ubs[iold];
        var.var_rep->index = static_cast<int>(inew);
        ++inew;
      }
      else
        delete var.var_rep;
    }
    m_vars.resize(inew);
    m_lbs.resize(inew);
    m_ubs.resize(inew);
  }
  {
    size_t inew = 0;
    for (unsigned iold = 0; iold < m_cnts.size(); ++iold)
    {
      const Cnt& cnt = m_cnts[iold];
      if (!cnt.cnt_rep->removed)
      {
        m_cnts[inew] = cnt;
        m_cntExprs[inew] = m_cntExprs[iold];
        m_cntTypes[inew] = m_cntTypes[iold];
        cnt.cnt_rep->index = static_cast<int>(inew);
        ++inew;
      }
      else
        delete cnt.cnt_rep;
    }
    m_cnts.resize(inew);
    m_cntExprs.resize(inew);
    m_cntTypes.resize(inew);
  }
}

void BPMPDModel::setVarBounds(const VarVector& vars, const DblVec& lower, const DblVec& upper)
{
  for (unsigned i = 0; i < vars.size(); ++i)
  {
    size_t varind = static_cast<size_t>(vars[i].var_rep->index);
    m_lbs[varind] = lower[i];
    m_ubs[varind] = upper[i];
  }
}
DblVec BPMPDModel::getVarValues(const VarVector& vars) const
{
  DblVec out(vars.size());
  for (unsigned i = 0; i < vars.size(); ++i)
  {
    size_t varind = static_cast<size_t>(vars[i].var_rep->index);
    out[i] = m_soln[varind];
  }
  return out;
}

#define DBG(expr)  // cout << #expr << ": " << CSTR(expr) << std::endl

CvxOptStatus BPMPDModel::optimize()
{
  update();
  //
  //
  // int    m, n, nz, qn, qnz, acolcnt[maxn+1], acolidx[maxnz], qcolcnt[maxn+1],
  // qcolidx[maxqnz],
  //        status[maxn+maxm], code, memsiz;
  // double acolnzs[maxnz], qcolnzs[maxqnz], rhs[maxm], obj[maxn],
  // lbound[maxn+maxm],
  //        ubound[maxn+maxm], primal[maxn+maxm], dual[maxn+maxm], big, opt;

  size_t n = m_vars.size();
  size_t m = m_cnts.size();

  IntVec acolcnt(n), acolidx, qcolcnt(n), qcolidx, status(m + n);
  DblVec acolnzs, qcolnzs, rhs(m), obj(n, 0), lbound(m + n), ubound(m + n), primal(m + n), dual(m + n);

  DBG(m_lbs);
  DBG(m_ubs);
  for (size_t iVar = 0; iVar < n; ++iVar)
  {
    lbound[iVar] = fmax(m_lbs[iVar], -BPMPD_BIG);
    ubound[iVar] = fmin(m_ubs[iVar], BPMPD_BIG);
  }

  std::vector<IntVec> var2cntinds(n);
  std::vector<DblVec> var2cntvals(n);
  for (size_t iCnt = 0; iCnt < m; ++iCnt)
  {
    const AffExpr& aff = m_cntExprs[iCnt];
    // cout << "adding constraint " << aff << endl;
    IntVec inds = vars2inds(aff.vars);

    for (size_t i = 0; i < aff.vars.size(); ++i)
    {
      var2cntinds[static_cast<size_t>(inds[i])].push_back(static_cast<int>(iCnt));
      var2cntvals[static_cast<size_t>(inds[i])].push_back(aff.coeffs[i]);  // xxx maybe repeated/
    }

    lbound[n + iCnt] = (m_cntTypes[iCnt] == INEQ) ? -BPMPD_BIG : 0;
    ubound[n + iCnt] = 0;
    rhs[iCnt] = -aff.constant;
  }

  for (size_t iVar = 0; iVar < n; ++iVar)
  {
    simplify2(var2cntinds[iVar], var2cntvals[iVar]);
    acolcnt[iVar] = static_cast<int>(var2cntinds[iVar].size());
    acolidx.insert(acolidx.end(), var2cntinds[iVar].begin(), var2cntinds[iVar].end());
    acolnzs.insert(acolnzs.end(), var2cntvals[iVar].begin(), var2cntvals[iVar].end());
  }
  // cout << CSTR(acolidx) << endl;
  // cout << CSTR(acolnzs) << endl;

  std::vector<DblVec> var2qcoeffs(n);
  std::vector<IntVec> var2qinds(n);
  for (size_t i = 0; i < m_objective.size(); ++i)
  {
    size_t idx1 = static_cast<size_t>(m_objective.vars1[i].var_rep->index);
    size_t idx2 = static_cast<size_t>(m_objective.vars2[i].var_rep->index);
    if (idx1 < idx2)
    {
      var2qinds[idx1].push_back(static_cast<int>(idx2));
      var2qcoeffs[idx1].push_back(m_objective.coeffs[i]);
    }
    else if (idx1 == idx2)
    {
      var2qinds[idx1].push_back(static_cast<int>(idx2));
      var2qcoeffs[idx1].push_back(m_objective.coeffs[i] * 2);
    }
    else
    {
      var2qinds[idx2].push_back(static_cast<int>(idx1));
      var2qcoeffs[idx2].push_back(m_objective.coeffs[i]);
    }
  }

  for (size_t iVar = 0; iVar < n; ++iVar)
  {
    simplify2(var2qinds[iVar], var2qcoeffs[iVar]);
    qcolidx.insert(qcolidx.end(), var2qinds[iVar].begin(), var2qinds[iVar].end());
    qcolnzs.insert(qcolnzs.end(), var2qcoeffs[iVar].begin(), var2qcoeffs[iVar].end());
    qcolcnt[iVar] = static_cast<int>(var2qinds[iVar].size());
  }

  for (size_t i = 0; i < m_objective.affexpr.size(); ++i)
  {
    obj[static_cast<size_t>(m_objective.affexpr.vars[i].var_rep->index)] += m_objective.affexpr.coeffs[i];
  }

#define VECINC(vec)                                                                                                    \
  for (unsigned i = 0; i < vec.size(); ++i)                                                                            \
    ++vec[i];
  VECINC(acolidx);
  VECINC(qcolidx);
#undef VECINC

  // cout << "objective: " << m_objective << endl;

  int nz = static_cast<int>(acolnzs.size()), qn = static_cast<int>(n), qnz = static_cast<int>(qcolnzs.size());

  DBG(m);
  DBG(n);
  DBG(nz);
  DBG(qn);
  DBG(qnz);
  DBG(acolcnt);
  DBG(acolidx);
  DBG(acolnzs);
  DBG(qcolcnt);
  DBG(qcolidx);
  DBG(qcolnzs);
  DBG(rhs);
  DBG(obj);
  DBG(lbound);
  DBG(ubound);

#if 0
  bpmpd(&m, &n, &nz, &qn, &qnz, acolcnt.data(), acolidx.data(), acolnzs.data(), qcolcnt.data(), qcolidx.data(), qcolnzs.data(),
      rhs.data(), obj.data(), lbound.data(), ubound.data(), 
      primal.data(), dual.data(), status.data(), &BIG, &code, &opt, &memsiz);

  // opt += m_objective.affexpr.constant;
  m_soln = DblVec(primal.begin(), primal.begin()+n);


  if (1) {
    DBG(primal);
    DBG(dual);
    DBG(status);
    DBG(code);
    DBG(opt);
#undef DBG

#else

  bpmpd_io::bpmpd_input bi(static_cast<int>(m),
                           static_cast<int>(n),
                           nz,
                           qn,
                           qnz,
                           acolcnt,
                           acolidx,
                           acolnzs,
                           qcolcnt,
                           qcolidx,
                           qcolnzs,
                           rhs,
                           obj,
                           lbound,
                           ubound);
  bpmpd_io::ser(gPipeIn, bi, bpmpd_io::SER);

  // std::cout << "serialization time:" << end-start << std::endl;

  bpmpd_io::bpmpd_output bo;
  bpmpd_io::ser(gPipeOut, bo, bpmpd_io::DESER);

  m_soln = DblVec(bo.primal.begin(), bo.primal.begin() + static_cast<long int>(n));
  int retcode = bo.code;

  if (retcode == 2)
    return CVX_SOLVED;
  else if (retcode == 3 || retcode == 4)
    return CVX_INFEASIBLE;
  else
    return CVX_FAILED;

#endif

  // exit(0);
}
void BPMPDModel::setObjective(const AffExpr& expr) { m_objective.affexpr = expr; }
void BPMPDModel::setObjective(const QuadExpr& expr) { m_objective = expr; }
void BPMPDModel::writeToFile(const std::string& /*fname*/)
{
  // assert(0 && "NOT IMPLEMENTED");
}
VarVector BPMPDModel::getVars() const { return m_vars; }
}
