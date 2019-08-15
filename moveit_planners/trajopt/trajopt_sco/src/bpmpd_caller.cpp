#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <errno.h>
#include <iostream>
#include <string.h>
#include <unistd.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sco/bpmpd_io.hpp>
#include <trajopt_utils/stl_to_string.hpp>

extern "C" {
extern void bpmpd(int*,
                  int*,
                  int*,
                  int*,
                  int*,
                  int*,
                  int*,
                  double*,
                  int*,
                  int*,
                  double*,
                  double*,
                  double*,
                  double*,
                  double*,
                  double*,
                  double*,
                  int*,
                  double*,
                  int*,
                  double*,
                  int*);
}

int main(int /*argc*/, char** /*argv*/)
{
  std::string working_dir = BPMPD_WORKING_DIR;
  int err = chdir(working_dir.c_str());
  if (err != 0)
  {
    std::cerr << "error going to BPMPD working dir\n";
    std::cerr << strerror(err) << std::endl;
    abort();
  }
  // int counter=0;
  while (true)
  {
    bpmpd_io::bpmpd_input bi;
    bpmpd_io::ser(STDIN_FILENO, bi, bpmpd_io::DESER);

    int memsiz = 0;
    double BIG = 1e30;
    bpmpd_io::bpmpd_output bo;
    bo.primal.resize(static_cast<unsigned long>(bi.m + bi.n));
    bo.dual.resize(static_cast<unsigned long>(bi.m + bi.n));
    bo.status.resize(static_cast<unsigned long>(bi.m + bi.n));

#define DBG(expr)  // cerr << #expr << ": " << CSTR(expr) << std::endl
    DBG(bi.m);
    DBG(bi.n);
    DBG(bi.nz);
    DBG(bi.qn);
    DBG(bi.qnz);
    DBG(bi.acolcnt);
    DBG(bi.acolidx);
    DBG(bi.acolnzs);
    DBG(bi.qcolcnt);
    DBG(bi.qcolidx);
    DBG(bi.qcolnzs);
    DBG(bi.rhs);
    DBG(bi.obj);
    DBG(bi.lbound);
    DBG(bi.ubound);

    bpmpd(&bi.m,
          &bi.n,
          &bi.nz,
          &bi.qn,
          &bi.qnz,
          bi.acolcnt.data(),
          bi.acolidx.data(),
          bi.acolnzs.data(),
          bi.qcolcnt.data(),
          bi.qcolidx.data(),
          bi.qcolnzs.data(),
          bi.rhs.data(),
          bi.obj.data(),
          bi.lbound.data(),
          bi.ubound.data(),
          bo.primal.data(),
          bo.dual.data(),
          bo.status.data(),
          &BIG,
          &bo.code,
          &bo.opt,
          &memsiz);

    bpmpd_io::ser(STDOUT_FILENO, bo, bpmpd_io::SER);
  }
}
