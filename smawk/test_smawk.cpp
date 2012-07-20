
//#define SMAWK_DEBUG

#include "Smawk.h"
#include "SimpleMatrix.h"
#include "SmawkDebug.h"

#include <ctime>
#include <assert.h>

//////////////////////////////////////////////////////////////////////

typedef SimpleMatrix_t<int> IntMatrix;
typedef IndexedMatrix_t<IntMatrix> IntIndexedMatrix;

//////////////////////////////////////////////////////////////////////

void fillRandomMatrix(IntMatrix& M, int range) {

  M(M.rows()-1, M.cols()-1) = rand() % range + 1;

  IntMatrix A(M, 0, 0, M.rows()-1, M.cols()-1);

  if (!A.empty()) { 
    fillRandomMatrix(A, range);
  }

  IntMatrix C(M, M.rows()-1, 0, 1, M.cols()-1);

  if (!C.empty()) {

    C(0, C.cols()-1) = rand() % range + 1;

    for (size_t j0=C.cols()-2; j0<C.cols(); --j0) {

      int cmin = 1;

      for (size_t i=0; i<A.rows(); ++i) {
        int a = A(i, j0);
        for (size_t j1=j0+1; j1<C.cols(); ++j1) {
          int b = A(i, j1);
          int d = C(0, j1);
          if (a >= b) {
            cmin = std::max(cmin, d+1);
          } else if (a > b) {
            cmin = std::max(cmin, d);
          }
        }
      }

      C(0, j0) = rand() % range + cmin;

    }

  }

  if (M.cols() > 1) {

    size_t j1 = M.cols()-1;
    
    for (size_t i0=M.rows()-2; i0<M.rows(); --i0) {
      int bmin = 1;
      for (size_t i1=i0+1; i1<M.rows(); ++i1) {
        for (size_t j0 = 0; j0<j1; ++j0) {
          int a = M(i0, j0);
          int c = M(i1, j0);
          int d = M(i1, j1);
          if (c < d) {
            bmin = std::max(bmin, a+1);
          } else {
            bmin = std::max(bmin, a);
          }
        }
      }
      M(i0, j1) = rand() % range + bmin;
    }

  }

#ifdef SMAWK_DEBUG
  std::less<int> cmp;
  debugPrintMatrix(M, cmp);
#endif
  
}


//////////////////////////////////////////////////////////////////////

typedef DebugMatrix_t< IntMatrix > DebugIntMatrix;

//////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {

  
  srand(time(NULL));

  size_t rows = 10;
  size_t cols = 16;
  int delay = 0;

  if (argc >= 3) {
    rows = atoi(argv[1]);
    cols = atoi(argv[2]);
  } 
  if (argc >= 4) {
    delay = atoi(argv[3]);
  }

  std::vector<int> data(rows*cols, 0);

  IntMatrix morig(rows, cols, &data[0]);
  std::less<int> cmp;

  Timer ctime;
  fillRandomMatrix(morig, 20);
  std::cout << "construction took " << ctime.elapsed() << " sec.\n\n";

  DebugIntMatrix dm(morig, delay);

  IndexArray bopt, sopt;

  Timer btime;
  dm.count = 0;
  bruteForceSearch(dm, cmp, bopt);
  std::cout << "brute ran " <<  dm.count 
            << " exaoptations total in " << btime.elapsed() << " seconds.\n\n";

  Timer stime;

  dm.count = 0;
  smawk(dm, cmp, sopt);
  std::cout << "smawk ran " << dm.count
            << " evaluations total in " << stime.elapsed() << " seconds.\n\n";

  if (cols <= 16) {
    debugPrintMatrixAndOptima(morig, cmp, sopt);
  }
  
  assert(bopt.size() == sopt.size());
  for (size_t i=0; i<bopt.size(); ++i) {
    assert(morig(i,bopt[i]) == morig(i,sopt[i]));
  }

  return 0;

}
