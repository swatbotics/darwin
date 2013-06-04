
//#define SMAWK_DEBUG
//#define SMAWK_REDUCE_DEBUG

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


enum { 
  test_rows = 9,
  test_cols = 18
};

int test_data[test_rows*test_cols] = {
   25, 21, 13, 10, 20, 13, 19, 35, 37, 41, 58, 66, 82, 99, 124, 133, 156, 178,
   42, 35, 26, 20, 29, 21, 25, 37, 36, 39, 56, 64, 76, 91, 116, 125, 146, 164, 
   57, 48, 35, 28, 33, 24, 28, 40, 37, 37, 54, 61, 72, 83, 107, 113, 131, 146,
   78, 65, 51, 42, 44, 35, 38, 48, 42, 42, 55, 61, 70, 80, 100, 106, 120, 135,
   90, 76, 58, 48, 49, 39, 42, 48, 39, 35, 47, 51, 56, 63, 80, 86, 97, 110, 
   103, 85, 67, 56, 55, 44, 44, 49, 39, 33, 41, 44, 49, 56, 71, 75, 84, 96,
   123, 105, 86, 75, 73, 59, 57, 62, 51, 44, 50, 52, 55, 59, 72, 74, 80, 92,
   142, 123, 100, 86, 82, 65, 61, 62, 50, 43, 47, 45, 46, 46, 58, 59, 65, 73, 
   151, 130, 104, 88, 80, 59, 52, 49, 37, 29, 29, 24, 23, 20, 28, 25, 31, 39
};

int main(int argc, char** argv) {

  
  srand(time(NULL));
  std::less<int> cmp;

  size_t rows = test_rows;
  size_t cols = test_cols;
  bool random = false;
  int delay = 0;

  if (argc >= 3) {
    rows = atoi(argv[1]);
    cols = atoi(argv[2]);
    random = true;
  } 
  if (argc >= 4) {
    delay = atoi(argv[3]);
  }

  std::vector<int> data(rows*cols, 0);

  IntMatrix morig(rows, cols, random ? &data[0] : test_data);

  if (random) {
    Timer ctime;
    fillRandomMatrix(morig, 20);
    ctime.stop();
    std::cout << "construction took " << ctime.elapsed() << " sec.\n\n";
  }

  DebugIntMatrix dm(morig, delay);

  IndexArray bopt, sopt, dopt;

  dm.count = 0;
  Timer btime;
  bruteForceSearch(dm, cmp, bopt);
  btime.stop();
  std::cout << "brute ran " <<  dm.count 
            << " evaluations total in " << btime.elapsed() << " seconds.\n\n";


  dm.count = 0;
  Timer stime;
  smawk(dm, cmp, sopt);
  stime.stop();
  std::cout << "smawk ran " << dm.count
            << " evaluations total in " << stime.elapsed() << " seconds.\n\n";

  dm.count = 0;
  Timer dtime;
  divideAndConquer(dm, cmp, dopt);
  dtime.stop();
  std::cout << "dandc ran " << dm.count
            << " evaluations total in " << dtime.elapsed() << "seconds.\n\n";

  if (cols <= test_cols) {
    debugPrintMatrixAndOptima(morig, cmp, sopt);
  }
  
  assert(bopt.size() == sopt.size());
  assert(dopt.size() == sopt.size());
  for (size_t i=0; i<bopt.size(); ++i) {
    assert(morig(i,bopt[i]) == morig(i,sopt[i]));
    assert(morig(i,bopt[i]) == morig(i,dopt[i]));
  }

  return 0;

}
