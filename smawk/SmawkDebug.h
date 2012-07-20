#ifndef _SMAWKDEBUG_H_
#define _SMAWKDEBUG_H_

#include "IndexedMatrix.h"
#include "SimpleMatrix.h"
#include <sys/time.h>

template <class Tmat, class Tcmp>
inline void checkMonotone(const Tmat& m, Tcmp cmp) {

  typedef typename Tmat::value_type Tval;
  
  for (size_t r0=0; r0<m.rows(); ++r0) {

    for (size_t c0=0; c0<m.cols(); ++c0) {

      Tval a = m(r0,c0);

      for (size_t r1=r0+1; r1<m.rows(); ++r1) {

        Tval c = m(r1,c0);

        for (size_t c1=c0+1; c1<m.cols(); ++c1) {

          Tval b = m(r0,c1);
          Tval d = m(r1,c1);
          
          bool ok = true;

          if (cmp(c,d)) {
            if (!cmp(a,b)) {
              ok = false;
            }
          } else if (!cmp(d,c)) {
            if (cmp(b,a)) {
              ok = false;
            }
          }

          if (!ok) { 
            std::cerr << "a at (" << r0 << ", " << c0 << ") = " << a << "; ";
            std::cerr << "b at (" << r0 << ", " << c1 << ") = " << b << "\n";
            std::cerr << "c at (" << r1 << ", " << c0 << ") = " << c << "; ";
            std::cerr << "d at (" << r1 << ", " << c1 << ") = " << d << "\n";
            exit(1);
          }

        }
      }

    }
  }

  std::cout << m.rows() << "x" << m.cols() << " matrix is monotone!\n";

};

template <class Tmat, class Tcmp>
inline void debugPrintMatrix(const Tmat& mat,
                             Tcmp cmp) {
  std::cout << pmat(mat) << "\n\n";
  checkMonotone(mat, cmp);
  std::cout << "\n";
}

template <class Tmat, class Tcmp>
inline void debugPrintMatrixAndOptima(const Tmat& mat, 
                                      Tcmp cmp,
                                      const IndexArray& optima) {

  debugPrintMatrix(mat, cmp);

  assert(optima.size() == mat.rows());
  for (size_t i=0; i<optima.size(); ++i) {
    std::cout << "optimum of row " << i+1 << " is at column " << optima[i]+1 << " with value " << mat(i, optima[i]) << "\n";
    for (size_t j=0; j<mat.cols(); ++j) {
      assert( !cmp(mat(i, j), mat(i, optima[i])) );
    }
  }
  std::cout << "\n";

}

//////////////////////////////////////////////////////////////////////

template <class Tmat, class Tcmp>
inline void bruteForceSearch(const Tmat& m,
                             Tcmp cmp,
                             IndexArray& optima) {
  
  optima.resize(m.rows());

  for (size_t i=0; i<m.rows(); ++i) {
    optima[i] = 0;
    int optval = m(i,0); 
    for (size_t j=1; j<m.cols(); ++j) {
      int val = m(i,j);
      if (cmp(val, optval)) { 
        optima[i] = j;
        optval = val;
      }
    }
  }

}

//////////////////////////////////////////////////////////////////////

class Timer {
private:
  double _start;

public:

  static double getTimeAsDouble() {
    struct timeval tp;
    gettimeofday(&tp, NULL);
    return tp.tv_sec * 1.0 + tp.tv_usec * 1e-6;
  }

  Timer(): _start(getTimeAsDouble()) {}

  double elapsed() const {
    return getTimeAsDouble() - _start;
  }

};

//////////////////////////////////////////////////////////////////////

template <class Tmat>
class DebugMatrix_t {
private:
  
  Tmat& _m;
  const int _u;

public:

  mutable size_t count;

  typedef typename Tmat::value_type value_type;

  DebugMatrix_t(Tmat& mat, int usec): _m(mat), _u(usec), count(0) {}

  size_t rows() const { return _m.rows(); }
  size_t cols() const { return _m.cols(); }
  
  value_type operator()(size_t r, size_t c) const {
    if (_u) { usleep(_u); }
    ++count;
    return _m(r,c);
  }

};

#endif
