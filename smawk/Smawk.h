#ifndef _SMAWK_H_
#define _SMAWK_H_

#include "IndexedMatrix.h"
#include "SmawkDebug.h"
#include <assert.h>


#ifdef SMAWK_DEBUG
enum { smawk_verbose = 1 };
#else
enum { smawk_verbose = 0 };
#endif

#define smawk_debug if (smawk_verbose) std::cout

#ifdef SMAWK_REDUCE_DEBUG
enum { smawk_reduce_verbose = 1 };
#else
enum { smawk_reduce_verbose = 0 };
#endif

#define smawk_reduce_debug if (smawk_reduce_verbose) std::cout


//////////////////////////////////////////////////////////////////////

template <class Tmat, class Tcmp>
inline void bruteForceSearch(const Tmat& m,
                             Tcmp cmp,
                             IndexArray& optima) {

  typedef typename Tmat::value_type Tval;
  
  optima.resize(m.rows());

  for (size_t i=0; i<m.rows(); ++i) {
    optima[i] = 0;
    Tval optval = m(i,0); 
    for (size_t j=1; j<m.cols(); ++j) {
      Tval val = m(i,j);
      if (cmp(val, optval)) { 
        optima[i] = j;
        optval = val;
      }
    }
  }

}

template <class Tmat, class Tcmp>
inline void ascendingSearch(const Tmat& m,
                            Tcmp cmp,
                            IndexArray& optima) {

  typedef typename Tmat::value_type Tval;
  
  optima.resize(m.rows());

  size_t p = 0;

  for (size_t i=0; i<m.rows(); ++i) {
    Tval optval = m(i,p); 
    for (size_t j=p+1; j<m.cols(); ++j) {
      Tval val = m(i,j);
      if (cmp(val, optval)) { 
        p = j;
        optval = val;
      }
    }
    optima[i] = p;
  }

}

//////////////////////////////////////////////////////////////////////

template <class Tmat, class Tcmp>
void divideAndConquer_r(const Tmat& m,
                        Tcmp cmp,
                        IndexArray& rowoptima,
                        size_t i0, size_t i1,
                        size_t j0, size_t j1) {

  typedef typename Tmat::value_type Tval;

  size_t i = i0 + (i1-i0)/2;

  Tval optval = m(i, j0);
  rowoptima[i] = j0;

  for (size_t j=j0+1; j<j1; ++j) {
    Tval val = m(i, j);
    if (cmp(val, optval)) {
      optval = val;
      rowoptima[i] = j;
    }
  }

  if (i > i0) {
    divideAndConquer_r(m, cmp, rowoptima,
                       i0, i,
                       j0, rowoptima[i]+1);
  } 

  if (i1 > i+1) {
    divideAndConquer_r(m, cmp, rowoptima,
                       i+1, i1,
                       rowoptima[i], j1);
  }


}

template <class Tmat, class Tcmp>
void divideAndConquer(const Tmat& m,
                      Tcmp cmp,
                      IndexArray& rowoptima) {

  rowoptima.resize(m.rows());
  divideAndConquer_r(m, cmp, rowoptima, 
                     0, m.rows(), 0, m.cols());

}

template <class Tmat, class Tcmp>
void reduce(IndexedMatrix_t<Tmat>& mat, 
            Tcmp cmp,
            size_t* keptcols) {

  size_t n = mat.rows();
  size_t m = mat.cols();

  size_t j1 = 0;
  size_t j2 = 1;

  keptcols[0] = 0;

  while (j2 < m) {
    
    if (smawk_reduce_verbose) {
      for (size_t i=0; i<n; ++i) {
        smawk_reduce_debug << "[ ";
        for (size_t j=0; j<=j1; ++j) {
          char c = (i == j1 && j == j1) ? '*' : ' ';
          smawk_reduce_debug << c << std::setw(4) << mat(i, j) << c << " ";
        }
        smawk_reduce_debug << " | ";
        for (size_t j=j2; j<m; ++j) {
          smawk_reduce_debug << std::setw(4) << mat(i, j) << " ";
        }
        smawk_reduce_debug << "]\n";
      }
      smawk_reduce_debug << "C" << j2+1 << " with value " << mat(j1,j2) << " challenges C" << keptcols[j1]+1 << " with value " << mat(j1,j1) << "\n";
    }


    if ( cmp( mat(j1, j1), mat(j1, j2)) ) {

      smawk_reduce_debug << "C" << j2+1 << " is defeated";

      if (j1 == n-1) {
        smawk_reduce_debug << " and eliminated.\n";
        ++j2;
      } else {
        smawk_reduce_debug << " but survives.\n";
        mat.replaceColumn(++j1, j2);
        keptcols[j1] = j2++;
      }
        
    } else {

      smawk_reduce_debug << "C" << keptcols[j1]+1 << " is eliminated\n";

      if (j1) {

        smawk_reduce_debug << "popping stack\n";
        --j1;

      } else {

        smawk_reduce_debug << "placing C" << j1+1 << " onto emtpy stack.\n";

        mat.replaceColumn(j1,j2);
        keptcols[j1] = j2++;

      }

    }

  }

  if (smawk_reduce_verbose) {
    assert(j1 == n-1);
  }

  mat.makeSquare();


}





template <class Tmat, class Tcmp>
void smawk_r(IndexedMatrix_t<Tmat>& wmat,
             Tcmp cmp,
             IndexArray& rowoptima) {

#ifdef SMAWK_DEBUG
  smawk_debug << "at top:\n";
  debugPrintMatrix(wmat, cmp);
#endif

  typedef typename Tmat::value_type Tval;

  size_t orig_cols = wmat.cols();
  size_t orig_rows = wmat.rows();

  IndexedMatrix_t<Tmat> mat(wmat);
  
  size_t* keptcols = 0;

  if (orig_cols > orig_rows) {

    keptcols = new size_t[orig_rows];

    reduce(mat, cmp, keptcols);

#ifdef SMAWK_DEBUG
    smawk_debug << "after col reduce:\n";
    debugPrintMatrix(mat, cmp);
#endif

  }

  if (orig_rows == 1) {

    rowoptima = IndexArray(1, 0);

  } else if (0) {

    //divideAndConquer(mat, cmp, rowoptima);
    //bruteForceSearch(mat, cmp, rowoptima);
    //ascendingSearch(mat, cmp, rowoptima);

  } else {

    mat.decimateRows();           

    smawk_r(mat, cmp, rowoptima);

    mat.restoreRows();
    rowoptima.resize(orig_rows);

    for (size_t i=orig_rows-1; i<orig_rows; --i) {

      if (i % 2) {

        rowoptima[i] = rowoptima[i/2];

      } else {

        size_t i0 = i/2-1;
        size_t i1 = i+1;
        size_t j0 = (i0 < orig_rows/2) ? rowoptima[i0] : 0;
        size_t j1 = (i1 < orig_rows) ? rowoptima[i1]+1 : mat.cols();

        size_t optidx = j0;
        Tval optval = mat(i, j0);

        for (size_t j=j0+1; j<j1; ++j) {
          Tval val = mat(i, j);
          if (cmp(val, optval)) {
            optidx = j;
            optval = val;
          }
        }

        rowoptima[i] = optidx;

      }

    }

#ifdef SMAWK_DEBUG
    smawk_debug << "after rows restored:\n";
    debugPrintMatrixAndOptima(mat, cmp, rowoptima);
#endif

  }

#ifdef SMAWK_DEBUG
  assert(rowoptima.size() == mat.rows());
#endif

  if (keptcols) {
    
    for (size_t i=0; i<mat.cols(); ++i) {
      rowoptima[i] = keptcols[rowoptima[i]];
    }

    delete[] keptcols;

#ifdef SMAWK_DEBUG
    smawk_debug << "after columns restored:\n";
    debugPrintMatrixAndOptima(wmat, cmp, rowoptima);
#endif

  }


}

template <class Tmat, class Tcmp>
void smawk(const Tmat& m,
           Tcmp cmp,
           IndexArray& rowoptima) {

  IndexedMatrix_t<Tmat> mat(m);
  
  smawk_r(mat, cmp, rowoptima);

}





#endif
