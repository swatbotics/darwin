#ifndef _SMAWK_H_
#define _SMAWK_H_

#include "IndexedMatrix.h"
#include "SmawkDebug.h"
#include <assert.h>

#define SMAWK_USE_PTR

#ifdef SMAWK_DEBUG
#define smawk_debug std::cout
#else
#define smawk_debug if (0) std::cout
#endif

//////////////////////////////////////////////////////////////////////


template <class Tmat, class Tcmp>
void reduce(IndexedMatrix_t<Tmat>& mat, 
            Tcmp cmp,
#ifdef SMAWK_USE_PTR
            size_t* keptcols) {
#else
            IndexArray& keptcols) {
#endif

  typedef typename Tmat::value_type Tval;


  size_t n = mat.rows();

  size_t m = mat.cols();
  size_t j2 = 0;

#ifdef SMAWK_USE_PTR
  size_t ss = 0;
#else
  keptcols.clear();
#endif

  while (j2 < m) {

#ifdef SMAWK_USE_PTR
    if (ss == 0) { 
#else
    if (keptcols.empty()) {
#endif

#ifdef SMAWK_REDUCE_DEBUG
      smawk_debug << "pushing C" << j2+1 << " onto empty stack.\n";
#endif

#ifdef SMAWK_USE_PTR
      keptcols[ss++] = j2;
#else
      keptcols.push_back(j2);
#endif
      ++j2;

    } 

    bool defeated = false;

#ifdef SMAWK_USE_PTR    
    while (!defeated && ss && j2 < m) {
#else
    while (!defeated && !keptcols.empty() && j2 < m) {
#endif

#ifdef SMAWK_USE_PTR
      size_t i = ss-1;
#else
      size_t i = keptcols.size()-1;
#endif
      size_t j1 = keptcols[i];

#ifdef SMAWK_REDUCE_DEBUG

      smawk_debug << "\n";

      for (size_t ii=0; ii<n; ++ii) {
        smawk_debug << "[ ";
#ifdef SMAWK_USE_PTR
        for (size_t j=0; j<ss; ++j) {
#else
        for (size_t j=0; j<keptcols.size(); ++j) {
#endif
          char c = (j == ii) ? '*' : ' ';
          smawk_debug << c << std::setw(4) << mat(ii, keptcols[j]) << c << " ";
        }
        smawk_debug << " | ";
        for (size_t j=j2; j<mat.cols(); ++j) {
          smawk_debug << std::setw(4) << mat(ii, j) << " ";
        }
        smawk_debug << "]\n";
      }

      smawk_debug << "we are at row " << i+1 << "\n";
      smawk_debug << "C" << j2+1 << " with value " << mat(i,j2) << " challenges C" << j1+1 << " with value " << mat(i,j1) << "\n";

#endif


      if ( cmp( mat(i,j1), mat(i,j2) ) ) {

#ifdef SMAWK_REDUCE_DEBUG          
        smawk_debug << "C" << j2+1 << " is defeated";
#endif
        defeated = true;

        if (i == n-1) {
#ifdef SMAWK_REDUCE_DEBUG          
          smawk_debug << " and eliminated.\n";
#endif
          ++j2;
        } else {
#ifdef SMAWK_REDUCE_DEBUG
          smawk_debug << " but survives.\n";
#endif
#ifdef SMAWK_USE_PTR
          keptcols[ss++] = j2;
#else
          keptcols.push_back(j2);
#endif
          ++j2;
        }

#ifdef SMAWK_REDUCE_DEBUG
        smawk_debug << "\n";
#endif

      } else {

#ifdef SMAWK_REDUCE_DEBUG
        smawk_debug << "C" << j1+1 << " is eliminated.\n";
#endif

#ifdef SMAWK_USE_PTR        
        --ss;
#else
        keptcols.pop_back();
#endif
          
      }

    }

      
  }

#ifdef SMAWK_USE_PTR        
  assert(ss == n);
#else
  assert(keptcols.size() == n);
#endif

  for (size_t i=0; i<n; ++i) {
    mat.replaceColumn(i, keptcols[i]);
  }

  mat.makeSquare();

}





template <class Tmat, class Tcmp>
void smawk_r(IndexedMatrix_t<Tmat>& wmat,
             Tcmp cmp,
             IndexArray& rowoptima,
             IndexArray& work) {

#ifdef SMAWK_DEBUG
  smawk_debug << "at top:\n";
  debugPrintMatrix(wmat, cmp);
#endif

  typedef typename Tmat::value_type Tval;

  size_t orig_cols = wmat.cols();

  IndexedMatrix_t<Tmat> mat(wmat);
  

#ifdef SMAWK_USE_PTR
  size_t* keptcols = 0;
#else
  IndexArray keptcols;
#endif

  if (orig_cols > mat.rows()) {

#ifdef SMAWK_USE_PTR
    keptcols = new size_t[mat.rows()];
#endif

    reduce(mat, cmp, keptcols);

#ifdef SMAWK_DEBUG
    smawk_debug << "after col reduce:\n";
    debugPrintMatrix(mat, cmp);
#endif

  }

  if (mat.rows() == 1) {

    rowoptima.clear();
    rowoptima.push_back(0);

  } else {

    size_t orig_rows = mat.rows();

    mat.decimateRows();           

    size_t new_rows = mat.rows();

    smawk_r(mat, cmp, rowoptima, work);

    rowoptima.swap(work);

    mat.restoreRows();
    rowoptima.resize(orig_rows);

    for (size_t i=0; i<orig_rows; ++i) {

      if (i % 2) {

        rowoptima[i] = work[i/2];

      } else {

        size_t i0 = i/2-1;
        size_t i1 = i/2;
        size_t j0 = (i0 < new_rows) ? work[i0] : 0;
        size_t j1 = (i1 < new_rows) ? work[i1]+1 : mat.cols();

        rowoptima[i] = j0;
        Tval optval = mat( i, j0 );

        for (size_t j=j0+1; j<j1; ++j) {
          Tval val = mat( i, j );
          if (cmp(val, optval)) { 
            rowoptima[i] = j;
            optval = val;
          }
        }

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

  if (orig_cols > mat.cols()) {
    
    for (size_t i=0; i<rowoptima.size(); ++i) {
      smawk_debug << "in row " << i+1 << " moving col " << rowoptima[i]+1 << " to " << keptcols[rowoptima[i]]+1 << "\n";
      rowoptima[i] = keptcols[rowoptima[i]];
    }

#ifdef SMAWK_USE_PTR
    delete[] keptcols;
#endif

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
  
  IndexArray work;

  smawk_r(mat, cmp, rowoptima, work);

}




#endif
