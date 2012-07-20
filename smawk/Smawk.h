#ifndef _SMAWK_H_
#define _SMAWK_H_

#include "IndexedMatrix.h"
#include "SmawkDebug.h"
#include <assert.h>



template <class Tval>
struct StackElem_t {

  size_t i;
  size_t l;

  Tval   value;

  StackElem_t() {}

  StackElem_t(size_t r, size_t c, Tval v): 
    i(r), l(c), value(v) {}

};

//////////////////////////////////////////////////////////////////////


template <class Tmat, class Tcmp>
void reduce(const IndexedMatrix_t<Tmat>& mat, 
            Tcmp cmp,
            IndexArray& cmap,
            std::vector< StackElem_t<typename Tmat::value_type > >& S) {

  typedef typename Tmat::value_type Tval;
  typedef StackElem_t<Tval> StackElem;
  typedef std::vector<StackElem> Stack;

  S.clear();
  size_t j = 0;

  while (j < mat.cols()) {

    bool defeated = false;
    
    if (S.empty()) {

      S.push_back( StackElem(0, j, mat(0, j)) );
      ++j;

    } else {

      while (!S.empty() && !defeated) {
      
        const StackElem& el = S.back();
        StackElem ej( el.i, j, mat(el.i, j) );
      
        if (cmp(el.value, ej.value)) {
        
          ++j;
          defeated = true;

          if (ej.i < mat.rows()-1) {
            ej.i += 1;
            ej.value = mat(ej.i, ej.l);
            S.push_back(ej);
          }

        } else { 

          S.pop_back();
          
        }
      
      }

    }

  }

  cmap.clear();

  for (size_t s=0; s<S.size(); ++s) {
    cmap.push_back(S[s].l);
  }

}





template <class Tmat, class Tcmp>
void smawk_r(IndexedMatrix_t<Tmat>& mat,
             Tcmp cmp,
             IndexArray& rowoptima,
             IndexArray& work,
             std::vector< StackElem_t<typename Tmat::value_type > >& stack) {


#ifdef SMAWK_DEBUG
  std::cout << "at top:\n";
  debugPrintMatrix(mat, cmp);
#endif

  typedef typename Tmat::value_type Tval;

  size_t orig_cols = mat.cols();

  if (orig_cols > mat.rows()) {

    reduce(mat, cmp, work, stack);
    mat.reduceCols(work);

#ifdef SMAWK_DEBUG
    std::cout << "after col reduce:\n";
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

    smawk_r(mat, cmp, rowoptima, work, stack);

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
    std::cout << "after rows restored:\n";
    debugPrintMatrixAndOptima(mat, cmp, rowoptima);
#endif

  }

#ifdef SMAWK_DEBUG
  assert(rowoptima.size() == mat.rows());
#endif

  if (mat.cols() != orig_cols) {
    
    mat.restoreCols(orig_cols, rowoptima, work);

#ifdef SMAWK_DEBUG
    std::cout << "after columns restored:\n";
    debugPrintMatrixAndOptima(mat, cmp, rowoptima);
#endif

  }

}

template <class Tmat, class Tcmp>
void smawk(const Tmat& m,
           Tcmp cmp,
           IndexArray& rowoptima) {

  IndexedMatrix_t<Tmat> mat(m);
  
  IndexArray work;
  std::vector< StackElem_t<typename Tmat::value_type > > stack;

  smawk_r(mat, cmp, rowoptima, work, stack);

}




#endif
