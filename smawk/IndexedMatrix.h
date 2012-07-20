#ifndef _INDEXEDMATRIX_H_
#define _INDEXEDMATRIX_H_

#include "Index.h"
#include <sstream>
#include <iomanip>
#include <assert.h>

//#define IMAT_USE_PTR

template <class Tmat>
class IndexedMatrix_t {
private:

  const Tmat& mat;
#ifdef IMAT_USE_PTR
  size_t* colset;
  size_t ncols;
#else
  IndexArray colset;
#endif
  size_t lod;

public:

  typedef typename Tmat::value_type value_type;
  
  IndexedMatrix_t(const Tmat& m, int l=0):
    mat(m), lod(l) {

#ifdef IMAT_USE_PTR
    ncols = m.cols();
    colset = new size_t[m.cols()];
#else
    colset.resize(m.cols());
#endif

    for (size_t i=0; i<m.cols(); ++i) { colset[i] = i; }

  }

#ifdef IMAT_USE_PTR

  IndexedMatrix_t(const IndexedMatrix_t<Tmat>& imat):
    mat(imat.mat), lod(imat.lod)
  {
    ncols = imat.ncols;
    colset = new size_t[imat.ncols];
    for (size_t i=0; i<ncols; ++i) { colset[i] = imat.colset[i]; }
  }

  ~IndexedMatrix_t() { delete[] colset; }
  
  size_t cols() const { return ncols; }

#else 

  size_t cols() const { return colset.size(); }

#endif

  size_t rows() const { return mat.rows() >> lod; }

  void decimateRows() { ++lod; }

  void restoreRows() { --lod; }

  void makeSquare() { 
#ifdef IMAT_USE_PTR
    ncols = rows();
#else
    colset.resize(rows()); 
#endif
  }

  void replaceColumn(size_t j1, size_t j2) {
    colset[j1] = colset[j2];
  }

  value_type operator()(size_t i, size_t j) const {
    return mat( ((i+1)<<lod)-1, colset[j] );
  }

};

template <class Tmat>
class PrintableMatrix {
public:
  const Tmat& m;
  PrintableMatrix(const Tmat& mat): m(mat) {}
};

template <class Tmat>
inline PrintableMatrix<Tmat> pmat(const Tmat& mat) {
  return PrintableMatrix<Tmat>(mat);
}

template <class Tmat>
inline std::ostream& operator<<(std::ostream& ostr, 
                                const PrintableMatrix< Tmat >& p) {

  const Tmat& m = p.m;

  size_t w = 0;
  for (size_t i=0; i<m.rows(); ++i) {
    for (size_t j=0; j<m.cols(); ++j) {
      std::ostringstream ostr;
      ostr << m(i,j);
      w = std::max(w, ostr.str().length());
    }
  }

  for (size_t i=0; i<m.rows(); ++i) {
    if (i) { ostr << "\n"; }
    ostr << "[ ";
    for (size_t j=0; j<m.cols(); ++j) {
      ostr << std::setw(w) << m(i,j) << " ";
    }
    ostr << "]";
  }
  return ostr;

}

#endif
