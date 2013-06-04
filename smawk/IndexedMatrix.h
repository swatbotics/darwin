#ifndef _INDEXEDMATRIX_H_
#define _INDEXEDMATRIX_H_

#include "Index.h"
#include <sstream>
#include <iomanip>


template <class Tmat>
class IndexedMatrix_t {
private:

  const Tmat& mat;
  IndexArray colset;
  size_t lod;
  size_t roffs;

  static size_t getOffset(size_t l) { 
    return (1<<l)-1;
  }

public:

  typedef typename Tmat::value_type value_type;
  
  IndexedMatrix_t(const Tmat& m, int l=0):
    mat(m), lod(l), roffs(getOffset(l)) {

    colset.resize(m.cols());
    for (size_t i=0; i<m.cols(); ++i) { colset[i] = i; }

  }

  size_t cols() const { return colset.size(); }

  size_t rows() const { return mat.rows() >> lod; }

  void decimateRows() { roffs = getOffset(++lod); }

  void restoreRows() { roffs = getOffset(--lod); }

  void makeSquare() { 
    colset.resize(rows()); 
  }

  void replaceColumn(size_t j1, size_t j2) {
    colset[j1] = colset[j2];
  }
  
  size_t trueRowIndex(size_t i) const {
    return (i<<lod) + roffs;
  }

  size_t trueColIndex(size_t j) const {
    return colset[j];
  }

  value_type operator()(size_t i, size_t j) const {
    return mat( (i<<lod) + roffs, colset[j] );
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
