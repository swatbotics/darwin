#ifndef _INDEXEDMATRIX_H_
#define _INDEXEDMATRIX_H_

#include "Index.h"
#include <sstream>
#include <iomanip>

template <class Tmat>
class IndexedMatrix_t {
private:

  const Tmat& mat;
  IndexSet colset;
  size_t lod;



public:

  typedef typename Tmat::value_type value_type;
  
  IndexedMatrix_t(const Tmat& m, int l=0):
    mat(m), colset(m.cols()), lod(l) {}

  size_t rows() const { return mat.rows() >> lod; }
  size_t cols() const { return colset.size(); }

  void decimateRows() { ++lod; }

  void restoreRows() { --lod; }

  size_t reduceCols(IndexArray& subs) {
    return colset.reduce(subs);
  }
  
  void restoreCols(size_t orig_cols, IndexArray& work) {
    colset.restore(orig_cols, work);
  }

  void restoreCols(size_t orig_cols, IndexArray& optima, IndexArray& work) {
    colset.restore(orig_cols, optima, work);
  }

  size_t trueRowIndex(size_t i) const {
    return ((i+1)<<lod)-1;
  }
   
  size_t trueColIndex(size_t j) const {
    return colset[j];
  }

  value_type operator()(size_t i, size_t j) const {
    return mat(trueRowIndex(i), trueColIndex(j));
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
