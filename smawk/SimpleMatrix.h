#ifndef _SIMPLEMATRIX_H_
#define _SIMPLEMATRIX_H_

#include <cstdlib>
#include <sstream>
#include <iomanip>
#include <assert.h>

template <class Tval>
class SimpleMatrix_t {
public:

  typedef Tval value_type;
  const size_t _rows;
  const size_t _cols;
  const size_t _stride;
  Tval* const _data;

  bool empty() const {
    return _rows == 0 || _cols == 0;
  }

  SimpleMatrix_t(size_t nr, size_t nc, Tval* d, size_t s=0):
    _rows(nr), _cols(nc), _stride(s ? s : nc), _data(d) {}
  
  SimpleMatrix_t(SimpleMatrix_t& orig, size_t r, size_t c, size_t nr, size_t nc):
    _rows(nr), 
    _cols(nc), 
    _stride(orig._stride), 
    _data(orig._data + r*orig._stride + c) 
  { 
    assert(c + nc <= orig.cols());
    assert(r + nr <= orig.rows());
  }

  size_t rows() const { return _rows; }
  size_t cols() const { return _cols; }

  Tval& operator()(size_t r, size_t c) {
    return _data[r*_stride + c];
  }

  const Tval& operator()(size_t r, size_t c) const {
    return _data[r*_stride + c];
  }

  
};

template <class Tval>
inline std::ostream& operator<<(std::ostream& ostr, const SimpleMatrix_t<Tval>& m) {

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
