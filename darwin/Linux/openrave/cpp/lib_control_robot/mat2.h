#ifndef _MAT2_H_
#define _MAT2_H_

#include "vec2.h"

template <class real> class mat2_t {
public:


  /** Default constructor initializes to identity matrix. */
  mat2_t() { *this = identity(); }

  /** Construct with optional initialization.
   *
   *  @param init If this is true, then initialize to the identity
   *              matrix, otherwise, leave data uninitialized.
   */
  mat2_t(bool init) { if (init) { *this = identity(); } }

  /** Copy from row-major 3-by-3 matrix. */
  mat2_t(const real src[4]) { memcpy(data, src, 4*sizeof(real)); }

  /** Copy constructor. */
  mat2_t(const mat2_t& src) { memcpy(data, src.data, 4*sizeof(real)); }

  /** Assignment operator. */
  mat2_t& operator=(const mat2_t& src) { 
    memcpy(data, src.data, 4*sizeof(real));
    return *this;
  }

  /** Return the transpose of this matrix. */
  mat2_t& transpose(mat2_t& m) const {
    const int tidx[4] = {
      0, 2,
      1, 3,
    };
    for (int i=0; i<4; ++i) {
      m.data[i] = data[tidx[i]];
    }
    return m;
  };

  /** Return the transpose of this matrix. */
  mat2_t transpose() const {
    mat2_t rval(false);
    return transpose(rval);
  }

  /** Return the zero matrix. */
  static mat2_t zero() {
    const real id[4] = {
      0.0, 0.0, 
      0.0, 0.0, 
    };
    return mat2_t(id);
  };

  /** Return the identity matrix. */
  static mat2_t identity() {
    const real id[4] = {
      1.0, 0.0, 
      0.0, 1.0, 
    };
    return mat2_t(id);
  };

  /** Elementwise access. Data is stored in a row-major format. */
  const real& operator[](size_t i) const {
    return data[i];
  }

  /** Subscript access. */
  const real& operator()(size_t row, size_t col) const {
    return data[row*2 + col];
  }

  /** Elementwise access. Data is stored in a row-major format. */
  real& operator[](size_t i)  {
    return data[i];
  }
  
  /** Subscript access. */
  real& operator()(size_t row, size_t col)  {
    return data[row*2 + col];
  }

  /** Determinant */
  real determinant() const {
    return data[0]*data[3] - data[1]*data[2];
  }
  
  /** Inverse. Determinant should be returned by method above. */
  mat2_t& inverse(mat2_t& m, real det) const {
    real invdet = 1/det;
    m[0] = data[3] * invdet;
    m[1] = -data[1] * invdet;
    m[2] = -data[2] * invdet;
    m[3] = data[0] * invdet;
    return m;
  }

  /** Inverse. */
  mat2_t inverse() const {
    real det = determinant();
    mat2_t rval(false);
    return inverse(rval, det);
  }

  /** 2D rotation matrix. */
  static mat2_t rotMat(real theta) {

    real ct = cos(theta);
    real st = sin(theta);

    mat2_t rval(false);

    rval[0] = ct;
    rval[1] = -st;
    rval[2] = st;
    rval[3] = ct;

    return rval;

  }

  /** Outer product matrix. */
  static mat2_t outer(const vec2_t<real>& a, 
                      const vec2_t<real>& b) {
    mat2_t rval(false);
    for (int row=0; row<2; ++row) {
      for (int col=0; col<2; ++col) {
        rval(row,col) = a[row]*b[col];
      }
    }
    return rval;
  }

  /** Underlying data. */
  real data[4];

};


/** Multiplication by scalar. */
template <class real, class real2> inline mat2_t<real> operator*(real2 s, const mat2_t<real>& m) {
  mat2_t<real> rval(false);
  for (int i=0; i<4; ++i) {
    rval.data[i] = m.data[i] * s;
  }
  return rval;
}

/** Multiplication by scalar. */
template <class real, class real2> inline mat2_t<real> operator*(const mat2_t<real>& m, real2 s) {
  return s * m;
}

/** Return the product of two matrices. */
template <class real> inline mat2_t<real> operator*(const mat2_t<real>& A, const mat2_t<real>& B) {
  mat2_t<real> rval(false);
  for (int i=0; i<2; ++i) {
    for (int j=0; j<2; ++j) {
      real accum = 0.0;
      // out(i,j) = ith row of A dotted with jth col of B
      for (int k=0; k<2; ++k) {
	accum += A(i, k) * B(k, j);
      }
      rval(i, j) = accum;
    }
  }
  return rval;
}

/** Return the product of a matrix and a 3D vector. */
template <class real> inline vec2_t<real> operator*(const mat2_t<real>& m, const vec2_t<real>& v) {
  
  vec2_t<real> rval(v[0] * m(0,0) + v[1] * m(0,1),
                    v[0] * m(1,0) + v[1] * m(1,1));

  return rval;

}

/** Matrix stream output. */
template <class real> inline std::ostream& operator<<(std::ostream& ostr, const mat2_t<real>& m) {
  for (int i=0; i<2; ++i) {
    for (int j=0; j<2; ++j) {
      ostr << std::setw(10) << m(i,j) << " ";
    }
    ostr << "\n";
  }
  return ostr;
}

/** Matrix negation. */
template <class real> inline mat2_t<real> operator-(const mat2_t<real>& m) {
  mat2_t<real> rval(false);
  for (int i=0; i<2; ++i) {
    for (int j=0; j<2; ++j) {
      rval(i,j) = -m(i,j);
    }
  }
  return rval;
}

/** Matrix addition */
template <class real> inline mat2_t<real> operator+(const mat2_t<real>& m1,
                                                    const mat2_t<real>& m2) {
  mat2_t<real> rval(false);
  for (int i=0; i<2; ++i) {
    for (int j=0; j<2; ++j) {
      rval(i,j) = m1(i,j) + m2(i,j);
    }
  }
  return rval;
}


/** Matrix subtraction */
template <class real> inline mat2_t<real> operator-(const mat2_t<real>& m1,
                                                    const mat2_t<real>& m2) {
  mat2_t<real> rval(false);
  for (int i=0; i<2; ++i) {
    for (int j=0; j<2; ++j) {
      rval(i,j) = m1(i,j) - m2(i,j);
    }
  }
  return rval;
}



typedef mat2_t<double> mat2d;
typedef mat2_t<float> mat2f;

#endif
