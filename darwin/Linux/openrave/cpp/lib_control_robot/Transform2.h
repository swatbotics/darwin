#ifndef _MZTRANSFORM2_H_
#define _MZTRANSFORM2_H_

#include "mat2.h"
#include "AngleUtil.h"

template <class real> class Transform2_t {
public:

  typedef vec2_t<real> vec2;
  typedef mat2_t<real> mat2;

  vec2 translation;
  real rotation;
  
  Transform2_t(): translation(0), rotation(0) {}

  Transform2_t(const vec2& tx, real rot): 
    translation(tx), rotation(rot) {}

  void setTransform(const vec2& tx, real rot) {
    translation = tx;
    rotation = rot;
  }

  vec2 transformFwd(const vec2& p) const {
    return rotFwd() * p + translation;
  }

  vec2 transformInv(const vec2& p) const {
    return rotInv() * (p - translation);
  }
  
  Transform2_t inverse() const {
    return Transform2_t( transformInv(vec2(0,0)), -rotation );
  }
  
  mat2 rotFwd() const {
    return mat2::rotMat(rotation);
  }

  mat2 rotInv() const {
    return mat2::rotMat(-rotation);
  }

};

template <class real> inline vec2_t<real>
operator*(const Transform2_t<real>& tx, const vec2_t<real>& v) {
  return tx.transformFwd(v);
}

template <class real> inline Transform2_t<real>
operator*(const Transform2_t<real>& t1, const Transform2_t<real>& t2) {
  real r2 = clamp_angle(t1.rotation + t2.rotation);
  return Transform2_t<real>(t1.transformFwd(t2.translation), r2);
}

typedef Transform2_t<double> Transform2d;
typedef Transform2_t<float> Transform2f;

template <class real> 
inline std::ostream& operator<<(std::ostream& ostr, const Transform2_t<real>& t) {
  return ostr << "< " << t.translation.x() 
              << ", " << t.translation.y() 
              << ", " << t.rotation * 180 / M_PI 
              << ">";
}

#endif

