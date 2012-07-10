#ifndef _QUAT_H_
#define _QUAT_H_

#include "mat4.h"
#include "mat3.h"

template <class real> class quat_t: public vec4_t<real> {
public:

  quat_t(): vec4_t<real>(0,0,0,1) {}

  quat_t(real x, real y, real z, real w): vec4_t<real>(x,y,z,w) {}

  quat_t(const vec4_t<real>& v): vec4_t<real>(v) {}  

  static quat_t fromAxisAngle(const vec3_t<real>& axis, real angle) {
  
    real l = axis.norm();
    real ca = cos(angle*0.5);
    real sa = sin(angle*0.5);
    real a, b, c, d;
    
    a = ca;
    b = sa*axis.x()/l;
    c = sa*axis.y()/l;
    d = sa*axis.z()/l;
    
    return quat_t(b,c,d,a);

  }

  static quat_t fromOmega(const vec3_t<real>& omega) {
    float angle = omega.norm();
    if (angle < 1e-6) { return quat_t(); }
    return fromAxisAngle(omega/angle, angle);
  }

  static quat_t fromMatrix(const mat4_t<real>& m) {
    
    // this was derived using transpose!!
    real r11 = m(0,0), r12 = m(1,0), r13 = m(2,0);
    real r21 = m(0,1), r22 = m(1,1), r23 = m(2,1);
    real r31 = m(0,2), r32 = m(1,2), r33 = m(2,2);

    real qx, qy, qz, qw;
    
    real trace = 1 + r11 + r22 + r33;
    const real eps = 1e-1;

    if (trace > eps) {
      real n = sqrt(trace);
      qw = n;
      qx = (r23 - r32) / n;
      qy = (r31 - r13) / n;
      qz = (r12 - r21) / n;
    } else {
      trace = 1 + r11 - r22 - r33;
      if (trace > eps) {
        real n = sqrt(trace);
        qw = (r23 - r32) / n;
        qx = n;
        qy = (r12 + r21) / n;
        qz = (r31 + r13) / n;
      } else {
        trace = 1 - r11 + r22 - r33;
        if (trace > eps) {
          real n = sqrt(trace);
          qw = (r31 - r13) / n;
          qx = (r12 + r21) / n;
          qy = n;
          qz = (r23 + r32) / n;
        } else {
          trace = 1 - r11 - r22 + r33;
          real n = sqrt(trace);
          qw = (r12 - r21) / n;
          qx = (r31 + r13) / n;
          qy = (r23 + r32) / n;
          qz = n;
        }
      }
    }

    return quat_t(0.5*qx, 0.5*qy, 0.5*qz, 0.5*qw);

  }

  static quat_t fromMat3(const mat3_t<real>& m) {
    
    // this was derived using transpose!!
    real r11 = m(0,0), r12 = m(1,0), r13 = m(2,0);
    real r21 = m(0,1), r22 = m(1,1), r23 = m(2,1);
    real r31 = m(0,2), r32 = m(1,2), r33 = m(2,2);

    real qx, qy, qz, qw;
    
    real trace = 1 + r11 + r22 + r33;
    const real eps = 1e-1;

    if (trace > eps) {
      real n = sqrt(trace);
      qw = n;
      qx = (r23 - r32) / n;
      qy = (r31 - r13) / n;
      qz = (r12 - r21) / n;
    } else {
      trace = 1 + r11 - r22 - r33;
      if (trace > eps) {
        real n = sqrt(trace);
        qw = (r23 - r32) / n;
        qx = n;
        qy = (r12 + r21) / n;
        qz = (r31 + r13) / n;
      } else {
        trace = 1 - r11 + r22 - r33;
        if (trace > eps) {
          real n = sqrt(trace);
          qw = (r31 - r13) / n;
          qx = (r12 + r21) / n;
          qy = n;
          qz = (r23 + r32) / n;
        } else {
          trace = 1 - r11 - r22 + r33;
          real n = sqrt(trace);
          qw = (r12 - r21) / n;
          qx = (r31 + r13) / n;
          qy = (r23 + r32) / n;
          qz = n;
        }
      }
    }

    return quat_t(0.5*qx, 0.5*qy, 0.5*qz, 0.5*qw);
    
  }

  static quat_t fromEuler(const vec3_t<real>& e) {

    // TODO: we need to do this properly!!!
    real C1 = cos(0.5*e[0]);
    real C2 = cos(0.5*e[1]);
    real C3 = cos(0.5*e[2]);
    
    real S1 = sin(0.5*e[0]);
    real S2 = sin(0.5*e[1]);
    real S3 = sin(0.5*e[2]);

    real w =  C1*C2*C3 - S1*S2*S3;
    real z =  C1*C2*S3 + S1*C3*S2;
    real y =  C1*C3*S2 - S1*C2*S3;
    real x =  C1*S2*S3 + C2*C3*S1;
    
    quat_t q(x,y,z,w);
    
    return q;

  }
  
  real angle() const {
    return 2 * acos(fabs(this->w()) / this->norm());
  }

  static real dist(const quat_t& u, quat_t v) {
    float duv = dot(u,v);
    if (duv < 0) { v = -v; }
    real d = fabs(duv / u.norm() * v.norm());
    if (d > 1.0) { return 0; }
    return 2*acos(d);
  }

  quat_t conj() const {
    return quat_t(-this->v[0], -this->v[1], -this->v[2], this->v[3]);
 }

  quat_t inverse() const {
    float n2 = this->norm2();
    return quat_t(-this->v[0]/n2, -this->v[1]/n2, -this->v[2]/n2, this->v[3]/n2);
  }


  void toEuler(vec3_t<real>& e) const {

    const real& qw = this->w();
    const real& qx = this->x();
    const real& qy = this->y();
    const real& qz = this->z();

    real qw2 = qw*qw;
    real qz2 = qz*qz;
    real qy2 = qy*qy;
    real qx2 = qx*qx;

    e[0] = atan2(-2*qz*qy + 2*qw*qx, qz2+qw2-qx2-qy2);
    e[1] = asin(2*qz*qx + 2*qw*qy);
    e[2] = atan2(-2*qy*qx + 2*qw*qz, qx2-qy2-qz2+qw2);

  }

  vec3_t<real> toEuler() const {

    vec3_t<real> rval;
    toEuler(rval);
    return rval;

  }


  mat3_t<real>& toMat3(mat3_t<real>& m) const {

    const real& qw = this->w();
    const real& qx = this->x();
    const real& qy = this->y();
    const real& qz = this->z();

    real sqw = qw*qw;
    real sqx = qx*qx;
    real sqy = qy*qy;
    real sqz = qz*qz;
    
    // invs (inverse square length) is only required if quaternion is not already normalised
    real invs = 1.0f / (sqx + sqy + sqz + sqw);
    m(0,0) = ( sqx - sqy - sqz + sqw)*invs ; // since sqw + sqx + sqy + sqz =1/invs*invs
    m(1,1) = (-sqx + sqy - sqz + sqw)*invs ;
    m(2,2) = (-sqx - sqy + sqz + sqw)*invs ;
    
    real tmp1 = qx*qy;
    real tmp2 = qz*qw;
    m(1,0) = 2.0f * (tmp1 + tmp2)*invs ;
    m(0,1) = 2.0f * (tmp1 - tmp2)*invs ;
    
    tmp1 = qx*qz;
    tmp2 = qy*qw;
    m(2,0) = 2.0f * (tmp1 - tmp2)*invs ;
    m(0,2) = 2.0f * (tmp1 + tmp2)*invs ;

    tmp1 = qy*qz;
    tmp2 = qx*qw;
    m(2,1) = 2.0f * (tmp1 + tmp2)*invs ;
    m(1,2) = 2.0f * (tmp1 - tmp2)*invs ;

    return m;

  }


  mat4_t<real>& toMatrix(mat4_t<real>& m) const {

    const real& qw = this->w();
    const real& qx = this->x();
    const real& qy = this->y();
    const real& qz = this->z();

    real sqw = qw*qw;
    real sqx = qx*qx;
    real sqy = qy*qy;
    real sqz = qz*qz;
    
    // invs (inverse square length) is only required if quaternion is not already normalised
    real invs = 1.0f / (sqx + sqy + sqz + sqw);
    m(0,0) = ( sqx - sqy - sqz + sqw)*invs ; // since sqw + sqx + sqy + sqz =1/invs*invs
    m(1,1) = (-sqx + sqy - sqz + sqw)*invs ;
    m(2,2) = (-sqx - sqy + sqz + sqw)*invs ;
    
    real tmp1 = qx*qy;
    real tmp2 = qz*qw;
    m(1,0) = 2.0f * (tmp1 + tmp2)*invs ;
    m(0,1) = 2.0f * (tmp1 - tmp2)*invs ;
    
    tmp1 = qx*qz;
    tmp2 = qy*qw;
    m(2,0) = 2.0f * (tmp1 - tmp2)*invs ;
    m(0,2) = 2.0f * (tmp1 + tmp2)*invs ;

    tmp1 = qy*qz;
    tmp2 = qx*qw;
    m(2,1) = 2.0f * (tmp1 + tmp2)*invs ;
    m(1,2) = 2.0f * (tmp1 - tmp2)*invs ;

    m(0,3) = m(1,3) = m(2,3) = 0.0;
    m(3,0) = m(3,1) = m(3,2) = 0.0;
    m(3,3) = 1.0;

    return m;

  }

  mat3_t<real> toMat3() const {
    mat3_t<real> rval;
    toMat3(rval);
    return rval;
  }

  mat4_t<real> toMatrix() const {
    mat4_t<real> rval;
    toMatrix(rval);
    return rval;
  }

  quat_t log() const {

    quat_t rval = *this;

    rval.w() = 0;
    real w = this->w();

    // handle roundoff
    if (w < -1) { w = -1; } 
    if (w > 1) { w = 1; }

    real theta = acos(w);
    real sin_theta = sin(theta);

    if ( fabs(sin_theta) > 1e-7 ) {
      rval.x() = this->x() / sin_theta * theta;
      rval.y() = this->y() / sin_theta * theta;
      rval.z() = this->z() / sin_theta * theta;
    }

    return rval;

  }

  quat_t exp() const {
    real a = this->trunc().norm();
    real sa = sin(a);
    quat_t rval;
    rval.w() = cos(a);
    if (a >= 1e-7) {
      sa /= a;
      rval.x() *= sa;
      rval.y() *= sa;
      rval.z() *= sa;
    } else {
      rval.x() = rval.y() = rval.z() = 0;
    }
    return rval;
  }

  quat_t pow(real p) const {
    return (log()*p).exp();
  }
    

  static quat_t slerp(const quat_t& u, quat_t v, real t) {

    real cosom = dot(u,v);
    
    if (cosom < 0.0f) {
      cosom = -cosom;
      v = -v;
    }
    
    if (cosom > 1.0f) { cosom = 1.0f; } // handle roundoff error!
    if (fabs(cosom - 1.0f) < 1e-5) { return u; }
    
    real omega = acos(cosom);
    real sinom = sin(omega);
    
    real scl0 = sin((1-t)*omega)/ sinom;
    real scl1 = sin(t*omega) / sinom;
    
    return quat_t(scl0 * u.x() + scl1 * v.x(),
		  scl0 * u.y() + scl1 * v.y(),
		  scl0 * u.z() + scl1 * v.z(),
		  scl0 * u.w() + scl1 * v.w());
    
  }

  static quat_t slerp_d(const quat_t& u, const quat_t& v, real t) {
    quat_t s = slerp(u, v, t);    
    return slerp_d(u, v, s);
  }

  static quat_t slerp_d(const quat_t& u, const quat_t& v, const quat_t& s) {
    return s*omega(u,v);
  }

  static vec3_t<real> omega(const quat_t& u, quat_t v) {
    float duv = dot(u,v);
    if (duv < 0) { v = -v; }
    quat_t p = (v*u.inverse()).log();
    return 2*vec3_t<real>(p.x(), p.y(), p.z());
  }

  static quat_t<real> squad(const quat_t& p,
                            const quat_t& a,
                            const quat_t& b,
                            const quat_t& q,
                            real t) {
    return slerp(slerp(p,q,t),
                 slerp(a,b,t),
                 2*t*(1-t));
  }

  static quat_t<real> squad_d(const quat_t& p,
                              const quat_t& a,
                              const quat_t& b,
                              const quat_t& q,
                              real t) {
    quat_t U = slerp(p,q,t);
    quat_t V = slerp(a,b,t);
    quat_t W = U.inverse()*V;
    quat_t Up = U*(p.inverse()*q).log();
    quat_t Vp = V*(a.inverse()*b).log();
    quat_t Wp = U.inverse()*Vp - U.pow(-2)*Up*V;
    return U*((2-4*t)*W.pow(2*t*(1-t))*W.log() + 2*t*(1-t)*W.pow(2*t*(1-t)-1)*Wp) + Up*W.pow(2*t*(1-t));
  }

  static quat_t<real> spline_t(const quat_t& q0,
                               const quat_t& q1,
                               const quat_t& q2) {
    quat_t t1 = (q1.inverse()*q2).log();
    quat_t t2 = (q1.inverse()*q0).log();
    quat_t t3 = quat_t( -(t1+t2)/4 ).exp();
    return q1 * t3;
  }
                             

  static quat_t<real> spline(const quat_t& q0,
                             const quat_t& q1,
                             const quat_t& q2,
                             const quat_t& q3,
                             real t) {
    quat_t a1 = spline_t(q0,q1,q2);
    quat_t a2 = spline_t(q1,q2,q3);
    return squad(q1,a1,a2,q3,t);
  }

  static quat_t<real> spline_d(const quat_t& q0,
                               const quat_t& q1,
                               const quat_t& q2,
                               const quat_t& q3,
                               real t) {
    quat_t a1 = spline_t(q0,q1,q2);
    quat_t a2 = spline_t(q1,q2,q3);
    return squad_d(q1,a1,a2,q3,t);
  }

  template <class real2>
  quat_t& operator*=(real2 s) {
    this->v[0] *= s;
    this->v[1] *= s;
    this->v[2] *= s;
    this->v[3] *= s;
    return *this;
  }

  template <class real2>
  quat_t& operator/=(real2 s) {
    this->v[0] /= s;
    this->v[1] /= s;
    this->v[2] /= s;
    this->v[3] /= s;
    return *this;
  }

  /*
  quat_t spin(const vec3_t<real>& angvel) const {
    quat_t rval(0.5*angvel[0], 0.5*angvel[1], 0.5*angvel[2], 0);
    rval /= rval.norm();
    return rval * (*this);
  }

  static quat_t integrate(const quat_t& q0, const vec3_t<real>& angvel, real dt) {
    quat_t q1 = q0 + q0.spin(angvel) * dt;
    q1 /= q1.norm();
    return q1;
  }

  static vec3_t<real> diff(const quat_t& q1, const quat_t& q0, real dt) {
    
  }
  */

};

/** Quaternion multiplication by scalar. */
template <class real, class real2> inline quat_t<real> operator*(const quat_t<real>& v, real2 s) {
  return quat_t<real>(v[0]*s, v[1]*s, v[2]*s, v[3]*s);
}

/** Quaternion multiplication by scalar. */
template <class real, class real2> inline quat_t<real> operator*(real2 s, const quat_t<real>& v) {
  return quat_t<real>(v[0]*s, v[1]*s, v[2]*s, v[3]*s);
}

/** Quaternion division. */
template <class real, class real2> inline quat_t<real> operator/(const quat_t<real>& v, real2 s) {
  return quat_t<real>(v[0]/s, v[1]/s, v[2]/s, v[3]/s);
}

template <class real> inline quat_t<real> operator-(const quat_t<real>& q) {
  return quat_t<real>(-q.x(), -q.y(), -q.z(), -q.w());
}

template <class real> quat_t<real> operator*(const quat_t<real>& u, 
						const quat_t<real>& v) {

  return quat_t<real>(u.w()*v.x() + u.x()*v.w() + u.y()*v.z() - u.z()*v.y(),
			 u.w()*v.y() - u.x()*v.z() + u.y()*v.w() + u.z()*v.x(),
			 u.w()*v.z() + u.x()*v.y() - u.y()*v.x() + u.z()*v.w(),
			 u.w()*v.w() - u.x()*v.x() - u.y()*v.y() - u.z()*v.z());


}

typedef quat_t<double> quatd;
typedef quat_t<float> quatf;

#endif
