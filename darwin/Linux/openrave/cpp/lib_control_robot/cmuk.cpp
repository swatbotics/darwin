#include "cmuk.h"
#include <math/Transform3.h>
#include <string.h>
#include <iostream>
#include <iomanip>
#include <assert.h>
#include "util_mz.h"

#ifdef _NDEBUG
#define debug if (0) std::cerr
#else
#define debug if(_debugOutput) std::cerr
#endif

bool cmuk::isFront(int legIndex) {
  return legIndex == FL || legIndex == FR;
}

bool cmuk::isLeft(int legIndex) {
  return legIndex == FL || legIndex == HL;
}

static mat3f inertia_tensor(float m, float a, float b, float c) {

  a *= 0.5;
  b *= 0.5;
  c *= 0.5;

  float a2 = a*a;
  float b2 = b*b;
  float c2 = c*c;

  mat3f I;
  
  I(0,0) = (b2+c2)*m/3;
  I(1,1) = (a2+c2)*m/3;
  I(2,2) = (a2+b2)*m/3;

  return I;
  
}


static const vec3f default_joint_offsets[cmuk::NUM_OFFSETS * cmuk::NUM_LEGS] = {

  vec3f(0.101, 0.036, 0),   // FL_HIP_RX
  vec3f(0, 0.03, 0),     // FL_HIP_RY
  vec3f(0, 0, -0.075),   // FL_KNEE_RY
  vec3f(-0.0265, 0, -0.0985),   // FL_FOOT
       
  vec3f(0.101, -0.036, 0),   // FR_HIP_RX
  vec3f(0, -0.03, 0),    // FR_HIP_RY
  vec3f(0, 0, -0.075),   // FR_KNEE_RY
  vec3f(-0.0265, 0, -0.0985),   // FR_FOOT
       
  vec3f(-0.101, 0.036, 0),   // HL_HIP_RX
  vec3f(0, 0.03, 0),     // HL_HIP_RY
  vec3f(0, 0, -0.075),   // HL_KNEE_RY
  vec3f(0.0265, 0, -0.0985),   // HL_FOOT
       
  vec3f(-0.101, -0.036, 0),   // HR_HIP_RX
  vec3f(0, -0.03, 0),    // HR_HIP_RY
  vec3f(0, 0, -0.075),   // HR_KNEE_RY
  vec3f(0.0265, 0, -0.0985)   // HR_FOOT

};

static const vec3f old_joint_offsets[cmuk::NUM_OFFSETS * cmuk::NUM_LEGS] = {

  vec3f( 0.101,  0.0365,  0.0),      // FL_HIP_RX
  vec3f( 0.0,    0.0302,  0.0),      // FL_HIP_RY
  vec3f( 0.0,    0.0,    -0.0751),   // FL_KNEE_RY
  vec3f(-0.021,  0.0,    -0.1027),   // FL_FOOT

  vec3f( 0.101, -0.0365,  0.0),      // FR_HIP_RX
  vec3f( 0.0,   -0.0302,  0.0),      // FR_HIP_RY
  vec3f( 0.0,    0.0,    -0.0751),   // FR_KNEE_RY
  vec3f(-0.021,  0.0,    -0.1027),   // FR_FOOT

  vec3f(-0.101,  0.0365,  0.0),      // HL_HIP_RX
  vec3f( 0.0,    0.0302,  0.0),      // HL_HIP_RY
  vec3f( 0.0,    0.0,    -0.0751),   // HL_KNEE_RY
  vec3f( 0.021,  0.0,    -0.1027),   // HL_FOOT

  vec3f(-0.101, -0.0365,  0.0),      // HR_HIP_RX
  vec3f( 0.0,   -0.0302,  0.0),      // HR_HIP_RY
  vec3f( 0.0,    0.0,    -0.0751),   // HR_KNEE_RY
  vec3f( 0.021,  0.0,    -0.1027),   // HR_FOOT

};


static const float default_joint_limits[cmuk::NUM_JOINTS * cmuk::NUM_LEGS * 2] = {

  -0.6,  0.6,  // FL_HIP_RX
  -3.5,  2.4,  // FL_HIP_RY
  -2.8,  1.0,  // FL_KNEE_RY
  
  -0.6,  0.6,  // FR_HIP_RX
  -3.5,  2.4,  // FR_HIP_RY
  -2.8,  1.0,  // FR_KNEE_RY

  -0.6,  0.6,  // HL_HIP_RX
  -2.4,  3.5,  // HL_HIP_RY
  -1.0,  2.8,  // HL_KNEE_RY

  -0.6,  0.6,  // HR_HIP_RX
  -2.4,  3.5,  // HR_HIP_RY
  -1.0,  2.8,  // HR_KNEE_RY
  
};



static const bool default_prefer_knee_forward[cmuk::NUM_LEGS] = {
  false, false, true, true
};

static const vec3f default_body_cg(0.0, 0.0, 0.012); // body CG offset

/*
static const cmuk::MzDogConstants dc = { 

  0.143, 0.304, 0.01, // h_body, l_body, r_foot

  0.040, // d_hip_body_bottom

  -(joint_offsets[0].z() + joint_offsets[1].z() + 
    joint_offsets[2].z() + joint_offsets[3].z()), // natural_height

};
*/


static inline vec3f jo(const cmuk::KConstants& kc, int leg, int offset, bool cfoot) {
  if ((offset % 4 == 3) && cfoot) {
    return kc.joint_offsets[cmuk::NUM_OFFSETS * leg + offset] + 
      vec3f(0, 0, kc.r_foot);
  } else {
    return kc.joint_offsets[cmuk::NUM_OFFSETS * leg + offset];
  }
}


static inline float le(const cmuk::KConstants& kc, int leg, bool cfoot) {
  vec3f v = jo(kc, leg, 3, cfoot);
  return atan2(-v.x(), -v.z());
}

static inline float ol(const cmuk::KConstants& kc, int leg, int offset, bool cfoot) {
  vec3f v = jo(kc, leg, offset, cfoot);
  return v.norm();
}

static inline float ol2(const cmuk::KConstants& kc, int leg, int offset, bool cfoot) {
  vec3f v = jo(kc, leg, offset, cfoot);
  return v.norm2();
}


static inline float jl(const cmuk::KConstants& kc, int leg, int joint, int minormax) {
  return kc.joint_limits[cmuk::NUM_JOINTS * 2 * leg + 
			 2 * joint + 
			 minormax];
}


static int parentFrame(int f) {

  if (f == cmuk::FRAME_FL_HIP_RX ||
      f == cmuk::FRAME_FR_HIP_RX ||
      f == cmuk::FRAME_HL_HIP_RX ||
      f == cmuk::FRAME_HR_HIP_RX) {
    return cmuk::FRAME_TRUNK;
  } else if (f <= 0) {
    return -1;
  } else {
    return f-1;
  }
  
}

static inline bool check_foot_ik(const cmuk::KConstants& kc, int leg, CMUK_ERROR_CODE status) {
  if (kc.prefer_knee_forward[leg]) {
    return (status == CMUK_OKAY) ||
      (status == CMUK_CANNOT_REACH_IK_KNEE_BENT_REARWARD);
  } else {
    return (status == CMUK_OKAY) ||
      (status == CMUK_CANNOT_REACH_IK_KNEE_BENT_FORWARD);
  }
}

static inline const vec3f& choose_leg_angles(const cmuk::KConstants& kc, 
					    int leg,
                                            const vec3f& qfwd,
                                            const vec3f& qback) {
  if (kc.prefer_knee_forward[leg]) {
    return qfwd;
  } else {
    return qback;
  }
}

static inline float compute_badness(float val, float min, float max) {
  if (val < min) { 
    return val - min; 
  } else if (val > max) { 
    return val - max; 
  } else {
    return 0;
  }
}
  

static inline void check_wrap(const cmuk::KConstants& kc, vec3f& q, cmuk::LegIndex leg) {

  float min[2], max[2];

  min[0] = jl(kc, leg, cmuk::HIP_RY, 0);
  min[1] = jl(kc, leg, cmuk::KNEE_RY, 0);

  max[0] = jl(kc, leg, cmuk::HIP_RY, 1);
  max[1] = jl(kc, leg, cmuk::KNEE_RY, 1);

  for (int i=0; i<2; ++i) {

    float& angle = q[i+1];
    float b_old = compute_badness(angle, min[i], max[i]);      
    if (!b_old) { continue; }

    float a_new = angle;

    if (b_old > 0) {
      a_new = angle - 2*M_PI;
    } else {
      a_new = angle + 2*M_PI;
    }

    float b_new = compute_badness(a_new, min[i], max[i]);

    if (fabs(b_new) < fabs(b_old)) {
      angle = a_new;
    }

  }

  
}


static bool check_limits(const cmuk::KConstants& kc, vec3f& angles, int leg) {
  bool rval = true;
  for (int i=0; i<3; ++i) {
    const float& min = jl(kc, leg, i, 0);
    const float& max = jl(kc, leg, i, 1);
    if (angles[i] < min) { angles[i] = min; rval = false; }
    if (angles[i] > max) { angles[i] = max; rval = false; }
  }
  return rval;
}

CMUK_ERROR_CODE cmuk::computeFootFK( LegIndex leg,
                                     const vec3f& q,
                                     vec3f* pos,
                                     vec4f* orient ) const {

  Transform3f t[4];

  CMUK_ERROR_CODE e = computeLegTransforms( leg, q, t );
  if (e != CMUK_OKAY) { return e; }

  Transform3f& tfoot = t[3];

  if (pos) { *pos = tfoot.translation(); }
  if (orient) { *orient = static_cast<const vec4f&>(tfoot.rotation()); }

  return CMUK_OKAY;

}

CMUK_ERROR_CODE cmuk::getAbsTransform( const cmuk::XformCache& cache,
                                       FrameIndex frame,
                                       Transform3f* xform ) const {

  if (!xform) { return CMUK_INSUFFICIENT_ARGUMENTS; }

  if (frame == FRAME_GROUND) {
    *xform = Transform3f();
  } else {
    if ((int)frame >= NUM_FRAMES) {
      return CMUK_BAD_FRAME_INDEX;
    }
    *xform = cache.absXforms[frame];
  }

  return CMUK_OKAY;

}

CMUK_ERROR_CODE cmuk::getRelTransform( const cmuk::XformCache& cache,
					     FrameIndex f0,
					     FrameIndex f1,
					     Transform3f* xform ) const {

  int if0 = int(f0);
  int if1 = int(f1);

  if (if0 != FRAME_GROUND && (if0 < 0 || if0 >= NUM_FRAMES)) {
    return CMUK_BAD_FRAME_INDEX;
  }

  if (if1 !=FRAME_GROUND && (if1 < 0 || if1 >= NUM_FRAMES)) {
    return CMUK_BAD_FRAME_INDEX;
  }

  if (!xform) {
    return CMUK_INSUFFICIENT_ARGUMENTS;
  }

  int leg0 = -1;
  int leg1 = -1;
  if (if0 > int(FRAME_TRUNK)) { leg0 = (if0 - 1) / 4; }
  if (if1 > int(FRAME_TRUNK)) { leg1 = (if0 - 1) / 4; }

  if (leg0 >= 0 && leg1 >= 0 && leg0 != leg1) {

    Transform3f t1, t2;
    CMUK_ERROR_CODE status;

    status = getRelTransform(cache, f0, FRAME_TRUNK, &t1);
    if (status != CMUK_OKAY) { return status; }

    status = getRelTransform(cache, FRAME_TRUNK, f1, &t2);
    if (status != CMUK_OKAY) { return status; }

    *xform = t1 * t2;

    return CMUK_OKAY;

  }

  if (if0 == if1) {
    *xform = Transform3f();
    return CMUK_OKAY;
  }

  // if1 should be smaller than if0
  bool swap = if0 < if1;

  if (swap) {
    int tmp = if0;
    if0 = if1;
    if1 = tmp;
  }

  Transform3f rval = cache.relXforms[if0];
  if0 = parentFrame(if0);
  
  // now we are going to decrement if0
  while (if1 < if0 && if0 > 0) {
    rval = rval * cache.relXforms[if0];
    if0 = parentFrame(if0);
  }

  if (swap) {
    rval = rval.inverse();
  }

  *xform = rval;
  return CMUK_OKAY;

}




cmuk::CmuDogKinematics() {

  _centeredFootIK = true;
  _debugOutput = false;
  
  memcpy(_kc.joint_offsets, default_joint_offsets, sizeof(default_joint_offsets));
  memcpy(_kc.joint_limits, default_joint_limits, sizeof(default_joint_limits));
  memcpy(_kc.prefer_knee_forward, default_prefer_knee_forward, sizeof(default_prefer_knee_forward));

  //_kc.h_body = 0.143;
  _kc.h_body = 0.120;
  _kc.l_body = 0.30;
  _kc.w_body_bottom = 0.0648;
  _kc.r_foot = 0.01;
  //_kc.d_hip_body_bottom = 0.060;
  _kc.d_hip_body_bottom = 0.048;
  _kc.d_foot_shin = 0.008;

  _kc.natural_height = -(_kc.joint_offsets[0].z() + _kc.joint_offsets[1].z() + 
			 _kc.joint_offsets[2].z() + _kc.joint_offsets[3].z());
  
  _kc.body_cg_offset = default_body_cg;

  _kc.m_body = 2.240;
  _kc.m_uleg = 0.120;
  _kc.m_lleg = 0.070;

  float m_total = 4*_kc.m_uleg + _kc.m_body;
  _kc.I_body = inertia_tensor(m_total, 
                              _kc.l_body, 
                              _kc.w_body_bottom,
                              _kc.h_body);

  _kc.max_hip_speed = 7.0;
  _kc.max_knee_speed = 10.0;

  _kc.skel_body_thickness = 0.005;
  _kc.skel_uleg_rad = 0.015;
  _kc.skel_knee_rad = 0.020;
  _kc.skel_lleg_rad = 0.008;
  
    
}


void cmuk::cacheTransforms(const KState& q, XformCache* c) const {

  c->absXforms[0] = c->relXforms[0] = Transform3f(q.body_rot, q.body_pos);
  const bool& cfik = _centeredFootIK;

  size_t fidx = 0;
  for (int leg=0; leg<NUM_LEGS; ++leg) {

    ++fidx;

    c->relXforms[fidx] = Transform3f::rx(q.leg_rot[leg][0], 
				       jo(_kc, leg, HIP_RX_OFFSET, cfik));

    c->absXforms[fidx] = c->absXforms[FRAME_TRUNK] * c->relXforms[fidx];

    ++fidx;

    c->relXforms[fidx] = Transform3f::ry(q.leg_rot[leg][1], 
				     jo(_kc, leg, HIP_RY_OFFSET, cfik));

    c->absXforms[fidx] = c->absXforms[fidx-1] * c->relXforms[fidx];
    
    ++fidx;
    
    c->relXforms[fidx] = Transform3f::ry(q.leg_rot[leg][2], 
				       jo(_kc, leg, KNEE_RY_OFFSET, cfik));
    
    c->absXforms[fidx] = c->absXforms[fidx-1] * c->relXforms[fidx];

    ++fidx;
    
    c->relXforms[fidx] = Transform3f(quatf(0.0f, 0.0f, 0.0f, 1.0f),
				   jo(_kc, leg, FOOT_OFFSET, cfik));

    c->absXforms[fidx] = c->absXforms[fidx-1] * c->relXforms[fidx];
    
  }
  
}


CMUK_ERROR_CODE cmuk::computeFootDK( LegIndex leg,
                                     const vec3f& q, 
                                     mat3f* jac,
                                     vec3f* pos,
                                     vec4f* orient,
                                     mat3f* axes ) const {

  if (!jac) {
    return CMUK_INSUFFICIENT_ARGUMENTS;
  }

  Transform3f t[4];

  CMUK_ERROR_CODE e = computeLegTransforms( leg, q, t );
  if (e != CMUK_OKAY) { return e; }

  mat3f a;

  computeJointAxes( t, &a );

  const vec3f& fpos = t[3].translation();

  computeJacobian( t, a, fpos, jac );

  if (pos) { *pos = fpos; }
  if (orient) { *orient = t[3].rotation(); }
  if (axes) { *axes = a; }

  return CMUK_OKAY;

}


float fix_angle(float angle, float min, float max) {
  if (angle < min) { 
    return angle + 2*M_PI;
  } else if (angle > max) { 
    return angle - 2*M_PI;
  } else {
    return angle;
  }
}

enum IK_CORRECTION_FLAGS {
  IK_UPPER_DISTANCE         = 0x01,
  IK_UPPER_ANGLE_RANGE      = 0x02,
  IK_LOWER_DISTANCE         = 0x04,
  IK_LOWER_ANGLE_RANGE_FWD  = 0x08,
  IK_LOWER_ANGLE_RANGE_REAR = 0x10,
};

static inline int howbad(int flags) {
  int rval = 0;
  if (flags & IK_UPPER_DISTANCE) { rval += 10; }
  if (flags & IK_UPPER_ANGLE_RANGE) { rval += 10; }
  if (flags & IK_LOWER_DISTANCE) { rval += 10; }
  if (flags & IK_LOWER_ANGLE_RANGE_FWD) { rval += 1; }
  if (flags & IK_LOWER_ANGLE_RANGE_REAR) { rval += 1; }
  return rval;
}

static inline CMUK_ERROR_CODE flags_to_errcode(int flags) {


  if ((flags & IK_UPPER_DISTANCE) ||
      (flags & IK_UPPER_ANGLE_RANGE) ||
      (flags & IK_LOWER_DISTANCE)) {
    return CMUK_CANNOT_REACH_IK_KNEE_BENT_EITHER;
  } 

  bool f_bad = (flags & IK_LOWER_ANGLE_RANGE_FWD);
  bool r_bad = (flags & IK_LOWER_ANGLE_RANGE_REAR);

  if (f_bad && r_bad) {
    return CMUK_CANNOT_REACH_IK_KNEE_BENT_EITHER;
  } else if (f_bad) {
    return CMUK_CANNOT_REACH_IK_KNEE_BENT_FORWARD;
  } else if (r_bad) {
    return CMUK_CANNOT_REACH_IK_KNEE_BENT_REARWARD;
  } else {
    return CMUK_OKAY;
  }      

}

CMUK_ERROR_CODE cmuk::computeFootIK( LegIndex leg,
                                     const vec3f& pos,
                                     vec3f* q_bent_forward,
                                     vec3f* q_bent_rearward ) const {

  if ((int)leg < 0 || (int)leg >= NUM_LEGS) {
    return CMUK_BAD_LEG_INDEX;
  } else if (!q_bent_forward || !q_bent_rearward) {
    return CMUK_INSUFFICIENT_ARGUMENTS;
  }

  debug << "*** computing IK...\n";

  int hipflags = 0;

  // subtract off hip position
  vec3f p = pos - jo(_kc, leg, HIP_RX_OFFSET, _centeredFootIK); 
  vec3f orig = pos;

  // get dist from hip rx joint to y rotation plane
  const float& d = jo(_kc, leg, HIP_RY_OFFSET, _centeredFootIK)[1]; 

  // get the squared length of the distance on the plane
  float yz = p[1]*p[1] + p[2]*p[2];

  // alpha is the angle of the foot in the YZ plane with respect to the Y axis
  float alpha = atan2(p[2], p[1]);

  // h is the distance of foot from hip in YZ plane
  float h = sqrt(yz);

  // beta is the angle between the foot-hip vector (projected in YZ
  // plane) and the top hip link.
  float cosbeta = d / h;

  debug << "p = " << p << ", d = " << d << ", yz = " << yz << "\nalpha = " << alpha << ", h = " << h << ", cosbeta=" << cosbeta << "\n";

  if (fabs(cosbeta) > 1) {
    debug << "violated triangle inequality when calculating hip_rx_angle!\n" ;
    if (fabs(cosbeta) - 1 > 1e-4) {
      hipflags = hipflags | IK_UPPER_DISTANCE;
    }
    cosbeta = (cosbeta < 0) ? -1 : 1;
    if (yz < 1e-4) {
      p[1] = d;
      p[2] = 0;
    } else {
      float scl = fabs(d) / h;
      p[1] *= scl;
      p[2] *= scl;
      orig = p + jo(_kc, leg, HIP_RX_OFFSET, _centeredFootIK);
    }
  }

  float beta = acos(cosbeta);

  // Now compute the two possible hip angles
  float hip_rx_angles[2], badness[2];
  int flags[2];

  flags[0] = hipflags;
  flags[1] = hipflags;

  hip_rx_angles[0] = fix_angle(alpha - beta, -M_PI, M_PI);
  hip_rx_angles[1] = fix_angle(alpha + beta, -M_PI, M_PI);

  const float& min = jl(_kc, leg, HIP_RX, 0);
  const float& max = jl(_kc, leg, HIP_RX, 1);

  // See how badly we violate the joint limits for this hip angles
  for (int i=0; i<2; ++i) {
    float& angle = hip_rx_angles[i];
    badness[i] = fabs(compute_badness(angle, min, max));
    if (badness[i]) { flags[i] = flags[i] | IK_UPPER_ANGLE_RANGE; }
  }
  
  // Put the least bad (and smallest) hip angle first
  bool swap = false;

  if ( badness[1] <= badness[0] ) {
    // We want the less bad solution for hip angle
    swap = true;
  } else if (badness[0] == 0 && badness[1] == 0) {
    // We want the solution for hip angle that leaves the hip up.
    if ((leg == FL || leg == HL) && hip_rx_angles[0] > hip_rx_angles[1]) {
      swap = true;
    } else if ((leg == FR || leg == HR) && hip_rx_angles[0] < hip_rx_angles[1]) {
      swap = true;
    }
  } 

  if (swap) {
    std::swap(hip_rx_angles[0], hip_rx_angles[1]);
    std::swap(badness[0], badness[1]);  
    std::swap(flags[0], flags[1]);
  }
  
  int hip_solution_cnt = 2;

  if (badness[0] == 0 && badness[1] != 0) {
    hip_solution_cnt = 1;
  } 

  debug << "hip_rx_angles[0]=" << hip_rx_angles[0] 
        << ", badness=" << badness[0]
        << ", flags=" << flags[0] << "\n";

  debug << "hip_rx_angles[1]=" << hip_rx_angles[1] 
        << ", badness=" << badness[1]
        << ", flags=" << flags[1] << "\n";
  
  debug << "hip_solution_cnt = " << hip_solution_cnt << "\n";

  vec3f qfwd[2], qrear[2];
  
  for (int i=0; i<hip_solution_cnt; ++i) {

    debug << "** computing ll solution " << (i+1) << " of " << (hip_solution_cnt) << "\n";

    float hip_rx = hip_rx_angles[i];
    
    // now make inv. transform to get rid of hip rotation
    Transform3f tx = Transform3f::rx(hip_rx, jo(_kc, leg, HIP_RX_OFFSET, _centeredFootIK));
    vec3f ptx = tx.transformInv(orig);

    debug << "tx=[" << tx.translation() << ", " << tx.rotation() << "], ptx = " << ptx << "\n";
    
    // calculate lengths for cosine law
    float l1sqr = ol2(_kc, leg, KNEE_RY_OFFSET, _centeredFootIK);
    float l2sqr = ol2(_kc, leg, FOOT_OFFSET, _centeredFootIK);
    float l1 = ol(_kc, leg, KNEE_RY_OFFSET, _centeredFootIK);
    float l2 = ol(_kc, leg, FOOT_OFFSET, _centeredFootIK);
    
    float ksqr = ptx[0]*ptx[0] + ptx[2]*ptx[2];
    float k = sqrt(ksqr);

    debug << "l1=" << l1 << ", l2=" << l2 << ", k=" << k << "\n";
    
    // check triangle inequality
    if (k > l1 + l2) { 
      debug << "oops, violated the triangle inequality for lower segments: "
            << "k = " << k << ", "
            << "l1 + l2 = " << l1 + l2 << "\n";
      if (k - (l1 + l2) > 1e-4) {
        flags[i] = flags[i] | IK_LOWER_DISTANCE;
      }
      k = l1 + l2;
      ksqr = k * k;
    }
    
    // 2*theta is the acute angle formed by the spread
    // of the two hip rotations... 
    float costheta = (l1sqr + ksqr - l2sqr) / (2 * l1 * k);
    if (fabs(costheta) > 1) {
      debug << "costheta = " << costheta << " > 1\n";
      if (fabs(costheta) - 1 > 1e-4) {
        flags[i] = flags[i] | IK_LOWER_DISTANCE;
      }
      costheta = (costheta < 0) ? -1 : 1;
    }
    float theta = acos(costheta);
    
    // gamma is the angle of the foot with respect to the z axis
    float gamma = atan2(-ptx[0], -ptx[2]);
    
    // hip angles are just offsets off of gamma now
    float hip_ry_1 = gamma - theta;
    float hip_ry_2 = gamma + theta;
    
    // phi is the obtuse angle of the parallelogram
    float cosphi = (l1sqr + l2sqr - ksqr) / (2 * l1 * l2);
    if (fabs(cosphi) > 1) {
      debug << "cosphi = " << cosphi << " > 1\n";
      if (fabs(cosphi) - 1 > 1e-4) {
        flags[i] = flags[i] | IK_LOWER_DISTANCE;
      }
      cosphi = (cosphi < 0) ? -1 : 1;
    }
    float phi = acos(cosphi);
    
    // epsilon is the "error" caused by not having feet offset directly
    // along the z-axis (if they were, epsilon would equal zero)
    float epsilon = le(_kc, leg, _centeredFootIK);
    
    // now we can directly solve for knee angles
    float knee_ry_1 =  M_PI - phi - epsilon;
    float knee_ry_2 =  -M_PI + phi - epsilon;

    // now fill out angle structs and check limits
    qfwd[i] = vec3f(hip_rx, hip_ry_1, knee_ry_1);
    qrear[i] = vec3f(hip_rx, hip_ry_2, knee_ry_2);
    
    debug << "before wrap, qfwd =  " << qfwd[i] << "\n";
    debug << "before wrap, qrear = " << qrear[i] << "\n";

    check_wrap(_kc, qfwd[i], leg);
    check_wrap(_kc, qrear[i], leg);

    debug << "after wrap, qfwd =  " << qfwd[i] << "\n";
    debug << "after wrap, qrear = " << qrear[i] << "\n";
    
    if (!check_limits(_kc, qfwd[i], leg)) {
      debug << "violated limits forward!\n";
      flags[i] = flags[i] | IK_LOWER_ANGLE_RANGE_FWD;
    }
    if (!check_limits(_kc, qrear[i], leg)) {
      debug << "violated limits rearward!\n";
      flags[i] = flags[i] | IK_LOWER_ANGLE_RANGE_REAR;
    }
    
  } // for each viable hip solution

  int best = 0;

  if (hip_solution_cnt == 2) {
    if (howbad(flags[0]) > howbad(flags[1]))  {
      best = 1;
    }
    debug << "best overall solution is " << (best+1) << "\n";
  }


  *q_bent_forward = qfwd[best];
  *q_bent_rearward = qrear[best];
  return flags_to_errcode(flags[best]);

}

// get joint offsets
//   - CMUK_OK
//   - CMUK_BAD_LEG_INDEX
//   - CMUK_INSUFFICIENT_ARGUMENTS
CMUK_ERROR_CODE cmuk::getJointOffset( LegIndex leg,
					    JointOffset j,
					    vec3f* offset ) const {
  if ((int)leg < 0 || (int)leg >= NUM_LEGS) {
    return CMUK_BAD_LEG_INDEX;
  } else if ((int)j < 0 || (int) j >= NUM_OFFSETS) {
    return CMUK_BAD_JOINT_OFFSET;
  } else if (!offset) {
    return CMUK_INSUFFICIENT_ARGUMENTS;
  }

  *offset = jo(_kc, leg, j, _centeredFootIK);

  return CMUK_OKAY;

}


CMUK_ERROR_CODE cmuk::computePreferredFootIK( LegIndex leg,
						    const vec3f& pos,
						    vec3f* qpreferred ) const {
  
  vec3f q0, q1;
  CMUK_ERROR_CODE status = computeFootIK( leg, pos, &q0, &q1 );
  //std::cerr << "in mzComputeFootIK, q0=" << q0 << ", q1=" << q1 << "\n";
  *qpreferred = choose_leg_angles(_kc, leg, q0, q1);
  if (!check_foot_ik(_kc, leg, status)) { return status; }
  return CMUK_OKAY;

}


CMUK_ERROR_CODE cmuk::computeStanceIK( const vec3f& body_pos,
                                       const quatf& body_rot,
                                       const StanceIKMode fmode[4],
                                       const vec3f fpos[4],
                                       KState* state,
                                       int errflags[4] ) const {
 

  if (!state) {
    return CMUK_INSUFFICIENT_ARGUMENTS;
  }

  Transform3f tx(body_rot, body_pos);

  state->body_pos = body_pos;
  state->body_rot = body_rot;

  CMUK_ERROR_CODE rval = CMUK_OKAY;
 
  for (int i=0; i<4; ++i) {

    if (errflags) { errflags[i] = 0; }

    if (fmode && fmode[i] == STANCE_IK_IGNORE) {
      continue;
    }

    vec3f pos;

    pos = (fmode && fmode[i] == STANCE_IK_BODY) ? 
      fpos[i] : tx.transformInv(fpos[i]);

    vec3f q;
    CMUK_ERROR_CODE status = computePreferredFootIK(LegIndex(i), pos, &q);

    if (status != CMUK_OKAY) {
      if (errflags) { errflags[i] = STANCE_ERR_UNREACHABLE; }
      rval = status;
    }

    state->leg_rot[i] = q;

  }

  return rval;

}

    

CMUK_ERROR_CODE cmuk::computeDKInverse( const mat3f& j,
                                              mat3f* jinv,
                                              float* pdet,
                                              float det_tol,
                                              float lambda ) {

  if (!jinv) { return CMUK_INSUFFICIENT_ARGUMENTS; }

  float det = j.determinant();
  if (pdet) { *pdet = det; }

  if (fabs(det) > det_tol) {
    j.inverse(*jinv, det);
    return CMUK_OKAY;
  } else {
    const float lambda = 0.1;
    mat3f jt, tmp;
    j.transpose(jt);
    tmp = jt * j;
    for (size_t j=0; j<3; ++j) {
      tmp(j,j) += lambda;
    }
    det = tmp.determinant();
    tmp.inverse(*jinv, det);
    *jinv = *jinv * jt;
    return CMUK_SINGULAR_MATRIX;
  }

}
  
CMUK_ERROR_CODE cmuk::computeStanceDK( const vec3f& body_pos,
                                       const quatf& body_rot,
                                       const vec3f& body_vel,
                                       const vec3f& body_angvel,
                                       const StanceIKMode fmode[4],
                                       const vec3f fpos[4],
                                       const vec3f fvel[4],
                                       DState* state,
                                       int errflags[4] ) const {

  if (!state) {
    return CMUK_INSUFFICIENT_ARGUMENTS;
  }

  Transform3f tx(body_rot, body_pos);

  state->body_pos = body_pos;
  state->body_rot = body_rot;
  state->body_vel = body_vel;
  state->body_angvel = body_angvel;

  mat3f j(false), jinv(false);

  CMUK_ERROR_CODE rval = CMUK_OKAY;
 
  for (int i=0; i<4; ++i) {

    errflags[i] = 0;

    if (fmode[i] == STANCE_IK_IGNORE) {
      continue;
    }

    LegIndex leg = LegIndex(i);

    vec3f pos;

    if (fmode[i] == STANCE_IK_BODY) {
      pos = fpos[i];
    } else {
      pos = tx.transformInv(fpos[i]);
    }

    vec3f q;

    CMUK_ERROR_CODE status = computePreferredFootIK(leg, pos, &q);
    state->leg_rot[i] = q;

    // couldn't exactly get there
    if (status != CMUK_OKAY) { 
      errflags[i] = (errflags[i] | STANCE_ERR_UNREACHABLE);
      // No IK -- zero out velocity and set rval
      if (rval == CMUK_OKAY || rval == CMUK_SINGULAR_MATRIX) { 
        rval = status;
      }
      quatf ignore;
      computeFootFK(leg, q, &pos, &ignore);
    }

    // relative velocity of foot in rotating body frame
    vec3f rvel;

    if (fmode[i] == STANCE_IK_BODY) {

      rvel = fvel[i];

    } else {

      // foot offset in world frame
      vec3f r = tx.transformFwd(pos) - body_pos;

      // foot velocity in world frame due to body if no joint movement
      vec3f fvel_w = body_vel + vec3f::cross(body_angvel, r);    
      
      // rel velocity in world frame
      vec3f fvel_rel_w = fvel[i] - fvel_w;
      
      // rel velocity in body frame
      // TODO: double-check whether this is fwd or inv?
      rvel = tx.rotFwd() * fvel_rel_w;

    }

    computeFootDK(leg, q, &j);
    status = computeDKInverse(j, &jinv);

    state->leg_rotvel[i] = jinv * rvel;

    // if jacobian was singular, make a note of it
    if (status != CMUK_OKAY) {
      errflags[i] = (errflags[i] | STANCE_ERR_SINGULAR);
      if (rval == CMUK_OKAY) { rval = status; }
    }

  }

  return rval;


}


CMUK_ERROR_CODE cmuk::getJointLimits( LegIndex leg,
                                      vec3f* minrot,
                                      vec3f* maxrot ) const {
  
  if ((int)leg < 0  || (int) leg >= NUM_LEGS) {
    return CMUK_BAD_LEG_INDEX;
  }
  if (!minrot || !maxrot) { return CMUK_INSUFFICIENT_ARGUMENTS; }
  
  for (int i=0; i<3; ++i) {
    (*minrot)[i] = jl(_kc, leg, i, 0);
    (*maxrot)[i] = jl(_kc, leg, i, 1);
  }
  
  return CMUK_OKAY;

}

const vec3f& cmuk::getBodyCGOffset() const {
  return _kc.body_cg_offset;
};

const cmuk::KConstants& cmuk::getConstants() const {
  return _kc;
}

// get body CG dist from support quadrilateral / triangle
//    positve = inside, negative = outside, 
float cmuk::getBodyCGDist( const XformCache& cache, 
			   int flight_foot, 
			   vec3f* normal,
			   vec3f* centroid,
			   vec3f* bodyCG,
			   size_t* ncvx,
			   cmuk::LegIndex* cidx,
			   vec3f* cpos) const {

  static const cmuk::LegIndex loop[4] = {
    FL, FR, HL, HR
  };

  size_t num_stance = 0;
  vec3f fpos[4];
  cmuk::LegIndex lused[4];
  vec3f ctr(0,0,0);
  
  const Transform3f& ftrunk = cache.absXforms[FRAME_TRUNK];

  for (size_t i=0; i<4; ++i) {
    LegIndex leg = loop[i];
    if (flight_foot == leg) { continue; }
    fpos[num_stance] = getFootWorldPosition(cache, leg);
    lused[num_stance] = leg;
    ctr = ctr + fpos[num_stance];
    ++num_stance;
  }

  if (centroid) {
    ctr = ctr / float(num_stance);
    *centroid = ctr;
  }

  vec3f cg = ftrunk * getBodyCGOffset();

  if (bodyCG) { *bodyCG = cg; }

  size_t idx[4];
  size_t num_convex = graham2D(num_stance, fpos, idx);
  
  if (ncvx) { *ncvx = num_convex; }

  if (cidx) {
    for (size_t i=0; i<num_convex; ++i) {
      cidx[i] = lused[idx[i]];
    }
  }

  if (cpos) {
    for (size_t i=0; i<num_convex; ++i) {
      cpos[i] = fpos[idx[i]];
    }
  }

  return pcpDist(cg, num_convex, fpos, idx, normal);

}


vec3f cmuk::getFootWorldPosition(const XformCache& cache, LegIndex leg) const {
  assert((int)leg >= 0 && (int)leg <= 3);
  const Transform3f& t = cache.absXforms[FRAME_FL_FOOT + 4*(int)leg];
  return t.translation();
}

const char* cmuk::getErrorString(int status) {

#define HANDLE(arg) case arg: return #arg

  switch (status) {
    HANDLE(CMUK_OKAY);
    HANDLE(CMUK_BAD_LEG_INDEX);
    HANDLE(CMUK_BAD_JOINT_OFFSET);
    HANDLE(CMUK_ANGLE_OUT_OF_RANGE);
    HANDLE(CMUK_SINGULAR_MATRIX);
    HANDLE(CMUK_INSUFFICIENT_ARGUMENTS);
    HANDLE(CMUK_CANNOT_REACH_IK_KNEE_BENT_FORWARD);
    HANDLE(CMUK_CANNOT_REACH_IK_KNEE_BENT_REARWARD);
    HANDLE(CMUK_CANNOT_REACH_IK_KNEE_BENT_EITHER);
    HANDLE(CMUK_BAD_QUATERNION_VECTOR);
    HANDLE(CMUK_BAD_FRAME_INDEX);
  default: break;
  }
  return "*** INVALID STATUS ***";

#undef HANDLE

}

bool cmuk::usingCenteredFootIK() const {
  return _centeredFootIK;
}

void cmuk::setCenteredFootIK(bool c) {
  if (_centeredFootIK != c) {
    _centeredFootIK = c;
  }
}

void cmuk::setDebugOutput(bool d) {
  _debugOutput = d;
}

bool cmuk::debugOutput() const {
  return _debugOutput;
}

void cmuk::setConstants(const KConstants& kc) {
  _kc = kc;
}


const char* cmuk::getJointOffsetString(int j) {

#define HANDLE(arg) case arg: return #arg

  switch (j) {
    HANDLE(HIP_RX_OFFSET);
    HANDLE(HIP_RY_OFFSET);
    HANDLE(KNEE_RY_OFFSET);
    HANDLE(FOOT_OFFSET);
  default: break;
  }
  return "*** INVALID JOINT OFFSET ***";

#undef HANDLE

}

const char* cmuk::getFrameString(int i) {

#define HANDLE(arg) case arg: return #arg

  switch (i) {
    HANDLE(FRAME_GROUND);
    HANDLE(FRAME_TRUNK);
    HANDLE(FRAME_FL_HIP_RX);
    HANDLE(FRAME_FL_HIP_RY);
    HANDLE(FRAME_FL_KNEE_RY);
    HANDLE(FRAME_FL_FOOT);
    HANDLE(FRAME_FR_HIP_RX);
    HANDLE(FRAME_FR_HIP_RY);
    HANDLE(FRAME_FR_KNEE_RY);
    HANDLE(FRAME_FR_FOOT);
    HANDLE(FRAME_HL_HIP_RX);
    HANDLE(FRAME_HL_HIP_RY);
    HANDLE(FRAME_HL_KNEE_RY);
    HANDLE(FRAME_HL_FOOT);
    HANDLE(FRAME_HR_HIP_RX);
    HANDLE(FRAME_HR_HIP_RY);
    HANDLE(FRAME_HR_KNEE_RY);
    HANDLE(FRAME_HR_FOOT);
  default: break;
  }
  return "*** INVALID FRAME ***";

#undef HANDLE

}

const char* cmuk::getLegAbbrev(int f) {
  switch (f) {
  case FL: return "fl";
  case FR: return "fr";
  case HL: return "hl";
  case HR: return "hr";
  default: break;
  }
  return "none"; 
}

const char* cmuk::getLegString(int f) {
  switch (f) {
  case FL: return "front left";
  case FR: return "front right";
  case HL: return "hind left";
  case HR: return "hind right";
  default: break;
  }
  return "none"; 
}

const char* cmuk::getJointString(int j) {
  switch (j) {
  case 0: return "hip_rx";
  case 1: return "hip_ry";
  case 2: return "knee_ry";
  default: break;
  }
  return "none";
}


cmuk::KState::KState():
  body_pos(0,0,0), 
  body_rot(0,0,0,1) {


  for (int i=0; i<cmuk::NUM_LEGS; ++i) {
    leg_rot[i] = vec3f(0,0,0);
  }
  
}


cmuk::DState::DState():
  body_vel(0),
  body_angvel(0)
{
  for (int i=0; i<cmuk::NUM_LEGS; ++i) {
    leg_rotvel[i] = vec3f(0);
  }
}

void cmuk::computeVelocities(const DState& state,
                             const XformCache& cache,
                             const vec3f& roffs,
                             vec3f linvel[],
                             vec3f angvel[]) const {

  if (!linvel || !angvel) { return; }

  vec3f woffs = cache.absXforms[0].rotFwd() * roffs;

  linvel[0] = state.body_vel + vec3f::cross(angvel[0], -woffs);
  angvel[0] = state.body_angvel;

  for (int i=0; i<NUM_LEGS; ++i) {

    static const vec3f raxis[3] = {
      vec3f(1, 0, 0),
      vec3f(0, 1, 0),
      vec3f(0, 1, 0)
    };

    for (int j=0; j<4; ++j) {

      int fidx = FRAME_FL_HIP_RX + 4*i + j;
      int pidx = parentFrame(fidx);

      const Transform3f& t0 = cache.absXforms[pidx];
      const Transform3f& t1 = cache.absXforms[fidx];
      
      const vec3f& x0 = t0.translation();
      const vec3f& x1 = t1.translation();

      vec3f d = x1 - x0;
      if (pidx == 0) {
        d -= woffs;
      }

      linvel[fidx] = linvel[pidx] + vec3f::cross(angvel[pidx], d);

      if (j < 3) {
        vec3f v = t0.rotFwd() * raxis[j];
        angvel[fidx] = angvel[pidx] + state.leg_rotvel[i][j] * v;
      } else {
        angvel[fidx] = angvel[pidx];
      }

    }

  }
  

}
                             
// Get position of feet
void cmuk::getFootWorldPositions( const KState& state, vec3f fpos[4] ) const {
  Transform3f tbody(state.body_rot, state.body_pos);
  vec3f p;
  quatf q;
  for (int i=0; i<4; ++i) {
    computeFootFK( LegIndex(i), state.leg_rot[i], &p, &q );
    fpos[i] = tbody*p;
  }
}

const char* cmuk::getStanceIKString(int mode) {
  switch (mode) {
  case STANCE_IK_IGNORE:  return "STANCE_IK_IGNORE";
  case STANCE_IK_BODY:    return "STANCE_IK_BODY";
  case STANCE_IK_WORLD:   return "STANCE_IK_WORLD";
  case STANCE_IK_SUPPORT: return "STANCE_IK_SUPPORT";
  default:
    break;
  }
  return "INVALID";
}

const char* cmuk::getStanceErrString(int err) {
  if (err & STANCE_ERR_UNREACHABLE) {
    if (err & STANCE_ERR_SINGULAR) {
      return "UNREACHABLE, SINGULAR";
    } else {
      return "UNREACHABLE";
    }
  } else if (err & STANCE_ERR_SINGULAR) {
    return "SINGULAR";
  } else {
    return "OKAY";
  }
}

/*
vec3f cmuk::jointToMotor(const vec3f& q) {
  return vec3f(q[0]+q[1], q[0]-q[1], 0.25*q[2]);
}

vec3f cmuk::motorToJoint(const vec3f& m) {
  return vec3f(0.5*(m[0]+m[1]), 0.5*(m[0]-m[1]), 4*m[2]);
}
*/

vec3f cmuk::clampToLimits(cmuk::LegIndex leg, const vec3f& q) const {
  vec3f rval = q;
  check_limits(_kc, rval, leg);
  return rval;
}

CMUK_ERROR_CODE cmuk::computeLegTransforms( LegIndex leg,
                                            const vec3f& q,
                                            Transform3f t[4] ) const {
  
  if ((int)leg < 0 || (int)leg >= NUM_LEGS) {
    return CMUK_BAD_LEG_INDEX;
  }
  if (!t) {
    return CMUK_INSUFFICIENT_ARGUMENTS;
  }

  t[0] = Transform3f::rx(q[0], jo(_kc, leg, HIP_RX_OFFSET, _centeredFootIK));
  t[1] = t[0] * Transform3f::ry(q[1], jo(_kc, leg, HIP_RY_OFFSET, _centeredFootIK));
  t[2] = t[1] * Transform3f::ry(q[2], jo(_kc, leg, KNEE_RY_OFFSET, _centeredFootIK));
  t[3] = t[2] * Transform3f(quatf(0.0f, 0.0f, 0.0f, 1.0f),
		    jo(_kc, leg, FOOT_OFFSET, _centeredFootIK));
  
  return CMUK_OKAY;

}

void cmuk::computeJointAxes( const Transform3f t[4], mat3f* a ) {

  assert(t);
  assert(a);

  a->setCol(0, t[0].rotFwd() * vec3f(1,0,0));
  a->setCol(1, t[1].rotFwd() * vec3f(0,1,0));
  a->setCol(2, t[2].rotFwd() * vec3f(0,1,0));

}

void cmuk::computeJacobian( const Transform3f t[4],
                            const mat3f& a, 
                            const vec3f& p,
                            mat3f* jac,
                            int maxlink ) {

  for (int joint=0; joint<3; ++joint) {
    const vec3f& j = t[joint].translation();
    vec3f dpdj(0);
    if (joint <= maxlink) { dpdj = vec3f::cross(a.col(joint), (p - j)); }
    for (int axis=0; axis<3; ++axis) {
      (*jac)(axis, joint) = dpdj[axis];
    }
  }
  
}



CMUK_ERROR_CODE cmuk::computePointDK( LegIndex leg,
                                      JointOffset link,
                                      const vec3f& q,
                                      const vec3f& p,
                                      mat3f* jac,
                                      mat3f* axes) const {

  if ((int)link < HIP_RX_OFFSET || (int)link >= NUM_OFFSETS) {
    return CMUK_BAD_JOINT_OFFSET;
  }
  if (!jac) {
    return CMUK_INSUFFICIENT_ARGUMENTS;
  }

  Transform3f t[4];

  CMUK_ERROR_CODE e = computeLegTransforms( leg, q, t );
  if (e != CMUK_OKAY) { return e; }

  mat3f a;

  computeJointAxes( t, &a );

  computeJacobian( t, a, p, jac, link );

  if (axes) { *axes = a; }

  return CMUK_OKAY;

}
  
CMUK_ERROR_CODE cmuk::computePointFootDK( LegIndex leg,
                                          JointOffset  link,
                                          const vec3f& q,
                                          const vec3f& p,
                                          mat3f* jac,
                                          vec3f* fpos,
                                          float* det,
                                          float det_tol,
                                          float lambda ) const {

  if ((int)link < HIP_RX_OFFSET || (int)link >= NUM_OFFSETS) {
    return CMUK_BAD_JOINT_OFFSET;
  }
  if (!jac) {
    return CMUK_INSUFFICIENT_ARGUMENTS;
  }

  if (link == FOOT_OFFSET) { link = KNEE_RY_OFFSET; }

  Transform3f t[4];

  CMUK_ERROR_CODE e = computeLegTransforms( leg, q, t );
  if (e != CMUK_OKAY) { return e; }

  mat3f a;

  computeJointAxes( t, &a );

  mat3f J_p, J_f, J_f_inv;

  vec3f f;
  f = t[3].translation();
  
  computeJacobian( t, a, f, &J_f );
  computeJacobian( t, a, p, &J_p, link );

  CMUK_ERROR_CODE err = computeDKInverse( J_f, &J_f_inv, 
                                          det, det_tol, lambda );
  
  *jac = J_p * J_f_inv;
  if (fpos) { *fpos = f; }

  return err;
    
}

vec3f cmuk::computeWorldPointRel( FrameIndex frame,
                                  const KState& s0,
                                  const vec3f& p0,
                                  const KState& s1 ) const {

  if (frame < FRAME_TRUNK || frame > FRAME_HR_FOOT) {
    return p0;
  }

  vec3f b = s0.xform().transformInv(p0);

  if (frame == FRAME_TRUNK) {
    return s1.xform().transformFwd(b);
  } else {
    // get the leg offset and deal
    LegIndex leg = LegIndex((frame-1)/4);
    if (s1.leg_rot[leg] == s0.leg_rot[leg]) {
      return s1.xform().transformFwd(b);
    }
    JointOffset joint = JointOffset((frame-1)%4);
    Transform3f x0[4];
    Transform3f x1[4];
    computeLegTransforms(leg, s0.leg_rot[leg], x0);
    computeLegTransforms(leg, s1.leg_rot[leg], x1);
    vec3f l = x0[joint].transformInv(b);
    b = x1[joint].transformFwd(l);
    return s1.xform().transformFwd(b);
  }
  
}

CMUK_ERROR_CODE cmuk::computeWorldPointFootDK( LegIndex leg,
                                               JointOffset link,
                                               const KState& state,
                                               const vec3f& pworld,
                                               mat3f* J_trans,
                                               mat3f* J_rot,
                                               float* det,
                                               float det_tol,
                                               float lambda ) const {

  if ((int)leg < 0 || (int)leg >= NUM_LEGS) {
    return CMUK_BAD_LEG_INDEX;
  }
  if (!J_trans || !J_rot) {
    return CMUK_INSUFFICIENT_ARGUMENTS;
  }
    
  Transform3f xform = state.xform();
  vec3f pbody = xform.transformInv(pworld);
  vec3f fbody;

  mat3f J;

  CMUK_ERROR_CODE err = 
    computePointFootDK( leg, link, state.leg_rot[leg], pbody, &J,
                        &fbody, det, det_tol, lambda );

  if (err != CMUK_OKAY && err != CMUK_SINGULAR_MATRIX) {
    return err;
  }

  const mat3f& R = xform.rotFwd();
  const mat3f& Rinv = xform.rotInv();

  *J_trans = mat3f::identity() - R * J * Rinv;
  //*J_rot = R * (-mat3f::cross(pbody) + J * mat3f::cross(fbody));
  *J_rot = R * ( J * mat3f::cross(fbody) - mat3f::cross(pbody) ) * Rinv;

  return err;

}

vec3f cmuk::clampSpeed(const vec3f& q0, 
                       const vec3f& q1,
                       float dt) const {

  vec3f diff = q1 - q0;
  float t = computeMinLegTime(diff);

  if (t > dt) {
    return q0 + diff * (dt/t);
  } else {
    return q1;
  }

}


  // compute the time it takes to move from q0 to q1
float cmuk::computeMinLegTime(const vec3f& q0, const vec3f& q1) const {
  return computeMinLegTime(q1-q0);
}

float cmuk::computeMinLegTime(const vec3f& diff) const {  

  float d_hip = fabs(diff[0]) + fabs(diff[1]);
  float d_knee = fabs(diff[2]);

  float t_hip = d_hip / _kc.max_hip_speed;
  float t_knee = d_knee / _kc.max_knee_speed;

  return std::max(t_hip, t_knee);

}
