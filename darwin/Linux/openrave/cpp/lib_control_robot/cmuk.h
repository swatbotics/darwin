#ifndef _CMUDOGKINEMATICS_H_
#define _CMUDOGKINEMATICS_H_

#include <math/Transform3.h>
#include <math/mat3.h>

enum CMUK_ERROR_CODE {
  CMUK_OKAY = 0,                           //!<  No error
  CMUK_BAD_LEG_INDEX,                      //!<  Invalid leg index
  CMUK_BAD_JOINT_OFFSET,                   //!<  Invalid joint offset
  CMUK_ANGLE_OUT_OF_RANGE,                 //!<  Passed angle is out of accepted range. Value clipped.
  CMUK_SINGULAR_MATRIX,                    //!<  A mathematical operation produced a singularity
  CMUK_INSUFFICIENT_ARGUMENTS,             //!<  Not enough arguments
  CMUK_CANNOT_REACH_IK_KNEE_BENT_FORWARD,  //!<  
  CMUK_CANNOT_REACH_IK_KNEE_BENT_REARWARD, //!<  
  CMUK_CANNOT_REACH_IK_KNEE_BENT_EITHER,   //!<  
  CMUK_BAD_QUATERNION_VECTOR,              //!<  Invalid unit quaternion (not unit length)
  CMUK_BAD_FRAME_INDEX,                    //!<  An invalid frame was passed to the robot
};

class CmuDogKinematics;

// ahh, less typing!
typedef CmuDogKinematics cmuk;

class CmuDogKinematics {
public:

  // Can be safely cast to LittleDog::LegIndex
  enum LegIndex {
    FL=0, FR, HL, HR, NUM_LEGS
  };

  // Can be safely cast to LittleDog::JointOffset
  enum JointOffset {
    HIP_RX_OFFSET=0,
    HIP_RY_OFFSET,
    KNEE_RY_OFFSET,
    FOOT_OFFSET,
    NUM_OFFSETS
  };

  enum JointIndex {
    HIP_RX=0,
    HIP_RY,
    KNEE_RY,
    NUM_JOINTS
  };

  // Coordinate frame of various parts of the dog
  enum FrameIndex {

    FRAME_GROUND = -1,

    FRAME_TRUNK  = 0,

    FRAME_FL_HIP_RX,
    FRAME_FL_HIP_RY,
    FRAME_FL_KNEE_RY,
    FRAME_FL_FOOT,

    FRAME_FR_HIP_RX,
    FRAME_FR_HIP_RY,
    FRAME_FR_KNEE_RY,
    FRAME_FR_FOOT,

    FRAME_HL_HIP_RX,
    FRAME_HL_HIP_RY,
    FRAME_HL_KNEE_RY,
    FRAME_HL_FOOT,

    FRAME_HR_HIP_RX,
    FRAME_HR_HIP_RY,
    FRAME_HR_KNEE_RY,
    FRAME_HR_FOOT,

    NUM_FRAMES

  };

  enum StanceIKMode {
    STANCE_IK_IGNORE,   // Joint angles are specified, no transform needed
    STANCE_IK_BODY,     // Body-frame cartesian
    STANCE_IK_WORLD,    // World-frame cartesian
    STANCE_IK_SUPPORT,  // World-frame, and the leg is holding up the dog
  };

  enum StanceErrorFlags {
    STANCE_ERR_UNREACHABLE = 0x01,
    STANCE_ERR_SINGULAR    = 0x02,
  };

  // Constants for a dog kinematic model. 
  struct KConstants {

    float h_body;               // Height of body in m
    float l_body;               // Length of body in m
    float w_body_bottom;        // Width of bottom of body
    float r_foot;               // Radius of foot in m
    float d_hip_body_bottom;    // Vertical distance between hip joint & bottom of body
    float natural_height;       // Height of body when all joints at zero and feet on ground
    vec3f body_cg_offset;       // Offset of body CG relative to dog origin
    float d_foot_shin;          // Horiz offset between foot & shin

    // Joint offsets - each leg is 4 consecutive entries
    vec3f  joint_offsets[NUM_OFFSETS * NUM_LEGS];   
    
    // Joint limits - each leg is 6 consecutive entries, with each
    // joint being 2 consecutive entries
    float joint_limits[NUM_JOINTS * NUM_LEGS * 2]; 
    
    // Prefer knee forward for specified joint?
    bool  prefer_knee_forward[NUM_LEGS];      

    // Mass of body
    float m_body;

    // Upper leg mass
    float m_uleg;

    // Lower leg mass
    float m_lleg;

    // inertia tensor of body
    mat3f I_body;

    // Max hip joints speed (sum of hip_rx speed and hip_ry speed)
    float max_hip_speed;

    // Max knee speed
    float max_knee_speed;

    float skel_body_thickness;
    float skel_uleg_rad;
    float skel_knee_rad;
    float skel_lleg_rad;
    
  };

  // Kinematic state of dog
  struct KState {

    vec3f body_pos;
    quatf body_rot;
    vec3f leg_rot[NUM_LEGS];
    
    KState();

    Transform3f xform() const {
      return Transform3f(body_rot, body_pos);
    }
    
  };

  // Dynamic state of dog -- I'll do something with this someday!
  struct DState: public KState {
    
    vec3f body_vel;
    vec3f body_angvel;
    vec3f leg_rotvel[NUM_LEGS];
    
    DState();
    
  };

  // Transform cache of dog (created from KState, used when querying
  // multiple transformations)

  struct XformCache {

    Transform3f absXforms[NUM_FRAMES];
    Transform3f relXforms[NUM_FRAMES];

  };

  //////////////////////////////////////////////////////////////////////

  CmuDogKinematics();

  static bool isFront(int legIndex);
  static bool isLeft(int legIndex);

  // Get the constants used by this model
  const KConstants& getConstants() const;
  bool usingCenteredFootIK() const;
  bool debugOutput() const;

  void setCenteredFootIK(bool c);
  void setDebugOutput(bool d);

  // Probably don't want to do this, but allows changing model without recompilation
  void setConstants(const KConstants& c);

  //////////////////////////////////////////////////////////////////////
  // Here is the part that mirrors the LittleDog API:

  // compute foot forward kinematics (position & orientation)
  //   - CMUK_OKAY
  //   - CMUK_BAD_LEG_INDEX
  //   - CMUK_INSUFFICIENT_ARGUMENTS (if stuff is null)
  CMUK_ERROR_CODE computeFootFK( LegIndex leg,
				 const vec3f& q,
				 vec3f* pos,
				 vec4f* orient=0 ) const;

  // compute IK
  //   - CMUK_OKAY
  //   - CMUK_BAD_LEG_INDEX
  //   - CMUK_INSUFFICIENT_ARGUMENTS
  //   - CMUK_CANNOT_REACH_IK_KNEE_BENT_FORWARD
  //   - CMUK_CANNOT_REACH_IK_KNEE_BENT_REARWARD
  //   - CMUK_CANNOT_REACH_IK_KNEE_BENT_EITHER
  CMUK_ERROR_CODE computeFootIK( LegIndex leg,
				 const vec3f& pos,
				 vec3f* q_bent_forward,
				 vec3f* q_bent_rearward ) const;
  
  // compute DK (jacobian)
  //   - CMUK_OKAY
  //   - CMUK_BAD_LEG_INDEX
  //   - CMUK_INSUFFICIENT_ARGUMENTS
  CMUK_ERROR_CODE computeFootDK( LegIndex leg,
				 const vec3f& q,
				 mat3f* jac,
                                 vec3f* pos=0,
                                 vec4f* orient=0,
                                 mat3f* axes=0 ) const;


  // compute either inverse or regularized pseudoinverse, depending on determinant
  // returns either
  //   - CMUK_OKAY
  //   - CMUK_SINGULAR_MATRIX
  //   - CMUK_INSUFFICIENT_ARGUMENTS
  static CMUK_ERROR_CODE computeDKInverse( const mat3f& jac,
                                           mat3f* jinv,
                                           float* det=0,
                                           float det_tol=1e-6,
                                           float lambda=1e-1 );
  
  // get joint offsets
  //   - CMUK_OKAY
  //   - CMUK_BAD_LEG_INDEX
  //   - CMUK_INSUFFICIENT_ARGUMENTS
  CMUK_ERROR_CODE getJointOffset( LegIndex leg,
				  JointOffset joint_offset,
				  vec3f* offset ) const;

  
  //////////////////////////////////////////////////////////////////////
  // Here is the part that is beyond the LittleDog API:

  // Get position of feet
  void getFootWorldPositions( const KState& state, vec3f fpos[4] ) const;

  // Get the body cg offset
  const vec3f& getBodyCGOffset() const;

  // Get the joint limits for the given joint
  CMUK_ERROR_CODE getJointLimits( LegIndex leg, vec3f* minrot, vec3f* maxrot ) const;

  // Compute the joint angles for the legs that leave them
  // in the given world positions, and puts the body where it is
  // supposed to go
  //   - CMUK_OKAY
  //   - CMUK_BAD_QUATERNION_VECTOR
  //   - CMUK_CANNOT_REACH_IK_KNEE_BENT_FORWARD,
  //   - CMUK_CANNOT_REACH_IK_KNEE_BENT_REARWARD,
  //   - CMUK_CANNOT_REACH_IK_KNEE_BENT_EITHER
  //   - CMUK_INSUFFICIENT_ARGUMENTS
  CMUK_ERROR_CODE computeStanceIK( const vec3f& body_pos,
				   const quatf& body_rot,
                                   const StanceIKMode fmode[4],
				   const vec3f cmd_pos[4],
				   KState* state,
                                   int errflags[4] ) const;

  // Compute the joint angles for the legs that leave them
  // in the given world positions, and puts the body where it is
  // supposed to go
  //   - CMUK_OKAY
  //   - CMUK_BAD_QUATERNION_VECTOR
  //   - CMUK_CANNOT_REACH_IK_KNEE_BENT_FORWARD,
  //   - CMUK_CANNOT_REACH_IK_KNEE_BENT_REARWARD,
  //   - CMUK_CANNOT_REACH_IK_KNEE_BENT_EITHER
  //   - CMUK_SINGULAR_MATRIX
  //   - CMUK_INSUFFICIENT_ARGUMENTS
  CMUK_ERROR_CODE computeStanceDK( const vec3f& body_pos,
                                   const quatf& body_rot,
                                   const vec3f& body_vel,
                                   const vec3f& body_angvel,
                                   const StanceIKMode fmode[4],
                                   const vec3f cmd_pos[4],
                                   const vec3f cmd_vel[4],
                                   DState* state,
                                   int errflags[4] ) const;
  
  // Compute foot IK but return an error if we can't bend the leg in
  // the desired direction (front legs have knees to rear, hind legs
  // have knees to front).
  //   - CMUK_OKAY
  //   - CMUK_BAD_LEG_INDEX
  //   - CMUK_INSUFFICIENT_ARGUMENTS
  //   - CMUK_CANNOT_REACH_IK_KNEE_BENT_FORWARD,
  //   - CMUK_CANNOT_REACH_IK_KNEE_BENT_REARWARD,
  //   - CMUK_CANNOT_REACH_IK_KNEE_BENT_EITHER
  CMUK_ERROR_CODE computePreferredFootIK( LegIndex leg,
					  const vec3f& pos,
					  vec3f* qpreferred ) const;

  // Create a transform cache (various functions below use it)
  void cacheTransforms( const KState& state,
			XformCache* cache ) const;
    
  // Get the absolute transform to the given frame
  //   - CMUK_OKAY
  //   - CMUK_BAD_FRAME_INDEX
  //   - CMUK_INSUFFICIENT_ARGUMENTS
  CMUK_ERROR_CODE getAbsTransform( const XformCache& cache,
				   FrameIndex frame, 
				   Transform3f* transform ) const;

  // Get the rigid body transform to go between the two frames
  //   - CMUK_OKAY
  //   - CMUK_BAD_FRAME_INDEX
  //   - CMUK_INSUFFICIENT_ARGUMENTS
  CMUK_ERROR_CODE getRelTransform( const XformCache& cache,
				   FrameIndex f0,
				   FrameIndex f1,
				   Transform3f* transform ) const;

  // This computes the linear and angular velocities of all of the
  // frames given a dynamics state and a transform cache which was
  // created around that state.
  //
  // The velocities are indexed by frame.
  
  void computeVelocities(const DState& state,
                         const XformCache& cache,
                         const vec3f& roffs,
                         vec3f linvel[],
                         vec3f angvel[]) const;
  
  
  // Get body CG dist from support quadrilateral / triangle
  //    positve = inside, negative = outside, 
  float getBodyCGDist( const XformCache& cache,
		       int flight_foot,
		       vec3f* normal=0,
		       vec3f* centroid=0,
		       vec3f* bodycg=0,
		       size_t* ncvx=0,
		       LegIndex* cidx=0,
		       vec3f* cpos=0 ) const;
  
  // helper function used by computeFootFK, computeFootDK,
  // computePointDK, computePointFootDK
  CMUK_ERROR_CODE computeLegTransforms( LegIndex leg,
                                        const vec3f& q,
                                        Transform3f xforms[4] ) const;

  // helper function used by computeFootDK, computePointDK,
  // computePointFootDK
  static void computeJointAxes( const Transform3f xforms[4],
                                mat3f* axes );

  // helper function used by computeFootDK, computePointDK,
  // computePointFootDK
  static void computeJacobian( const Transform3f xforms[4],
                               const mat3f& axes,
                               const vec3f& p,
                               mat3f* jac,
                               int maxlink = FOOT_OFFSET );


  // compute the "point jacobian" in body frame.  P must be expressed
  // in BODY coordinates.
  //
  // if JointOffset is FOOT_OFFSET or KNEE_RY_OFFSET and p is equal to
  // the foot position in BODY coordinates for the configuration q,
  // this is equal to computeFootDK.
  //
  //   - CMUK_OKAY
  //   - CMUK_BAD_LEG_INDEX
  //   - CMUK_BAD_JOINT_OFFSET
  //   - CMUK_INSUFFICIENT_ARGUMENTS
  CMUK_ERROR_CODE computePointDK( LegIndex leg,
                                  JointOffset link,
                                  const vec3f& q,
                                  const vec3f& p,
                                  mat3f* J,
                                  mat3f* axes=0 ) const;
  
  // compute a matrix dp/df showing how the point p moves in body
  // coords if the foot is moved via IK.  p must be expressed in BODY
  // coordinates. since this calls computeDKInverse, the final 3
  // arguments are the same as for computeDKInverse
  //
  // if JointOffset is FOOT_OFFSET or KNEE_RY_OFFSET and p is equal to
  // the foot position in BODY coordinates for the configuration q,
  // this should return the identity matrix.
  //
  // if fpos is supplied, the vector is set to foot position in body
  // coords
  //
  //   - CMUK_OKAY
  //   - CMUK_BAD_LEG_INDEX
  //   - CMUK_BAD_JOINT_OFFSET
  //   - CMUK_INSUFFICIENT_ARGUMENTS
  //   - CMUK_SINGULAR_MATRIX
  CMUK_ERROR_CODE computePointFootDK( LegIndex leg,
                                      JointOffset  link,
                                      const vec3f& q,
                                      const vec3f& p,
                                      mat3f* J,
                                      vec3f* fpos=0,
                                      float* det=0,
                                      float  det_tol=1e-6,
                                      float  lambda=1e-1 ) const;

  // just as above, but do this in world coords.
  // calls the above function
  //
  // can return:
  //
  //   - CMUK_OKAY
  //   - CMUK_BAD_LEG_INDEX
  //   - CMUK_BAD_JOINT_OFFSET
  //   - CMUK_INSUFFICIENT_ARGUMENTS
  //   - CMUK_SINGULAR_MATRIX

  CMUK_ERROR_CODE computeWorldPointFootDK( LegIndex leg,
                                           JointOffset link,
                                           const KState& state,
                                           const vec3f& pworld,
                                           mat3f* J_trans,
                                           mat3f* J_rot,
                                           float* det=0,
                                           float  det_tol=1e-6,
                                           float  lambda=1e-1 ) const;

  // compute new world position for a point attached to the given frame.
  // at state s0, the point has position p0 in world coordinates.
  // this returns the world coordinates of the point at state s1
  vec3f computeWorldPointRel( FrameIndex frame,
                              const KState& s0,
                              const vec3f& p0,
                              const KState& s1 ) const;

  // Get the position of the foot in the world
  vec3f getFootWorldPosition(const XformCache& cache, LegIndex leg) const;

  vec3f clampToLimits(LegIndex leg, const vec3f& q) const;

  // we want to go from q0 to q1 in dt, but might not get there.
  // so return the furthest you can get in that direction in time dt
  vec3f clampSpeed(const vec3f& q0, 
                   const vec3f& q1,
                   float dt) const;

  // compute the time it takes to move from q0 to q1
  float computeMinLegTime(const vec3f& q0, const vec3f& q1) const;

  // compute the time it takes to move delta q
  float computeMinLegTime(const vec3f& dq) const;

  static const char* getErrorString(int status);
  static const char* getStanceIKString(int sik);
  static const char* getStanceErrString(int err);
  static const char* getLegAbbrev(int leg);
  static const char* getJointString(int j);
  static const char* getLegString(int leg);
  static const char* getJointOffsetString(int j);
  static const char* getFrameString(int i);


private:

  KConstants _kc;
  bool _centeredFootIK;
  bool _debugOutput;

};

inline std::ostream& operator<<(std::ostream& ostr, const cmuk::KState& state) {
  ostr << "state:\n";
  ostr << "  pos:    " << state.body_pos << "\n";
  ostr << "  rot:    " << state.body_rot << "\n";
  for (int i=0; i<4; ++i) {
    ostr << "  " << cmuk::getLegAbbrev(i) << " rot: " << state.leg_rot[i] << "\n";
  }
  return ostr;
}

#endif
