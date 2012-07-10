#ifndef _KINEMATICS_
#define _KINEMATICS_

#include "ForwardKinematics.h"
#define IKFAST_NAMESPACE leftleg
#include "ik_leftleg.cpp"
#undef IKFAST_NAMESPACE
#define IKFAST_NAMESPACE rightleg
#include "ik_rightleg.cpp"
#undef IKFAST_NAMESPACE

enum FootFrame{LeftFoot, RightFoot};

class Kinematics{
 public:
  ForwardKinematics fKin;
  
  Kinematics();
  void getJacobian(int attachedFrame, float position[], float result[][14]);
  void changeReferenceFrame();
  void changeReferenceFrame(FootFrame foot);

  // use floats
  bool IKleft(float eetrans[], float eerot[][3],float solution[]);
  bool IKright(float eetrans[], float eerot[][3], float solution[]);

 private:
  FootFrame Frame;

};

#endif
