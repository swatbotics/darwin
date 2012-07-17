
#ifndef _DARWIN_KINEMATICS_
#define _DARWIN_KINEMATICS_

#include "ForwardKinematics.h"

enum FootFrame{LeftFoot, RightFoot};

class Kinematics{
 public:
  
  Kinematics();
  void getJacobian(int attachedFrame, vec3f position, vec3f result[14]);
  Transform3f getTransform(int frame);
  void changeReferenceFrame();
  void changeReferenceFrame(FootFrame foot);
  void setAngle(int index, float value);
  Transform3f getTransform();
  bool setTransform(Transform3f trans);
  bool setT_PostOffset(Transform3f dt);
  bool setT_PreOffset(Transform3f dt);
  ForwardKinematics getFKinObj();
  FootFrame getFrame();

  // use floats
  bool IKleft(Transform3f transform , float solution[]);
  bool IKright(Transform3f transform, float solution[]);

 private:
  ForwardKinematics fKin;
  FootFrame Frame;
  Transform3f t_foot;
  Transform3f b_transform;
  
};

#endif

