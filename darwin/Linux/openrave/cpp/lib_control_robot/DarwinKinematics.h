
#ifndef _DARWIN_KINEMATICS_
#define _DARWIN_KINEMATICS_

#include "ForwardKinematics.h"

enum FootFrame{LeftFoot, RightFoot};

class Kinematics{
 public:
  
  Kinematics();
  void getJacobian(int attachedFrame, vec3f position, vec3f result[14]);
  void getCOMJacobian(int attachedFrame, vec3f result[14]);
  void getCOMJacobian(vec3f result[14]);
  vec3f getCOM();
  void changeReferenceFrame();
  void changeReferenceFrame(FootFrame foot);
  void setAngle(int index, float value);
  Transform3f getTransform();
  Transform3f getTransform(int frame);
  bool setTransform(Transform3f trans);
  bool setT_Offset(Transform3f dt);
  bool setT_Offset(vec3f trans);
  bool setT_Offset(quatf q);
  ForwardKinematics getFKinObj();
  FootFrame getFrame();

  // testing
  void testing(vec3f result[]);

  // use floats
  bool IKleft(Transform3f transform , float solution[]);
  bool IKright(Transform3f transform, float solution[]);
  bool IKleft(float solution[]);
  bool IKright(float solution[]);


 private:
  ForwardKinematics fKin;
  FootFrame Frame;
  Transform3f t_foot;
  Transform3f b_transform;
  
};

#endif

