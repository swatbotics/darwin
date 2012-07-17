#ifndef _FORWARD_KIN_ 
#define _FORWARD_KIN_

#include "DarwinDynamicDefinitions.h"

#include "Transform3.h"
#include <cstdio>
#include <cstdlib>


class ForwardKinematics {
 public:
  ForwardKinematics();
  void update();
  vec3f getCOM(int index);
  vec3f getCOM();
  vec3f getCOM_raw(int index);
  float getMass(int index);
  Transform3f getTransform(int index);
  void setAngle(int index, float value);
  void setAngleOffset(int index, float value);
  float getAngle(int index);
  vec3f getAxis(int index);
  bool isUpdated();
  void printTrans(int index);
  void printTrans(const Transform3f trans);
  void getCOMJacobian(vec3f result[20]);
  void getCOMJacobian(int index, vec3f result[20]);
  void getJacobian(int attachedFrame, 
		   const vec3f position, vec3f result[20]);

 private:
  Darwin _myDar;
  int _numJoints;
  float _angles[20];
  int _requireUpdate[5]; // stores INT_MAX if updated, 
                         // else the first link that need update in the chain
  Transform3f _transforms[21];

  int getChainNum(int index);
  void crossX(const float v1[], const float v2[], float result[]);
  Transform3f generateJointTrans(const vec3f axis, const float angle);
};

#endif
