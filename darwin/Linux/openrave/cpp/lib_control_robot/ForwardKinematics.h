#ifndef _FORWARD_KIN_ 
#define _FORWARD_KIN_

#include "DarwinDynamicDefinitions.h"

#include <cstdio>
#include <cstdlib>

class ForwardKinematics {
 public:
  ForwardKinematics();
  void update();
  void getCOM(int index, float result[]);
  void getCOM(float result[]);
  void getTransform(int index, float result[][4]);
  void setAngle(int index, float value);
  void setAngleOffset(int index, float value);
  float getAngle(int index);
  bool isUpdated();
  void printTrans(int index);
  void printTrans(const float trans[][4]);
  void getCOMJacobian(float result[][20]);
  void getCOMJacobian(int index, float result[][20]);
  void getJacobian(int attachedFrame, 
		   const float position[], float result[][20]);

 private:
  Darwin _myDar;
  int _numJoints;
  float _angles[20];
  int _requireUpdate[5]; // stores INT_MAX if updated, 
                         // else the first link that need update in the chain
  float _transforms[21][3][4];

  int getChainNum(int index);
  void crossX(const float v1[], const float v2[], float result[]);
  void applyTransform(const float transform[][4], 
		      const float coordinate[], float result[]);
  void applyRotation(const float transform[][4], 
		     const float coordinate[], float result[]);
  void multiplyTransforms(const float trans1[][4],
			  const float trans2[][4], float result[][4]);
  void getInverse(const float trans[][4], float result[][4]);
  void generateJointTrans(const float axis[], const float angle, float result[][4]);
};

#endif