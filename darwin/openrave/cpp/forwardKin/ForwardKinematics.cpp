#include "ForwardKinematics.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include "limits.h"

enum chainNames{BODY=-1, HEAD, LEFTLEG, RIGHTLEG, LEFTARM, RIGHTARM};

ForwardKinematics::ForwardKinematics(){
  _numJoints = 20;
  // Set all chains to need update
  for (int i=0; i<5; i++){
    _requireUpdate[i]=INT_MAX;
  }
  // Initialize angle values
  for (int i=0; i<_numJoints; i++){
    setAngle(i,0);
  }
  // Update body link, then the rest of the body
  for (int i=0; i<3; i++){
    for (int j=0; j<4; j++){
      _transforms[0][i][j] = (i==j)? 1 : 0;
    }
  }
  update();
}

void printTrans(const float trans[][4]){
  for (int i=0; i<3; i++){
    printf("%f, %f, %f, %f \n", trans[i][0], trans[i][1]
	   , trans[i][2], trans[i][3]);
  }
  printf("\n");
}


void ForwardKinematics::update(){
  
  int linkNum = 0;
  float jointTrans[3][4];
  float jointBefore[3][4];

  for (int chainNum=0; chainNum<5; chainNum++){
    while (_requireUpdate[chainNum]!=INT_MAX){
      linkNum = _requireUpdate[chainNum];
      generateJointTrans(_myDar.Links[linkNum].AXIS,
			 _angles[linkNum-1],
			 jointTrans);
      if (&(_myDar.Links[linkNum]) 
	  == _myDar.Chains[chainNum].FIRST){
	multiplyTransforms(_myDar.Chains[chainNum].T_FROM_BODY,
			   jointTrans,
			   _transforms[linkNum]);
      } else {
	multiplyTransforms(_transforms[linkNum-1],
			   _myDar.Links[linkNum-1].T_PREV2NEXT,
			   jointBefore);
 	multiplyTransforms(jointBefore, 
			   jointTrans,
			   _transforms[linkNum]);
      }
      // Set the next link in the chain to be updated
      if (_myDar.Links[_requireUpdate[chainNum]].NEXT==NULL){
	_requireUpdate[chainNum]=INT_MAX;
      } else {
	_requireUpdate[chainNum]++;
      }
    }
  }
  return;
}

void ForwardKinematics::generateJointTrans(const float axis[], 
					   const float angle, float result[][4]){

  result[0][0] = cos(axis[1]*angle) * cos(axis[2]*angle); 
  result[1][1] = cos(axis[0]*angle) * cos(axis[2]*angle); 
  result[2][2] = cos(axis[0]*angle) * cos(axis[1]*angle); 

  result[0][1] = -sin(axis[2]*angle);  
  result[1][2] = -sin(axis[0]*angle);  
  result[2][0] = -sin(axis[1]*angle);

  result[1][0] = sin(axis[2]*angle);
  result[2][1] = sin(axis[0]*angle);
  result[0][2] = sin(axis[1]*angle);

  result[0][3] = 0;
  result[1][3] = 0;
  result[2][3] = 0;

}

void ForwardKinematics::multiplyTransforms(const float trans1[][4],
			const float trans2[][4], float result[][4]){
  // trans1 is the premultiplier, trans2 is the postmultiplier
  for (int i=0; i<3; i++){
    for (int j=0; j<3; j++){
      result[i][j] = trans1[i][0]*trans2[0][j] + 
	trans1[i][1]*trans2[1][j] + trans1[i][2]*trans2[2][j];
    }
    result[i][3] = trans1[i][0]*trans2[0][3] + 
      trans1[i][1]*trans2[1][3] + trans1[i][2]*trans2[2][3] + 
      trans1[i][3];
  }
  return;
}

/* This function has yet to be tested
void ForwardKinematics::getInverse(float trans[][4], float result[][4]){
  for (int i=0; i<3; i++){
    for (int j=0; j<3; j++){
      result[i][j] = trans[j][i];
    }
    result[i][3] = trans[0][i]*trans[0][3] + 
      trans[1][i]*trans[1][3] + trans[2][i]*trans[2][3];

  }
  return;
}
*/

void ForwardKinematics::applyTransform(const float transform[][4], 
				       const float coordinate[],
				       float result[]){
  for (int i=0; i<3; i++){
    result[i] = coordinate[0]*transform[i][0] + 
      coordinate[1]*transform[i][1] + 
      coordinate[2]*transform[i][2] + 
      transform[i][3];
  }
  return;
}

void ForwardKinematics::getCOM(float result[]){
  float total_mass = 0;
  float accumalate[3] = {0,0,0};
  float linkCOM[3] = {0,0,0};
  for (int i=0; i<_numJoints+1; i++){
    applyTransform(_transforms[i], _myDar.Links[i].COM, linkCOM); 
    for (int j=0; j<3; j++){
      accumalate[j] += linkCOM[j]*_myDar.Links[i].MASS;
    }
    total_mass += _myDar.Links[i].MASS;
  }
  for (int j=0; j<3; j++){
    result[j] = accumalate[j] / total_mass;
  }
  return;
}



void ForwardKinematics::getTransform(int index, float result[][4]){
  if (index<0 || index >=_numJoints+1){
    printf("attempting to get transform on link %d.\n", index);
    std::exit(-1);
  } else {
    result = _transforms[index];
  }
}

void ForwardKinematics::setAngle(int index, float value){
  if (index<0 || index >=_numJoints){
    printf("attempting to change angle on joint %d.\n", index);
    std::exit(-1);
  } else {
    _angles[index] = value;
    _requireUpdate[getChainNum(index)] = 
      std::min(index,_requireUpdate[getChainNum(index)]) ;
  }
  return;
}

int ForwardKinematics::getChainNum(int index){
  switch(index){
  case 0:
    return BODY;
  case 1: case 2:
    return HEAD;
  case 3: case 4: case 5: case 6: case 7: case 8:
    return LEFTLEG;
  case 9: case 10: case 11: case 12: case 13: case 14:
    return RIGHTLEG;
  case 15: case 16: case 17:
    return LEFTARM;
  case 18: case 19: case 20:
    return RIGHTARM;
  default:
    printf("attempting to change angle on joint %d.\n", index);
    std::exit(-1);
  }
}

float ForwardKinematics::getAngle(int index){
  if ( index<0 || index>=_numJoints){
    printf("attempting to get angle on joint %d.\n", index);
    std::exit(-1);
  } else {
    return _angles[index];
  }
}

bool ForwardKinematics::isUpdated(){
  for (int i=0; i<5; i++){
    if (_requireUpdate[i]!=0){
      return false;
    }
  }
  return true;
}


