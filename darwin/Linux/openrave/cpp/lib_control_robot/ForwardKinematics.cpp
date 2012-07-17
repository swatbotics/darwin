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
  _transforms[0] = Transform3f();
  update();
}

void ForwardKinematics::printTrans(int index){
  printTrans(_transforms[index]);
}

void ForwardKinematics::printTrans(const Transform3f trans){
  std::cout<<trans.matrix()<<std::endl;
}

void ForwardKinematics::update(){
  
  int linkNum = 0;
  Transform3f jointTrans;
  Transform3f jointBefore;

  for (int chainNum=0; chainNum<5; chainNum++){
    while (_requireUpdate[chainNum]!=INT_MAX){
      linkNum = _requireUpdate[chainNum];
      jointTrans = generateJointTrans(_myDar.Links[linkNum].AXIS,
			 _angles[linkNum-1]);
      if (_myDar.Links[linkNum].PREVIOUS==0){
	_transforms[linkNum] = _myDar.Chains[chainNum].T_FROM_BODY * jointTrans;
      } else {
	jointBefore = _transforms[linkNum-1] *  _myDar.Links[linkNum-1].T_PREV2NEXT;
	_transforms[linkNum] = jointBefore * jointTrans;
      }
      // Set the next link in the chain to be updated
      if (_myDar.Links[linkNum].NEXT==-1){
	_requireUpdate[chainNum]=INT_MAX;
      } else {
	_requireUpdate[chainNum]++;
      }
    }
  }
  return;
}

Transform3f ForwardKinematics::generateJointTrans(const vec3f axis, 
						  const float angle){
  return Transform3f(quatf::fromAxisAngle(axis, angle), vec3f(0,0,0));
}

void ForwardKinematics::getCOMJacobian(vec3f result[20]){
  float total_mass = 0;
  vec3f accum[20];
  for (int j=0; j<20; j++){
    accum[j] = vec3f(0,0,0);
  }

  vec3f linkJacobian[20];
  for (int i=0; i<_numJoints+1; i++){
    getCOMJacobian(i, linkJacobian);
    for (int j=0; j<20; j++){
      accum[j] = linkJacobian[j] * _myDar.Links[i].MASS + accum[j];
    }
    total_mass += _myDar.Links[i].MASS;
  }
  for (int j=0; j<20; j++){
    result[j] = accum[j] / total_mass;
  }
  return;
}

void ForwardKinematics::getCOMJacobian(int index, vec3f result[20]){
  getJacobian(index, _myDar.Links[index].COM ,result);
  return;
}

void ForwardKinematics::getJacobian(int attachedFrame, 
				    const vec3f position,
				    vec3f result[20]){
  // Initialize the Jacobian to zero throughout, what data structure 
  // to store jacobian?
  for (int i=0; i<20; i++){
    result[i] = vec3f(0,0,0);
  }

  if (attachedFrame<0 || attachedFrame>20){
    printf("There is no link number %d for Jacobian\n", attachedFrame);
    exit(-1);
  }
  if (attachedFrame==0){
    return;
  }

  // The joint corresponding to the attached frame is frame-1
  int i = attachedFrame;
  vec3f column, axisVec, positionVec, GlobalPosition;
  GlobalPosition = _transforms[attachedFrame] * position;
  while (1){
    axisVec = getAxis(i);
    positionVec = GlobalPosition - _transforms[i].translation();
    column = vec3f::cross(axisVec , positionVec);
    // update result
    result[i-1] = column;
    if (_myDar.Links[i].PREVIOUS == 0){
      return;
    }
    i--;
  }
}

vec3f ForwardKinematics::getCOM_raw(int index){
  if (index<0 || index >20){
    printf("asking for COM of link %d\n", index);
    exit(-1);
  }
  return _myDar.Links[index].COM; 
}

vec3f ForwardKinematics::getCOM(int index){ 
  if (index<0 || index >20){
    printf("asking for COM of link %d\n", index);
    exit(-1);
  }
  return _transforms[index] * _myDar.Links[index].COM; 
}

vec3f ForwardKinematics::getCOM(){
  float total_mass = 0;
  vec3f accum = vec3f(0,0,0);
  vec3f linkCOM;
  for (int i=0; i<_numJoints+1; i++){
    accum = accum + getCOM(i) * _myDar.Links[i].MASS;
    total_mass += _myDar.Links[i].MASS;
  }
  return accum / total_mass;
}

float ForwardKinematics::getMass(int index){
  if (index<0 || index >20){
    printf("asking for mass of link %d\n", index);
    exit(-1);
  }
  return _myDar.Links[index].MASS;
}


vec3f ForwardKinematics::getAxis(int index){
  if (index<=0 || index >=_numJoints+1){
    printf("attempting to get axis on link %d.\n", index);
    std::exit(-1);
  } else {
    return _transforms[index].rotFwd() * _myDar.Links[index].AXIS;
  }
}

Transform3f ForwardKinematics::getTransform(int index){
  if (index<0 || index >=_numJoints+1){
    printf("attempting to get transform on link %d.\n", index);
    std::exit(-1);
  } else {
    return _transforms[index];
  }
}

void ForwardKinematics::setAngleOffset(int index, float value){
  setAngle(index, getAngle(index)+value);
  return;
}

void ForwardKinematics::setAngle(int index, float value){
  // the link corresponding to joint i is link i+1
  if (index<0 || index >=_numJoints){
    printf("attempting to change angle on joint %d.\n", index);
    std::exit(-1);
  } else {
    _angles[index] = value;
    _requireUpdate[getChainNum(index+1)] = 
      std::min(index+1,_requireUpdate[getChainNum(index+1)]) ;
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


