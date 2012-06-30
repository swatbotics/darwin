#include "ForwardKinematics.h"

int main(int argc, char** argv){
  
  float test_angles[3] = {.045662, -.037544, .008118};
  ForwardKinematics myKin;
  //  myKin.setAngle(4,1.517);
  //  myKin.setAngle(10,1.517);
  for (int i=0; i<3; i++){
    myKin.setAngle(i+4, test_angles[i]);
    myKin.setAngle(i+10, -test_angles[i]);
  }
  myKin.update();
  for (int i=0; i<21; i++){
    printf("%d \n",i);
    myKin.printTrans(i);
  }
  float COM[3] = {0,0,0};
  myKin.getCOM(COM);
  printf("%f, %f, %f \n", COM[0], COM[1], COM[2]);
  return 0;
}
