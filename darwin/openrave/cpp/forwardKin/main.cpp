#include "ForwardKinematics.h"

int main(int argc, char** argv){
  ForwardKinematics myKin;
  myKin.setAngle(4,1.517);
  myKin.setAngle(10,1.517);
  myKin.update();
  float COM[3] = {0,0,0};
  myKin.getCOM(COM);
  printf("%f, %f, %f \n", COM[0], COM[1], COM[2]);
  return 0;
}
