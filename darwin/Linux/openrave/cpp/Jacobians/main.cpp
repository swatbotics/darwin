#include "ForwardKinematics.h"

int main(int argc, char** argv){
  
  float step = .01;
  ForwardKinematics myKin;
  vec3f Jacobian[20];
  vec3f before, after;
  int link = 2;

  printf(" Generated Jacobian \n");

  myKin.getCOMJacobian(Jacobian);
  for (int i=0; i<20; i++){
    std::cout<<Jacobian[i]<<std::endl;
  }

  printf("\n Numeric Jacobian Difference \n");

  for (int i=0; i<20; i++){
    myKin.setAngleOffset(i, -step);
    myKin.update();
    before = myKin.getCOM();
    myKin.setAngleOffset(i, 2*step);
    myKin.update();
    after = myKin.getCOM();
    
    std::cout<<
      ((after-before)/(2*step) - Jacobian[i])
	     <<std::endl;
    
    myKin.setAngleOffset(i, -step);
  }

  return 0;
}
