#include "ForwardKinematics.h"

int main(int argc, char** argv){
  
  float step = .001;
  ForwardKinematics myKin;
  float Jacobian[3][20];
  float COMbefore[3];
  float COMafter[3];


  myKin.getCOMJacobian(Jacobian);
  /*
  for (int i=0; i<20; i++){
    printf("column %d, %f, %f, %f \n", i, Jacobian[0][i], Jacobian[1][i], Jacobian[2][i]);
  }
  printf("\n\n");
  */

  printf("Jacobian in the body frame");

  for (int i=0; i<20; i++){
    myKin.setAngleOffset(i, -step);
    myKin.update();
    myKin.getCOM(COMbefore);
    myKin.setAngleOffset(i, 2*step);
    myKin.update();
    myKin.getCOM(COMafter);
    
    printf("difference in column %d, %f, %f, %f \n", i,
	   (COMafter[0]-COMbefore[0])/step/2 - Jacobian[0][i],
	   (COMafter[1]-COMbefore[1])/step/2 - Jacobian[1][i],
	   (COMafter[2]-COMbefore[2])/step/2 - Jacobian[2][i]	   );
    
    myKin.setAngleOffset(i, -step);
  }

  return 0;
}
