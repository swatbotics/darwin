#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <libgen.h>
#include <string>
#include <iostream>
#include <sstream>
#include <cassert>

#include "Body.h"
#include "Camera.h"
#include "mjpg_streamer.h"
#include "LinuxDARwIn.h"
#include "Joystick.h"
#include "DarwinKinematics.h"

/* Initialization stuff */

int motion_initialization(){
  //////////////////// Framework Initialize ////////////////////////////
  LinuxCM730* linux_cm730 = new LinuxCM730("/dev/ttyUSB0");
  CM730* cm730 = new CM730(linux_cm730);
  if(MotionManager::GetInstance()->Initialize(cm730) == false)
  {
    printf("Fail to initialize Motion Manager!\n");
    return -1;
  }
  MotionManager::GetInstance()->AddModule((MotionModule*)Body::GetInstance());	
  LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
  motion_timer->Start();

  MotionManager::GetInstance()->SetEnable(true);
  for (int i=1; i<=20; i++){
    Body::GetInstance()->m_Joint.SetEnable(i,true,true);
    Body::GetInstance()->m_Joint.SetAngle(i,0);
  }

  return 0;

}

int main(int argc, char** argv)
{
  if (motion_initialization()){
    printf("Failed to initialize control!\n");
    return -1;
  }
  printf("i started");
  Kinematics myKin;
  float alpha = .0005;
  float sol_l[6], sol_r[6];
  vec3f Jac[14];
  myKin.setT_PreOffset(Transform3f(vec3f(0,-.001,0)));

  vec3f goal = myKin.getCOM() - vec3f(0, .05, 0);
  getchar();


  while(1){
    myKin.getCOMJacobian(Jac);
    mat3f Jt;
    for (int i=0; i<3; i++){
      Jt.setRow(i, Jac[3+i]);
    }
    std::cout<<myKin.getCOM()<<std::endl;
    assert(myKin.setT_PreOffset(Transform3f(alpha * vec3f::normalize (Jt * (goal - myKin.getCOM())))));
    if (myKin.IKleft(sol_l) && myKin.IKright(sol_r)){
      std::cout<<"yo"<<std::endl;
      for (int i=0; i<6; i++){
	Body::GetInstance()->m_Joint.SetRadian(2*i+8,sol_l[i]);
	Body::GetInstance()->m_Joint.SetRadian(2*i+7,sol_r[i]);
       }
    } else {
      printf("no solution!\n");
      return 1;
    }

    usleep(50000);
  }
  return 0;
}
