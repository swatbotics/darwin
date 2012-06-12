#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <libgen.h>
#include <string>
#include <iostream>
#include <sstream>

#include "Body.h"
#include "Camera.h"
#include "mjpg_streamer.h"
#include "LinuxDARwIn.h"
#include "ik.cpp"


using namespace std;


int motion_initialization(){
  //////////////////// Framework Initialize ////////////////////////////
  LinuxCM730 linux_cm730("/dev/ttyUSB0");
  CM730 cm730(&linux_cm730);
  if(MotionManager::GetInstance()->Initialize(&cm730) == false)
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
  }
  return 0;
}

void display_intro(){
  cout<< "control the left foot of the darwin from your keyboard\n";
  cout<< "------------------------------------------------------\n";
  cout<< " i -- up    j -- left   u -- forward\n";
  cout<< " k -- down  l -- right  o -- backward";
  cout<< " Anything else to quit" <<endl;
  return;
}

IKSolution* checkSol(std::vector<IKSolution> solutions){
  static float lowerBound[6] = {1,1,1,1,1,1};
  static float upperBound[6] = {1,1,1,1,1,1};
  std::vector<IKReal> sol(6);
  int i,j;
  for (i=0; i<(int)solutions.size() ;i++){
    solutions[i].GetSolution(&sol[0],NULL);
    for (j=0; j<6; j++){
      if (sol[j]<lowerBound[j] || sol[j]>upperBound[j]){
	break;
      }
    }
    if (j==6){
      return &solutions[i];
    }
  }
  return NULL;
}

int main(void)
{
  display_intro();
  IKReal eerot[9] = {0,0,1,  1,0,0, 0,1,0};
  IKReal eetrans[3] = {-.037,-.3416,0};
  IKReal* pfree = NULL;
  IKSolver solver;
  std::vector<IKSolution> solutions;
  char control;
  while(1){
    if (solver.ik((IKReal*)eetrans,(IKReal*) eerot, pfree , solutions)){
      // Check and return one correct solution
      Body::GetInstance()->m_Joint.SetAngle(0,0);
    } else {
      fprintf(stderr,"Failed to get ik solution\n");
      return -1;
    }
    control = getchar();
    switch (control){
    case 'i':
      eetrans[1]+=.01; break;
    case 'k':
      eetrans[1]+=-.01; break;
    case 'j':
      eetrans[0]+=.01; break;
    case 'l':
      eetrans[0]+=-.01; break;
    case 'u':
      eetrans[2]+=.01; break;
    case 'o':
      eetrans[2]+=-.01; break;
    default:
      return 0;
    }
  }
  return 0;
}
