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
#include "Joystick.h"
#include "ForwardKinematics.h"
#define IKFAST_NAMESPACE leftleg
#include "ik_leftleg.cpp"
#undef IKFAST_NAMESPACE
#define IKFAST_NAMESPACE rightleg
#include "ik_rightleg.cpp"
#undef IKFAST_NAMESPACE

using namespace std;
typedef leftleg::IKReal IKReal;

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

/* Printing information stuff */

void printtrans(IKReal* trans, string transname){
  printf("%s is :[ %.3f, %.3f, %.3f ]\n", transname.c_str(), trans[0], trans[1], trans[2] );
  return;
}

void printrot(IKReal* trans, string transname){
  printf("%s is :[ %.3f, %.3f, %.3f ]\n", transname.c_str(), trans[0], trans[1], trans[2] );  
  printf("          [ %.3f, %.3f, %.3f ]\n", trans[3], trans[4], trans[5] );
  printf("          [ %.3f, %.3f, %.3f ]]\n", trans[6], trans[7], trans[8] );
  return;
}

/* transform conversion stuff */

IKReal IKsin(IKReal in){ return leftleg::IKsin(in);}

IKReal IKcos(IKReal in){ return leftleg::IKcos(in);}

void getRot(const IKReal* euler, IKReal* eerot){
   eerot[0] = IKcos(euler[1])*IKcos(euler[2]);
   eerot[1] = -IKcos(euler[0])*IKsin(euler[2])+IKsin(euler[0])*IKsin(euler[1])*IKcos(euler[2]);
   eerot[2] = IKsin(euler[0])*IKsin(euler[2])+IKcos(euler[0])*IKsin(euler[1])*IKcos(euler[2]);
   eerot[3] = IKcos(euler[1])*IKsin(euler[2]);
   eerot[4] = IKcos(euler[0])*IKcos(euler[2])+IKsin(euler[0])*IKsin(euler[1])*IKsin(euler[2]);
   eerot[5] = -IKsin(euler[0])*IKcos(euler[2])+IKcos(euler[0])*IKsin(euler[1])*IKsin(euler[2]);
   eerot[6] = -IKsin(euler[1]);
   eerot[7] = IKsin(euler[0])*IKcos(euler[1]);
   eerot[8] = IKcos(euler[0])*IKcos(euler[1]);
   return;
 }

void transpose(const IKReal* rotin, IKReal* rotout ){
  rotout[0] = rotin[0];
  rotout[1] = rotin[3];
  rotout[2] = rotin[6];
  rotout[3] = rotin[1];
  rotout[4] = rotin[4];
  rotout[5] = rotin[7];
  rotout[6] = rotin[2];
  rotout[7] = rotin[5];
  rotout[8] = rotin[8];
  return;
}

void applyTransform(const IKReal* trans, const IKReal* rot, const float* point, float* result){
  for (int i=0; i<3; i++){
    result[i] = rot[3*i]*point[0] + rot[3*i+1]*point[1] + 
      rot[3*i+2]*point[2] + trans[i];
  }
  return;
}

/* Check solution stuff */

static const float leftLowerBound[6] = {-1.57,  -.87,  -.52,  -2.24,  -1.39,  -.78};
static const float leftUpperBound[6] = {.52,    .78,   1.57,  0,      .96,    .78};

static const float rightUpperBound[6] = {1.57,   .87,   .52,   2.24,   1.39,   .78};
static const float rightLowerBound[6] = {-.52,   -.78,  -1.57, -0,     -.96,   -.78};

template <class Tik> std::pair<const float*, const float*> getBounds() {
  return std::make_pair((const float*)NULL, (const float*)NULL);
}

template <> std::pair<const float*, const float*> getBounds<leftleg::IKSolution>() {
  return std::make_pair(leftLowerBound, leftUpperBound);
}

template <> std::pair<const float*, const float*> getBounds<rightleg::IKSolution>() {
  return std::make_pair(rightLowerBound, rightUpperBound);
}


template <class IKSolution>
int checkSol(std::vector<IKSolution> solutions){
  
  std::pair<const float*, const float*> bounds = getBounds<IKSolution>();
  const float* lowerBound = bounds.first;
  const float* upperBound = bounds.second;

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
      return i;
    }
  }
  return -1;
}

/* Check center of mass stuff */

static const int jointInd_robot2rave[20] = {17,14,18,15,19,16,//arms
					    8,2,9,3,10,4,11,5,12,6,13,7,//legs
					    0,1};//head

void getCOM(std::vector<IKReal> sol_l, std::vector<IKReal> sol_r,
	    IKReal* bodytrans, IKReal* bodyrot, ForwardKinematics myKin, 
	    float* COM){

  for (int i=0; i<(int)sol_l.size(); i++){
    // 2i+8 is the index for the joints on the left foot. 2i+7 for the right
    // TODO: clean this up to make it readable, NO MAGIC NUMBERS
    myKin.setAngle(i+2, sol_l[i]); 
    myKin.setAngle(i+8, sol_r[i]);
  }  
  myKin.update();
  float bodyCOM[3] = {0,0,0};
  myKin.getCOM(bodyCOM);
  // bodytrans is the translation of the body relative to the body 
  // at 0 position, so we must adjust to have it centered at the foot
  applyTransform(bodytrans, bodyrot, bodyCOM, COM);
  return;
}

/* Input from keyboard and joystick */

void moveTo(IKReal* trans, int value, IKReal trans_size){ 
   const IKReal step_size = 10;
   if (*trans - trans_size * value < -step_size){
     *trans += step_size * trans_size;
   } else if (*trans - trans_size * value > step_size){
     *trans -= step_size * trans_size;
   } else {
     *trans = trans_size * value;
   }
   return;
}

void getNext(IKReal* trans_l, IKReal* trans_body, float* COM, float* COMprev){
  if (trans_body[1] > .335){
    trans_body[1] += -.00006;
  } else {
    trans_body[0] -= .1*(COM[0] - 0) - .03 * (COM[0] - COMprev[0]);
    trans_body[2] -= .1*(COM[2] - 0) - .03 * (COM[2] - COMprev[2]);
    if (fabs(COM[0])<.001 ) {
      trans_l[1] -= .01*(trans_l[1] - .1);
    }
  }
  return;
}

int main(int argc, char** argv)
{
  if (motion_initialization()){
    printf("Failed to initialize control!\n");
    return -1;
  }
  printf("i startd");
  ForwardKinematics myKin;
 
  IKReal rotation[9] = {1,0,0, 0,1,0, 0,0,1};
  // Base frame is the right foot
  IKReal trans_l[3] = {.037*2,0,0};
  IKReal trans_body[3] = {.037,.3416,0};
  IKReal eetrans_l[3] = {0,0,0};
  IKReal eetrans_r[3] = {0,0,0};
  IKReal* pfree = NULL;
  leftleg::IKSolver solver_l;
  rightleg::IKSolver solver_r;
  std::vector<leftleg::IKSolution> solutions_l;
  std::vector<rightleg::IKSolution> solutions_r;
  int solution_index;
  std::vector<IKReal> sol_l(6), sol_r(6);
  bool left_has_solution, right_has_solution;
  float COM[3] = {0,0,0};
  float COMprev[3] = {0,0,0};
  sleep(1);
  while(1){
    for (int i=0; i<3; i++){
      eetrans_r[i] = -trans_body[i];
      eetrans_l[i] = eetrans_r[i] + trans_l[i];
    }
    printf("%f, %f, %f \n", trans_body[0], trans_body[1], trans_body[2]);
    
    left_has_solution = false;
    right_has_solution = false;
    if (solver_l.ik((IKReal*)eetrans_l, (IKReal*) rotation, 
		    pfree , solutions_l)){
      // Check and return one correct solution
      solution_index = checkSol(solutions_l);
       if (solution_index==-1){
	 printf("No valid solutions\n");
       } else {
	 solutions_l[solution_index].GetSolution(&sol_l[0],NULL);
	 left_has_solution = true;
       }
    } else {
      printf("Failed to get left foot ik solution\n");
    }
    if (solver_r.ik((IKReal*)eetrans_r, (IKReal*) rotation, 
		    pfree , solutions_r)){
      // Check and return one correct solution
      solution_index = checkSol(solutions_r);
       if (solution_index==-1){
	 printf("No valid solutions\n");
       } else {
	 solutions_r[solution_index].GetSolution(&sol_r[0],NULL);
	 right_has_solution = true;
       }
    } else {
      printf("Failed to get right foot ik solution\n");
    }
    
    if (left_has_solution && right_has_solution){
      for (int i=0; i<6; i++){
	Body::GetInstance()->m_Joint.SetRadian(2*i+8,sol_l[i]);
	Body::GetInstance()->m_Joint.SetRadian(2*i+7,sol_r[i]);
       }
    } else {
      printf("no solution!\n");
      return 1;
    }

    for (int i=0; i<3; i++){
      COMprev[i] = COM[i];
    }
    getCOM(sol_l, sol_r, trans_body, rotation, myKin, COM);
    getNext(trans_l, trans_body, COM, COMprev);
    usleep(50000);
  }
  return 0;
}
