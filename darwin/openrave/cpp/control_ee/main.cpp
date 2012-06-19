#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <libgen.h>
#include <string>
#include <iostream>
#include <sstream>

// Includes for _getch()
#include <unistd.h>
#include <termios.h>
#include <term.h>
#include <fcntl.h>
#include <ncurses.h>
/******************/

#include "Body.h"
#include "Camera.h"
#include "mjpg_streamer.h"
#include "LinuxDARwIn.h"
#include "Joystick.h"
#define IKFAST_NAMESPACE leftleg
#include "ik_leftleg.cpp"
#undef IKFAST_NAMESPACE
#define IKFAST_NAMESPACE rightleg
#include "ik_rightleg.cpp"
#undef IKFAST_NAMESPACE

using namespace std;
typedef leftleg::IKReal IKReal;
/*****************************************************/
int _getch()
{
	struct termios oldt, newt;
	int ch;
	tcgetattr( STDIN_FILENO, &oldt );
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr( STDIN_FILENO, TCSANOW, &newt );
	ch = getchar();
	tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
	return ch;
}

int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
}

struct termios oldterm, new_term;
void set_stdin(void)
{
	tcgetattr(0,&oldterm);
	new_term = oldterm;
	new_term.c_lflag &= ~(ICANON | ECHO | ISIG); // ÀÇ¹Ì´Â struct termios¸¦ Ã£À¸¸é µÊ.
	new_term.c_cc[VMIN] = 1;
	new_term.c_cc[VTIME] = 0;
	tcsetattr(0, TCSANOW, &new_term);
}

void reset_stdin(void)
{
	tcsetattr(0, TCSANOW, &oldterm);
}
/****************************************************/

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

IKReal IKsin(IKReal in){ return leftleg::IKsin(in);}

IKReal IKcos(IKReal in){ return leftleg::IKcos(in);}

void dance(IKReal* trans, IKReal* Euler){
   const IKReal trans_size = .001;
   const IKReal rot_size = .5 * 3.14 / 180;
   char control;
   set_stdin();
   while (!kbhit()) {}
   control = char(_getch());
   reset_stdin();
   switch (control){
     case 'i':
       trans[1]+=trans_size; break;
     case 'k':
       trans[1]+=-trans_size; break;
     case 'j':
       trans[0]+=trans_size; break;
     case 'l':
       trans[0]+=-trans_size; break;
     case 'u':
       trans[2]+=trans_size;  break;
     case 'o':
       trans[2]+=-trans_size; break;
     case 'w':
       Euler[1]+=rot_size; break;
     case 's':
       Euler[1]+=-rot_size; break;
     case 'a':
       Euler[0]+=rot_size; break;
     case 'd':
       Euler[0]+=-rot_size; break;
     case 'q':
       Euler[2]+=rot_size; break;
     case 'e':
       Euler[2]+=-rot_size; break;
     default:
       exit(1);
   }
   printf("\n");
   printf("new transform: %.2f, %.2f, %.2f \n",trans[0],trans[1],trans[2]);
   printf("new euler angles: %.2f, %.2f, %.2f \n",Euler[0],Euler[1],Euler[2]);
   return;
 }

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

void seteetrans(const IKReal* bodytrans, const IKReal* eeoff, 
		const IKReal* bodyrot, IKReal* eetrans){
  IKReal T[3];
  for (int i=0; i<3; i++){
    T[i] = eeoff[i] - bodytrans[i];
  }
  for (int i=0; i<3; i++){
    eetrans[i] = T[0]*bodyrot[3*i]+T[1]*bodyrot[3*i+1]+T[2]*bodyrot[3*i+2];
  }
  return;
}

void joystickfunz(Joystick* joy){
  joy->update();
  if (joy->getHat(0)){
    printf("yo\n");
  }
  return;
}

void moveTo(IKReal* trans, int value, IKReal trans_size){ 
  //   const IKReal trans_size = .000001;  
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

void dance(IKReal* trans, IKReal* Euler, Joystick* joy){
  if (joy==NULL){
    dance(trans, Euler);
  } else {
  const IKReal trans_size = .000001;  
  const IKReal rot_size = .00001;
   joy->update();
   switch (joy->getHat(0)){
     case 1:
       trans[1]+=trans_size; break;
     case 2:
       trans[1]+=-trans_size; break;
     case 4:
       Euler[1]+=rot_size; break;
     case 8:
       Euler[1]+=-rot_size; break;
   }
   moveTo(&trans[0], -joy->getAxis(0), trans_size);
   moveTo(&trans[2], -joy->getAxis(1), trans_size);
   moveTo(&Euler[0], -joy->getAxis(4), rot_size);   
   moveTo(&Euler[2], -joy->getAxis(3), -rot_size);
   

   /*
   if (joy->getButton(4)){trans[1]+=trans_size;}
   if (joy->getButton(5)){trans[1]+=-trans_size;}
   if (joy->getButton(4)){trans[1]+=trans_size;}
   if (joy->getButton(4)){trans[1]+=trans_size;} 

   printf("\n");
   printf("new transform: %.2f, %.2f, %.2f \n",trans[0],trans[1],trans[2]);
   printf("new euler angles: %.2f, %.2f, %.2f \n",Euler[0],Euler[1],Euler[2]);
   */
  }
  return;
}


 int main(void)
 {
   //   display_intro();

   if (motion_initialization()){
     printf("Failed to initialize control!\n");
     return -1;
   }

   Joystick* joy = new Joystick(0);

   IKReal bodyEuler[3] = {0,0,0};
   IKReal bodytrans[3] = {0,0,0};
   IKReal eerot_l[9], eerot_r[9], bodyrot[9], eetrans_l[3], eetrans_r[3];
   const IKReal transOffset_l[3] = {.037,-.3416,0};
   const IKReal transOffset_r[3] = {-.037,-.3416,0};
   IKReal* pfree = NULL;
   leftleg::IKSolver solver_l;
   rightleg::IKSolver solver_r;
   std::vector<leftleg::IKSolution> solutions_l;
   std::vector<rightleg::IKSolution> solutions_r;
   int solution_index;
   std::vector<IKReal> sol(6);
   while(1){
     getRot(bodyEuler,bodyrot);
     transpose(bodyrot,eerot_l);
     transpose(bodyrot,eerot_r);
     seteetrans(bodytrans, transOffset_l, eerot_l, eetrans_l);
     seteetrans(bodytrans, transOffset_r, eerot_r, eetrans_r);
     /*
     printtrans(eetrans_l, "left trans");
     printtrans(eetrans_r, "right trans");

     printrot(eerot_l, "ee rot");
     printrot(bodyrot, "body rot");
     */
     if (solver_l.ik((IKReal*)eetrans_l,(IKReal*) eerot_l, pfree , solutions_l)){
       // Check and return one correct solution
       solution_index = checkSol(solutions_l);
       if (solution_index==-1){
	 printf("No valid solutions\n");
       } else {
	 solutions_l[solution_index].GetSolution(&sol[0],NULL);
	 //	 sol[0]*=-1;
	 for (int i=0; i<6; i++){
	   Body::GetInstance()->m_Joint.SetRadian(2*i+8,sol[i]);
	 }
       }
     } else {
       printf("Failed to get left foot ik solution\n");
     }
     if (solver_r.ik((IKReal*)eetrans_r,(IKReal*) eerot_r, pfree , solutions_r)){
       // Check and return one correct solution
       solution_index = checkSol(solutions_r);
       if (solution_index==-1){
	 printf("No valid solutions\n");
       } else {
	 solutions_r[solution_index].GetSolution(&sol[0],NULL);
	 //	 sol[0]*=-1;
	 for (int i=0; i<6; i++){
	   Body::GetInstance()->m_Joint.SetRadian(2*i+7,sol[i]);
	 }
       }
     } else {
       printf("Failed to get right foot ik solution\n");
     }
     dance(bodytrans, bodyEuler, joy);
     //    joystickfunz(joy);
  }
  joy->close();
  return 0;
}
