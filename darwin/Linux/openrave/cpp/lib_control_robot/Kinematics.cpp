#include "Kinematics.h"
#include "Transform3.h"

Kinematics::Kinematics(){
  changeReferenceFrame(LeftFoot);
  return;
}

void Kinematics::getJacobian(int attachedFrame,
			     float position[],
			     float result[][14]){
  for (int i=0; i<3; i++){
    for (int j=0; j<14; j++){
      result[i][j] = 0;
    }
  }
  if (attachedFrame<0 || attachedFrame>20){
    printf("There is no link number %d for Jacobian\n", attachedFrame);
    exit(-1);
  }
  
}

void Kinematics::changeReferenceFrame(){
  Frame = Frame==LeftFoot ? RightFoot : LeftFoot;
  return;
}

void Kinematics::changeReferenceFrame(FootFrame foot){
  if (foot == LeftFoot){
    Frame = RightFoot;
  } else if (foot==RightFoot){
    Frame = LeftFoot;
  } else {
    printf("Error changing reference frame, no such foot");
    exit(-1);
  }
  return;
}

 
/* The IK functions here should call the IK on the legs and 
   check bounds and return either one single solution or
   return false.
*/

// What kind of input should it take? 1x9 and 1x3? quatf and vec3f?

bool Kinematics::IKleft(float eetrans[], float eerot[][3], float solution[]){
  
  static const float leftLowerBound[6] = {-1.57,  -.87,  -.52,  -2.24,  -1.39,  -.78};
  static const float leftUpperBound[6] = {.52,    .78,   1.57,  0,      .96,    .78};
  
  leftleg::IKReal IK_eetrans[3];
  leftleg::IKReal IK_eerot[9];
  leftleg::IKReal* IK_pfree = NULL;
  std::vector<leftleg::IKSolution> solutions_vec;
  std::vector<leftleg::IKReal> sol(6);
  leftleg::IKSolver solver;

  // Initialize
    
  if(solver.ik(IK_eetrans, IK_eerot, IK_pfree, solutions_vec)){
    int i,j;
    for (i=0; i<(int)solutions_vec.size() ;i++){
      solutions_vec[i].GetSolution(&sol[0],NULL);
      for (j=0; j<6; j++){
	if (sol[j]<leftLowerBound[j] || sol[j]>leftUpperBound[j]){
	break;
	}
      }
      // If every angle passed angle limit test, then return this solution
      if (j==6){
	break;
      }
    }
    if (i==(int)solutions_vec.size()){
      return false;
    } else {
      for (j=0; j<6; j++){
	solution[j] = sol[j];
      }
      return true;
    }      
  }
  return false;
}

bool Kinematics::IKright(float eetrans[], float eerot[][3], float solution[]){
  
  static const float rightUpperBound[6] = {1.57,   .87,   .52,   2.24,   1.39,   .78};
  static const float rightLowerBound[6] = {-.52,   -.78,  -1.57, -0,     -.96,   -.78};
  
  rightleg::IKReal IK_eetrans[3];
  rightleg::IKReal IK_eerot[9];
  rightleg::IKReal* IK_pfree = NULL;
  std::vector<rightleg::IKSolution> solutions_vec;
  std::vector<rightleg::IKReal> sol(6);
  rightleg::IKSolver solver;

  // Initialize
    
  if(solver.ik(IK_eetrans, IK_eerot, IK_pfree, solutions_vec)){
    int i,j;
    for (i=0; i<(int)solutions_vec.size() ;i++){
      solutions_vec[i].GetSolution(&sol[0],NULL);
      for (j=0; j<6; j++){
	if (sol[j]<rightLowerBound[j] || sol[j]>rightUpperBound[j]){
	break;
	}
      }
      // If every angle passed angle limit test, then return this solution
      if (j==6){
	break;
      }
    }
    if (i==(int)solutions_vec.size()){
      return false;
    } else {
      for (j=0; j<6; j++){
	solution[j] = sol[j];
      }
      return true;
    }      
  }
  return false;
}
