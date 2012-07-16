#include "DarwinKinematics.h"
#define IKFAST_NAMESPACE leftleg
#include "ik_leftleg.cpp"
#undef IKFAST_NAMESPACE
#define IKFAST_NAMESPACE rightleg
#include "ik_rightleg.cpp"
#undef IKFAST_NAMESPACE


Kinematics::Kinematics(){
  changeReferenceFrame(LeftFoot);
  b_transform = Transform3f(vec3f(-.037, .3416, 0));
  t_foot = Transform3f(vec3f(-2*.037,0,0));
  return;
}

Transform3f Kinematics::getTransform(int frame){
  return b_transform * fKin.getTransform(frame);
}


void Kinematics::getJacobian(int attachedFrame,
			     vec3f position,
			     vec3f result[14]){
  for (int i=0; i<14; i++){
    result[i] = vec3f(0,0,0);
  }
  if (attachedFrame<0 || attachedFrame>20){
    printf("There is no link number %d for Jacobian\n", attachedFrame);
    exit(-1);
  }

  mat3f R = b_transform.rotFwd(); // This is where the body is relative to foot
  Transform3f trans_from_body = fKin.getTransform(attachedFrame);
  vec3f Jac[20];
  fKin.getJacobian(attachedFrame, position, Jac);
 
  // Columns in rot | trans | head | arm_l | arm_r

  // If the attached frame is above body

  if (attachedFrame <= 2 || attachedFrame >=15){
    mat3f rotJac = -R*mat3f::cross((trans_from_body.transformFwd(position)));
    for (int i=0; i<3; i++){
      result[i] = rotJac.col(i);
    }
    result[3] = vec3f(1,0,0);
    result[4] = vec3f(0,1,0);
    result[5] = vec3f(0,0,1);
    result[6] = R * Jac[0];
    result[7] = R * Jac[1];
    for (int i=8; i<14; i++){
      result[i] = R * Jac[i+6];
    }
  }
  // If the attached frame is leftleg
  else {
    
  }
}

void Kinematics::changeReferenceFrame(){
  Frame = Frame==LeftFoot ? RightFoot : LeftFoot;
  t_foot = t_foot.inverse();
  b_transform = t_foot * b_transform;
  return;
}

void Kinematics::changeReferenceFrame(FootFrame foot){
  if (foot!=LeftFoot && foot!=RightFoot){
    printf("Error changing reference frame, no such foot");
    exit(-1);
  }
  if (foot != Frame){
    changeReferenceFrame();
  }
  return;
}

 
/* The IK functions here should call the IK on the legs and 
   check bounds and return either one single solution or
   return false.
*/

// What kind of input should it take? 1x9 and 1x3? quatf and vec3f?

bool Kinematics::IKleft(Transform3f transform, float solution[]){
  
  static const float leftLowerBound[6] = {-1.57,  -.87,  -.52,  -2.24,  -1.39,  -.78};
  static const float leftUpperBound[6] = {.52,    .78,   1.57,  0,      .96,    .78};

  leftleg::IKReal IK_eetrans[3];
  leftleg::IKReal IK_eerot[9];
  leftleg::IKReal* IK_pfree = NULL;
  std::vector<leftleg::IKSolution> solutions_vec;
  std::vector<leftleg::IKReal> sol(6);
  leftleg::IKSolver solver;

  // Initialize
  mat3f rotation = transform.rotFwd();
  vec3f translation = transform.translation();
  
  for (int i=0; i<9; i++){
    IK_eerot[i] = rotation[i];
  }
  for (int i=0; i<3; i++){
    IK_eetrans[i] = translation[i];
  }
    
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

bool Kinematics::IKright(Transform3f transform, float solution[]){
  
  static const float rightUpperBound[6] = {1.57,   .87,   .52,   2.24,   1.39,   .78};
  static const float rightLowerBound[6] = {-.52,   -.78,  -1.57, -0,     -.96,   -.78};
  
  rightleg::IKReal IK_eetrans[3];
  rightleg::IKReal IK_eerot[9];
  rightleg::IKReal* IK_pfree = NULL;
  std::vector<rightleg::IKSolution> solutions_vec;
  std::vector<rightleg::IKReal> sol(6);
  rightleg::IKSolver solver;
  
  // Initialize
  mat3f rotation = transform.rotFwd();
  vec3f translation = transform.translation();
  
  for (int i=0; i<9; i++){
    IK_eerot[i] = rotation[i];
  }
  for (int i=0; i<3; i++){
    IK_eetrans[i] = translation[i];
  }
    
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

ForwardKinematics Kinematics::getFKinObj(){
  return fKin;
}

void Kinematics::setAngle(int index, float value){
  if (index>=0 && index < 2){
    fKin.setAngle(index, value);
    fKin.update();
  } else if (index>=2 && index<8){
    fKin.setAngle(index+12, value);
    fKin.update();
  } else {
    printf("set angle index out of bounds \n ");
    exit(-1);
  }
  return;
}

FootFrame Kinematics::getFrame(){
  return Frame;
}

Transform3f Kinematics::getTransform(){
  return b_transform;
}

bool Kinematics::setTransform(Transform3f trans){
  float sol_l[6], sol_r[6];
  if (Frame==LeftFoot){
    if(!IKleft(trans.inverse(), sol_l) || 
       !IKright(t_foot * trans.inverse(), sol_r)){
      return false;
    }
  } else {
    if(!IKleft(t_foot * trans.inverse(), sol_l) || 
       !IKright(trans.inverse(), sol_r)){
      return false;
    }
  }
  b_transform = trans;
  for (int i=0; i<6; i++){
    fKin.setAngle(i+2, sol_l[i]); // joints 2 to 7 are left leg
    fKin.setAngle(i+8, sol_r[i]); // joints 8 to 13 are right leg
  }
  fKin.update();
  return true;
}

bool Kinematics::setTransformOffset(Transform3f dt){
  return setTransform(b_transform * dt);
}
