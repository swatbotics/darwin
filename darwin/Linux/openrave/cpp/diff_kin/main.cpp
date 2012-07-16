#include "DarwinKinematics.h"
#include <cassert>

#define PI 3.1415926

int main(int argc, char** argv){
  
  float step = .001;
  Kinematics myKin;
  vec3f Jacobian[14];
  vec3f before, after;
  vec3f position = vec3f(0,1,0);
  int frame = 2;
  vec3f delta[3] = {vec3f(step,0,0), vec3f(0,step,0), vec3f(0,0,step)};

  assert(myKin.setTransformOffset(Transform3f(vec3f(0,-.1,0))));
  assert(myKin.setTransformOffset(Transform3f(quatf::fromOmega(vec3f(.1,.1,.1)))));
  

  myKin.getJacobian(frame,position,Jacobian);
  for (int i=0; i<14; i++){
    std::cout<<Jacobian[i]<<std::endl;
  }
  std::cout<<myKin.getTransform(frame).transformFwd(position);
  std::cout<<std::endl<<std::endl;

  for (int i=0; i<3; i++){
    assert(myKin.setTransformOffset(Transform3f(quatf::fromOmega(-delta[i]))));
    before = myKin.getTransform(frame).transformFwd(position);
    assert(myKin.setTransformOffset(Transform3f(quatf::fromOmega(2*delta[i]))));
    after = myKin.getTransform(frame).transformFwd(position);
    assert(myKin.setTransformOffset(Transform3f(quatf::fromOmega(-delta[i]))));
    std::cout<<(after-before)/(2*step)<<std::endl;
  }



  std::cout<<std::endl<<myKin.getTransform().matrix()<<std::endl;
  
  

  return 0;
}
