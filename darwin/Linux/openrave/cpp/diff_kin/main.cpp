#include "DarwinKinematics.h"
#include <opencv2/core/core.hpp>
#include <cassert>

#define PI 3.1415926

float step = 1e-3;

void mattTest() {

  ForwardKinematics myKin;

  //float test_angles[6] = { .15, .05, -.03, .07, -.09, 0.08 };
  
  float test_angles[6] = {
    0.162854, 0.626912, 0.912151, -1.9116, -0.915347, 0.320088
  };
  
  //float test_angles[6] = { 0, 0, 0, 0, 0, 0 };
  
  for (int i=0; i<6; i++){
    myKin.setAngle(i+2, test_angles[i]);
  }
  myKin.update();

  Transform3f footTransform = myKin.getTransform(8);

  quatf q = footTransform.rotation();
  vec3f f = footTransform.translation();

  std::cout << "got q = " << q << ", f = " << f << "\n";

  vec3f Jf_func[20];
  vec3f Jr_func[6];

  myKin.getJacobian(8, vec3f(0), Jf_func);

  for (int i=0; i<6; ++i) {
    Jr_func[i] = myKin.getAxis(i+3);
  }

  vec3f Jf_num[6], Jr_num[6];

  for (int i=0; i<6; ++i) {

    myKin.setAngle(i+2, test_angles[i]+step);
    myKin.update();

    Transform3f tp = myKin.getTransform(8);
    quatf qp = tp.rotation();
    vec3f fp = tp.translation();

    myKin.setAngle(i+2, test_angles[i]-step);
    myKin.update();

    Transform3f tm = myKin.getTransform(8);
    quatf qm = tm.rotation();
    vec3f fm = tm.translation();

    Jf_num[i] = (fp - fm)/(2*step);

    myKin.setAngle(i+2, test_angles[i]);
    myKin.update();

    Jr_num[i] = quatf::omega(qm, qp) / (2*step);
    
  }

  std::cout << "Jf:\n";
  for (int i=0; i<6; ++i) {
    std::cout << "func: " << Jf_func[i+2] << ", num: " << Jf_num[i] << ", err: " << (Jf_func[i+2]-Jf_num[i]).norm() << "\n";
  }

  std::cout << "Jr:\n";
  for (int i=0; i<6; ++i) {
    std::cout << "func: " << Jr_func[i] << ", num: " << Jr_num[i] << ", err: " << (Jr_func[i]-Jr_num[i]).norm() << "\n";
  }


}

void mattTest2() {

  Kinematics myKin;
  ForwardKinematics extraFKin;

  assert(myKin.setT_Offset(vec3f(0,-.01,0)));
  assert(myKin.setT_Offset(quatf::fromOmega(vec3f(-.01,.02,.03))));
  vec3f delta[3] = {vec3f(step,0,0), vec3f(0,step,0), vec3f(0,0,step)};
  
  quatf rot = myKin.getTransform().inverse().rotation();
  vec3f trans = myKin.getTransform().inverse().translation();

  float theta[6];

  assert(myKin.IKleft(Transform3f(rot, trans), theta));


  for (int a=0; a<6; ++a) {
    extraFKin.setAngle(a+2, theta[a]);
  }
  extraFKin.update();
  Transform3f xp = extraFKin.getTransform(8);

  std::cout << "rot error: " << quatf::dist(xp.rotation(), rot) << "\n";
  std::cout << "pos error: " << (xp.translation()-trans) << "\n";

  vec3f dik_omega_num[6];
  vec3f dik_omega_func[6];

  std::cout << "theta = ";
  for (int a=0; a<6; ++a) { std::cout << theta[a] << " "; }
  std::cout << "\n";

  cv::Mat_<float> J(6,6);

  vec3f Jf_func[20];

  myKin.fKin.update();
  myKin.fKin.getJacobian(8, vec3f(0), Jf_func);

  for (int i=0; i<6; ++i) {
    vec3f Jai = myKin.fKin.getAxis(i+3);
    vec3f Jbi = Jf_func[i+2];
    for (int c=0; c<3; ++c) {
      J(c+0,i) = Jai[c];
      J(c+3,i) = Jbi[c];
    }
  }

  std::cout << "J = \n" << J << "\n";
  /*
  cv::SVD svd(J);

  std::cout << "svd.u:\n" << svd.u << "\n";
  std::cout << "svd.w:\n" << svd.w << "\n";
  std::cout << "svd.vt:\n" << svd.vt << "\n";
  */
  cv::Mat_<float> Jinv = J.inv();

  std::cout << "Jinv = \n" << Jinv << "\n";

  for (int i=0; i<6; ++i) {
    for (int j=0; j<3; ++j) {
      dik_omega_func[i][j] = Jinv(i,j);
    }
  }

  std::cout << "Jinv * J = \n" << Jinv * J << "\n\n";


  for (int i=0; i<3; ++i) {

    float tp[6];
    float tm[6];

    quatf qp = quatf::fromOmega( delta[i])*rot;
    quatf qm = quatf::fromOmega(-delta[i])*rot;

    assert(myKin.IKleft(Transform3f(qp, trans), tp));

    assert(myKin.IKleft(Transform3f(qm, trans), tm));

    for (int j=0; j<6; ++j) {
      dik_omega_num[j][i] = (tp[j] - tm[j])/(2*step);
    }

  }

  std::cout << "Jr:\n";
  for (int j=0; j<6; ++j) {
    const vec3f& f = dik_omega_func[j];
    const vec3f& n = dik_omega_num[j];
    std::cout << "func: " << f << ", "
	      << "num: " << n << ", "
	      << "err: " << (f-n).norm() << ", "
	      << "dot: " << vec3f::dot(f,n)/(f.norm()*n.norm()) << "\n";
  }



}

void keliangTest() {

  Kinematics myKin;
  assert(myKin.setT_Offset(vec3f(0,-.1,0)));
  assert(myKin.setT_Offset(quatf::fromOmega(vec3f(-.1,.2,.3))));
  vec3f delta[3] = {vec3f(step,0,0), vec3f(0,step,0), vec3f(0,0,step)};
  


  quatf rot = myKin.getTransform().inverse().rotation();
  vec3f trans = myKin.getTransform().inverse().translation();

  vec3f result[6];
  myKin.testing(result);
  for (int i=0; i<6; i++){
    std::cout<<result[i]<<std::endl;
  }

  std::cout<<std::endl<<std::endl;

  float before[3][6], after[3][6];
  if (1){
    for (int i=0; i<3; i++){
      assert(myKin.IKleft(Transform3f(quatf::fromOmega(-delta[i])*rot, trans), before[i]));
      assert(myKin.IKleft(Transform3f(quatf::fromOmega(delta[i])*rot, trans), after[i]));
    }
  } else {
    for (int i=0; i<3; i++){
      assert(myKin.IKleft(Transform3f(rot, -delta[i]+trans), before[i]));
      assert(myKin.IKleft(Transform3f(rot, delta[i]+trans), after[i]));
    }
  }

  for (int j=0; j<6; j++){
    for (int i=0; i<3; i++){
      std::cout<< (after[i][j]-before[i][j])/(2*step) <<", ";
    }
    std::cout<<std::endl;
  }
    

  /*
  vec3f Jacobian[14];
  vec3f before, after;
  vec3f position = vec3f(-1,1,2);
  int frame = 6;

  
  myKin.getJacobian(frame,position,Jacobian);

  for (int i=0; i<14; i++){
    std::cout<<Jacobian[i]<<std::endl;
  }
  std::cout<<std::endl<<myKin.getTransform(frame).transformFwd(position);
  std::cout<<std::endl<<std::endl;


  // rotation
  for (int i=0; i<3; i++){
    assert(myKin.setT_Offset(quatf::fromOmega(-delta[i])));
    before = myKin.getTransform(frame).transformFwd(position);
    assert(myKin.setT_Offset(quatf::fromOmega(2*delta[i])));
    after = myKin.getTransform(frame).transformFwd(position);
    assert(myKin.setT_Offset(quatf::fromOmega(-delta[i])));
    std::cout<<(after-before)/(2*step)<<std::endl;
  }

  // translation
  for (int i=0; i<3; i++){
    assert(myKin.setT_Offset(-delta[i]));
    before = myKin.getTransform(frame).transformFwd(position);
    assert(myKin.setT_Offset(2*delta[i]));
    after = myKin.getTransform(frame).transformFwd(position);
    assert(myKin.setT_Offset(-delta[i]));
    std::cout<<(after-before)/(2*step)<<std::endl;
  }

  std::cout<<std::endl<<myKin.getTransform().matrix()<<std::endl;
  */

}

int main(int argc, char** argv){


  
  
  //  mattTest();

  mattTest2();

  //  keliangTest();


  

  return 0;
}
