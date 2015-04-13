#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <libgen.h>
#include <string>
#include <iostream>
#include <sstream>
#include <vector>
#include <assert.h>

#include "Body.h"
#include "Camera.h"
#include "mjpg_streamer.h"
#include "LinuxDARwIn.h"

using namespace std;

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

class SimpleTrajectory {
public:

  double dt;
  size_t njoints;
  size_t nticks;

  std::vector<double> angles_rad;


};

enum { NUM_JOINTS = 20 } ;

bool parse_file(const char* filename, SimpleTrajectory& traj) {

  std::ifstream istr(filename);
  if (!istr.is_open()) {
    std::cerr << "error opening file " << filename << "!\n";
    return false;
  }

  std::string header_str;

  // parse our header
  istr >> header_str;

  if (header_str != "TRAJ") {
    std::cerr << "expected TRAJ header!\n";
    return false;
  }

  if (!(istr >> traj.njoints >> traj.nticks >> traj.dt)) {
    std::cerr << "error parsing header!\n";
    return false;
  }

  if (traj.njoints != NUM_JOINTS) {
    std::cerr << "incorrect # joints: got " << traj.njoints << ", expected " << NUM_JOINTS << "\n";
    return false;
  }

  if (traj.dt <= 0) {
    std::cerr << "expected dt > 0!\n";
    return false;
  }

  if (!traj.nticks) {
    std::cerr << "refuse to read empty trajectory!\n";
    return false;
  }

  size_t offset = 0;

  traj.angles_rad.resize(traj.nticks * traj.njoints);

  for (size_t t=0; t<traj.nticks; ++t) {
    for (size_t i=0; i<traj.njoints; ++i) { 
      if (!(istr >> traj.angles_rad[offset])) {
	std::cerr << "error parsing angle!\n";
	return false;
      }
      ++offset;
    }
  }
  
  return true;

}

int main(int argc, char** argv)
{

  SimpleTrajectory traj;

  if (argc != 2) {
    std::cerr << "usage: " << argv[0] << " TRAJ_FILENAME.txt\n";
    return 1;
  }

  if (!parse_file(argv[1], traj)) {
    std::cerr << "no trajectory loaded, exiting.\n";
    return 1;
  }

  std::cout << "loaded " << argv[1] << "\n";

  if (motion_initialization()){
    printf("Failed to initialize control!\n");
    return -1;
  }

  // this is frightening but I think it needs to wait for motion initialization to do stuff
  sleep(1);

  // convert delta t from trajectory to microseconds
  size_t dt_usec = size_t(traj.dt * 1e6);

  // the offset of the current tick within trajectory data
  size_t offset = 0;

  assert( traj.angles_rad.size() == traj.nticks * NUM_JOINTS );

  // loop over trajectory
  for (size_t t=0; t<traj.nticks; ++t) {

    // put trajectory data into motors
    for (int i=0; i<NUM_JOINTS; ++i) {
      // note motor indices start at 1, so need to add i+1 for motor_number
      Body::GetInstance()->m_Joint.SetRadian(i+1, traj.angles_rad[offset]);
      ++offset;
    }

    usleep(dt_usec);

  }

  return 0;

}
