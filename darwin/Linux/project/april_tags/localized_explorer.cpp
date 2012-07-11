#include "localized_explorer.hpp"

#include <iostream>

#include <gflags/gflags.h>
#include <LinuxDARwIn.h>

#include "util.hpp"

#define INI_FILE_PATH "config.ini"
#define U2D_DEV_NAME "/dev/ttyUSB0"

DEFINE_string(server_name, "192.168.1.7",
              "IP address or DNS name of the status server to query.");
DEFINE_int32(server_port, 9000,
             "Port on the status server to connect to.");

namespace Robot {

LocalizedExplorer::LocalizedExplorer() :
    cm730_(NULL),
    client_(FLAGS_server_name, FLAGS_server_port) {
}

LocalizedExplorer::~LocalizedExplorer() {}

void LocalizedExplorer::Initialize() {
  printf("\n===== INIT EXPLORER =====\n\n");
  // To make relative filenames work.
  change_dir_from_root("darwin/Linux/project/april_tags");
  InitializeMotionFramework();
  printf("Initialize robot position? (hit enter) "); getchar();
  InitializeMotionModules();
  printf("Start tracking mode? (hit enter) "); getchar();
  client_.Run();
}

void LocalizedExplorer::InitializeMotionFramework() {
  std::cout << "Initializing motion framework..." << std::endl;
  LinuxCM730* linux_cm730 = new LinuxCM730(U2D_DEV_NAME);
  cm730_ = new CM730(linux_cm730);
  MotionManager* manager = MotionManager::GetInstance();
  if (manager->Initialize(cm730_) == false) {
    printf("Failed to initialize MotionManager!\n");
    exit(1);
  }
  LinuxMotionTimer* motion_timer = new LinuxMotionTimer(manager);
  motion_timer->Start();
}

void LocalizedExplorer::InitializeMotionModules() {
  std::cout << "Initializing motion modules..." << std::endl;
  MotionManager* manager = MotionManager::GetInstance();
  Head* head = Head::GetInstance();
  manager->AddModule(head);
  head->m_Joint.SetEnableHeadOnly(true, true);
  head->m_Joint.SetPGain(JointData::ID_HEAD_PAN, 4); //8);
  head->m_Joint.SetPGain(JointData::ID_HEAD_TILT, 4); //8);
  MotionStatus::m_CurrentJoints.SetEnableBodyWithoutHead(false);
  manager->SetEnable(true);
}

void LocalizedExplorer::Process() {
  static const double kPanRate = 1.0;
  static const double kScanTiltAngle = 40.0;
  Head* head = Head::GetInstance();
  static double direction = 1.0;
  double angle = head->GetPanAngle();
  double new_angle = angle + direction * kPanRate;
  if (new_angle <= head->GetRightLimitAngle()) direction = 1;
  if (new_angle >= head->GetLeftLimitAngle()) direction = -1;
  new_angle = angle + direction * kPanRate;
  head->MoveByAngle(new_angle, kScanTiltAngle);
  std::cout << "PAUSING WALKER (Waiting for goal)" << std::endl;


  std::cout << client_.GetData() << std::endl;
  usleep(50 * 1000);

  //  head->MoveByAngle(0, 0);
}

}  // namespace Robot

int main(void) {
  printf( "\n===== April Tag Test for DARwIn =====\n\n");
  Robot::LocalizedExplorer explorer;
  explorer.Initialize();
  record_elapsed_time();
  while (true) {
    explorer.Process();
    double frame_time = record_elapsed_time();
    printf("FPS: % 2d\n",  (int) (1 / frame_time));
  }
  return 0;
}
