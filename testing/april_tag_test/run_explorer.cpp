#include <iostream>

#include "Camera.h"
#include "mjpg_streamer.h"
#include "LinuxDARwIn.h"

#include "april/CameraUtil.h"
#include "april/TagDetector.h"

#include "explorer.hpp"
#include "util.hpp"

#define INI_FILE_PATH "config.ini"
#define U2D_DEV_NAME "/dev/ttyUSB0"

// NOTE: MUST INITIALIZE CAMERA BEFORE FRAMEWORK!
void initialize_camera() {
  minIni* ini = new minIni(INI_FILE_PATH);
  LinuxCamera* camera = LinuxCamera::GetInstance();
  camera->Initialize(0);
  camera->LoadINISettings(ini);
}

void initialize_motion_framework() {
  LinuxCM730* linux_cm730 = new LinuxCM730(U2D_DEV_NAME);
  CM730* cm730 = new CM730(linux_cm730);
  MotionManager* manager = MotionManager::GetInstance();
  if (manager->Initialize(cm730) == false) {
    printf("Failed to initialize MotionManager!\n");
    exit(1);
  }
  LinuxMotionTimer* motion_timer = new LinuxMotionTimer(manager);
  motion_timer->Start();
}

void initialize_motion_modules() {
  MotionManager* manager = MotionManager::GetInstance();
  manager->AddModule(Explorer::GetInstance());
  Explorer::GetInstance()->SetupHead();
  manager->SetEnable(true);
}

int main(void) {
  printf( "\n===== April Tag Test for DARwIn =====\n\n");
  change_current_dir();  // To make relative filenames work.

  minIni* ini = new minIni(INI_FILE_PATH);
  initialize_camera();
  ColorFinder* ball_finder = new ColorFinder();
  ball_finder->LoadINISettings(ini);
  httpd::ball_finder = ball_finder;

  initialize_motion_framework();
  initialize_motion_modules();

  Explorer* explorer = Explorer::GetInstance();
  record_elapsed_time();
  while (true) {
    explorer->ProcessImage();
    // Compute and print current frames-per-second value.
    double frame_time = record_elapsed_time();
    printf("FPS: % 2d\n",  (int) (1 / frame_time));
  }
  return 0;
}
