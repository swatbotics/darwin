/*
 * main.cpp
 *
 *  Created on: 2011. 1. 4.
 *      Author: robotis
 */

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <libgen.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "april/CameraUtil.h"
#include "april/TagDetector.h"

// Necessary because of conflict with opencv and httpd?
#undef MAX
#undef MIN

#include "Camera.h"
#include "mjpg_streamer.h"
#include "LinuxDARwIn.h"

#define INI_FILE_PATH       "config.ini"
#define U2D_DEV_NAME        "/dev/ttyUSB0"

#define DEFAULT_TAG_FAMILY "Tag36h11"
// For the tag family above, measured in meters.
#define DEFAULT_TAG_SIZE 0.1905
// From Nick Rhinehart's code, TagPositionWriter.java
#define DARWIN_FOCAL_LENGTH 265.45

void change_current_dir() {
  char exepath[1024] = {0};
  if (readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1) {
    chdir(dirname(exepath));
  }
}

void print_double_visually(double min, double max, double value) {
  static const int meter_width = 50;
  int bucket = (value - min) * meter_width / (max - min);
  bucket = (bucket < 0) ? 0 : bucket;
  bucket = (bucket >= meter_width) ? meter_width : bucket;
  printf("% 4.2f [", min);
  for (int i = 0; i < meter_width; ++i) {
    printf("%c", (i == bucket) ? '+' : ' ');
  }
  printf("] % 4.2f", max);
  printf(" (%.4f)\n", value);
}

int main(void) {
  printf( "\n===== April Tag Test for DARwIn =====\n\n");
  change_current_dir();
  
  minIni* ini = new minIni(INI_FILE_PATH);
  LinuxCamera::GetInstance()->Initialize(0);
  LinuxCamera::GetInstance()->LoadINISettings(ini);
  
  mjpg_streamer* streamer = new mjpg_streamer(Camera::WIDTH, Camera::HEIGHT);
  ColorFinder* ball_finder = new ColorFinder();
  ball_finder->LoadINISettings(ini);
  httpd::ball_finder = ball_finder;
  BallTracker tracker = BallTracker();

  //////////////////// Framework Initialize ////////////////////////////
  LinuxCM730 linux_cm730(U2D_DEV_NAME);
  CM730 cm730(&linux_cm730);
  MotionManager* manager = MotionManager::GetInstance();
  if (manager->Initialize(&cm730) == false) {
    printf("Failed to initialize MotionManager!\n");
    return 1;
  }
  manager->AddModule(Head::GetInstance());	
  LinuxMotionTimer* motion_timer = new LinuxMotionTimer(manager);
  motion_timer->Start();
  
  MotionStatus::m_CurrentJoints.SetEnableBodyWithoutHead(false);
  manager->SetEnable(true);
  /////////////////////////////////////////////////////////////////////
  
  Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
  Head::GetInstance()->m_Joint.SetPGain(JointData::ID_HEAD_PAN, 8);
  Head::GetInstance()->m_Joint.SetPGain(JointData::ID_HEAD_TILT, 8);

  TagFamily family(DEFAULT_TAG_FAMILY);
  TagDetector detector(family);
  detector.segDecimate = true;
  cv::Point2d opticalCenter;
  opticalCenter.x = Camera::WIDTH * 0.5;
  opticalCenter.y = Camera::HEIGHT * 0.5;
  TagDetectionArray detections;
  
  LinuxCamera* camera = LinuxCamera::GetInstance();
  while (true) {
    Point2D pos;
    camera->CaptureFrame();
    Image* hsv_image = camera->fbuffer->m_HSVFrame;
    tracker.Process(ball_finder->GetPosition(hsv_image));

    Image* rgb_ball = LinuxCamera::GetInstance()->fbuffer->m_RGBFrame;
    for (int i = 0; i < rgb_ball->m_NumberOfPixels; i++) {
      if (ball_finder->m_result->m_ImageData[i] == 1) {
	rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 0] = 255;
	rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 1] = 0;
	rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 2] = 0;
      }
    }
    streamer->send_image(rgb_ball);

    unsigned char* raw_frame = camera->fbuffer->m_RGBFrame->m_ImageData;
    cv::Mat frame(Camera::HEIGHT, Camera::WIDTH, CV_8UC3, raw_frame);
    detector.process(frame, opticalCenter, detections);
    //    TagDetector::reportTimers();
    for (TagDetectionArray::iterator it = detections.begin();
	 it != detections.end(); ++it) {
      printf("Found tag id: %zd\n", it->id);
      static const double f = DARWIN_FOCAL_LENGTH;
      at::Mat r, t;
      CameraUtil::homographyToPoseCV(f, f, DEFAULT_TAG_SIZE,
				     it->homography,
				     r, t);
      std::cout << "r = " << r << std::endl;
      std::cout << "t = " << t << std::endl;
      std::cout << "t(0): ";
      print_double_visually(-0.5, 0.5, t[0][0]);
      std::cout << "t(1): ";
      print_double_visually(-0.5, 0.5, t[1][0]);
      std::cout << "t(2): ";
      print_double_visually(0.0, 2.0, t[2][0]);
    }
  }
  return 0;
}
