#include "explorer.hpp"

#include <cmath>
#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "april/CameraUtil.h"

#include "Camera.h"
#include "LinuxDARwIn.h"

#include "util.hpp"

#define INI_FILE_PATH "config.ini"
#define U2D_DEV_NAME "/dev/ttyUSB0"

// Focal length of the camera on the Darwin.
#define DARWIN_FOCAL_LENGTH 265.45  // In pixels.

// Params specifying the tags to look for.
#define DEFAULT_TAG_FAMILY "Tag36h11"
#define DEFAULT_TAG_SIZE 0.05 //0.1905  // In meters.

// Paths to MP3 files to play amusing noises when discovering tags.
#define TAG_FOUND_MP3_FILE "../../darwin/Data/mp3/Wow.mp3"
#define TAG_LOST_MP3_FILE "../../darwin/Data/mp3/Oops.mp3"

// Params for marking a ring around detected tags.
#define TAG_RING_RADIUS 20  // In pixels.
#define TAG_RING_THICKNESS 3  // In pixels.
#define TAG_RING_DEFAULT_COLOR_RGB {0, 255, 0}

// Whether to show a timing report for tag detection.
#define REPORT_TAG_TIMING false

//static const double PI = 3.1415926;
static const rgb_color TAG_RING_DEFAULT_COLOR = TAG_RING_DEFAULT_COLOR_RGB;

void mark_point_on_image(const Robot::Point2D& point, Robot::Image* rgb_image,
                         rgb_color color=TAG_RING_DEFAULT_COLOR) {
  unsigned char* framebuf = rgb_image->m_ImageData;
  for (int i = 0; i < rgb_image->m_NumberOfPixels; i++) {
    int x = i % rgb_image->m_Width;
    int y = i / rgb_image->m_Width;
    int dist = sqrt(pow(x - point.X, 2) + pow(y - point.Y, 2));
    if (dist >= TAG_RING_RADIUS &&
        dist <= TAG_RING_RADIUS + TAG_RING_THICKNESS) {
      size_t offset = i * rgb_image->m_PixelSize;
      framebuf[offset + 0] = color.R;
      framebuf[offset + 1] = color.G;
      framebuf[offset + 2] = color.B;
    }
  }
}


namespace Robot {

const cv::Point2d Explorer::kOpticalCenter(Camera::WIDTH / 2.0,
                                           Camera::HEIGHT / 2.0);

Explorer::Explorer() :
    cm730_(NULL),
    streamer_(NULL),
    tag_family_(DEFAULT_TAG_FAMILY),
    tag_detector_(tag_family_),
    tracker_(),
    current_goal_() {
}

Explorer::~Explorer() {}

void Explorer::Initialize() {
  printf("\n===== INIT EXPLORER =====\n\n");

  // NOTE: Must initialize camera before framework!
  InitializeCamera();

  // NOTE: Must initialize streamer before framework!
  streamer_ = new mjpg_streamer(Camera::WIDTH, Camera::HEIGHT);

  // Initialize April Tag detection code.
  tag_detector_.segDecimate = true;

  InitializeMotionFramework();
  InitializeMotionModules();
  /*
  printf("Initialize robot position? (hit enter) ");
  getchar();
  InitializeRobotPosition();
  */
}

void Explorer::InitializeCamera() {
  std::cout << "Initializing camera..." << std::endl;
  minIni* ini = new minIni(INI_FILE_PATH);
  LinuxCamera* camera = LinuxCamera::GetInstance();
  camera->Initialize(0);
  camera->LoadINISettings(ini);
}

void Explorer::InitializeMotionFramework() {
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

void Explorer::InitializeMotionModules() {
  std::cout << "Initializing motion modules..." << std::endl;
  MotionManager* manager = MotionManager::GetInstance();
  Head* head = Head::GetInstance();
  Walking* walker = Walking::GetInstance();
  manager->AddModule(head);
  manager->AddModule(walker);
  head->m_Joint.SetEnableHeadOnly(true, true);
  head->m_Joint.SetPGain(JointData::ID_HEAD_PAN, 8);
  head->m_Joint.SetPGain(JointData::ID_HEAD_TILT, 8);
  //  MotionStatus::m_CurrentJoints.SetEnableBodyWithoutHead(false);
  walker->m_Joint.SetEnableBodyWithoutHead(true, true);
  manager->SetEnable(true);
}

void Explorer::InitializeRobotPosition() {
  Head* head = Head::GetInstance();
  head->MoveByAngle(0, 40);

  int n = 0;
  int param[JointData::NUMBER_OF_JOINTS * 5];
  int wGoalPosition, wStartPosition, wDistance;
  for(int id = JointData::ID_R_SHOULDER_PITCH;
      id < JointData::NUMBER_OF_JOINTS; ++id) {
    wStartPosition = MotionStatus::m_CurrentJoints.GetValue(id);
    wGoalPosition = Walking::GetInstance()->m_Joint.GetValue(id);
    wDistance = abs(wStartPosition - wGoalPosition) >> 2;
    if (wDistance < 8) wDistance = 8;
    param[n++] = id;
    param[n++] = CM730::GetLowByte(wGoalPosition);
    param[n++] = CM730::GetHighByte(wGoalPosition);
    param[n++] = CM730::GetLowByte(wDistance);
    param[n++] = CM730::GetHighByte(wDistance);
  }
  cm730_->SyncWrite(MX28::P_GOAL_POSITION_L, 5, JointData::NUMBER_OF_JOINTS - 1, param);
}

void Explorer::Process() {
  LinuxCamera::GetInstance()->CaptureFrame();
  Image* rgb_image = LinuxCamera::GetInstance()->fbuffer->m_RGBFrame;
  TagDetectionArray detections;
  FindDetections(rgb_image, &detections);

  static bool found_tag = false;
  if (detections.size() >= 1) {
    // We found a tag - make an excited noise if this is a change.
    if (!found_tag) LinuxActionScript::PlayMP3(TAG_FOUND_MP3_FILE);
    found_tag = true;
    TagInfoMap map = ProcessDetections(detections, rgb_image);
    MoveToGoal(map.begin()->second);
  } else {
    // We didn't find a tag - make a sad noise if this is a change.
    if (found_tag) LinuxActionScript::PlayMP3(TAG_LOST_MP3_FILE);
    found_tag = false;
    /*
    static const Point2D& no_goal_value = Point2D(-1, -1);
    current_goal_ = no_goal_value;
    */
    LookForGoal();
  }

  // Send the modified camera image to streaming server.
  if (streamer_ == NULL) {
    std::cerr << "mjpg-streamer not initialized!" << std::endl;
    exit(1);
  }
  streamer_->send_image(rgb_image);
}

void Explorer::FindDetections(const Image* camera_image,
                              TagDetectionArray* detections) {
  unsigned char* raw_frame = camera_image->m_ImageData;
  cv::Mat frame(Camera::HEIGHT, Camera::WIDTH, CV_8UC3, raw_frame);
  tag_detector_.process(frame, kOpticalCenter, *detections);
  if (REPORT_TAG_TIMING) TagDetector::reportTimers();
}

Explorer::TagInfoMap Explorer::ProcessDetections(
    const TagDetectionArray& detections, Image* display_image) {
  Head* head = Head::GetInstance();
  static const double TILT_OFFSET = 40.0;
  double pan_angle = head->GetPanAngle();  // Left is positive.
  double tilt_angle = head->GetTiltAngle() - TILT_OFFSET;  // Up is positive.
  //  print_double_visually("pan", -180, 180, pan_angle);
  //  print_double_visually("tilt", -90, 90, tilt_angle);
  // Convert to radians, and invert angles to invert the rotations.
  double p = pan_angle * M_PI / 180 * -1;
  double t = tilt_angle * M_PI / 180 * -1;
  cv::Mat pan_mat = (cv::Mat_<double>(3, 3) <<
                     cos(p), -sin(p),      0,
                     sin(p),  cos(p),      0,
                          0,       0,      1);
  cv::Mat tilt_mat = (cv::Mat_<double>(3, 3) <<
                       cos(t),       0, sin(t),
                            0,       1,      0,
                      -sin(t),       0, cos(t));
  cv::Mat transform = pan_mat * tilt_mat;

  TagInfoMap tagmap;
  // Print info on each detected tag and mark it on the image.
  for (size_t i = 0; i < detections.size(); ++i) {
    const TagDetection& d = detections[i];
    TagInfo& tag = tagmap[d.id];
    tag.id = d.id;
    tag.center = d.cxy;
    static const double f = DARWIN_FOCAL_LENGTH;
    CameraUtil::homographyToPoseCV(f, f, DEFAULT_TAG_SIZE,
                                   d.homography, tag.raw_r, tag.raw_t);
    tag.head_x =  tag.raw_t[2][0];  // Robot x is forward; camera z is forward.
    tag.head_y = -tag.raw_t[0][0];  // Robot y is left; camera x is right.
    tag.head_z = -tag.raw_t[1][0];  // Robot z is up; camera y is down.
    tag.head_t = (cv::Mat_<double>(3, 1) <<
                  tag.head_x, tag.head_y, tag.head_z);
    tag.t = transform * tag.head_t;
    tag.x = tag.t[0][0];
    tag.y = tag.t[1][0];
    tag.z = tag.t[2][0];
  }

  for (TagInfoMap::iterator it = tagmap.begin(); it != tagmap.end(); ++it) {
    TagInfo& tag = it->second;
    std::cout << "Found tag id: " << tag.id << std::endl;
    print_double_visually("x",  0.0, 2.0, tag.x);
    print_double_visually("y", -2.0, 2.0, tag.y);
    print_double_visually("z", -1.0, 1.0, tag.z);
    Point2D tag_image_center(tag.center.x, tag.center.y);
    static rgb_color blue = {0, 0, 255};
    static rgb_color green = {0, 255, 0};
    bool main_tag = (it == tagmap.begin());
    mark_point_on_image(tag_image_center, display_image,
                        main_tag ? green : blue);
    if (main_tag) {
      current_goal_ = tag_image_center;
      tracker_.Process(current_goal_);
    }
  }
  return tagmap;
}

void Explorer::MoveToGoal(const TagInfo& goal_tag) {
  static const double kXLimClose = 0.1;
  static const double kXLimFar = 0.2;
  static const double kYLimLeft = -0.05;
  static const double kYLimRight = 0.05;
  bool at_goal = (in_range(goal_tag.x, kXLimClose, kXLimFar) &&
                  in_range(goal_tag.y, kYLimLeft, kYLimRight));
  Walking* walker = Walking::GetInstance();
  if (at_goal) {
    walker->X_MOVE_AMPLITUDE = 0.0;
    walker->A_MOVE_AMPLITUDE = 0.0;
    walker->Stop();
    std::cout << "STOPPING WALKER (Reached goal!)" << std::endl;
  } else {
    if (goal_tag.x < kXLimClose) {
      walker->X_MOVE_AMPLITUDE = -5.0;
      walker->A_MOVE_AMPLITUDE = 0.0;
    } else {
      double goal_angle = atan2(goal_tag.y, goal_tag.x);
      double goal_angle_deg = goal_angle * 180 / M_PI;
      double goal_angle_sign = copysign(1.0, goal_angle_deg);
      printf("GOAL ANGLE: %f\n", goal_angle_deg);

      if (in_range(goal_angle_deg, -10, 10)) {
        walker->X_MOVE_AMPLITUDE = 10.0;
        walker->A_MOVE_AMPLITUDE = 0.0;
      } else {
        walker->X_MOVE_AMPLITUDE = 0.0;
        walker->A_MOVE_AMPLITUDE = 15.0 * -goal_angle_sign;
      }
    }
    if (!walker->IsRunning()) {
      walker->Start();
      std::cout << "STARTING WALKER" << std::endl;
    }
  }
  printf("WALKER PARAMS: X = %f, A = %f\n",
         walker->X_MOVE_AMPLITUDE, walker->A_MOVE_AMPLITUDE);
}

void Explorer::LookForGoal() {
  Head::GetInstance()->MoveToHome();
  Walking* walker = Walking::GetInstance();
  double direction_guess = (current_goal_.X < kOpticalCenter.x ? 1.0 : -1.0);
  walker->X_MOVE_AMPLITUDE = 0.0;
  walker->A_MOVE_AMPLITUDE = 15.0 * direction_guess;
  if (!walker->IsRunning()) {
    walker->Start();
  }
  std::cout << "PAUSING WALKER (Waiting for goal)" << std::endl;
}

}  // namespace Robot
