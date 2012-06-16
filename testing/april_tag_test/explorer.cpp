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
#define REACHED_GOAL_MP3_FILE "../../darwin/Data/mp3/Yes.mp3"

// Whether to show a timing report for tag detection.
#define REPORT_TAG_TIMING false

// Params for marking a ring around detected tags.
#define TAG_RING_RADIUS 20  // In pixels.
#define TAG_RING_THICKNESS 3  // In pixels.
#define TAG_RING_DEFAULT_COLOR_RGB {0, 255, 0}

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

  tag_detector_.segDecimate = true;

  InitializeMotionFramework();
  printf("Initialize robot position? (hit enter) ");
  getchar();
  InitializeRobotPosition();
  InitializeMotionModules();
  printf("Start walking mode? (hit enter) ");
  getchar();
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
  head->m_Joint.SetPGain(JointData::ID_HEAD_PAN, 4); //8);
  head->m_Joint.SetPGain(JointData::ID_HEAD_TILT, 4); //8);
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
  for (int id = JointData::ID_R_SHOULDER_PITCH;
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
  if (cm730_ == NULL) {
    std::cerr << "CM730 is not initialized!" << std::endl;
    exit(1);
  }
  cm730_->SyncWrite(MX28::P_GOAL_POSITION_L, 5,
                    JointData::NUMBER_OF_JOINTS - 1, param);
}

void Explorer::Process() {
  LinuxCamera::GetInstance()->CaptureFrame();
  Image* rgb_image = LinuxCamera::GetInstance()->fbuffer->m_RGBFrame;
  TagDetectionArray detections;
  FindDetections(rgb_image, &detections);

  static bool found_tag = false;
  static bool at_goal = false;
  static size_t goal_tag_id = 0;  // Make this smarter?
  TagInfoMap tag_map = ProcessDetections(detections, goal_tag_id, rgb_image);
  if (tag_map.count(goal_tag_id) > 0) {
    // We found a tag - make an excited noise if this is a change.
    if (!found_tag) LinuxActionScript::PlayMP3(TAG_FOUND_MP3_FILE);
    found_tag = true;
    cv::Point2d goal_point(tag_map[goal_tag_id].x, tag_map[goal_tag_id].y);
    bool reached_goal = MoveToGoal(goal_point);
    if (reached_goal && !at_goal) {
      LinuxActionScript::PlayMP3(REACHED_GOAL_MP3_FILE);
      usleep(10 * 1000);
      printf("\nReached goal tag %d! Continue? (hit enter) ", goal_tag_id);
      getchar();
      ++goal_tag_id;
      found_tag = false;
    }
    at_goal = reached_goal;
  } else {
    // We didn't find a tag - make a sad noise if this is a change.
    if (found_tag) LinuxActionScript::PlayMP3(TAG_LOST_MP3_FILE);
    found_tag = false;
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
    const TagDetectionArray& detections, size_t goal_tag_id,
    Image* display_image) {
  Head* head = Head::GetInstance();
  static const double TILT_OFFSET = 40.0;
  double pan_angle = head->GetPanAngle();  // Left is positive.
  double tilt_angle = head->GetTiltAngle() - TILT_OFFSET;  // Up is positive.
  print_double_visually("p", -80, 80, pan_angle);
  print_double_visually("t", -90, 90, tilt_angle);
  // Convert to radians.
  double p = pan_angle * M_PI / 180;
  double t = tilt_angle * M_PI / 180;
  cv::Mat pan_mat = (cv::Mat_<double>(3, 3) <<
                     cos(p), -sin(p),      0,
                     sin(p),  cos(p),      0,
                          0,       0,      1);
  cv::Mat tilt_mat = (cv::Mat_<double>(3, 3) <<
                       cos(t),       0, sin(t),
                            0,       1,      0,
                      -sin(t),       0, cos(t));
  cv::Mat transform = pan_mat * tilt_mat;

  TagInfoMap tag_map;
  // Print info on each detected tag and mark it on the image.
  for (size_t i = 0; i < detections.size(); ++i) {
    const TagDetection& d = detections[i];
    TagInfo& tag = tag_map[d.id];
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

  for (TagInfoMap::iterator it = tag_map.begin(); it != tag_map.end(); ++it) {
    TagInfo& tag = it->second;
    std::cout << "Found tag id: " << tag.id << std::endl;
    //    print_double_visually("X",  0.0, 2.0, tag.head_x);
    //    print_double_visually("Y", -2.0, 2.0, tag.head_y);
    //    print_double_visually("Z", -1.0, 1.0, tag.head_z);
    print_double_visually("x",  0.0, 2.0, tag.x);
    print_double_visually("y", -2.0, 2.0, tag.y);
    print_double_visually("z", -1.0, 1.0, tag.z);
    Point2D tag_image_center(tag.center.x, tag.center.y);
    static rgb_color blue = {0, 0, 255};
    static rgb_color green = {0, 255, 0};
    bool main_tag = (it->first == goal_tag_id);
    mark_point_on_image(tag_image_center, display_image,
                        main_tag ? green : blue);
    if (main_tag) {
      current_goal_ = tag_image_center;
      tracker_.Process(current_goal_);
    }
  }
  return tag_map;
}

bool Explorer::MoveToGoal(const cv::Point2d& goal) {
  static const double kXLimClose = 0.1;
  static const double kXLimFar = 0.2;
  static const double kYLimLeft = -0.05;
  static const double kYLimRight = 0.05;
  bool at_goal = (in_range(goal.x, kXLimClose, kXLimFar) &&
                  in_range(goal.y, kYLimLeft, kYLimRight));
  Walking* walker = Walking::GetInstance();
  if (at_goal) {
    walker->X_MOVE_AMPLITUDE = 0.0;
    walker->A_MOVE_AMPLITUDE = 0.0;
    walker->Stop();
    std::cout << "STOPPING WALKER (Reached goal!)" << std::endl;
    return true;
  } else {
    if (goal.x < kXLimClose) {
      walker->X_MOVE_AMPLITUDE = -5.0;
      walker->A_MOVE_AMPLITUDE = 0.0;
    } else {
      double goal_angle = atan2(goal.y, goal.x);
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
  return false;
}

void Explorer::LookForGoal() {
  /*
  Head::GetInstance()->MoveToHome();
  Walking* walker = Walking::GetInstance();
  double direction_guess = (current_goal_.X < kOpticalCenter.x ? 1.0 : -1.0);
  walker->X_MOVE_AMPLITUDE = 0.0;
  walker->A_MOVE_AMPLITUDE = 15.0 * direction_guess;
  if (!walker->IsRunning()) {
    walker->Start();
  }
  */
  Walking* walker = Walking::GetInstance();
  walker->X_MOVE_AMPLITUDE = 0.0;
  walker->A_MOVE_AMPLITUDE = 0.0;
  walker->Stop();
  static const double kPanRate = 1.0;
  static const double kScanTiltAngle = 40.0;
  Head* head = Head::GetInstance();
  static double direction = (current_goal_.X < kOpticalCenter.x ? 1.0 : -1.0);
  double angle = head->GetPanAngle();
  double new_angle = angle + direction * kPanRate;
  if (new_angle <= head->GetRightLimitAngle()) direction = 1;
  if (new_angle >= head->GetLeftLimitAngle()) direction = -1;
  new_angle = angle + direction * kPanRate;
  head->MoveByAngle(new_angle, kScanTiltAngle);
  std::cout << "PAUSING WALKER (Waiting for goal)" << std::endl;
}

}  // namespace Robot
