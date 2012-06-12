#include "explorer.hpp"

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
#define DEFAULT_TAG_SIZE 0.1905  // In meters.

// Paths to MP3 files to play amusing noises when discovering tags.
#define TAG_FOUND_MP3_FILE "../../darwin/Data/mp3/Wow.mp3"
#define TAG_LOST_MP3_FILE "../../darwin/Data/mp3/Oops.mp3"

// Params for marking a ring around detected tags.
#define TAG_RING_RADIUS 20  // In pixels.
#define TAG_RING_THICKNESS 3  // In pixels.
#define TAG_RING_DEFAULT_COLOR_RGB {0, 255, 0}

// Whether to show a timing report for tag detection.
#define REPORT_TAG_TIMING false

static const rgb_color TAG_RING_DEFAULT_COLOR = TAG_RING_DEFAULT_COLOR_RGB;

void print_tag_detection_info(const TagDetection& d) {
  std::cout << "Found tag id: " << d.id << std::endl;
  at::Mat r, t;
  static const double f = DARWIN_FOCAL_LENGTH;
  CameraUtil::homographyToPoseCV(f, f, DEFAULT_TAG_SIZE,
				 d.homography, r, t);
  std::cout << "r = " << r << std::endl;
  std::cout << "t = " << t << std::endl;
  print_double_visually("t_x", -0.5, 0.5, t[0][0]);
  print_double_visually("t_y", -0.5, 0.5, t[1][0]);
  print_double_visually("t_z",  0.0, 2.0, t[2][0]);
}

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

Explorer* Explorer::m_UniqueInstance = NULL;
const cv::Point2d Explorer::kOpticalCenter(Camera::WIDTH / 2.0,
					   Camera::HEIGHT / 2.0);

Explorer::Explorer() :
  streamer_(NULL),
  tag_family_(DEFAULT_TAG_FAMILY),
  tag_detector_(tag_family_),
  tracker_(),
  current_goal_() {
}

Explorer::~Explorer() {}

Explorer* Explorer::GetInstance() {
  if (m_UniqueInstance == NULL) {
    m_UniqueInstance = new Explorer();
  }
  return m_UniqueInstance;
}

void Explorer::Initialize() {
  printf("\n===== INIT EXPLORER MODULE =====\n\n");
  change_current_dir();  // To make relative filenames work.

  streamer_ = new mjpg_streamer(Camera::WIDTH, Camera::HEIGHT);

  // Initialize April Tag detection code.
  tag_detector_.segDecimate = true;

  // Initialize actual Darwin framework and motion modules.
  MotionStatus::m_CurrentJoints.SetEnableBodyWithoutHead(false);
}

void Explorer::SetupHead() {
  MotionManager* manager = MotionManager::GetInstance();
  Head* head = Head::GetInstance();
  manager->AddModule(head);
  head->m_Joint.SetEnableHeadOnly(true, true);
  head->m_Joint.SetPGain(JointData::ID_HEAD_PAN, 8);
  head->m_Joint.SetPGain(JointData::ID_HEAD_TILT, 8);
}

void Explorer::Process() {
  //  std::cout << "Process! Goal is: (" << current_goal_.X << ","
  //	    << current_goal_.Y << ")" << std::endl;
  //  tracker_.Process(current_goal_);
}

void Explorer::ProcessImage() {
  //  std::cout << "ProcessImage()!" << std::endl;
  LinuxCamera::GetInstance()->CaptureFrame();
  Image* rgb_image = LinuxCamera::GetInstance()->fbuffer->m_RGBFrame;
  unsigned char* raw_frame = rgb_image->m_ImageData;
  cv::Mat frame(Camera::HEIGHT, Camera::WIDTH, CV_8UC3, raw_frame);
  TagDetectionArray detections;
  tag_detector_.process(frame, kOpticalCenter, detections);
  if (REPORT_TAG_TIMING) TagDetector::reportTimers();

  static bool found_tag = false;
  if (detections.size() >= 1) {
    // We found a tag - make an excited noise if this is a change.
    if (!found_tag) LinuxActionScript::PlayMP3(TAG_FOUND_MP3_FILE);
    found_tag = true;
    size_t min_id = detections[0].id;
    Point2D min_center;
    // Print info on each detected tag and mark it on the image.
    for (size_t i = 0; i < detections.size(); ++i) {
      TagDetection& d = detections[i];
      print_tag_detection_info(d);
      Point2D tag_center(d.cxy.x, d.cxy.y);
      min_id = std::min(min_id, d.id);
      if (d.id == min_id) min_center = tag_center;
      static rgb_color blue = {0, 0, 255};
      mark_point_on_image(tag_center, rgb_image, blue);
    }
    // Highlight the lowest-id tag and set the Darwin's head to track it.
    current_goal_ = min_center;
    tracker_.Process(current_goal_);
    static rgb_color green = {0, 255, 0};
    mark_point_on_image(min_center, rgb_image, green);

  } else {
    // We didn't find a tag - make a sad noise if this is a change.
    if (found_tag) LinuxActionScript::PlayMP3(TAG_LOST_MP3_FILE);
    found_tag = false;
    static const Point2D& no_goal_value = Point2D(-1, -1);
    current_goal_ = no_goal_value;
  }

  // Send the modified camera image to streaming server.
  if (streamer_ == NULL) {
    std::cerr << "mjpg-streamer not initialized!" << std::endl;
    exit(1);
  }
  streamer_->send_image(rgb_image);
}

}  // namespace Robot
