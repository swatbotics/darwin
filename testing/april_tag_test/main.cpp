#include <algorithm>  // For min() and max().
#include <cmath>
#include <cstdio>
#include <libgen.h>  // For dirname().
#include <string>
#include <sys/time.h>
#include <unistd.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "april/CameraUtil.h"
#include "april/TagDetector.h"

#include "Camera.h"
#include "mjpg_streamer.h"
#include "LinuxDARwIn.h"

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

typedef struct {
  unsigned char R;
  unsigned char G;
  unsigned char B;
} rgb_color;

static const rgb_color TAG_RING_DEFAULT_COLOR = TAG_RING_DEFAULT_COLOR_RGB;

void change_current_dir() {
  char exepath[1024] = {0};
  if (readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1) {
    if (chdir(dirname(exepath)) != 0) {
      std::cerr << "Error changing directory to: " << exepath << std::endl;
    }
  }
}

double record_elapsed_time() {
  static double last_time = -1.0;
  struct timeval tval;
  gettimeofday(&tval, NULL);
  double this_time = tval.tv_sec + tval.tv_usec / 1000000.0;
  double elapsed_time = (last_time < 0) ? 0 : this_time - last_time;
  last_time = this_time;
  return elapsed_time;
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
  manager->AddModule(Head::GetInstance());
  MotionStatus::m_CurrentJoints.SetEnableBodyWithoutHead(false);
  manager->SetEnable(true);
  Head* head = Head::GetInstance();
  head->m_Joint.SetEnableHeadOnly(true, true);
  head->m_Joint.SetPGain(JointData::ID_HEAD_PAN, 8);
  head->m_Joint.SetPGain(JointData::ID_HEAD_TILT, 8);
}

void print_double_visually(const char* label, double min, double max,
			   double value) {
  if (label != NULL) printf("%s: ", label);
  static const int meter_width = 50;
  int bucket = (value - min) * meter_width / (max - min);
  bucket = std::max(0, bucket);
  bucket = std::min(meter_width - 1, bucket);
  printf("% 4.2f [", min);
  for (int i = 0; i < meter_width; ++i) {
    printf("%c", (i == bucket) ? '+' : ' ');
  }
  printf("] % 4.2f", max);
  printf(" (%.4f)\n", value);
}

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

void mark_point_on_image(const Point2D& point, Image* rgb_image,
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


int main(void) {
  printf( "\n===== April Tag Test for DARwIn =====\n\n");
  change_current_dir();  // To make relative filenames work.

  // Initialize camera-related code.
  minIni* ini = new minIni(INI_FILE_PATH);
  LinuxCamera* camera = LinuxCamera::GetInstance();
  camera->Initialize(0);
  camera->LoadINISettings(ini);
  ColorFinder* ball_finder = new ColorFinder();
  ball_finder->LoadINISettings(ini);

  // Initialize MJPG-Streamer code.
  httpd::ball_finder = ball_finder;
  mjpg_streamer* streamer = new mjpg_streamer(Camera::WIDTH, Camera::HEIGHT);

  // Initialize actual Darwin framework and motion modules.
  initialize_motion_framework();
  initialize_motion_modules();
  BallTracker tracker = BallTracker();

  // Initialize April Tag detection code.
  TagFamily tag_family(DEFAULT_TAG_FAMILY);
  TagDetector detector(tag_family);
  detector.segDecimate = true;
  TagDetectionArray detections;
  cv::Point2d optical_center(Camera::WIDTH / 2.0, Camera::HEIGHT / 2.0);

  // Start main motion loop and timer.
  record_elapsed_time();
  while (true) {
    // Capture camera frame and process to detect April Tags.
    camera->CaptureFrame();
    Image* rgb_image = LinuxCamera::GetInstance()->fbuffer->m_RGBFrame;
    unsigned char* raw_frame = rgb_image->m_ImageData;
    cv::Mat frame(Camera::HEIGHT, Camera::WIDTH, CV_8UC3, raw_frame);
    detector.process(frame, optical_center, detections);
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
      tracker.Process(min_center);
      static rgb_color green = {0, 255, 0};
      mark_point_on_image(min_center, rgb_image, green);

    } else {
      // We didn't find a tag - make a sad noise if this is a change.
      if (found_tag) LinuxActionScript::PlayMP3(TAG_LOST_MP3_FILE);
      found_tag = false;
    }

    // Send the modified camera image to streaming server.
    streamer->send_image(rgb_image);

    // Compute and print current frames-per-second value.
    double frame_time = record_elapsed_time();
    printf("FPS: % 2d\n",  (int) (1 / frame_time));
  }
  return 0;
}
