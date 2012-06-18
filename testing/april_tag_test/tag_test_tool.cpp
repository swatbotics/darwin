#include <cstdio>
#include <iostream>
#include <set>

#include <opencv2/core/core.hpp>

#include "april/TagDetector.h"
#include "april/TagFamily.h"

#include "Camera.h"
#include "LinuxDARwIn.h"
#include "mjpg_streamer.h"

#include "util.hpp"

#define INI_FILE_PATH "config.ini"
#define U2D_DEV_NAME "/dev/ttyUSB0"

// Params specifying the tags to look for.
#define DEFAULT_TAG_FAMILY "Tag36h11"

// Whether to show a timing report for tag detection.
#define REPORT_TAG_TIMING false

// Params for marking a ring around detected tags.
#define TAG_RING_RADIUS 20  // In pixels.
#define TAG_RING_THICKNESS 1  // In pixels.
#define TAG_RING_DEFAULT_COLOR_RGB {255, 0, 0}

namespace {

static const rgb_color TAG_RING_DEFAULT_COLOR = TAG_RING_DEFAULT_COLOR_RGB;

void print_usage(const char* tool_name) {
  fprintf(stderr,
          "Usage: %s [-d] [-t] [-f FAMILY]\n"
          "Run a tool to test Darwin's tag detection. Options:\n"
          "  -d           Use decimation for segmentation stage.\n"
          "  -t           Show timing information for tag detection.\n"
          "  -v           Show verbose debug info from tag detection.\n"
          "  -f FAMILY    Look for the given tag family (default \"%s\")\n",
          tool_name, DEFAULT_TAG_FAMILY);
  fprintf(stderr, "Known tag families:");
  TagFamily::StringArray known = TagFamily::families();
  for (size_t i = 0; i < known.size(); ++i) {
    fprintf(stderr, " %s", known[i].c_str());
  }
  fprintf(stderr, "\n");
}

void InitializeCamera() {
  std::cout << "Initializing camera..." << std::endl;
  minIni* ini = new minIni(INI_FILE_PATH);
  LinuxCamera* camera = LinuxCamera::GetInstance();
  camera->Initialize(0);
  camera->LoadINISettings(ini);
}

void InitializeMotionFramework() {
  std::cout << "Initializing motion framework..." << std::endl;
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

void InitializeMotionModules() {
  std::cout << "Initializing motion modules..." << std::endl;
  MotionManager* manager = MotionManager::GetInstance();
  Head* head = Head::GetInstance();
  manager->AddModule(head);
  head->m_Joint.SetEnableHeadOnly(true, true);
  head->m_Joint.SetPGain(JointData::ID_HEAD_PAN, 4); //8);
  head->m_Joint.SetPGain(JointData::ID_HEAD_TILT, 4); //8);
  MotionStatus::m_CurrentJoints.SetEnableBodyWithoutHead(false);
  manager->SetEnable(true);
  const double kTiltUpperLimit = 40.0;
  head->MoveByAngle(0, kTiltUpperLimit);
}

void mark_point_on_image(const Robot::Point2D& point, Robot::Image* rgb_image,
                         rgb_color color=TAG_RING_DEFAULT_COLOR) {
  unsigned char* framebuf = rgb_image->m_ImageData;
  const int kScanRadius = TAG_RING_RADIUS + TAG_RING_THICKNESS + 1;
  for (int x = point.X - kScanRadius; x <= point.X + kScanRadius; ++x) {
    for (int y = point.Y - kScanRadius; y <= point.Y + kScanRadius; ++y) {
      if (x < 0 || x >= rgb_image->m_Width ||
          y < 0 || y >= rgb_image->m_Height) continue;
      int dist = sqrt(pow(x - point.X, 2) + pow(y - point.Y, 2));
      if (dist >= TAG_RING_RADIUS &&
          dist <= TAG_RING_RADIUS + TAG_RING_THICKNESS) {
        size_t offset = (y * rgb_image->m_Width + x) * rgb_image->m_PixelSize;
        framebuf[offset + 0] = color.R;
        framebuf[offset + 1] = color.G;
        framebuf[offset + 2] = color.B;
      }
    }
  }
}

}  // namespace

int main(int argc, char* argv[]) {
  const char* family_str = DEFAULT_TAG_FAMILY;
  bool decimate = false;
  bool use_timing = false;
  bool show_debug_info = false;
  const char* options = "f:dtv";
  int c;
  while ((c = getopt(argc, argv, options)) != -1) {
    switch (c) {
      case 'f':
        family_str = optarg;
        break;
      case 'd':
        decimate = true;
        break;
      case 't':
        use_timing = true;
        break;
      case 'v':  // For "verbose".
        show_debug_info = true;
        break;
      default:
        fprintf(stderr, "\n");
        print_usage(argv[0]);
        exit(1);
    }
  }

  TagFamily family(family_str);
  TagDetector detector(family);
  if (decimate) detector.segDecimate = true;
  if (show_debug_info) {
    detector.debug = true;
    detector.debugWindowName = "Tag test tool";
  }

  // NOTE: Must initialize camera before framework!
  InitializeCamera();
  // NOTE: Must initialize streamer before framework!
  mjpg_streamer streamer(Camera::WIDTH, Camera::HEIGHT);

  InitializeMotionFramework();
  printf("Initialize robot position? (hit enter) ");
  getchar();
  InitializeMotionModules();

  const cv::Point2d kOpticalCenter(Camera::WIDTH / 2.0,
                                   Camera::HEIGHT / 2.0);

  while (true) {
    LinuxCamera::GetInstance()->CaptureFrame();
    Image* rgb_image = LinuxCamera::GetInstance()->fbuffer->m_RGBFrame;
    TagDetectionArray detections;
    unsigned char* raw_frame = rgb_image->m_ImageData;
    cv::Mat frame(Camera::HEIGHT, Camera::WIDTH, CV_8UC3, raw_frame);
    detector.process(frame, kOpticalCenter, detections);
    if (use_timing) TagDetector::reportTimers();

    static std::set<size_t> seen_tags;
    std::set<size_t> new_seen_tags;
    for (TagDetectionArray::iterator it = detections.begin();
         it != detections.end(); ++it) {
      new_seen_tags.insert(it->id);
      if (seen_tags.count(it->id) == 0) {
        printf("Found tag %zd!\n", it->id);
      }
      Point2D tag_image_center(it->cxy.x, it->cxy.y);
      mark_point_on_image(tag_image_center, rgb_image);
    }
    seen_tags = new_seen_tags;
    
    streamer.send_image(rgb_image);
  }

}
