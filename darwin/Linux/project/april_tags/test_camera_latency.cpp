#include <cstdio>
#include <iostream>
#include <string>
#include <time.h>

#include <pthread.h>
#include <gflags/gflags.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

DEFINE_int32(device_num, 0,
             "Device number for video device to parse for tag information.");
DEFINE_int32(frame_width, 640, "Desired video frame width.");
DEFINE_int32(frame_height, 480, "Desired video frame height.");
DEFINE_int32(timing_interval, 1,
             "Target interval between timing printouts in nanoseconds.");

void runVideoCapture() {
  cv::VideoCapture vc;
  vc.open(FLAGS_device_num);
  // Use uvcdynctrl to figure this out dynamically at some point?
  vc.set(CV_CAP_PROP_FRAME_WIDTH, FLAGS_frame_width);
  vc.set(CV_CAP_PROP_FRAME_HEIGHT, FLAGS_frame_height);
  std::cout << "Set camera to resolution: "
            << vc.get(CV_CAP_PROP_FRAME_WIDTH) << "x"
            << vc.get(CV_CAP_PROP_FRAME_HEIGHT) << "\n";

  const std::string win = "Camera/OpenCV latency test";
  cv::Mat frame;
  while (true) {
    vc >> frame;
    if (frame.empty()) {
      std::cerr << "no frames!\n";
      exit(1);
    }
    cv::imshow(win, frame);
    int k = cv::waitKey(5);
    if (k % 256 == 27 /* ESC */) break;
  }
}

void* runTimingThread(void* /* unused */) {
  struct timespec ts, sleep_ts;
  sleep_ts.tv_sec = 0;
  sleep_ts.tv_nsec = FLAGS_timing_interval;
  while (true) {
    clock_gettime(CLOCK_REALTIME, &ts);
    fprintf(stderr, "%ld.%09ld\n", ts.tv_sec, ts.tv_nsec);
    nanosleep(&sleep_ts, NULL);
  }
  return NULL;
}

int main(int argc, char** argv) {
  std::string usage;
  usage += std::string("Usage: ") + argv[0] + std::string(" [OPTIONS]\n");
  gflags::SetUsageMessage(usage);
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  pthread_t time_thread;
  pthread_create(&time_thread, NULL, &runTimingThread, NULL);

  runVideoCapture();

  pthread_cancel(time_thread);
  pthread_join(time_thread, NULL);
  return 0;
}
