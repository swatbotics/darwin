#include "TagDetector.h"

#include <sys/time.h>
#include <iostream>

#include <gflags/gflags.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "CameraUtil.h"

DEFINE_int32(device_num, 0,
             "Device number for video device to parse for tag information.");
DEFINE_double(focal_length, 500,
              "Focal length of the camera represented by the video device.");
DEFINE_string(tag_family, "Tag36h11",
              "Tag family to use for detections.");
DEFINE_double(tag_size, 0.1905,
              "Tag size in meters of the tags to detect.");
DEFINE_int32(frame_width, 640, "Desired video frame width.");
DEFINE_int32(frame_height, 480, "Desired video frame height.");
DEFINE_bool(decimate, false, "Set to use decimation for tag detection.");
DEFINE_bool(mirror_display, false,
            "Whether to flip display window image horizontally.");

int main(int argc, char** argv) {
  std::string usage;
  usage += std::string("Usage: ") + argv[0] + std::string(" [OPTIONS]\n");
  usage += std::string("Known tag families:");
  TagFamily::StringArray known = TagFamily::families();
  for (size_t i = 0; i < known.size(); ++i) {
    usage += std::string(" ") + known[i];
  }
  gflags::SetUsageMessage(usage);
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  cv::VideoCapture vc;
  vc.open(FLAGS_device_num);

  // Use uvcdynctrl to figure this out dynamically at some point?
  vc.set(CV_CAP_PROP_FRAME_WIDTH, FLAGS_frame_width);
  vc.set(CV_CAP_PROP_FRAME_HEIGHT, FLAGS_frame_height);

  std::cout << "Set camera to resolution: "
            << vc.get(CV_CAP_PROP_FRAME_WIDTH) << "x"
            << vc.get(CV_CAP_PROP_FRAME_HEIGHT) << "\n";

  cv::Mat frame;
  cv::Point2d opticalCenter;

  vc >> frame;
  if (frame.empty()) {
    std::cerr << "no frames!\n";
    exit(1);
  }

  opticalCenter.x = frame.cols * 0.5;
  opticalCenter.y = frame.rows * 0.5;

  std::string win = "Cam tag test";

  TagFamily family(FLAGS_tag_family);
  TagDetectorParams params;
  if (FLAGS_decimate) {
    params.segDecimate = true;
    std::cout << "will decimate for segmentation!\n";
  }
  TagDetector detector(family, params);
  
  TagDetectionArray detections;

  int cvPose = 0;
  
  while (1) {

    vc >> frame;
    if (frame.empty()) { break; }

    detector.process(frame, opticalCenter, detections);

    cv::Mat show;
    if (detections.empty()) {

      show = frame;

    } else {

      //show = family.superimposeDetections(frame, detections);
      show = frame;

      double s = FLAGS_tag_size;
      double ss = 0.5*s;
      double sz = s;

      enum { npoints = 8, nedges = 12 };

      cv::Point3d src[npoints] = {
        cv::Point3d(-ss, -ss, 0),
        cv::Point3d( ss, -ss, 0),
        cv::Point3d( ss,  ss, 0),
        cv::Point3d(-ss,  ss, 0),
        cv::Point3d(-ss, -ss, sz),
        cv::Point3d( ss, -ss, sz),
        cv::Point3d( ss,  ss, sz),
        cv::Point3d(-ss,  ss, sz),
      };

      int edges[nedges][2] = {

        { 0, 1 },
        { 1, 2 },
        { 2, 3 },
        { 3, 0 },

        { 4, 5 },
        { 5, 6 },
        { 6, 7 },
        { 7, 4 },

        { 0, 4 },
        { 1, 5 },
        { 2, 6 },
        { 3, 7 }

      };

      cv::Point2d dst[npoints];

      double f = FLAGS_focal_length;

      double K[9] = {
        f, 0, opticalCenter.x,
        0, f, opticalCenter.y,
        0, 0, 1
      };

      cv::Mat_<cv::Point3d> srcmat(npoints, 1, src);
      cv::Mat_<cv::Point2d> dstmat(npoints, 1, dst);

      cv::Mat_<double>      Kmat(3, 3, K);

      cv::Mat_<double>      distCoeffs = cv::Mat_<double>::zeros(4,1);

      for (size_t i=0; i<detections.size(); ++i) {

        //for (cvPose=0; cvPose<2; ++cvPose) {
        if (1) {

          cv::Mat r, t;

          if (cvPose) {


            CameraUtil::homographyToPoseCV(f, f, s, 
                                           detections[i].homography,
                                           r, t);

          } else {

            cv::Mat_<double> M = 
              CameraUtil::homographyToPose(f, f, s, 
                                           detections[i].homography,
                                           false);

            cv::Mat_<double> R = M.rowRange(0,3).colRange(0, 3);
          
            t = M.rowRange(0,3).col(3);

            cv::Rodrigues(R, r);

          }

          cv::projectPoints(srcmat, r, t, Kmat, distCoeffs, dstmat);

          for (int j=0; j<nedges; ++j) {
            cv::line(show, 
                     dstmat(edges[j][0],0),
                     dstmat(edges[j][1],0),
                     cvPose ? CV_RGB(0,255,0) : CV_RGB(255,0,0),
                     1, CV_AA);
          }

        }

      }
                                                          

    }

    if (FLAGS_mirror_display) {
      cv::flip(show, show, 1);
    }

    cv::imshow(win, show);
    int k = cv::waitKey(5);
    if (k % 256 == 's') {
      cv::imwrite("frame.png", frame);
      std::cout << "wrote frame.png\n";
    } else if (k % 256 == 'p') {
      cvPose = !cvPose;
    } else if (k % 256 == 27 /* ESC */) {
      break;
    }

  }    

  TagDetector::reportTimers();

  return 0;


}
