#include "TagDetector.h"
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "CameraUtil.h"
#include <sys/time.h>

int main(int argc, char** argv) {

  const std::string dstr = "--decimate";

  if (argc < 3 || argc > 4 || (argc == 4 && argv[3] != dstr)) {
    std::cerr << "Usage: " << argv[0] << " TAGFAMILY DEVICE [--decimate]\n";
    std::cerr << "Known tag families:";
    TagFamily::StringArray known = TagFamily::families();
    for (size_t i=0; i<known.size(); ++i) { std::cerr << " " << known[i]; }
    std::cerr << "\n";
    return 1;
  }
  
  cv::VideoCapture vc;

  char* endptr;
  int device = strtol(argv[2], &endptr, 10);

  if (endptr && !*endptr) {
    vc.open(device);
  } else {
    vc.open(argv[2]);
  }

  // Use uvcdynctrl to figure this out dynamically at some point?
  vc.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
  vc.set(CV_CAP_PROP_FRAME_HEIGHT, 720);

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

  TagFamily family(argv[1]);
  TagDetectorParams params;
  if (argc == 4 && argv[3] == dstr) {
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

      double s = 0.1905;
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

      double f = 500;

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

    cv::flip(show, show, 1);

    cv::imshow(win, show);
    int k = cv::waitKey(5);
    if (k % 256 == 's') {
      cv::imwrite("frame.png", frame);
      std::cout << "wrote frame.png\n";
    } else if (k % 256 == 'p') {
      cvPose = !cvPose;
    } else if (k % 256 == 27) {
      break;
    }

  }    

  TagDetector::reportTimers();

  return 0;


}
