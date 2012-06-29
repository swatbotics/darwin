#include <iostream>
#include <string>
#include <ctype.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sys/stat.h>
#include "Geometry.h"

typedef std::vector<cv::Point2f> CvPoint2fArray;
typedef std::vector<cv::Point3f> CvPoint3fArray;

void saveImage(const cv::Mat& image) {
  
  for (int i=0; i<1000; ++i) {
    char buf[1024];
    sprintf(buf, "calib%04d.png", i);
    struct stat sb;
    if (stat(buf, &sb) < 0) {
      cv::imwrite(buf, image);
      std::cout << "wrote to " << buf << "\n";
      return;
    }
  }

}

void convertToRGB(const cv::Mat& src, cv::Mat& dst) {

  if (src.channels() == 1) {
    cv::cvtColor(src, dst, CV_GRAY2RGB);
  } else {
    dst = src.clone();
  }

}

////////////////////////////////////////////////////////////////////////////////

void convertToGray(const cv::Mat& src, cv::Mat& dst) {
 
  if (src.channels() == 1) {
    dst = src.clone();
  } else {
    cv::cvtColor(src, dst, CV_RGB2GRAY);
  }

}

////////////////////////////////////////////////////////////////////////////////

bool detectCorners(const cv::Mat& orig,
                   const cv::Size& pattern_size,
                   CvPoint2fArray& corners,
                   cv::Mat& rgb) {

  cv::Mat gray;

  convertToGray(orig, gray);
  convertToRGB(orig, rgb);

  bool ok = cv::findChessboardCorners(gray, pattern_size, corners);
  if (!ok) { return false; }

  const cv::TermCriteria SUBPIX_CRIT(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 1e-6);
  const cv::Size SUBPIX_WIN_SIZE(5,5);
  const cv::Size SUBPIX_ZERO_ZONE(-1,-1);

  cv::cornerSubPix(gray, corners, SUBPIX_WIN_SIZE, SUBPIX_ZERO_ZONE, SUBPIX_CRIT);
  
  cv::drawChessboardCorners(rgb, pattern_size, cv::Mat(corners), ok);

  return true;

}

int main(int argc, char** argv) {

  /*
  cv::Size pattern_size(7,7);
  float square_size = 0.067;
  */

  cv::Size pattern_size(8,6);
  float square_size = 0.030;
  
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " DEVICE\n";
    std::cerr << "   or: " << argv[0] << " IMAGE1 IMAGE2 ...\n";
  }

  cv::VideoCapture capture;
  int device = -1;

  std::vector<cv::Mat> calib_images;

  for (int arg=1; arg<argc; ++arg) {

    cv::Mat m = cv::imread(argv[arg]);

    if (!m.empty()) {

      std::cout << "read " << argv[arg] << "\n";
      calib_images.push_back(m);

    } else {

      char* endptr;

      if (capture.isOpened()) {

        std::cerr << "duplicate device " << argv[arg] << "?\n";
        exit(1);

      } else {

        device = strtol(argv[arg], &endptr, 10);
        if (endptr && !*endptr) {

          capture.open(device);
          if (!capture.isOpened()) {
            std::cerr << "error opening " << device << " for capture!\n";
            exit(1);
          }

        } else {

          capture.open(argv[arg]);
          if (!capture.isOpened()) {
            std::cerr << "error opening " << argv[arg] << " for capture!\n";
            exit(1);
          }
          
        }

      }

    }
  }

  cv::namedWindow("calib");

  if (capture.isOpened()) {

    /* open capture window */

    while (1) {

      cv::Mat frame;
      capture >> frame;

      if (frame.empty()) { 
        std::cout << "no more frames!\n";
      }


      cv::Mat detected;
      CvPoint2fArray corners;

      frame = frame.clone();
      bool ok = detectCorners(frame, pattern_size, corners, detected);

      char buf[1024];
      sprintf(buf, "%d images captured", (int)calib_images.size());
      labelImage(detected, buf);

      cv::imshow("capture", detected);

      int k = cv::waitKey(5);
      if (k%256 == 27) { // esc
        break;
      } else if (k%256 == ' ') { // space
        if (ok) {
          calib_images.push_back(frame);
          saveImage(frame);
        }
      }

    }

  }

  if (calib_images.empty()) {

    std::cout << "no images, returning!\n";
    exit(0);

  }

  std::cout << "got " << calib_images.size() << " images\n";
  
  CvPoint3fArray object_points;
  
  for (int y=0; y<pattern_size.height; ++y) {
    for (int x=0; x<pattern_size.width; ++x) {
      cv::Point3f p( x * square_size, y * square_size, 0.0f );
      object_points.push_back(p);
    }
  }

  std::vector<cv::Mat> all_object_points;
  std::vector<cv::Mat> all_corners;

  for (size_t i=0; i<calib_images.size(); ++i) {
    
    cv::Mat display;
    CvPoint2fArray corners;
    bool ok = detectCorners(calib_images[i], pattern_size, corners, display);
    
    if (!ok) {
      labelImage(display, "FAIL!!!\n");
    } else {
      all_object_points.push_back(cv::Mat(object_points).clone());
      all_corners.push_back(cv::Mat(corners).clone());
      char buf[1024];
      sprintf(buf, "Image %d (hit any key)", int(all_corners.size()));
      labelImage(display, buf);
    }

    cv::imshow("capture", display);
    cv::waitKey();
    
  }

  const int FLAGS_NODISTS = ( CV_CALIB_FIX_K1 | CV_CALIB_FIX_K2 | CV_CALIB_FIX_K3 |
                              CV_CALIB_ZERO_TANGENT_DIST );
  
  cv::Mat k, dists;
  
  std::vector< cv::Mat > rvecs, tvecs;

  int flags = FLAGS_NODISTS;

  cv::calibrateCamera( all_object_points, 
                       all_corners,
                       calib_images[0].size(),
                       k, dists, rvecs, tvecs, flags );

  std::cout << "K =\n" << k << "\n";
  std::cout << "dists =\n" << dists << "\n";

  return 0;

  

}
