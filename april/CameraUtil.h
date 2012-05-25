#ifndef _CAMERAUTIL_H_
#define _CAMERAUTIL_H_

#include "AprilTypes.h"

namespace CameraUtil {

  at::Mat homographyToPose(at::real fx, at::real fy, 
                           at::real tagSize,
                           const at::Mat& horig,
                           bool openGLStyle=false);

  void homographyToPoseCV(at::real fx, at::real fy,
                          at::real tagSize,
                          const at::Mat& horig,
                          cv::Mat& rvec,
                          cv::Mat& tvec);

};

#endif
