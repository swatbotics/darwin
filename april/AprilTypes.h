#ifndef _APRILTYPES_H_
#define _APRILTYPES_H_

#include <opencv2/core/core.hpp>
#include <stdint.h>

namespace at {

  typedef uint64_t code_t;
  typedef unsigned int uint;

#if 1

  typedef float real;
  enum { IMAGE_TYPE = CV_32F };
#define AT_REAL_MAX FLT_MAX
#define AT_EPSILON  0.0000001;  

#else

  typedef double real;
  enum { IMAGE_TYPE = CV_64F };
#define AT_REAL_MAX DBL_MAX
#define AT_EPSILON  0.000000001

#endif

  typedef cv::Point_<real> Point;
  typedef cv::Point3_<real> Point3;
  typedef cv::Vec<real, 3> Vec3;
  typedef cv::Mat_<real> Mat;

};

#endif
