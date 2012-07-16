#ifndef _REFINE_H_
#define _REFINE_H_

#include "AprilTypes.h"
#include <opencv2/imgproc/imgproc.hpp>

struct TPoint {

  at::Point p;
  at::real  t;

  TPoint();

  TPoint(const at::Point& point,
         at::real target): p(point), t(target) {}

  TPoint(at::real x, at::real y,
         at::real target): p(x,y), t(target) {}

};

typedef std::vector<TPoint> TPointArray;

cv::Mat shrink(const cv::Mat& image, int scl);
cv::Mat enlarge(const cv::Mat& image, int scl);
cv::Mat addNoise(const cv::Mat& src, cv::RNG& rng, double stddev);

void drawPoint(cv::Mat& m, const at::Point& p, 
               const cv::Scalar& color, 
               at::real sz=3, int thickness=1);

void drawArrow(cv::Mat& m, const at::Point& p, const at::Point& g, 
               const cv::Scalar& color, at::real scl=15.0/255.0);

cv::Rect boundingRect(const at::Point p[4], const cv::Size& sz);
void dilate(cv::Rect& r, int b, const cv::Size& sz);

at::Point interpolate(const at::Point p[4], const at::Point& uv, at::Mat* pJ=0);
at::Point interpolateH(const at::Mat& H, const at::Point& uv, at::Mat* pJxy=0);
void computeH(const at::Point p[4], at::Mat& H, at::Mat* pJh=0);

int refineQuad(const cv::Mat& gimage,
               const at::Mat& gx,
               const at::Mat& gy,
               at::Point p[4],
               const TPointArray& tpoints,
               bool debug = false,
               int max_iter = 25,
               at::real max_grad = 1e-2);

template<class Tval>
inline Tval bsample(const cv::Mat_<Tval>& image, int x, int y) {
  int yy = borderInterpolate(y, image.rows, cv::BORDER_REPLICATE);
  int xx = borderInterpolate(x, image.cols, cv::BORDER_REPLICATE);
  return image(yy,xx);
}

template<class Tval>
inline Tval bsample(const cv::Mat_<Tval>& image, const cv::Point& p) {
  return bsample<Tval>(image, p.x, p.y);
}

template<class Tval>
inline Tval bsample(const cv::Mat_<Tval>& image, const at::Point& p) {
  return bsample<Tval>(image, int(p.x+0.5), int(p.y+0.5));
}

inline at::real cubicInterpolate(at::real p[4], at::real x) {
  return p[1] + at::real(0.5) * x * 
    (p[2] - p[0] + x*(at::real(2.0)*p[0] - 
                      at::real(5.0)*p[1] + 
                      at::real(4.0)*p[2] - 
                      p[3] + x*(at::real(3.0)*(p[1] - p[2]) + p[3] - p[0])));
}

inline at::real bicubicInterpolate (at::real p[4][4], at::real x, at::real y) {
  at::real arr[4];
  arr[0] = cubicInterpolate(p[0], y);
  arr[1] = cubicInterpolate(p[1], y);
  arr[2] = cubicInterpolate(p[2], y);
  arr[3] = cubicInterpolate(p[3], y);
  return cubicInterpolate(arr, x);
}

template <class Tval>
inline at::real bicubicInterpolate(const cv::Mat_<Tval>& image, 
                            const at::Point& pt) {

  at::real fx = pt.x - 0.5;
  at::real fy = pt.y - 0.5;

  int ix = fx;
  int iy = fy;

  fx -= ix;
  fy -= iy;

  at::real p[4][4];

  for (int py=0; py<4; ++py) {
    for (int px=0; px<4; ++px) {
      p[px][py] = bsample(image, ix+px-1, iy+py-1);
    }
  }

  return bicubicInterpolate(p, fx, fy);

}

template <class Tval>
inline cv::Mat_<Tval> subimageWithBorder(const cv::Mat_<Tval>& image,
                                  const cv::Rect& r,
                                  int border) {
  
  cv::Mat_<Tval> bimage(r.height + 2*border, r.width + 2*border);
  
  for (int y=-border; y<r.height+border; ++y) {
    for (int x=-border; x<r.width+border; ++x) {
      bimage(y+border, x+border) = bsample(image, r.x+x, r.y+y);
    }
  }

  return bimage;

}


#endif
