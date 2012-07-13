#include "Refine.h"
#include <iostream>
#include <opencv2/highgui/highgui.hpp>

at::Point interpolate(const at::Point p[4], const at::Point& uv, at::Mat* pJ) {

  at::real tmp0 = p[1].x*p[2].y;
  at::real tmp1 = p[2].x*p[1].y;
  at::real tmp2 = tmp0 - tmp1;
  at::real tmp3 = p[1].x*p[3].y;
  at::real tmp4 = tmp2 - tmp3;
  at::real tmp5 = p[3].x*p[1].y;
  at::real tmp6 = p[2].x*p[3].y;
  at::real tmp7 = p[3].x*p[2].y;
  at::real tmp8 = tmp4 + tmp5 + tmp6 - tmp7;
  at::real tmp9 = p[0].x*p[2].x;
  at::real tmp10 = tmp9*p[1].y;
  at::real tmp11 = p[1].x*p[2].x;
  at::real tmp12 = p[0].x*p[3].x;
  at::real tmp13 = p[1].x*p[3].x;
  at::real tmp14 = tmp13*p[0].y;
  at::real tmp15 = tmp9*p[3].y;
  at::real tmp16 = tmp13*p[2].y;
  at::real tmp17 = tmp10 - tmp11*p[0].y - tmp12*p[1].y + tmp14 - tmp15 + tmp12*p[2].y + tmp11*p[3].y - tmp16;
  at::real tmp18 = p[0].x*p[1].x;
  at::real tmp19 = p[2].x*p[3].x;
  at::real tmp20 = tmp18*p[2].y - tmp10 - tmp18*p[3].y + tmp14 + tmp15 - tmp19*p[0].y - tmp16 + tmp19*p[1].y;
  at::real tmp21 = p[0].x*p[1].y;
  at::real tmp22 = p[1].x*p[0].y;
  at::real tmp23 = tmp22*p[2].y;
  at::real tmp24 = tmp21*p[3].y;
  at::real tmp25 = p[2].x*p[0].y;
  at::real tmp26 = p[3].x*p[0].y;
  at::real tmp27 = tmp26*p[2].y;
  at::real tmp28 = tmp1*p[3].y;
  at::real tmp29 = tmp21*p[2].y - tmp23 - tmp24 + tmp22*p[3].y - tmp25*p[3].y + tmp27 + tmp28 - tmp5*p[2].y;
  at::real tmp30 = p[0].x*p[2].y;
  at::real tmp31 = tmp23 - tmp25*p[1].y - tmp24 + tmp26*p[1].y + tmp30*p[3].y - tmp27 - tmp0*p[3].y + tmp28;
  at::real tmp32 = p[0].x*p[3].y;
  at::real tmp33 = tmp30 - tmp25 - tmp32 - tmp0 + tmp1 + tmp26 + tmp3 - tmp5;
  at::real tmp34 = tmp21 - tmp22;
  at::real tmp35 = tmp34 - tmp30 + tmp25 + tmp3 - tmp5 - tmp6 + tmp7;
  at::real hx = (tmp17/tmp8)*uv.x - (tmp20/tmp8)*uv.y + p[0].x;
  at::real hy = (tmp29/tmp8)*uv.x - (tmp31/tmp8)*uv.y + p[0].y;
  at::real hw = (tmp33/tmp8)*uv.x + (tmp35/tmp8)*uv.y + 1;

  if (pJ) {

    at::Mat& J = *pJ;
    if (J.rows != 2 || J.cols != 8) { J = at::Mat(2,8); }

    at::real tmp36 = tmp8*tmp8;
    at::real tmp37 = p[2].y - p[3].y;
    at::real tmp38 = p[1].y - p[2].y;
    at::real tmp39 = ((tmp37*tmp8)/tmp36)*uv.x + ((tmp38*tmp8)/tmp36)*uv.y;
    at::real tmp40 = hw*hw;
    J[0][0] = (((((tmp1 - tmp5 - tmp6 + tmp7)*tmp8)/tmp36)*uv.x - (((tmp4 + tmp6)*tmp8)/tmp36)*uv.y + 1)*hw - hx*tmp39)/tmp40;
    at::real tmp41 = -p[2].x + p[3].x;
    at::real tmp42 = -p[1].x + p[2].x;
    at::real tmp43 = ((tmp41*tmp8)/tmp36)*uv.x + ((tmp42*tmp8)/tmp36)*uv.y;
    J[0][1] = (((((-tmp11 + tmp13)*tmp8)/tmp36)*uv.x - (((tmp13 - tmp19)*tmp8)/tmp36)*uv.y)*hw - hx*tmp43)/tmp40;
    at::real tmp44 = tmp30 - tmp32;
    at::real tmp45 = (((-p[2].y + p[3].y)*tmp8 - tmp33*tmp37)/tmp36)*uv.x + (((-p[0].y + p[3].y)*tmp8 - tmp35*tmp37)/tmp36)*uv.y;
    J[0][2] = (((((-tmp25 + tmp26 + tmp6 - tmp7)*tmp8 - tmp17*tmp37)/tmp36)*uv.x - (((tmp44 + tmp26 - tmp7)*tmp8 - tmp20*tmp37)/tmp36)*uv.y)*hw - hx*tmp45)/tmp40;
    at::real tmp46 = (((p[2].x - p[3].x)*tmp8 - tmp33*tmp41)/tmp36)*uv.x + (((p[0].x - p[3].x)*tmp8 - tmp35*tmp41)/tmp36)*uv.y;
    J[0][3] = (((((tmp9 - tmp12)*tmp8 - tmp17*tmp41)/tmp36)*uv.x - (((-tmp9 + tmp19)*tmp8 - tmp20*tmp41)/tmp36)*uv.y)*hw - hx*tmp46)/tmp40;
    at::real tmp47 = -p[1].y + p[3].y;
    at::real tmp48 = (((-p[0].y + p[1].y)*tmp8 - tmp33*tmp47)/tmp36)*uv.x + (((p[0].y - p[3].y)*tmp8 - tmp35*tmp47)/tmp36)*uv.y;
    J[0][4] = (((((tmp34 - tmp32 + tmp3)*tmp8 - tmp17*tmp47)/tmp36)*uv.x - (((-tmp21 + tmp32 - tmp26 + tmp5)*tmp8 - tmp20*tmp47)/tmp36)*uv.y)*hw - hx*tmp48)/tmp40;
    at::real tmp49 = p[1].x - p[3].x;
    at::real tmp50 = (((p[0].x - p[1].x)*tmp8 - tmp33*tmp49)/tmp36)*uv.x + (((-p[0].x + p[3].x)*tmp8 - tmp35*tmp49)/tmp36)*uv.y;
    J[0][5] = (((((tmp12 - tmp13)*tmp8 - tmp17*tmp49)/tmp36)*uv.x - (((tmp18 - tmp13)*tmp8 - tmp20*tmp49)/tmp36)*uv.y)*hw - hx*tmp50)/tmp40;
    at::real tmp51 = -tmp21 + tmp22;
    at::real tmp52 = (((p[0].y - p[1].y)*tmp8 - tmp33*tmp38)/tmp36)*uv.x + (((-p[1].y + p[2].y)*tmp8 - tmp35*tmp38)/tmp36)*uv.y;
    J[0][6] = (((((tmp51 + tmp30 - tmp0)*tmp8 - tmp17*tmp38)/tmp36)*uv.x - (((tmp22 - tmp25 - tmp0 + tmp1)*tmp8 - tmp20*tmp38)/tmp36)*uv.y)*hw - hx*tmp52)/tmp40;
    at::real tmp53 = p[1].y*p[2].y;
    at::real tmp54 = (((-p[0].x + p[1].x)*tmp8 - tmp33*tmp42)/tmp36)*uv.x + (((p[1].x - p[2].x)*tmp8 - tmp35*tmp42)/tmp36)*uv.y;
    J[0][7] = (((((-tmp9 + tmp11)*tmp8 - tmp17*tmp42)/tmp36)*uv.x - (((-tmp18 + tmp9)*tmp8 - tmp20*tmp42)/tmp36)*uv.y)*hw - hx*tmp54)/tmp40;
    at::real tmp55 = p[1].y*p[3].y;
    at::real tmp56 = p[2].y*p[3].y;
    J[1][0] = (((((tmp53 - tmp55)*tmp8)/tmp36)*uv.x - (((-tmp55 + tmp56)*tmp8)/tmp36)*uv.y)*hw - hy*tmp39)/tmp40;
    J[1][1] = (((((-tmp0 + tmp3 - tmp6 + tmp7)*tmp8)/tmp36)*uv.x - (((tmp2 + tmp5 - tmp7)*tmp8)/tmp36)*uv.y + 1)*hw - hy*tmp43)/tmp40;
    at::real tmp57 = p[0].y*p[2].y;
    at::real tmp58 = p[0].y*p[3].y;
    J[1][2] = (((((-tmp57 + tmp58)*tmp8 - tmp29*tmp37)/tmp36)*uv.x - (((tmp57 - tmp56)*tmp8 - tmp31*tmp37)/tmp36)*uv.y)*hw - hy*tmp45)/tmp40;
    J[1][3] = (((((tmp44 + tmp6 - tmp7)*tmp8 - tmp29*tmp41)/tmp36)*uv.x - (((-tmp25 - tmp32 + tmp26 + tmp6)*tmp8 - tmp31*tmp41)/tmp36)*uv.y)*hw - hy*tmp46)/tmp40;
    at::real tmp59 = p[0].y*p[1].y;
    J[1][4] = (((((-tmp58 + tmp55)*tmp8 - tmp29*tmp47)/tmp36)*uv.x - (((-tmp59 + tmp55)*tmp8 - tmp31*tmp47)/tmp36)*uv.y)*hw - hy*tmp48)/tmp40;
    J[1][5] = (((((tmp34 + tmp26 - tmp5)*tmp8 - tmp29*tmp49)/tmp36)*uv.x - (((tmp22 + tmp32 - tmp26 - tmp3)*tmp8 - tmp31*tmp49)/tmp36)*uv.y)*hw - hy*tmp50)/tmp40;
    J[1][6] = (((((tmp57 - tmp53)*tmp8 - tmp29*tmp38)/tmp36)*uv.x - (((tmp59 - tmp57)*tmp8 - tmp31*tmp38)/tmp36)*uv.y)*hw - hy*tmp52)/tmp40;
    J[1][7] = (((((tmp51 - tmp25 + tmp1)*tmp8 - tmp29*tmp42)/tmp36)*uv.x - (((-tmp21 + tmp30 - tmp0 + tmp1)*tmp8 - tmp31*tmp42)/tmp36)*uv.y)*hw - hy*tmp54)/tmp40;

  }

  return at::Point(hx/hw, hy/hw);

}


cv::Mat shrink(const cv::Mat& image, int scl) {
  cv::Mat small;
  cv::resize(image, small, cv::Size(image.cols/scl, image.rows/scl), 0, 0,
             CV_INTER_AREA);
  return small;
}

cv::Mat enlarge(const cv::Mat& image, int scl) {
  cv::Mat big;
  cv::resize(image, big, cv::Size(scl*image.cols, scl*image.rows), 
             0, 0, CV_INTER_CUBIC);
  return big;
}

cv::Mat addNoise(const cv::Mat& src, cv::RNG& rng, double stddev) {

  cv::Mat input;

  if (src.depth() != at::IMAGE_TYPE) {
    src.convertTo(input, CV_MAKETYPE(at::IMAGE_TYPE, src.channels()));
  } else {
    input = src.clone();
  }

  std::vector<cv::Mat> chans;

  cv::split(input, chans);

  for (size_t c=0; c<chans.size(); ++c) {
    at::Mat m = chans[c];
    for (int y=0; y<m.rows; ++y) {
      for (int x=0; x<m.cols; ++x) {
        m(y,x) += rng.gaussian(stddev);
      }
    }
  }

  cv::merge(chans, input);

  cv::Mat output; 

  if (input.depth() != src.depth()) {
    input.convertTo(output, CV_MAKETYPE(src.depth(), src.channels()));
  } else {
    output = input;
  }

  return output;

}

void drawPoint(cv::Mat& m, const at::Point& p, 
               const cv::Scalar& color, at::real sz, int thickness) {
  //cv::Point dp(sz, sz);
  //cv::rectangle(m, p-dp, p+dp, color, 1, 4);
  const int shift = 2;
  const int ss = (1 << shift);
  cv::circle(m, p*ss, sz*ss, color, thickness, CV_AA, shift);
}

void drawArrow(cv::Mat& m, const at::Point& p, const at::Point& g, 
               const cv::Scalar& color, at::real scl) {

  const int shift = 2;
  const int ss = (1 << shift);

  cv::line(m, ss*p, ss*(p+g*scl), color, 1, CV_AA, shift);

}

cv::Rect boundingRect(const at::Point p[4], const cv::Size sz) {

  // get the rectangle
  int x0 = p[0].x, x1 = p[0].x, y0 = p[0].y, y1 = p[0].y;
  
  for (int i=0; i<4; ++i) {

    x0 = std::min(x0, int(p[i].x));
    y0 = std::min(y0, int(p[i].y));

    x1 = std::max(x1, int(p[i].x)+1);
    y1 = std::max(y1, int(p[i].y)+1);

  }

  x0 = std::max(x0, 0);
  x1 = std::min(x1, sz.width);

  y0 = std::max(y0, 0);
  y1 = std::min(y1, sz.height);

  int w = x1-x0;
  int h = y1-y0;

  return cv::Rect(x0, y0, w, h);

}


int refineQuad(const cv::Mat& gmat,
               const at::Mat& gx,
               const at::Mat& gy,
               at::Point p[4],
               const TPointArray& tpoints,
               bool debug,
               int max_iter,
               at::real max_grad) {

  assert( gmat.type() == CV_8UC1 );

  cv::Mat_<unsigned char> gimage = gmat;

  cv::Rect rect = boundingRect(p, gimage.size());

  cv::Mat subimage(gimage, rect);
  double dmin, dmax;
  cv::minMaxLoc(subimage, &dmin, &dmax);
  at::real amin = dmin;
  at::real amax = dmax;
  at::real ascl = 255 / (amax - amin);

  at::real gnscl = at::real(1) / (tpoints.size() * 255 * 255);

  bool done = false;
  int iter;
  
  for (iter=0; iter<max_iter && !done; ++iter) {

    at::Mat g = at::Mat::zeros(8, 1);
    at::real err = 0;

    cv::Mat debug_big;
    int debug_scl = 16;
    at::Point debug_delta;

    if (debug) {

      int border = 4;

      cv::Mat gsmall = subimageWithBorder(gimage, rect, border);
      gsmall = (gsmall - amin) * ascl;

      cv::Mat gbig = enlarge(gsmall, debug_scl);

      debug_delta = at::Point(border-rect.x, border-rect.y);

      cv::cvtColor(gbig, debug_big, CV_GRAY2RGB);
      for (int i=0; i<4; ++i) {
        drawPoint(debug_big, (p[i]+debug_delta)*debug_scl, CV_RGB(255,0,0));
      }
      
    }

    for (int i=0; i<tpoints.size(); ++i) {

      const TPoint& tpi = tpoints[i];
      
      at::Mat Ji(2,8);

      at::Point pi = interpolate(p, tpi.p, &Ji);

      at::real oi = (bicubicInterpolate(gimage, pi) - amin) * ascl;

      at::Point gi = at::Point( bicubicInterpolate(gx, pi), 
                                bicubicInterpolate(gy, pi) ) * ascl;

      at::real ti = tpi.t;
      
      at::real ei = ti - oi;
      err += ei * ei;
      
      for (int j=0; j<8; ++j) {
        at::real Jij = gi.x * Ji(0, j) + gi.y * Ji(1, j);
        g(j,0) += ei * Jij;
      }
      
      if (debug) {

        cv::Scalar color = tpi.t ? CV_RGB(0,255,0) : CV_RGB(0,0,255);

        drawPoint( debug_big, (pi+debug_delta)*debug_scl, color );
        drawArrow( debug_big, (pi+debug_delta)*debug_scl, ei * gi, color, 1e-3 );

      }
      
    }
    
    at::real gn = cv::norm(g)*gnscl;

    if (gn < max_grad || iter+1 == max_iter) { done = true; }

    at::real ss = 4e-6;

    for (int i=0; i<4; ++i) {
      p[i].x += ss*g(2*i+0, 0);
      p[i].y += ss*g(2*i+1, 0);
    }

    if (debug) {

      if (iter == 0 || done) {

        std::cout << "at iter " << iter+1 << ":\n";
        std::cout << "err = " << err << "\n";
        std::cout << "grad. norm = " << gn << " (scaled=" << gn/gnscl << ")\n";
        std::cout << "\n";

      }

      cv::imshow("image", debug_big);
      cv::waitKey();

    }

  }

  return iter;

}
