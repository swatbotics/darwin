#ifndef _IMAGEUTIL_H_
#define _IMAGEUTIL_H_

#include <opencv2/core/core.hpp>

enum ScaleType {
  ScaleNone,
  ScaleMinMax,
  ScaleAbs
};


int resizeToDisplay(const cv::Mat& src, cv::Mat& dst,
                    int w=1200, int h=780);

cv::Mat rescaleImage(const cv::Mat& img, ScaleType type);

void labelImage(cv::Mat& img, const std::string& text);

void labelAndWaitForKey(const std::string& window,
                        const std::string& text, 
                        const cv::Mat& img, 
                        ScaleType type,
                        bool resize);

#endif
