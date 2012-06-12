#ifndef EXPLORER_HPP
#define EXPLORER_HPP

#include <opencv2/core/core.hpp>

#include "april/TagFamily.h"
#include "april/TagDetector.h"
#include "mjpg_streamer.h"

#include "BallTracker.h"

namespace Robot {

class Explorer {
public:
  Explorer();
  ~Explorer();
  void Initialize();
  void Process();

private:
  static const cv::Point2d kOpticalCenter;
  void InitializeCamera();
  void InitializeMotionFramework();
  void InitializeMotionModules();
  mjpg_streamer* streamer_;
  TagFamily tag_family_;
  TagDetector tag_detector_;
  BallTracker tracker_;
  Point2D current_goal_;
};

}  // namespace Robot

#endif  // EXPLORER_HPP
