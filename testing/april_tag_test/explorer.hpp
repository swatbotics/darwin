#ifndef EXPLORER_HPP
#define EXPLORER_HPP

#include <opencv2/core/core.hpp>

#include "april/TagFamily.h"
#include "april/TagDetector.h"
#include "mjpg_streamer.h"

#include "BallTracker.h"
#include "MotionModule.h"

namespace Robot {

class Explorer : public MotionModule {
public:
  ~Explorer();
  static Explorer* GetInstance();
  void Initialize();
  void Process();
  void ProcessImage();
  void SetupHead();

private:
  static const cv::Point2d kOpticalCenter;
  Explorer();
  static Explorer* m_UniqueInstance;
  mjpg_streamer* streamer_;
  TagFamily tag_family_;
  TagDetector tag_detector_;
  BallTracker tracker_;
  Point2D current_goal_;
};

}  // namespace Robot

#endif  // EXPLORER_HPP
