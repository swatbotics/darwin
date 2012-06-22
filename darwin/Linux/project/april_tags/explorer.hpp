#ifndef EXPLORER_HPP
#define EXPLORER_HPP

#include <map>

#include <opencv2/core/core.hpp>

#include "april/TagFamily.h"
#include "april/TagDetector.h"
#include "mjpg_streamer.h"

#include "BallTracker.h"
#include "CM730.h"

namespace Robot {

class Explorer {
 public:
  Explorer();
  ~Explorer();
  void Initialize();
  void Process();

 private:
  typedef struct {
    size_t id;
    cv::Point2d center;
    cv::Mat_<double> raw_r;
    cv::Mat_<double> raw_t;
    cv::Mat_<double> head_t;
    double head_x;
    double head_y;
    double head_z;
    cv::Mat_<double> t;
    double x;
    double y;
    double z;
  } TagInfo;
  typedef std::map<size_t,TagInfo> TagInfoMap;

  static const cv::Point2d kOpticalCenter;
  void InitializeCamera();
  void InitializeMotionFramework();
  void InitializeMotionModules();
  void InitializeRobotPosition();
  void FindDetections(const Image* camera_image,
                      TagDetectionArray* detections);
  TagInfoMap ProcessDetections(const TagDetectionArray& detections,
                               size_t goal_tag_id, Image* display_image);
  bool MoveToGoal(const cv::Point2d& goal);
  void LookForGoal();
  CM730* cm730_;
  mjpg_streamer* streamer_;
  TagFamily tag_family_;
  TagDetector tag_detector_;
  BallTracker tracker_;
  Point2D current_goal_;
};

}  // namespace Robot

#endif  // EXPLORER_HPP
