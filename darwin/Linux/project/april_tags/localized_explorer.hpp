#ifndef LOCALIZED_EXPLORER_HPP
#define LOCALIZED_EXPLORER_HPP

#include <opencv2/core/core.hpp>

#include "CM730.h"

#include "pid_controller.hpp"
#include "status_client.hpp"

namespace Robot {

class LocalizedExplorer {
 public:
  LocalizedExplorer();
  ~LocalizedExplorer();
  void Initialize();
  void Process();

 private:
  struct LocalizedObject {
    LocalizedObject();
    LocalizedObject(const std::string& data);
    void Initialize();
    void ParseFromString(const std::string& data);
    std::string name;
    cv::Mat_<double> r;
    cv::Mat_<double> t;
  };
  typedef std::map<std::string, LocalizedObject> LocalizedObjectMap;

  void InitializeMotionFramework();
  void InitializeMotionModules();
  LocalizedObjectMap RetrieveObjectData();
  cv::Vec3d GetGoalDirection(const LocalizedObject& head,
                             const LocalizedObjectMap& obj_map);
  void PointHeadToward(const LocalizedObject& head_obj,
                       const cv::Vec3d& goal_dir);

  CM730* cm730_;
  StatusClient client_;
  PIDController pan_controller_;
  PIDController tilt_controller_;
};

}  // namespace Robot

#endif  // LOCALIZED_EXPLORER_HPP
