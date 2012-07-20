#ifndef LOCALIZED_EXPLORER_HPP
#define LOCALIZED_EXPLORER_HPP

#include <deque>

#include <opencv2/core/core.hpp>

#include "CM730.h"

#include "pid_controller.hpp"
#include "status_client.hpp"
#include "timestamp.hpp"

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

  struct HeadPos {
    static HeadPos Current();
    HeadPos();
    double pan;
    double tilt;
  };
  struct HeadData {
    static HeadData Current();
    HeadData();
    Timestamp ts;
    HeadPos pos;
  };

  void InitializeMotionFramework();
  void InitializeMotionModules();
  void MeasureSystemLatency();
  LocalizedObjectMap RetrieveObjectData(Timestamp* ts=NULL);
  cv::Vec3d GetGoalDirection(const LocalizedObject& head,
                             const LocalizedObjectMap& obj_map);
  cv::Vec3d ConvertGoalDirection(const LocalizedObject& head_obj,
                                 const cv::Vec3d& goal_dir);
  void PointHeadToward(const LocalizedObject& head_obj,
                       const cv::Vec3d& goal_dir);
  HeadData GetCachedHeadData();
  void SaveHeadData();

  CM730* cm730_;
  StatusClient client_;
  PIDController pan_controller_;
  PIDController tilt_controller_;
  std::deque<HeadData> head_data_cache_;
};

}  // namespace Robot

#endif  // LOCALIZED_EXPLORER_HPP
