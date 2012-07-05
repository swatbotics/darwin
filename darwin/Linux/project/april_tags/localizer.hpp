#ifndef LOCALIZER_HPP
#define LOCALIZER_HPP

#include <string>

#include <libconfig.h++>
#include <opencv2/highgui/highgui.hpp>

#include "april/TagDetector.h"

class Localizer {
 public:
  struct DataCallbackFunc :
      public std::unary_function<const std::string&, void> {
    virtual void operator() (const std::string& data) {}
  };
  Localizer();
  ~Localizer() {}
  void Run(DataCallbackFunc* data_callback);

 private:
  struct TagInfo {
    TagInfo();
    TagInfo(const TagDetection& d, double tag_size);
    void Reset();
    void DetectAt(const TagDetection& d);
    int id;
    bool detected;
    double size;
    cv::Point2d center;
    cv::Mat_<double> ref_r;  // Rotation vector
    cv::Mat_<double> ref_t;
    cv::Mat_<double> raw_r;  // Rotation vector
    cv::Mat_<double> raw_t;
    cv::Mat_<double> r;  // Rotation vector
    cv::Mat_<double> t;
  };
  typedef std::map<int, TagInfo> TagInfoMap;
  struct ReferenceSystem {
    TagInfo* origin;
    TagInfo* primary;
    TagInfo* secondary;
  };

  static const double kObjectTagSize;
  static const char* kWindowName;

  void InitializeConfiguration();
  void InitializeVideoDevice();
  void ResetTagDetections();
  void RunTagDetection();
  void FindGlobalTransform();
  cv::Mat_<double> ComputeTransformRigid(const cv::Mat_<double>& primary_vec,
      const cv::Mat_<double>& secondary_vec);
  cv::Mat_<double> ComputeTransformNonRigid(
      const cv::Mat_<double>& primary_vec,
      const cv::Mat_<double>& secondary_vec,
      const cv::Mat_<double>& primary_ref_vec,
      const cv::Mat_<double>& secondary_ref_vec);
  double GetLocalizationError();
  cv::Mat_<double> TransformToGlobal(const cv::Mat_<double>& vec);
  cv::Mat_<double> TransformToCamera(const cv::Mat_<double>& vec);
  void LocalizeObjects();
  void ShowVisualDisplay();
  void DrawTag(const TagInfo& tag, const cv::Scalar& color);
  void GenerateLocalizationData(DataCallbackFunc* data_callback);

  libconfig::Config config_;
  cv::VideoCapture vc_;
  cv::Mat frame_;
  cv::Point2d optical_center_;
  TagFamily tag_family_;
  TagDetector detector_;
  TagDetectionArray detections_;
  ReferenceSystem ref_system_;
  TagInfoMap ref_tags_;
  TagInfoMap obj_tags_;
  cv::Mat_<double> global_translation_;
  cv::Mat_<double> global_rotation_;
};

#endif  // LOCALIZER_HPP
