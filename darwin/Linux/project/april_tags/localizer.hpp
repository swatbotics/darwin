#ifndef LOCALIZER_HPP
#define LOCALIZER_HPP

#include <string>
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
    size_t id;
    double size;
    cv::Point2d center;
    cv::Mat_<double> raw_r;
    cv::Mat_<double> raw_t;
    cv::Mat_<double> t;
  };
  typedef std::map<size_t, TagInfo> TagInfoMap;
  struct ReferenceTagCoords {
    size_t id;
    double pos[3];
  };
  struct ReferenceTagSystem {
    size_t origin;
    size_t primary;
    size_t secondary;
  };
  static const double kObjectTagSize;
  static const double kReferenceTagSize;
  static const double kReferenceTagInterval = 0.605;
  static const ReferenceTagCoords reference_tag_coords_[];
  static const ReferenceTagSystem reference_tag_system_;
  static const char* kWindowName;

  void InitializeVideoDevice();
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
  void DrawTag(const TagInfo& tag, const cv::Scalar& color);
  void LocalizeObjects();
  void GenerateLocalizationData(DataCallbackFunc* data_callback);

  cv::VideoCapture vc_;
  cv::Mat frame_;
  cv::Point2d optical_center_;
  TagFamily tag_family_;
  TagDetector detector_;
  TagDetectionArray detections_;
  TagInfoMap ref_tags_;
  TagInfoMap obj_tags_;
  cv::Mat_<double> global_translation_;
  cv::Mat_<double> global_rotation_;
};

#endif  // LOCALIZER_HPP
