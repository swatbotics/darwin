#ifndef LOCALIZER_HPP
#define LOCALIZER_HPP

#include <map>
#include <set>
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
    void Reset();
    void DetectAt(const TagDetection& d);
    int id;
    bool detected;
    double size;
    double perimeter;
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
  struct TaggedObject {
    TaggedObject();
    void Reset();
    std::string name;
    std::set<int> tag_ids;
    bool localized;
    int primary_tag_id;
    cv::Mat_<double> r;  // Rotation vector
    cv::Mat_<double> t;
  };
  typedef std::map<std::string, TaggedObject> TaggedObjectMap;

  static const char* kWindowName;

  void InitializeConfiguration();
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
  void LocalizeObjects();
  bool LocalizeObjectFromTags(TaggedObject& obj);
  void ShowVisualDisplay();
  void DrawFrameAxes(const cv::Mat_<double>& frame_r,
                     const cv::Mat_<double>& frame_t,
                     double size, const cv::Scalar& color);
  void DrawTagBox(const TagInfo& tag, const cv::Scalar& color);
  void DrawProjectedPoints(const cv::Mat_<cv::Point3d>& points,
                           const std::vector<std::pair<int, int> >& edges,
                           const cv::Mat_<double>& r_vec,
                           const cv::Mat_<double>& t_vec,
                           const cv::Scalar& color);
  void GenerateLocalizationData(DataCallbackFunc* data_callback);
  void Reset();

  libconfig::Config config_;
  cv::VideoCapture vc_;
  cv::Mat frame_;
  cv::Point2d optical_center_;
  TagFamily tag_family_;
  TagDetector detector_;
  TagDetectionArray detections_;
  bool found_references_;
  ReferenceSystem ref_system_;
  TagInfoMap ref_tags_;
  TagInfoMap obj_tags_;
  TaggedObjectMap tagged_objects_;
  cv::Mat_<double> global_translation_;
  cv::Mat_<double> global_rotation_;
  cv::Mat display_;
};

#endif  // LOCALIZER_HPP
