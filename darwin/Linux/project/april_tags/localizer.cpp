#include "localizer.hpp"

#include <cstdio>
#include <iostream>
#include <sstream>

#include <april/CameraUtil.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <gflags/gflags.h>

#include "util.hpp"

DEFINE_int32(device_num, 0,
             "Device number for video device to parse for tag information.");
DEFINE_double(focal_length, 500,
              "Focal length of the camera represented by the video device.");
DEFINE_string(tag_family, "Tag36h11",
              "Tag family to use for detections.");
DEFINE_double(obj_tag_size, 0.075, "Size of object tags.");
DEFINE_double(ref_tag_size, 0.075, "Size of reference tags.");
DEFINE_int32(frame_width, 640, "Desired video frame width.");
DEFINE_int32(frame_height, 480, "Desired video frame height.");
DEFINE_bool(rigid_transform, true,
            "Require rigid transform from camera to world frame.");
DEFINE_bool(show_localization_error, false,
            "Show amount of error in localization of reference tags.");
DEFINE_bool(show_display, false, "Show a visual display of detections.");

#define DEBUG false

const double Localizer::kObjectTagSize = FLAGS_obj_tag_size;
const double Localizer::kReferenceTagSize = FLAGS_ref_tag_size;

const Localizer::ReferenceTagCoords
Localizer::reference_tag_coords_[] = {
  {0, {0, 0, 0}},
  {1, {0, kReferenceTagInterval, 0}},
  {2, {kReferenceTagInterval, 0, 0}},
  {3, {kReferenceTagInterval, kReferenceTagInterval, 0}}
};

const Localizer::ReferenceTagSystem
Localizer::reference_tag_system_ = {0, 2, 1};

const char* Localizer::kWindowName = "Localization Server";

Localizer::TagInfo::TagInfo() :
    id(0),
    size(0),
    center(0,0),
    raw_r(cv::Mat_<double>::zeros(3, 3)),
    raw_t(cv::Mat_<double>::zeros(3, 1)),
    t(cv::Mat_<double>::zeros(3, 1)) {
}

Localizer::TagInfo::TagInfo(const TagDetection& d, double tag_size) {
  id = d.id;
  size = tag_size;
  center = d.cxy;
  const double f = FLAGS_focal_length;
  CameraUtil::homographyToPoseCV(f, f, tag_size, d.homography, raw_r, raw_t);
  t = cv::Mat_<double>::zeros(3, 1);
}

Localizer::Localizer() :
    vc_(),
    frame_(),
    optical_center_(),
    tag_family_(FLAGS_tag_family),
    detector_(tag_family_),
    detections_(),
    ref_tags_(),
    obj_tags_(),
    global_translation_(3, 1),
    global_rotation_(3, 3)
{
  //  detector_.params.segDecimate = true;
  //  detector_.params.thetaThresh = 25;
  //  detector_.params.refineCornersSubPix = true;
  InitializeVideoDevice();
}

void Localizer::InitializeVideoDevice() {
  vc_.open(FLAGS_device_num);
  if (!vc_.isOpened()) {
    std::cerr << "Could not open video device!\n";
    exit(1);
  }
  vc_.set(CV_CAP_PROP_FRAME_WIDTH, FLAGS_frame_width);
  vc_.set(CV_CAP_PROP_FRAME_HEIGHT, FLAGS_frame_height);
  std::cout << "Camera resolution: "
            << vc_.get(CV_CAP_PROP_FRAME_WIDTH) << "x"
            << vc_.get(CV_CAP_PROP_FRAME_HEIGHT) << "\n";
}

void Localizer::Run(DataCallbackFunc* data_callback) {
  double lasttime = get_time_as_double();
  while (true) {
    RunTagDetection();
    FindGlobalTransform();
    if (FLAGS_show_localization_error) {
      printf("Localization error: %f\n", GetLocalizationError());
    }
    LocalizeObjects();
    if (FLAGS_show_display) {
      ShowVisualDisplay();
    }
    GenerateLocalizationData(data_callback);
    double thistime = get_time_as_double();
    printf("FPS: %d\n", (int) (1 / (thistime - lasttime)));
    lasttime = thistime;
  }
}

void Localizer::RunTagDetection() {
  vc_ >> frame_;
  if (frame_.empty()) {
    std::cerr << "Got empty frame!\n";
    exit(1);
  }
  optical_center_ = cv::Point2d(frame_.cols * 0.5, frame_.rows * 0.5);
  detector_.process(frame_, optical_center_, detections_);
  const size_t reference_tags_num = (sizeof(reference_tag_coords_) /
                                     sizeof(reference_tag_coords_[0]));
  ref_tags_.clear();
  obj_tags_.clear();
  for (size_t i = 0; i < detections_.size(); ++i) {
    const TagDetection& d = detections_[i];
    bool is_ref_tag = false;
    size_t j = 0;
    for (j = 0; j < reference_tags_num; ++j) {
      if ((is_ref_tag = (reference_tag_coords_[j].id == d.id))) break;
    }
    if (is_ref_tag) {
      TagInfo tag(d, kReferenceTagSize);
      if (DEBUG) {
        std::cout << "ref tag " << tag.id << " raw_t = " << tag.raw_t << "\n";
      }
      for (int k = 0; k < tag.t.rows; ++k) {
        tag.t[k][0] = reference_tag_coords_[j].pos[k];
      }
      ref_tags_[tag.id] = tag;
    } else {
      TagInfo tag(d, kObjectTagSize);
      obj_tags_[tag.id] = tag;
    }
  }
}

void Localizer::FindGlobalTransform() {
  if (ref_tags_.size() < 3) {
    // TODO: Localize with only 1-2 tags, or remember tags to avoid flickers?
    std::cerr << "Cannot localize with less than 3 tags!\n";
    return;
  }
  if (ref_tags_.count(reference_tag_system_.origin) == 0 ||
      ref_tags_.count(reference_tag_system_.primary) == 0 ||
      ref_tags_.count(reference_tag_system_.secondary) == 0) {
    std::cerr << "Cannot localize with this tag system!\n";
    return;
  }
  TagInfo origin_ref = ref_tags_[reference_tag_system_.origin];
  TagInfo primary_ref = ref_tags_[reference_tag_system_.primary];
  TagInfo secondary_ref = ref_tags_[reference_tag_system_.secondary];
  cv::Mat_<double> primary_vec = primary_ref.raw_t - origin_ref.raw_t;
  cv::Mat_<double> secondary_vec = secondary_ref.raw_t - origin_ref.raw_t;
  cv::Mat_<double> primary_ref_vec = primary_ref.t - origin_ref.t;
  cv::Mat_<double> secondary_ref_vec = secondary_ref.t - origin_ref.t;

  if (DEBUG) {
    std::cout << "primary_vec = " << primary_vec << "("
              << cv::norm(primary_vec) << ")\n";
    std::cout << "secondary_vec = " << secondary_vec << "("
              << cv::norm(secondary_vec) << ")\n";
  }

  if (FLAGS_rigid_transform) {
    global_rotation_ = ComputeTransformRigid(primary_vec, secondary_vec);
  } else {
    global_rotation_ =
        ComputeTransformNonRigid(primary_vec, secondary_vec,
                                 primary_ref_vec, secondary_ref_vec);
  }
  global_translation_ = -global_rotation_ * origin_ref.raw_t;

  if (DEBUG) {
    std::cout << "global_rotation_ = \n" << global_rotation_ << "\n";
    std::cout << "global_translation_ = \n" << global_translation_ << "\n";
  }
}

cv::Mat_<double> Localizer::ComputeTransformRigid(
    const cv::Mat_<double>& primary_vec,
    const cv::Mat_<double>& secondary_vec) {
  const cv::Mat_<double> zero = cv::Mat_<double>::zeros(3, 1);
  cv::Mat_<double> basis(3, 3);
  basis.col(0) = primary_vec + zero;
  basis.col(2) = primary_vec.cross(secondary_vec) + zero;
  basis.col(1) = basis.col(2).cross(primary_vec) + zero;
  if (DEBUG) std::cout << "basis raw = \n" << basis << "\n";

  basis.col(0) /= cv::norm(basis.col(0));
  basis.col(1) /= cv::norm(basis.col(1));
  basis.col(2) /= cv::norm(basis.col(2));
  if (DEBUG) std::cout << "basis normed = \n" << basis << "\n";

  if (DEBUG) std::cout << "basis check = \n" << basis * basis.t() << "\n";

  return basis.t();
}

cv::Mat_<double> Localizer::ComputeTransformNonRigid(
    const cv::Mat_<double>& primary_vec,
    const cv::Mat_<double>& secondary_vec,
    const cv::Mat_<double>& primary_ref_vec,
    const cv::Mat_<double>& secondary_ref_vec) {
  const cv::Mat_<double> zero = cv::Mat_<double>::zeros(3, 1);
  cv::Mat_<double> ref_points(3, 3);
  ref_points.col(0) = primary_ref_vec + zero;
  ref_points.col(1) = secondary_ref_vec + zero;
  ref_points.col(2) = ref_points.col(0).cross(ref_points.col(1)) + zero;
  cv::Mat_<double> cam_points(3, 3);
  cam_points.col(0) = primary_vec + zero;
  cam_points.col(1) = secondary_vec + zero;
  cam_points.col(2) = cam_points.col(0).cross(cam_points.col(1)) + zero;
  if (DEBUG) {
    std::cout << "ref_points = \n" << ref_points << "\n";
    std::cout << "cam_points = \n" << cam_points << "\n";
  }
  return ref_points * cam_points.inv();
}

double Localizer::GetLocalizationError() {
  double sum_err = 0;
  int count = 0;
  for (TagInfoMap::const_iterator it = ref_tags_.begin();
       it != ref_tags_.end(); ++it, ++count) {
    const TagInfo& tag = it->second;
    cv::Mat_<double> localized_t = TransformToGlobal(tag.raw_t);
    cv::Mat_<double> errs = tag.t - localized_t;
    sum_err += cv::norm(errs);
    //    std::cout << "Ref tag " << tag.id << " = " << localized_t << "\n";
  }
  return sum_err / count;
}

cv::Mat_<double> Localizer::TransformToGlobal(
    const cv::Mat_<double>& vec) {
  return global_rotation_ * vec + global_translation_;
}

cv::Mat_<double> Localizer::TransformToCamera(
    const cv::Mat_<double>& vec) {
  return global_rotation_.inv() * (vec - global_translation_);
}

void Localizer::LocalizeObjects() {
  for (TagInfoMap::iterator it = obj_tags_.begin();
       it != obj_tags_.end(); ++it) {
    TagInfo& tag = it->second;
    if (DEBUG) std::cout << "raw_t = " << tag.raw_t << "\n";
    tag.t = TransformToGlobal(tag.raw_t);
    printf("Object (id #%zd) at (%.2f, %.2f, %.2f)\n",
           tag.id, tag.t[0][0], tag.t[1][0], tag.t[2][0]);
  }
}

void Localizer::ShowVisualDisplay() {
  const cv::Scalar& ref_tag_color = CV_RGB(0, 0, 0);
  const cv::Scalar& obj_tag_color = CV_RGB(0, 255, 0);
  for (TagInfoMap::iterator it = ref_tags_.begin();
       it != ref_tags_.end(); ++it) {
    TagInfo& tag = it->second;
    DrawTag(tag, ref_tag_color);
  }
  for (TagInfoMap::iterator it = obj_tags_.begin();
       it != obj_tags_.end(); ++it) {
    TagInfo& tag = it->second;
    DrawTag(tag, obj_tag_color);
  }
  cv::imshow(kWindowName, frame_);
  cv::waitKey(5);
}

void Localizer::DrawTag(const TagInfo& tag, const cv::Scalar& color) {
  static const int npoints = 8;
  static const int nedges = 12;
  const double s = tag.size;
  const double ss = 0.5*s;
  cv::Point3d src[npoints] = {
    cv::Point3d(-ss, -ss, 0),
    cv::Point3d( ss, -ss, 0),
    cv::Point3d( ss,  ss, 0),
    cv::Point3d(-ss,  ss, 0),
    cv::Point3d(-ss, -ss, s),
    cv::Point3d( ss, -ss, s),
    cv::Point3d( ss,  ss, s),
    cv::Point3d(-ss,  ss, s),
  };
  static const int edges[nedges][2] = {
    { 0, 1 }, { 1, 2 }, { 2, 3 }, { 3, 0 },
    { 4, 5 }, { 5, 6 }, { 6, 7 }, { 7, 4 },
    { 0, 4 }, { 1, 5 }, { 2, 6 }, { 3, 7 }
  };
  cv::Point2d dst[npoints];
  const double f = FLAGS_focal_length;
  double K[9] = {
    f, 0, optical_center_.x,
    0, f, optical_center_.y,
    0, 0, 1
  };
  const cv::Mat_<cv::Point3d> srcmat(npoints, 1, src);
  cv::Mat_<cv::Point2d> dstmat(npoints, 1, dst);
  const cv::Mat_<double> Kmat(3, 3, K);
  const cv::Mat_<double> distCoeffs = cv::Mat_<double>::zeros(4,1);
  cv::projectPoints(srcmat, tag.raw_r, tag.raw_t, Kmat, distCoeffs, dstmat);
  for (int j=0; j<nedges; ++j) {
    cv::line(frame_,
             dstmat(edges[j][0],0),
             dstmat(edges[j][1],0),
             color,
             1, CV_AA);
  }
}

void Localizer::GenerateLocalizationData(DataCallbackFunc* data_callback) {
  if (data_callback != NULL) {
    std::stringstream sstream;
    for (TagInfoMap::iterator it = obj_tags_.begin();
         it != obj_tags_.end(); ++it) {
      TagInfo& tag = it->second;
      sstream << tag.id << " @ "
              << tag.t[0][0] << " "
              << tag.t[1][0] << " "
              << tag.t[2][0] << "\n";
    }
    // Call the callback with the generated data.
    (*data_callback)(sstream.str());
  }
}
