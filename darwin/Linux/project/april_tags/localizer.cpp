#include "localizer.hpp"

#include <cstdio>
#include <iostream>
#include <sstream>

#include <april/CameraUtil.h>
#include <libconfig.h++>
#include <opencv2/calib3d/calib3d.hpp>
#include <gflags/gflags.h>

#include "util.hpp"

DEFINE_int32(device_num, 0,
             "Device number for video device to parse for tag information.");
DEFINE_double(focal_length, 500,
              "Focal length of the camera represented by the video device.");
DEFINE_string(tag_family, "Tag36h11",
              "Tag family to use for detections.");
DEFINE_double(default_tag_size, 0.075, "Size of unassociated tags.");
DEFINE_int32(frame_width, 640, "Desired video frame width.");
DEFINE_int32(frame_height, 480, "Desired video frame height.");
DEFINE_bool(rigid_transform, true,
            "Require rigid transform from camera to world frame.");
DEFINE_bool(show_localization_error, false,
            "Show amount of error in localization of reference tags.");
DEFINE_bool(show_display, false, "Show a visual display of detections.");
DEFINE_string(config_file, "./localizer_env.cfg",
              "Configuration file for localization environment info.");

#define DEBUG false

const char* Localizer::kWindowName = "Localization Server";

Localizer::TagInfo::TagInfo() :
    id(0),
    detected(false),
    size(FLAGS_default_tag_size),
    ref_r(cv::Mat_<double>::zeros(3, 1)),
    ref_t(cv::Mat_<double>::zeros(3, 1)) {
  Reset();
}

void Localizer::TagInfo::Reset() {
  detected = false;
  center = cv::Point2d(0, 0);
  raw_r = cv::Mat_<double>::zeros(3, 1);
  raw_t = cv::Mat_<double>::zeros(3, 1);
  r = cv::Mat_<double>::zeros(3, 1);
  t = cv::Mat_<double>::zeros(3, 1);
}

void Localizer::TagInfo::DetectAt(const TagDetection& d) {
  Reset();
  id = d.id;
  detected = true;
  center = d.cxy;
  const double f = FLAGS_focal_length;
  CameraUtil::homographyToPoseCV(f, f, size, d.homography, raw_r, raw_t);
}


Localizer::Localizer() :
    config_(),
    vc_(),
    frame_(),
    optical_center_(),
    tag_family_(FLAGS_tag_family),
    detector_(tag_family_),
    detections_(),
    ref_system_(),
    ref_tags_(),
    obj_tags_(),
    global_translation_(3, 1),
    global_rotation_(3, 3)
{
  //  detector_.params.segDecimate = true;
  //  detector_.params.thetaThresh = 25;
  //  detector_.params.refineCornersSubPix = true;
  InitializeConfiguration();
  InitializeVideoDevice();
}

void Localizer::InitializeConfiguration() {
  try {
    config_.readFile(FLAGS_config_file);
  } catch (libconfig::ConfigException& e) {
    std::cerr << e.what() << ": Error reading configuration from file '"
              << FLAGS_config_file << "'.\n";
    exit(1);
  }
  try {
    const libconfig::Setting& ref_config = config_.lookup("references");
    const libconfig::Setting& ref_tag_configs = ref_config.lookup("tags");
    for (int i = 0; i < ref_tag_configs.getLength(); ++i) {
      const libconfig::Setting& tag_config = ref_tag_configs[i];
      TagInfo& tag = ref_tags_[tag_config["id"]];
      tag.id = tag_config["id"];
      tag.size = tag_config["size"];
      tag.ref_t[0][0] = tag_config["x"];
      tag.ref_t[1][0] = tag_config["y"];
      tag.ref_t[2][0] = tag_config["z"];
    }
    ref_system_.origin = &ref_tags_[ref_config["origin"]];
    ref_system_.primary = &ref_tags_[ref_config["primary"]];
    ref_system_.secondary = &ref_tags_[ref_config["secondary"]];
  } catch (libconfig::SettingException& e) {
    std::cerr << e.what() << ": Error with setting '" << e.getPath() << "'.\n";
    exit(1);
  }
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

void Localizer::ResetTagDetections() {
  for (TagInfoMap::iterator it = ref_tags_.begin();
       it != ref_tags_.end(); ++it) {
      it->second.Reset();
  }
  for (TagInfoMap::iterator it = obj_tags_.begin();
       it != obj_tags_.end(); ++it) {
      it->second.Reset();
  }
}

void Localizer::RunTagDetection() {
  ResetTagDetections();
  vc_ >> frame_;
  if (frame_.empty()) {
    std::cerr << "Got empty frame!\n";
    exit(1);
  }
  optical_center_ = cv::Point2d(frame_.cols * 0.5, frame_.rows * 0.5);
  detector_.process(frame_, optical_center_, detections_);
  for (size_t i = 0; i < detections_.size(); ++i) {
    const TagDetection& d = detections_[i];
    if (ref_tags_.count(d.id) > 0) {
      ref_tags_[d.id].DetectAt(d);
    } else {
      obj_tags_[d.id].DetectAt(d);
    }
  }
}

void Localizer::FindGlobalTransform() {
  if (ref_system_.origin == NULL || !ref_system_.origin->detected ||
      ref_system_.primary == NULL || !ref_system_.primary->detected ||
      ref_system_.secondary == NULL || !ref_system_.secondary->detected) {
    std::cerr << "Cannot find full reference system for localization!\n";
    return;
  }
  const TagInfo& origin_ref = *ref_system_.origin;
  const TagInfo& primary_ref = *ref_system_.primary;
  const TagInfo& secondary_ref = *ref_system_.secondary;
  cv::Mat_<double> primary_vec = primary_ref.raw_t - origin_ref.raw_t;
  cv::Mat_<double> secondary_vec = secondary_ref.raw_t - origin_ref.raw_t;
  cv::Mat_<double> primary_ref_vec = primary_ref.ref_t - origin_ref.ref_t;
  cv::Mat_<double> secondary_ref_vec = secondary_ref.ref_t - origin_ref.ref_t;

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
    cv::Mat_<double> errs = tag.ref_t - localized_t;
    sum_err += cv::norm(errs);
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
    cv::Mat_<double> raw_r_mat, r_mat;
    cv::Rodrigues(tag.raw_r, raw_r_mat);
    r_mat = global_rotation_ * raw_r_mat;
    cv::Rodrigues(r_mat, tag.r);
    tag.t = TransformToGlobal(tag.raw_t);
    if (DEBUG) std::cout << "raw_r = \n" << raw_r_mat << "\n";
    if (DEBUG) std::cout << "raw_t = " << tag.raw_t << "\n";
    printf("Object (id #%d) at (%.2f, %.2f, %.2f), x -> (%.2f, %.2f, %.2f)\n",
           tag.id, tag.t[0][0], tag.t[1][0], tag.t[2][0],
           r_mat[0][0], r_mat[1][0], r_mat[2][0]);
  }
}

void Localizer::ShowVisualDisplay() {
  const cv::Scalar& ref_tag_color = CV_RGB(0, 0, 0);
  const cv::Scalar& obj_tag_color = CV_RGB(0, 255, 0);
  for (TagInfoMap::iterator it = ref_tags_.begin();
       it != ref_tags_.end(); ++it) {
    TagInfo& tag = it->second;
    if (tag.detected) {
      DrawTag(tag, ref_tag_color);
    }
  }
  for (TagInfoMap::iterator it = obj_tags_.begin();
       it != obj_tags_.end(); ++it) {
    TagInfo& tag = it->second;
    if (tag.detected) {
      DrawTag(tag, obj_tag_color);
    }
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
              << tag.t[2][0] << " * "
              << tag.r[0][0] << " "
              << tag.r[1][0] << " "
              << tag.r[2][0] << "\n";
    }
    // Call the callback with the generated data.
    (*data_callback)(sstream.str());
  }
}
