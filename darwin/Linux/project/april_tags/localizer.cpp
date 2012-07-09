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
DEFINE_bool(video_background, true,
            "Use video as background of visual display.");
DEFINE_bool(show_tag_labels, true,
            "Label tags with IDs in the visual display.");
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
  perimeter = 0.0;
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
  perimeter = d.observedPerimeter;
  center = d.cxy;
  const double f = FLAGS_focal_length;
  CameraUtil::homographyToPoseCV(f, f, size, d.homography, raw_r, raw_t);
}


Localizer::TaggedObject::TaggedObject() :
    name(""),
    tag_ids() {
  Reset();
}

void Localizer::TaggedObject::Reset() {
  localized = false;
  primary_tag_id = -1;
  r = cv::Mat_<double>::zeros(3, 1);
  t = cv::Mat_<double>::zeros(3, 1);
}

Localizer::Localizer() :
    config_(),
    vc_(),
    frame_(),
    optical_center_(),
    tag_family_(FLAGS_tag_family),
    detector_(tag_family_),
    detections_(),
    found_references_(false),
    ref_system_(),
    ref_tags_(),
    obj_tags_(),
    tagged_objects_(),
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

    const libconfig::Setting& obj_configs = config_.lookup("objects");
    for (int i = 0; i < obj_configs.getLength(); ++i) {
      const libconfig::Setting& obj_config = obj_configs[i];
      TaggedObject& obj = tagged_objects_[obj_config["name"]];
      // Cast needed to disambiguate between std::string constructors.
      obj.name = (const char *) obj_config["name"];
      for (int j = 0; j < obj_config.lookup("tags").getLength(); ++j) {
        const libconfig::Setting& tag_config = obj_config["tags"][j];
        TagInfo& tag = obj_tags_[tag_config["id"]];
        tag.id = tag_config["id"];
        tag.size = tag_config["size"];
        tag.ref_t[0][0] = tag_config["x"];
        tag.ref_t[1][0] = tag_config["y"];
        tag.ref_t[2][0] = tag_config["z"];
        tag.ref_r[0][0] = tag_config["r_x"];
        tag.ref_r[1][0] = tag_config["r_y"];
        tag.ref_r[2][0] = tag_config["r_z"];
        obj.tag_ids.insert(tag.id);
      }
    }
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
    if (found_references_) {
      if (FLAGS_show_localization_error) {
        printf("Localization error: %f\n", GetLocalizationError());
      }
      LocalizeObjects();
      GenerateLocalizationData(data_callback);
    }
    if (FLAGS_show_display) {
      ShowVisualDisplay();
    }
    double thistime = get_time_as_double();
    printf("FPS: %d\n", (int) (1 / (thistime - lasttime)));
    lasttime = thistime;
    Reset();
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
  found_references_ = true;
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
    if (!tag.detected) continue;
    cv::Mat_<double> raw_r_mat, r_mat;
    cv::Rodrigues(tag.raw_r, raw_r_mat);
    r_mat = global_rotation_ * raw_r_mat;
    cv::Rodrigues(r_mat, tag.r);
    tag.t = TransformToGlobal(tag.raw_t);
    if (DEBUG) std::cout << "raw_r = \n" << raw_r_mat << "\n";
    if (DEBUG) std::cout << "raw_t = " << tag.raw_t << "\n";
    printf("Tag (id #%d) at (%.2f, %.2f, %.2f), x -> (%.2f, %.2f, %.2f)\n",
           tag.id, tag.t[0][0], tag.t[1][0], tag.t[2][0],
           r_mat[0][0], r_mat[1][0], r_mat[2][0]);
  }
  for (TaggedObjectMap::iterator it = tagged_objects_.begin();
       it != tagged_objects_.end(); ++it) {
    TaggedObject& obj = it->second;
    if (LocalizeObjectFromTags(obj)) {
      cv::Mat_<double> r_mat;
      cv::Rodrigues(obj.r, r_mat);
      printf("Object (%s) at (%.2f, %.2f, %.2f), x -> (%.2f, %.2f, %.2f)\n",
             obj.name.c_str(), obj.t[0][0], obj.t[1][0], obj.t[2][0],
             r_mat[0][0], r_mat[1][0], r_mat[2][0]);
    }
  }
}

bool Localizer::LocalizeObjectFromTags(TaggedObject& obj) {
  // Use the object-identifying tag with the largest visual perimeter.
  int best_id = -1;
  double max_perim = 0.0;
  for (std::set<int>::const_iterator it = obj.tag_ids.begin();
       it != obj.tag_ids.end(); ++it) {
    int id = *it;
    if (obj_tags_.count(id) > 0 && obj_tags_[id].detected
        && obj_tags_[id].perimeter > max_perim) {
      best_id = id;
      max_perim = obj_tags_[id].perimeter;
    }
  }
  if (best_id == -1) return false;
  obj.primary_tag_id = best_id;
  // TODO: This stuff is a mess, calls cv::Rodrigues three times. Need to
  // either just store full rotation matrices or implement wrappers.
  const TagInfo& tag = obj_tags_[best_id];
  cv::Mat_<double> tag_ref_r_mat, tag_r_mat, obj_r_mat;
  cv::Rodrigues(tag.ref_r, tag_ref_r_mat);
  cv::Mat_<double> obj_r_in_tag_frame = tag_ref_r_mat.inv();
  cv::Mat_<double> obj_origin_in_tag_frame = tag_ref_r_mat.inv() * -tag.ref_t;
  cv::Rodrigues(tag.r, tag_r_mat);
  obj_r_mat = tag_r_mat * obj_r_in_tag_frame;
  cv::Rodrigues(obj_r_mat, obj.r);
  obj.t = tag_r_mat * obj_origin_in_tag_frame + tag.t;
  obj.localized = true;
  return true;
}

void Localizer::ShowVisualDisplay() {
  if (FLAGS_video_background) {
    display_ = frame_;
  } else {
    display_ = cv::Mat(frame_.size(), frame_.type(), cv::Scalar(0, 0, 0, 0));
  }
  const cv::Scalar& ref_tag_color = (FLAGS_video_background ?
                                     CV_RGB(0, 0, 0) : CV_RGB(255, 255, 255));
  const cv::Scalar& obj_tag_color = CV_RGB(0, 127, 0);
  for (TagInfoMap::const_iterator it = ref_tags_.begin();
       it != ref_tags_.end(); ++it) {
    const TagInfo& tag = it->second;
    if (tag.detected) {
      DrawTagBox(tag, ref_tag_color);
    }
  }
  for (TagInfoMap::const_iterator it = obj_tags_.begin();
       it != obj_tags_.end(); ++it) {
    const TagInfo& tag = it->second;
    if (tag.detected) {
      DrawTagBox(tag, obj_tag_color);
    }
  }
  if (found_references_) {
    const double kGlobalFrameAxesSize = cv::norm(ref_system_.primary->ref_t);
    cv::Mat_<double> global_r_vec_inv;
    cv::Rodrigues(global_rotation_.inv(), global_r_vec_inv);
    DrawFrameAxes(global_r_vec_inv,
                  TransformToCamera(ref_system_.origin->ref_t),
                  kGlobalFrameAxesSize, ref_tag_color);
  }
  static const double kObjFrameAxesSize = 0.1;
  for (TaggedObjectMap::const_iterator it = tagged_objects_.begin();
       it != tagged_objects_.end(); ++it) {
    const TaggedObject& obj = it->second;
    if (obj.localized) {
      // For now, just draw this box over the standard object tag box.
      // TODO: add a "color" field to TagInfo? Or "type"? To control display
      // so we don't have to draw two boxes.
      const cv::Scalar& primary_tag_color = CV_RGB(0, 255, 0);
      DrawTagBox(obj_tags_[obj.primary_tag_id], primary_tag_color);
      cv::Mat_<double> obj_r_mat, cam_r_mat, cam_r_vec;
      cv::Rodrigues(obj.r, obj_r_mat);
      cam_r_mat = global_rotation_.inv() * obj_r_mat;
      cv::Rodrigues(cam_r_mat, cam_r_vec);
      cv::Mat_<double> cam_t = TransformToCamera(obj.t);
      DrawFrameAxes(cam_r_vec, cam_t, kObjFrameAxesSize, CV_RGB(0, 255, 0));
    }
  }
  cv::imshow(kWindowName, display_);
  cv::waitKey(5);
}

void Localizer::DrawFrameAxes(const cv::Mat_<double>& frame_r,
                              const cv::Mat_<double>& frame_t,
                              double size, const cv::Scalar& color) {
  static const int npoints = 4;
  static const int nedges = 3;
  cv::Point3d points_raw[npoints] = {
    cv::Point3d(0, 0, 0),
    cv::Point3d(size, 0, 0),
    cv::Point3d(0, size, 0),
    cv::Point3d(0, 0, size),
  };
  static const int edges_raw[nedges][2] = {
    { 0, 1 }, { 0, 2 }, { 0, 3 },
  };
  std::vector<std::pair<int, int> > edges;
  for (int i = 0; i < nedges; ++i) {
    edges.push_back(std::make_pair(edges_raw[i][0], edges_raw[i][1]));
  }
  const cv::Mat_<cv::Point3d> points(npoints, 1, points_raw);
  DrawProjectedPoints(points, edges, frame_r, frame_t, color);
  // TODO: Might be nice to draw little arrows at the axis tips.
}

void Localizer::DrawTagBox(const TagInfo& tag, const cv::Scalar& color) {
  static const int npoints = 8;
  static const int nedges = 12;
  const double s = tag.size;
  const double ss = 0.5*s;
  // TODO: These raw arrays could probably be initialized directly as
  // cv::Mat_ instances using the stream constructor (<<) syntax.
  cv::Point3d points_raw[npoints] = {
    cv::Point3d(-ss, -ss, 0),
    cv::Point3d( ss, -ss, 0),
    cv::Point3d( ss,  ss, 0),
    cv::Point3d(-ss,  ss, 0),
    cv::Point3d(-ss, -ss, s),
    cv::Point3d( ss, -ss, s),
    cv::Point3d( ss,  ss, s),
    cv::Point3d(-ss,  ss, s),
  };
  static const int edges_raw[nedges][2] = {
    { 0, 1 }, { 1, 2 }, { 2, 3 }, { 3, 0 },
    { 4, 5 }, { 5, 6 }, { 6, 7 }, { 7, 4 },
    { 0, 4 }, { 1, 5 }, { 2, 6 }, { 3, 7 }
  };
  std::vector<std::pair<int, int> > edges;
  for (int i = 0; i < nedges; ++i) {
    edges.push_back(std::make_pair(edges_raw[i][0], edges_raw[i][1]));
  }
  const cv::Mat_<cv::Point3d> points(npoints, 1, points_raw);
  DrawProjectedPoints(points, edges, tag.raw_r, tag.raw_t, color);
  if (FLAGS_show_tag_labels) {
    std::stringstream sstream;
    sstream << "#" << tag.id;
    const cv::Point3d text_point(0, 0, s / 2);
    DrawProjectedText(sstream.str(), text_point, tag.raw_r, tag.raw_t, color);
  }
}

void Localizer::DrawProjectedPoints(
    const cv::Mat_<cv::Point3d>& points,
    const std::vector<std::pair<int, int> >& edges,
    const cv::Mat_<double>& r_vec,
    const cv::Mat_<double>& t_vec,
    const cv::Scalar& color) {
  const double f = FLAGS_focal_length;
  double K[9] = {
    f, 0, optical_center_.x,
    0, f, optical_center_.y,
    0, 0, 1
  };
  const cv::Mat_<double> Kmat(3, 3, K);
  cv::Mat_<cv::Point2d> proj_points(points.size());
  const cv::Mat_<double> distCoeffs = cv::Mat_<double>::zeros(4,1);
  cv::projectPoints(points, r_vec, t_vec, Kmat, distCoeffs, proj_points);
  for (size_t i = 0; i < edges.size(); ++i) {
    cv::line(display_,
             proj_points(edges[i].first, 0),
             proj_points(edges[i].second, 0),
             color, 1, CV_AA);
  }
}

void Localizer::DrawProjectedText(const std::string& text,
                                  const cv::Point3d& point,
                                  const cv::Mat_<double>& r_vec,
                                  const cv::Mat_<double>& t_vec,
                                  const cv::Scalar& color) {
  cv::Mat_<cv::Point3d> points(1, 1, point);
  // TODO: This definition of the K matrix repeats code in DrawProjectedPoints,
  // above - should refactor projection part out.
  const double f = FLAGS_focal_length;
  double K[9] = {
    f, 0, optical_center_.x,
    0, f, optical_center_.y,
    0, 0, 1
  };
  const cv::Mat_<double> Kmat(3, 3, K);
  cv::Mat_<cv::Point2d> proj_points(points.size());
  const cv::Mat_<double> distCoeffs = cv::Mat_<double>::zeros(4,1);
  cv::projectPoints(points, r_vec, t_vec, Kmat, distCoeffs, proj_points);
  const int font_face = cv::FONT_HERSHEY_DUPLEX;
  const double font_scale = 0.75;
  const int thickness = 1;
  cv::Size text_size = cv::getTextSize(text, font_face, font_scale,
                                       thickness, NULL);
  cv::Point2d text_center(-text_size.width * 0.5, text_size.height * 0.5);
  cv::Point2d centered_point = proj_points(0, 0) + text_center;
  cv::putText(display_, text, centered_point, font_face,
              font_scale, color, thickness, CV_AA);
}

void Localizer::GenerateLocalizationData(DataCallbackFunc* data_callback) {
  if (data_callback != NULL) {
    std::stringstream sstream;
    // TODO: At some point, stop returning data for bare tags?
    for (TagInfoMap::const_iterator it = obj_tags_.begin();
         it != obj_tags_.end(); ++it) {
      const TagInfo& tag = it->second;
      if (!tag.detected) continue;
      // TODO: This should probably be tag.ToString() or something.
      sstream << tag.id << " @ "
              << tag.t[0][0] << " "
              << tag.t[1][0] << " "
              << tag.t[2][0] << " * "
              << tag.r[0][0] << " "
              << tag.r[1][0] << " "
              << tag.r[2][0] << "\n";
    }
    for (TaggedObjectMap::const_iterator it = tagged_objects_.begin();
         it != tagged_objects_.end(); ++it) {
      const TaggedObject& obj = it->second;
      if (!obj.localized) continue;
      // TODO: This should probably be obj.ToString() or something.
      sstream << obj.name << " @ "
              << obj.t[0][0] << " "
              << obj.t[1][0] << " "
              << obj.t[2][0] << " * "
              << obj.r[0][0] << " "
              << obj.r[1][0] << " "
              << obj.r[2][0] << "\n";
    }
    // Call the callback with the generated data.
    (*data_callback)(sstream.str());
  }
}

void Localizer::Reset() {
  for (TagInfoMap::iterator it = ref_tags_.begin();
       it != ref_tags_.end(); ++it) {
    it->second.Reset();
  }
  for (TagInfoMap::iterator it = obj_tags_.begin();
       it != obj_tags_.end(); ++it) {
    it->second.Reset();
  }
  for (TaggedObjectMap::iterator it = tagged_objects_.begin();
       it != tagged_objects_.end(); ++it) {
    it->second.Reset();
  }
}
