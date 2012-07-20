#include "localized_explorer.hpp"

#include <cmath>
#include <iostream>
#include <string>

#include <gflags/gflags.h>
#include <opencv2/calib3d/calib3d.hpp>

#include <LinuxDARwIn.h>

#include "util.hpp"

#define INI_FILE_PATH "config.ini"
#define U2D_DEV_NAME "/dev/ttyUSB0"

DEFINE_bool(quiet, false, "Set to reduce amount of output shown.");
DEFINE_double(fps_target, 50, "Target frames per second to run at.");

DEFINE_string(goal_object, "",
              "Object whose frame goal coordinates are specified in; the "
              "empty string represents the world frame.");
DEFINE_bool(goal_axis_only, false,
            "Treat goal coordinates as defining an axis to point head along, "
            "rather than a specific point in space to look toward.");
DEFINE_double(goal_x, 0.0, "Goal x-coordinate.");
DEFINE_double(goal_y, 0.0, "Goal y-coordinate.");
DEFINE_double(goal_z, 0.0, "Goal z-coordinate.");
DEFINE_double(pan_pgain, 0.1, "Pan controller proportional gain.");
DEFINE_double(pan_igain, 0.0, "Pan controller integral gain.");
DEFINE_double(pan_dgain, 0.0, "Pan controller derivative gain.");
DEFINE_double(tilt_pgain, 0.1, "Tilt controller proportional gain.");
DEFINE_double(tilt_igain, 0.0, "Tilt controller integral gain.");
DEFINE_double(tilt_dgain, 0.0, "Tilt controller derivative gain.");

DEFINE_double(servo_pan_pgain, 16, "Pan servo proportional gain.");
DEFINE_double(servo_tilt_pgain, 16, "Tilt servo proportional gain.");

DEFINE_bool(latency_test, true,
            "Run a test to determine overall system latency.");
DEFINE_bool(latency_test_square_wave, false,
            "Use a square wave instead of a sinusoid for latency testing.");
DEFINE_double(latency_test_length, 10.0,
              "Length of latency test in seconds.");
DEFINE_double(latency_test_amplitude, 30.0,
              "Amplitude of latency test sinusoid in degrees.");
DEFINE_double(latency_test_offset, 30.0,
              "Offset of latency test sinusoid in degrees (positive is "
              "turning the head to the left).");
DEFINE_double(latency_test_period, 2.0,
              "Period of latency test sinusoid in seconds.");

DEFINE_int32(head_data_cache_max_size, 1000,
             "Maximum size of cache for head data values.");

namespace Robot {

const double LocalizedExplorer::kTiltOffset = 40.0;

LocalizedExplorer::LocalizedObject::LocalizedObject() {
  Initialize();
}

LocalizedExplorer::LocalizedObject::LocalizedObject(const std::string& data) {
  Initialize();
  ParseFromString(data);
}

void LocalizedExplorer::LocalizedObject::Initialize() {
  r = cv::Mat_<double>::zeros(3, 1);
  t = cv::Mat_<double>::zeros(3, 1);
}

void LocalizedExplorer::LocalizedObject::ParseFromString(
    const std::string& data) {
  std::stringstream ss(data);
  std::string sep1, sep2;
  ss >> name
     >> sep1 >> t[0][0] >> t[1][0] >> t[2][0]
     >> sep2 >> r[0][0] >> r[1][0] >> r[2][0];
}


LocalizedExplorer::HeadPos::HeadPos() :
    pan(0),
    tilt(0) {
}

LocalizedExplorer::HeadPos LocalizedExplorer::HeadPos::Current() {
  Head* head = Head::GetInstance();
  HeadPos pos;
  pos.pan = head->m_Joint.GetAngle(JointData::ID_HEAD_PAN);
  pos.tilt = head->m_Joint.GetAngle(JointData::ID_HEAD_TILT) - kTiltOffset;
  return pos;
}


LocalizedExplorer::HeadData::HeadData() :
    ts(),
    pos() {
}

LocalizedExplorer::HeadData LocalizedExplorer::HeadData::Current() {
  HeadData data;
  data.ts = Timestamp::Now();
  data.pos = HeadPos::Current();
  return data;
}


LocalizedExplorer::LocalizedExplorer() :
    cm730_(NULL),
    client_(),
    pan_controller_(FLAGS_pan_pgain, FLAGS_pan_igain, FLAGS_pan_dgain),
    tilt_controller_(FLAGS_tilt_pgain, FLAGS_tilt_igain, FLAGS_tilt_dgain),
    head_data_cache_() {
}

LocalizedExplorer::~LocalizedExplorer() {}

void LocalizedExplorer::Initialize() {
  printf("\n===== INIT EXPLORER =====\n\n");
  // To make relative filenames work.
  change_dir_from_root("darwin/Linux/project/april_tags");
  InitializeMotionFramework();
  prompt("Initialize robot position?");
  InitializeMotionModules();

  client_.Run();
  if (FLAGS_latency_test) {
    prompt("Start latency detection?");
    MeasureSystemLatency();
  }
  prompt("Start tracking mode?");
}

void LocalizedExplorer::InitializeMotionFramework() {
  std::cout << "Initializing motion framework...\n";
  LinuxCM730* linux_cm730 = new LinuxCM730(U2D_DEV_NAME);
  cm730_ = new CM730(linux_cm730);
  MotionManager* manager = MotionManager::GetInstance();
  if (manager->Initialize(cm730_) == false) {
    printf("Failed to initialize MotionManager!\n");
    exit(1);
  }
  LinuxMotionTimer* motion_timer = new LinuxMotionTimer(manager);
  motion_timer->Start();
}

void LocalizedExplorer::InitializeMotionModules() {
  std::cout << "Initializing motion modules...\n";
  MotionManager* manager = MotionManager::GetInstance();
  Head* head = Head::GetInstance();
  manager->AddModule(head);
  head->m_Joint.SetEnableHeadOnly(true, true);
  head->m_Joint.SetPGain(JointData::ID_HEAD_PAN, FLAGS_servo_pan_pgain);
  head->m_Joint.SetPGain(JointData::ID_HEAD_TILT, FLAGS_servo_tilt_pgain);
  MotionStatus::m_CurrentJoints.SetEnableBodyWithoutHead(false);
  manager->SetEnable(true);
}

bool LocalizedExplorer::RetrieveObjectData() {
  // Retrieve data from localization client, extract head rotation info.
  std::string data = client_.GetData(&data_ts_);
  if (!FLAGS_quiet) {
    std::cout << "DATA:\n" << data << "\n";
  }
  obj_map_.clear();
  std::vector<std::string> lines = split(data, '\n');
  for (std::vector<std::string>::const_iterator it = lines.begin();
       it != lines.end(); ++it) {
    LocalizedObject obj(*it);
    obj_map_[obj.name] = obj;
  }
  if (obj_map_.count("head") == 0 || obj_map_.count("body") == 0) {
    // TODO: Should be able to get by with just one of these two, ideally.
    // See TODOs for GetGoalAngles() and GetHeadEstimateAngles().
    return false;
  }
  head_obj_ = obj_map_["head"];
  body_obj_ = obj_map_["body"];
  return true;
}

void LocalizedExplorer::MeasureSystemLatency() {
  double kLatencyTestTilt = 40;
  Head::GetInstance()->MoveByAngle(FLAGS_latency_test_offset,
                                   kLatencyTestTilt);
  prompt("Run latency detection?");

  double start_time = get_time_as_double();
  double elapsed_time = 0.0;
  while ((elapsed_time = (get_time_as_double() - start_time))
         < FLAGS_latency_test_length) {
    double A = FLAGS_latency_test_amplitude;
    double B = FLAGS_latency_test_offset;
    double T = FLAGS_latency_test_period;
    double t = elapsed_time;
    double angle = 0;
    if (FLAGS_latency_test_square_wave) {
      angle = A * (sin(t * (2 * M_PI) / T) >= 0 ? 1 : -1) + B;
    } else {
      angle = A * sin(t * (2 * M_PI) / T) + B;
    }
    Head* head = Head::GetInstance();
    head->MoveByAngle(angle, kLatencyTestTilt);
    double servo_angle = head->m_Joint.GetAngle(JointData::ID_HEAD_PAN);

    if (!RetrieveObjectData()) {
      std::cerr << "Cannot see head and body for latency measurements!\n";
      continue;
    }
    cv::Mat head_r_mat;
    cv::Mat body_r_mat;
    cv::Rodrigues(head_obj_.r, head_r_mat);
    cv::Rodrigues(body_obj_.r, body_r_mat);
    cv::Mat head_x = head_r_mat.col(0);
    cv::Mat body_x = body_r_mat.col(0);
    double seen_angle = acos(head_x.dot(body_x)) * 180 / M_PI;
    printf("Time: %7.3f  Output: %5.1f  Servo: %5.1f  Seen: %5.1f\n",
           elapsed_time, angle, servo_angle, seen_angle);
  }
}

void LocalizedExplorer::Process() {
  usleep(1000 * 1000 / FLAGS_fps_target);
  std::cout << "\n";
  if (RetrieveObjectData()) {
    HeadPos goal_angles = GetGoalAngles();
    HeadPos estimate_angles = GetHeadEstimateAngles();
    DriveHeadToward(goal_angles, estimate_angles);
  }
  SaveHeadData();
}

LocalizedExplorer::HeadPos LocalizedExplorer::GetGoalAngles() {
  // TODO: If we can't see the body, fall back to using head-relative
  // pan and tilt values, and then use cached head angles to find the
  // body-relative pan and tilt values.
  cv::Vec3d goal_dir = GetGoalDirection();
  HeadPos goal_angles = GetPanTiltForVector(goal_dir);
  //  std::cout << "Goal angles: " << goal_angles.pan
  //            << " " << goal_angles.tilt << "\n";
  return goal_angles;
}

LocalizedExplorer::HeadPos LocalizedExplorer::GetHeadEstimateAngles() {
  // TODO: If we can't see the body or the head, then fall back to just
  // using the current pan and tilt values from the head, instead of this
  // fancier estimate that tries to adapt for lag in the servo response.
  cv::Vec3d seen_head_dir = GetSeenHeadDirection();
  HeadPos seen_head_angles = GetPanTiltForVector(seen_head_dir);
  HeadPos cached_head_angles = GetCachedHeadData().pos;
  HeadPos current_head_angles = HeadPos::Current();
  // std::cout << "Seen head angles: " << seen_head_angles.pan
  //           << " " << seen_head_angles.tilt << "\n";
  // std::cout << "Cached head angles: " << cached_head_angles.pan
  //           << " " << cached_head_angles.tilt << "\n";
  // std::cout << "Current head angles: " << current_head_angles.pan
  //           << " " << current_head_angles.tilt << "\n";
  HeadPos estimate;
  estimate.pan = seen_head_angles.pan + (current_head_angles.pan -
                                         cached_head_angles.pan);
  estimate.tilt = seen_head_angles.tilt + (current_head_angles.tilt -
                                           cached_head_angles.tilt);
  return estimate;
}

LocalizedExplorer::HeadPos LocalizedExplorer::GetPanTiltForVector(
    const cv::Vec3d& vec) {
  HeadPos result;
  cv::Mat body_r_mat;
  cv::Rodrigues(body_obj_.r, body_r_mat);
  cv::Mat point_body = body_r_mat.inv() * (cv::Mat) vec;
  cv::Point3d new_point = (cv::Vec3d) point_body;
  double hypot = cv::norm(new_point);
  double hypot_xy = cv::norm(cv::Point2d(new_point.x, new_point.y));
  result.pan = asin(new_point.y / hypot_xy) * 180.0 / M_PI;
  result.tilt = asin(new_point.z / hypot) * 180.0 / M_PI;
  //  std::cout << "pan_rel = " << pan_rel << "\n";
  //  std::cout << "tilt_rel = " << tilt_rel << "\n";
  return result;
}

cv::Vec3d LocalizedExplorer::GetGoalDirection() {
  // Determine whether goal frame is an object or the world frame.
  static cv::Mat_<double> r_obj_to_world = cv::Mat_<double>::eye(3, 3);
  static cv::Mat_<double> t_obj_to_world = cv::Mat_<double>::zeros(3, 1);
  if (!FLAGS_goal_object.empty() && obj_map_.count(FLAGS_goal_object) > 0) {
    LocalizedObjectMap::const_iterator it = obj_map_.find(FLAGS_goal_object);
    if (it == obj_map_.end()) {
      std::cerr << "Object not found??\n";
    } else {
      const LocalizedObject& obj = it->second;
      cv::Rodrigues(obj.r, r_obj_to_world);
      t_obj_to_world = obj.t.clone();
    }
  }

  // Map goal point from object frame to world frame.
  cv::Vec3d origin = cv::Mat(t_obj_to_world);
  cv::Vec3d goal_point_raw(FLAGS_goal_x, FLAGS_goal_y, FLAGS_goal_z);
  cv::Mat goal_point_mat = (r_obj_to_world * cv::Mat(goal_point_raw) +
                            t_obj_to_world);
  cv::Vec3d goal_point = goal_point_mat;

  // Determine whether head should align with goal as a point or an axis.
  cv::Vec3d head_point(cv::Mat(head_obj_.t));
  cv::Vec3d goal_dir;
  if (FLAGS_goal_axis_only) {
    goal_dir = goal_point - origin;
  } else {
    goal_dir = goal_point - head_point;
  }
  return goal_dir;
}

cv::Vec3d LocalizedExplorer::GetSeenHeadDirection() {
  cv::Mat head_r_mat;
  cv::Rodrigues(head_obj_.r, head_r_mat);
  return head_r_mat.col(0);
}

void LocalizedExplorer::DriveHeadToward(HeadPos goal, HeadPos estimate) {
  HeadPos err;
  err.pan = goal.pan - estimate.pan;
  err.tilt = goal.tilt - estimate.tilt;
  Head* head = Head::GetInstance();
  double pan_output = pan_controller_.Update(err.pan);
  double tilt_output = tilt_controller_.Update(err.tilt);
  head->MoveByAngle(head->GetPanAngle() + pan_output,
                    head->GetTiltAngle() + tilt_output);
  std::cout << "Pan PID: " << pan_controller_.GetProportionalError()
            << " " << pan_controller_.GetIntegralError()
            << " " << pan_controller_.GetDerivativeError()
            << " ==> " << pan_output << "\n";
  std::cout << "Tilt PID: " << tilt_controller_.GetProportionalError()
            << " " << tilt_controller_.GetIntegralError()
            << " " << tilt_controller_.GetDerivativeError()
            << " ==> " << tilt_output << "\n";
}

LocalizedExplorer::HeadData LocalizedExplorer::GetCachedHeadData() {
  HeadData best;
  if (head_data_cache_.size() == 0) return best;
  best = head_data_cache_.front();
  double best_diff = std::abs((best.ts - data_ts_).ToDouble());
  while (head_data_cache_.size() > 0) {
    HeadData current = head_data_cache_.front();
    double cur_diff = std::abs((current.ts - data_ts_).ToDouble());
    if (cur_diff <= best_diff) {
      best = current;
      best_diff = cur_diff;
      head_data_cache_.pop_front();
    } else {
      break;
    }
  }
  //  std::cout << "Cache ts: " << best.ts << "\n";
  std::cout << "Cache diff: " << best.ts - data_ts_ << "\n";
  return best;
}

void LocalizedExplorer::SaveHeadData() {
  HeadData data = HeadData::Current();
  head_data_cache_.push_back(data);
  if (head_data_cache_.size() > (size_t) FLAGS_head_data_cache_max_size) {
    head_data_cache_.pop_front();
  }
  std::cout << "Cache size: " << head_data_cache_.size() << "\n";
}

}  // namespace Robot


int main(int argc, char* argv[]) {
  std::string usage;
  usage += std::string("Usage: ") + argv[0] + std::string(" [OPTIONS]");
  gflags::SetUsageMessage(usage);
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  printf( "\n===== April Tag Test for DARwIn =====\n\n");
  Robot::LocalizedExplorer explorer;
  explorer.Initialize();
  record_elapsed_time();
  while (true) {
    explorer.Process();
    double frame_time = record_elapsed_time();
    printf("FPS: % 2d\n",  (int) (1 / frame_time));
  }
  return 0;
}
