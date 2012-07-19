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
DEFINE_double(pan_pgain, 0.05, "Pan controller proportional gain.");
DEFINE_double(pan_igain, 0.0, "Pan controller integral gain.");
DEFINE_double(pan_dgain, 0.0, "Pan controller derivative gain.");
DEFINE_double(tilt_pgain, 0.05, "Tilt controller proportional gain.");
DEFINE_double(tilt_igain, 0.0, "Tilt controller integral gain.");
DEFINE_double(tilt_dgain, 0.0, "Tilt controller derivative gain.");

DEFINE_double(servo_pan_pgain, 8, "Pan servo proportional gain.");
DEFINE_double(servo_tilt_pgain, 8, "Tilt servo proportional gain.");

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

    LocalizedObjectMap obj_map = RetrieveObjectData();
    if (obj_map.count("head") == 0 || obj_map.count("body") == 0) {
      std::cerr << "Cannot see head and body for latency measurements!\n";
      continue;
    }
    LocalizedObject head_obj = obj_map["head"];
    LocalizedObject body_obj = obj_map["body"];
    cv::Mat head_r_mat;
    cv::Mat body_r_mat;
    cv::Rodrigues(head_obj.r, head_r_mat);
    cv::Rodrigues(body_obj.r, body_r_mat);
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
  Timestamp ts;
  LocalizedObjectMap obj_map = RetrieveObjectData(&ts);
  if (obj_map.count("head") == 0) {
    // TODO: also try to point head using body tag, if we can't see head?
    Head::GetInstance()->InitTracking();
    return;
  }
  // TODO: Make the head object an instance member, instead of a param?
  cv::Vec3d goal_dir = GetGoalDirection(obj_map["head"], obj_map);
  cv::Vec3d goal_rel = ConvertGoalDirection(obj_map["head"], goal_dir);
  PointHeadToward(obj_map["head"], goal_rel);
  SaveHeadData();
}

LocalizedExplorer::LocalizedObjectMap LocalizedExplorer::RetrieveObjectData(
    Timestamp* ts) {
  LocalizedObjectMap obj_map;
  // Retrieve data from localization client, extract head rotation info.
  std::string data = client_.GetData(ts);
  if (!FLAGS_quiet) {
    std::cout << "DATA:\n" << data << "\n";
  }
  std::vector<std::string> lines = split(data, '\n');
  for (std::vector<std::string>::const_iterator it = lines.begin();
       it != lines.end(); ++it) {
    LocalizedObject obj(*it);
    obj_map[obj.name] = obj;
  }
  return obj_map;
}

cv::Vec3d LocalizedExplorer::GetGoalDirection(
    const LocalizedObject& head,
    const LocalizedObjectMap& obj_map) {
  // Determine whether goal frame is an object or the world frame.
  static cv::Mat_<double> r_obj_to_world = cv::Mat_<double>::eye(3, 3);
  static cv::Mat_<double> t_obj_to_world = cv::Mat_<double>::zeros(3, 1);
  if (!FLAGS_goal_object.empty() && obj_map.count(FLAGS_goal_object) > 0) {
    LocalizedObjectMap::const_iterator it = obj_map.find(FLAGS_goal_object);
    if (it == obj_map.end()) {
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
  cv::Vec3d head_point(cv::Mat(head.t));
  cv::Vec3d goal_dir;
  if (FLAGS_goal_axis_only) {
    goal_dir = goal_point - origin;
  } else {
    goal_dir = goal_point - head_point;
  }
  return goal_dir;
}

cv::Vec3d LocalizedExplorer::ConvertGoalDirection(
    const LocalizedObject& head_obj, const cv::Vec3d& goal_dir) {
  cv::Mat head_r_mat;
  cv::Rodrigues(head_obj.r, head_r_mat);
  // TODO: adjust head_r_mat to compensate for current head position.
  cv::Mat goal_rel = head_r_mat.inv() * cv::Mat(goal_dir);
  std::cout << "goal_rel = " << goal_rel << "\n";
  return goal_rel;
}

void LocalizedExplorer::PointHeadToward(const LocalizedObject& head_obj,
                                        const cv::Vec3d& goal_rel) {
  // Compute relative angles of the goal axis from the head axis.
  cv::Point3d goal_rel_pt(goal_rel);
  double hypot = cv::norm(goal_rel_pt);
  double hypot_xy = cv::norm(cv::Point2d(goal_rel_pt.x, goal_rel_pt.y));
  double pan_rel = asin(goal_rel_pt.y / hypot_xy) * 180.0 / M_PI;
  double tilt_rel = asin(goal_rel_pt.z / hypot) * 180.0 / M_PI;
  std::cout << "pan_rel = " << pan_rel << "\n";
  std::cout << "tilt_rel = " << tilt_rel << "\n";

  // Use relative angles as inputs to PID controllers for head position.
  Head* head = Head::GetInstance();
  double pan_output = pan_controller_.Update(pan_rel);
  double tilt_output = tilt_controller_.Update(tilt_rel);
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

void LocalizedExplorer::SaveHeadData() {
  Head* head = Head::GetInstance();
  HeadData data;
  data.ts = Timestamp::Now();
  data.pan = head->m_Joint.GetAngle(JointData::ID_HEAD_PAN);
  data.tilt = head->m_Joint.GetAngle(JointData::ID_HEAD_TILT);
  head_data_cache_.push_back(data);
  if (head_data_cache_.size() > (size_t) FLAGS_head_data_cache_max_size) {
    head_data_cache_.pop_front();
  }
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
