#include "localized_explorer.hpp"

#include <cmath>
#include <iostream>
#include <string>

#include <gflags/gflags.h>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <LinuxDARwIn.h>

#include "util.hpp"

#define INI_FILE_PATH "config.ini"
#define U2D_DEV_NAME "/dev/ttyUSB0"

DEFINE_string(server_name, "192.168.1.7",
              "IP address or DNS name of the status server to query.");
DEFINE_int32(server_port, 9000,
             "Port on the status server to connect to.");
DEFINE_double(fps_target, 15, "Target frames per second to run at.");

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

namespace Robot {

LocalizedExplorer::LocalizedExplorer() :
    cm730_(NULL),
    client_(FLAGS_server_name, FLAGS_server_port),
    pan_controller_(FLAGS_pan_pgain, FLAGS_pan_igain, FLAGS_pan_dgain),
    tilt_controller_(FLAGS_tilt_pgain, FLAGS_tilt_igain, FLAGS_tilt_dgain) {
}

LocalizedExplorer::~LocalizedExplorer() {}

void LocalizedExplorer::Initialize() {
  printf("\n===== INIT EXPLORER =====\n\n");
  // To make relative filenames work.
  change_dir_from_root("darwin/Linux/project/april_tags");
  InitializeMotionFramework();
  printf("Initialize robot position? (hit enter) "); getchar();
  InitializeMotionModules();
  printf("Start tracking mode? (hit enter) "); getchar();
  client_.Run();
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
  head->m_Joint.SetPGain(JointData::ID_HEAD_PAN, 4); //8);
  head->m_Joint.SetPGain(JointData::ID_HEAD_TILT, 4); //8);
  MotionStatus::m_CurrentJoints.SetEnableBodyWithoutHead(false);
  manager->SetEnable(true);
}

void LocalizedExplorer::Process() {
  usleep(1000 * 1000 / FLAGS_fps_target);
  std::cout << "\n";

  // Retrieve data from localization client, extract head rotation info.
  std::string data = client_.GetData();
  std::cout << "DATA:\n" << data << "\n";
  cv::Vec3d head_r(0, 0, 0);
  cv::Vec3d head_t(0, 0, 0);
  bool found_head = false;
  std::vector<std::string> lines = split(data, '\n');
  for (std::vector<std::string>::const_iterator it = lines.begin();
       it != lines.end(); ++it) {
    std::stringstream ss(*it);
    std::string name, sep1, sep2;
    double t[3] = {};
    double r[3] = {};
    ss >> name >> sep1 >> t[0] >> t[1] >> t[2]
       >> sep2 >> r[0] >> r[1] >> r[2];
    if (name == "head") {
      found_head = true;
      for (int i = 0; i < 3; ++i) {
        head_r[i] = r[i];
        head_t[i] = t[i];
      }
    }
  }
  if (!found_head) {
    //    Head::GetInstance()->MoveToHome();
    Head::GetInstance()->InitTracking();
    return;
  }

  // Determine whether head should align with goal as a point or an axis.
  cv::Point3d origin(0, 0, 0);
  cv::Point3d goal_point(FLAGS_goal_x, FLAGS_goal_y, FLAGS_goal_z);
  cv::Point3d head_point(head_t);
  cv::Vec3d goal_dir;
  if (FLAGS_goal_axis_only) {
    goal_dir = goal_point - origin;
  } else {
    goal_dir = goal_point - head_point;
  }

  // Compute relative angles of the goal axis from the head axis.
  cv::Mat head_r_mat;
  cv::Rodrigues(head_r, head_r_mat);
  cv::Mat goal_rel = head_r_mat.inv() * cv::Mat(goal_dir);
  std::cout << "goal_rel = " << goal_rel << "\n";
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
