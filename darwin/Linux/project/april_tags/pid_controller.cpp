#include "pid_controller.hpp"

#include "util.hpp"

PIDController::PIDController(double pgain, double igain, double dgain) :
    pgain_(pgain),
    igain_(igain),
    dgain_(dgain),
    first_update_(true),
    previous_time_(0.0),
    previous_error_(0.0),
    integral_error_(0.0),
    deriv_error_(0.0) {
}

double PIDController::Update(double error) {
  double curtime = get_time_as_double();
  double delta = first_update_ ? 0.0 : curtime - previous_time_;
  previous_time_ = curtime;

  integral_error_ += error * delta;
  deriv_error_ = (first_update_ ? 0.0 :
                  (error - previous_error_) / delta);
  previous_error_ = error;
  first_update_ = false;
  return ((pgain_ * error) +
          (igain_ * integral_error_) +
          (dgain_ * deriv_error_));
}
