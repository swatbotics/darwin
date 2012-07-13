#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

class PIDController {
 public:
  PIDController(double pgain, double igain, double dgain);
  ~PIDController() {}
  double Update(double error);
  double GetProportionalError() const { return previous_error_; }
  double GetIntegralError() const { return integral_error_; }
  double GetDerivativeError() const { return deriv_error_; }

 private:
  const double pgain_;
  const double igain_;
  const double dgain_;
  bool first_update_;
  double previous_time_;
  double previous_error_;
  double integral_error_;
  double deriv_error_;
};

#endif  // PID_CONTROLLER_HPP
