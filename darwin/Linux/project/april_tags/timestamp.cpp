#include "timestamp.hpp"

#include <cmath>  // For floating-point abs().
#include <sstream>

#define K10E9 1000000000

Timestamp Timestamp::Now() {
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  return Timestamp(ts);
}

Timestamp::Timestamp(struct timespec ts) :
    sec_(ts.tv_sec),
    nsec_(ts.tv_nsec) {
  Canonicalize();
}

Timestamp::Timestamp(long int seconds, long int nseconds) :
    sec_(seconds),
    nsec_(nseconds) {
  Canonicalize();
}

Timestamp::Timestamp(const std::string& str) {
  std::istringstream ss(str);
  ss >> sec_ >> nsec_;
  Canonicalize();
}

void Timestamp::Canonicalize() {
  // If nseconds is outside the range 0 <= nseconds < K10E9, then we
  // mod it to within that range and add the appropriate number of seconds
  // to sec_. Since nseconds < 0 results in negative mod values, we
  // correct for that to ensure it is always in 0 <= nseconds < K10E9.
  sec_ = sec_ + nsec_ / K10E9 + (nsec_ >= 0 ? 0 : -1);
  nsec_ = nsec_ % K10E9 + (nsec_ >= 0 ? 0 : K10E9);
}

Timestamp Timestamp::operator-(const Timestamp& other) const {
  return Timestamp(sec_ - other.sec_, nsec_ - other.nsec_);
}

Timestamp Timestamp::operator*(double scale) const {
  // TODO: Convince myself this handles scale < 0, and sec_ < 0?
  return Timestamp(sec_ * scale, nsec_ * scale);
}

Timestamp& Timestamp::operator=(const Timestamp& other) {
  sec_ = other.sec_;
  nsec_ = other.nsec_;
  return *this;
}

bool Timestamp::operator==(const Timestamp& other) const {
  return sec_ == other.sec_ && nsec_ == other.nsec_;
}

bool Timestamp::operator<(const Timestamp& other) const {
  return sec_ < other.sec_ && nsec_ < other.nsec_;
}

bool Timestamp::operator>(const Timestamp& other) const {
  return !(*this < other || *this == other);
}

bool Timestamp::operator<=(const Timestamp& other) const {
  return *this < other || *this == other;
}

bool Timestamp::operator>=(const Timestamp& other) const {
  return *this > other || *this == other;
}

double Timestamp::ToDouble() const {
  return sec_ + nsec_ / (double) K10E9;
}

std::string Timestamp::ToString() const {
  std::ostringstream oss;
  oss << sec_ << " " << nsec_;
  return oss.str();
}

std::string Timestamp::ToStringReadable() const {
  double time_in_s = ToDouble();
  std::ostringstream ss;
  ss.precision(1);
  ss.width(5);
  ss << std::fixed;
  if (std::abs(time_in_s) >= 1.0) {
    ss << time_in_s << "s";
  } else {
    double time_in_ms = time_in_s * 1000;
    if (std::abs(time_in_ms) >= 1.0) {
      ss << time_in_ms << "ms";
    } else {
      double time_in_us = time_in_ms * 1000;
      ss << time_in_us << "us";
    }
  }
  return ss.str();
}

std::ostream& operator<<(std::ostream& stream, const Timestamp& ts) {
  return stream << ts.ToStringReadable();
}
