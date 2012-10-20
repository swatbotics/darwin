#ifndef TIMESTAMP_HPP
#define TIMESTAMP_HPP

#include <iostream>
#include <string>
#include <time.h>

class Timestamp {
 public:
  static Timestamp Now();

  // Intentionally not-explicit to allow automatic conversion from timespec
  // in, for example, overloaded operators below.
  Timestamp(struct timespec ts);
  explicit Timestamp(long int seconds=0, long int nseconds=0);
  explicit Timestamp(const std::string& str);
  ~Timestamp() {}

  void Canonicalize();

  Timestamp operator-(const Timestamp& other) const;
  Timestamp operator*(double scale) const;
  Timestamp& operator=(const Timestamp& other);
  bool operator==(const Timestamp& other) const;
  bool operator<(const Timestamp& other) const;
  bool operator>(const Timestamp& other) const;
  bool operator<=(const Timestamp& other) const;
  bool operator>=(const Timestamp& other) const;
  
  double ToDouble() const;
  std::string ToString() const;
  std::string ToStringReadable() const;

 private:
  long int sec_;
  long int nsec_;
};

std::ostream& operator<<(std::ostream& stream, const Timestamp& ts);

#endif  // TIMESTAMP_HPP
