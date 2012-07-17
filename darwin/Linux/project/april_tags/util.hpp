#ifndef UTIL_HPP
#define UTIL_HPP

#include <string>
#include <vector>

typedef struct {
  unsigned char R;
  unsigned char G;
  unsigned char B;
} rgb_color;

void print_double_visually(const char* label, double min, double max,
                           double value);

double get_time_as_double();

double record_elapsed_time();

void change_dir_from_root(const char* relpath);

void change_current_dir();

inline bool in_range(double value, double min, double max) {
  return min <= value && value <= max;
}

std::vector<std::string> &split(const std::string &s, char delim,
                                std::vector<std::string> &elems);

std::vector<std::string> split(const std::string &s, char delim);

bool prompt(const std::string& prompt_text, bool yesno=false);

#endif  // UTIL_HPP
