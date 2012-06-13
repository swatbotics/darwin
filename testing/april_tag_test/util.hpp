#ifndef UTIL_HPP
#define UTIL_HPP

typedef struct {
  unsigned char R;
  unsigned char G;
  unsigned char B;
} rgb_color;

void print_double_visually(const char* label, double min, double max,
                           double value);

double record_elapsed_time();

void change_current_dir();

#endif  // UTIL_HPP
