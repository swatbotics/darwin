#include "util.hpp"

#include <algorithm>  // For min() and max().
#include <cstdio>
#include <libgen.h>  // For dirname().
#include <sys/time.h>
#include <unistd.h>

void print_double_visually(const char* label, double min, double max,
                           double value) {
  if (label != NULL) printf("%s: ", label);
  static const int kMeterWidth = 50;
  int bucket = (value - min) * kMeterWidth / (max - min);
  bucket = std::max(0, bucket);
  bucket = std::min(kMeterWidth - 1, bucket);
  printf("% 4.2f [", min);
  for (int i = 0; i < kMeterWidth; ++i) {
    printf("%c", (i == bucket) ? '+' : ' ');
  }
  printf("] % 4.2f", max);
  printf(" (%.4f)\n", value);
}

double record_elapsed_time() {
  static double last_time = -1.0;
  struct timeval tval;
  gettimeofday(&tval, NULL);
  double this_time = tval.tv_sec + tval.tv_usec / 1000000.0;
  double elapsed_time = (last_time < 0) ? 0 : this_time - last_time;
  last_time = this_time;
  return elapsed_time;
}

void change_current_dir() {
  char exepath[1024] = {0};
  if (readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1) {
    if (chdir(dirname(exepath)) != 0) {
      fprintf(stderr, "Error changing directory to: %s\n", exepath);
    }
  }
}
