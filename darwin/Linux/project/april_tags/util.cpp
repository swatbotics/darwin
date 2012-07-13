#include "util.hpp"

#include <algorithm>  // For min() and max().
#include <cstdio>
#include <cstdlib>
#include <libgen.h>  // For dirname().
#include <sstream>  // For stringstream.
#include <string>
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

double get_time_as_double() {
  static struct timeval tval;
  gettimeofday(&tval, NULL);
  return tval.tv_sec + tval.tv_usec / 1000000.0;
}

double record_elapsed_time() {
  double this_time = get_time_as_double();
  static double last_time = this_time;
  double elapsed_time = this_time - last_time;
  last_time = this_time;
  return elapsed_time;
}

void change_dir_from_root(const char* relpath) {
  const char* root_dir_path = getenv("DARWIN_ROOT");
  if (root_dir_path != NULL) {
    std::string path(root_dir_path);
    path += std::string("/") + std::string(relpath);
    if (chdir(path.c_str()) != 0) {
      fprintf(stderr, "Error changing directory to: %s\n", path.c_str());
      exit(1);
    }
  } else {
    fprintf(stderr, "Error reading DARWIN_ROOT environment variable!\n");
    exit(1);
  }
}

void change_current_dir() {
  char exepath[1024] = {0};
  if (readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1) {
    if (chdir(dirname(exepath)) != 0) {
      fprintf(stderr, "Error changing directory to: %s\n", exepath);
    }
  }
}

// Split functions from http://stackoverflow.com/a/236803/1179226.
std::vector<std::string> &split(const std::string &s, char delim,
                                std::vector<std::string> &elems) {
  std::stringstream ss(s);
  std::string item;
  while(std::getline(ss, item, delim)) {
    elems.push_back(item);
  }
  return elems;
}

std::vector<std::string> split(const std::string &s, char delim) {
  std::vector<std::string> elems;
  return split(s, delim, elems);
}
