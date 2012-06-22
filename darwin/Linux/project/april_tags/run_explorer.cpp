#include <cstdio>

#include "explorer.hpp"
#include "util.hpp"

int main(void) {
  printf( "\n===== April Tag Test for DARwIn =====\n\n");
  change_current_dir();  // To make relative filenames work.

  Explorer explorer;
  explorer.Initialize();
  record_elapsed_time();
  while (true) {
    explorer.Process();
    double frame_time = record_elapsed_time();
    printf("FPS: % 2d\n",  (int) (1 / frame_time));
  }
  return 0;
}
