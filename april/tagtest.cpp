#include <cstdio>
#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "TagDetector.h"

#define DEFAULT_TAG_FAMILY "Tag36h11"

typedef struct TagTestOptions {
  TagTestOptions() :
      show_debug_info(false),
      show_timing(false),
      show_results(false),
      be_verbose(false),
      no_images(false),
      generate_output_files(false),
      params(),
      family_str(DEFAULT_TAG_FAMILY) {
  }
  bool show_debug_info;
  bool show_timing;
  bool show_results;
  bool be_verbose;
  bool no_images;
  bool generate_output_files;
  TagDetectorParams params;
  std::string family_str;
} TagTestOptions;


void print_usage(const char* tool_name, FILE* output=stderr) {
  fprintf(output, "\
Usage: %s [OPTIONS] IMAGE1 [IMAGE2 ...]\n\
Run a tool to test tag detection. Options:\n\
 -h              Show this help message.\n\
 -d              Show debug images and data during tag detection.\n\
 -t              Show timing information for tag detection.\n\
 -R              Show textual results of tag detection.\n\
 -v              Be verbose (includes -d -t -R).\n\
 -x              Do not generate any non-debug visuals.\n\
 -o              Generate debug visuals as output files vs. using X11.\n\
 -D              Use decimation for segmentation stage.\n\
 -s SEGSIGMA     Set the segmentation sigma value (default %.2f).\n\
 -a THETATHRESH  Set the theta threshold for clustering (default %.1f).\n\
 -m MAGTHRESH    Set the magnitude threshold for clustering (default %.1f).\n\
 -c              Re-detect quad corners to improve accuracy.\n\
 -C              Re-re-detect quad corners with subpixel accuracy.\n\
 -b              Set the block size for corner detection (default %d).\n\
 -r              Set the search radius for corner detection (default %d).\n\
 -f FAMILY       Look for the given tag family (default \"%s\")\n",
          tool_name,
          TagDetectorParams::kDefaultSegSigma,
          TagDetectorParams::kDefaultThetaThresh,
          TagDetectorParams::kDefaultMagThresh,
          TagDetectorParams::kDefaultCornerBlockSize,
          TagDetectorParams::kDefaultCornerSearchRadius,
          DEFAULT_TAG_FAMILY);

  fprintf(output, "Known tag families:");
  TagFamily::StringArray known = TagFamily::families();
  for (size_t i = 0; i < known.size(); ++i) {
    fprintf(output, " %s", known[i].c_str());
  }
  fprintf(output, "\n");
}

TagTestOptions parse_options(int argc, char** argv) {
  TagTestOptions opts;
  const char* options_str = "hdtRvxoDs:a:m:cCb:r:f:";
  int c;
  while ((c = getopt(argc, argv, options_str)) != -1) {
    switch (c) {
      // Reminder: add new options to 'options_str' above and print_usage()!
      case 'h': print_usage(argv[0], stdout); exit(0); break;
      case 'd': opts.show_debug_info = true; break;
      case 't': opts.show_timing = true; break;
      case 'R': opts.show_results = true; break;
      case 'v': opts.be_verbose = true; break;
      case 'x': opts.no_images = true; break;
      case 'o': opts.generate_output_files = true; break;
      case 'D': opts.params.segDecimate = true; break;
      case 's': opts.params.segSigma = atof(optarg); break;
      case 'a': opts.params.thetaThresh = atof(optarg); break;
      case 'm': opts.params.magThresh = atof(optarg); break;
      case 'c': opts.params.refineCorners = true; break;
      case 'C': opts.params.refineCornersSubPix = true; break;
      case 'b': opts.params.cornerBlockSize = atoi(optarg); break;
      case 'r': opts.params.cornerSearchRadius = atoi(optarg); break;
      case 'f': opts.family_str = optarg; break;
      default:
        fprintf(stderr, "\n");
        print_usage(argv[0], stderr);
        exit(1);
    }
  }
  if (opts.be_verbose) {
    opts.show_debug_info = opts.show_timing = opts.show_results = true;
  }
  if (opts.params.refineCornersSubPix) {
    opts.params.refineCorners = true;
  }
  return opts;
}

int main(int argc, char** argv) {
  const std::string win = "Tag test";
  TagTestOptions opts = parse_options(argc, argv);
  TagFamily family(opts.family_str);
  TagDetector detector(family, opts.params);
  detector.debug = opts.show_debug_info;
  detector.debugWindowName = win;
  if (opts.params.segDecimate && opts.be_verbose) {
    std::cout << "Will decimate for segmentation!\n";
  }

  TagDetectionArray detections;
  for (int i=optind; i<argc; ++i) {
    cv::Mat src = cv::imread(argv[i]);
    if (src.empty()) { continue; }

    while (std::max(src.rows, src.cols) > 800) {
      cv::Mat tmp;
      cv::resize(src, tmp, cv::Size(0,0), 0.5, 0.5);
      src = tmp;
    }
    cv::Point2d opticalCenter(0.5*src.rows, 0.5*src.cols);

    if (!detector.debug && !opts.no_images) {
      labelAndWaitForKey(win, "Original", src, ScaleNone);
    }

    clock_t start = clock();
    detector.process(src, opticalCenter, detections);
    clock_t end = clock();

    if (opts.show_results) {
      if (opts.show_debug_info) std::cout << "\n";
      std::cout << "Got " << detections.size() << " detections in "
                << double(end-start)/CLOCKS_PER_SEC << " seconds.\n";
      for (size_t i=0; i<detections.size(); ++i) {
        const TagDetection& d = detections[i];
        std::cout << " - Detection: id = " << d.id << ", "
                  << "code = " << d.code << ", "
                  << "rotation = " << d.rotation << "\n";
      }
    }
    if (!opts.no_images) {
      cv::Mat img = family.superimposeDetections(src, detections);
      labelAndWaitForKey(win, "Detected", img, ScaleNone);
    }
  }

  if (opts.show_timing) detector.reportTimers();
  return 0;
}
