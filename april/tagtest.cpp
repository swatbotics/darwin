#include <cstdio>
#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "TagDetector.h"

#define DEFAULT_TAG_FAMILY "Tag36h11"

typedef struct TagTestOptions {
  TagTestOptions() :
      show_timing(false),
      show_verbose_debug(false),
      generate_output_files(false),
      params(),
      family_str(DEFAULT_TAG_FAMILY) {
  }
  bool use_decimation;
  bool show_timing;
  bool show_verbose_debug;
  bool generate_output_files;
  TagDetectorParams params;
  std::string family_str;
} TagTestOptions;


void print_usage(const char* tool_name, FILE* output=stderr) {
  fprintf(output, "\
Usage: %s [OPTIONS] IMAGE1 [IMAGE2 ...]\n\
Run a tool to test tag detection. Options:\n\
 -h              Show this help message.\n\
 -d              Use decimation for segmentation stage.\n\
 -t              Show timing information for tag detection.\n\
 -v              Show verbose debug info from tag detection.\n\
 -o              Generate debug visuals as output files vs. using X11.\n\
 -s SEGSIGMA     Set the segmentation sigma value (default %.2f).\n\
 -a THETATHRESH  Set the theta threshold for clustering (default %.1f).\n\
 -m MAGTHRESH    Set the magnitude threshold for clustering (default %.1f).\n\
 -f FAMILY       Look for the given tag family (default \"%s\")\n",
          tool_name,
          TagDetectorParams::kDefaultSegSigma,
          TagDetectorParams::kDefaultThetaThresh,
          TagDetectorParams::kDefaultMagThresh,
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
  const char* option_str = "hdtvos:a:m:f:";
  int c;
  while ((c = getopt(argc, argv, option_str)) != -1) {
    switch (c) {
      case 'h': print_usage(argv[0], stdout); exit(0); break;
      case 'd': opts.params.segDecimate = true; break;
      case 't': opts.show_timing = true; break;
      case 'v': opts.show_verbose_debug = true; break;
      case 'o': opts.generate_output_files = true; break;
      case 's': opts.params.segSigma = atof(optarg); break;
      case 'a': opts.params.thetaThresh = atof(optarg); break;
      case 'm': opts.params.magThresh = atof(optarg); break;
      case 'f': opts.family_str = optarg; break;
      default:
        fprintf(stderr, "\n");
        print_usage(argv[0], stderr);
        exit(1);
    }
  }
  return opts;
}

int main(int argc, char** argv) {
  TagTestOptions opts = parse_options(argc, argv);
  const std::string win = "Tag test";

  TagFamily family(opts.family_str);
  TagDetector detector(family, opts.params);
  detector.debug = opts.show_verbose_debug;
  detector.debugWindowName = win;

  if (opts.params.segDecimate) {
    std::cout << "will decimate for segmentation!\n";
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

    if (!detector.debug) {
      labelAndWaitForKey(win, "Original", src, ScaleNone);
    }

    clock_t start = clock();
    
    detector.process(src, opticalCenter, detections);
    
    clock_t end = clock();
    
    std::cout << "got " << detections.size() << " detections in " <<
      double(end-start)/CLOCKS_PER_SEC << " seconds\n";
    
    for (size_t i=0; i<detections.size(); ++i) {
      
      const TagDetection& d = detections[i];
      std::cout << "id = " << d.id << ", code=" << d.code << "\n";
      std::cout << "rotation = " << d.rotation << "\n";

      
      
    }

    cv::Mat img = family.superimposeDetections(src, detections);
    labelAndWaitForKey(win, "Detected", img, ScaleNone);

  }

  detector.reportTimers();

  return 0;

  

}
