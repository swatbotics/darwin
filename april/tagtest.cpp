#include "TagDetector.h"
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int main(int argc, char** argv) {

  const std::string dstr = "--decimate";

  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " TAGFAMILY [--decimate] IMAGE1 [IMAGE2 ...]\n";
    std::cerr << "Known tag families:";
    TagFamily::StringArray known = TagFamily::families();
    for (size_t i=0; i<known.size(); ++i) { std::cerr << " " << known[i]; }
    std::cerr << "\n";
    return 1;
  }
    

  const std::string win = "Tag test";

  TagFamily family(argv[1]);
  TagDetector detector(family);

  detector.debug = false;
  detector.debugWindowName = win;

  int sarg = 2;
  if (argv[sarg] == dstr) {
    std::cout << "will decimate for segmentation!\n";
    detector.params.segDecimate = true;
    ++sarg;
  }

  TagDetectionArray detections;

  for (int i=sarg; i<argc; ++i) {

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
