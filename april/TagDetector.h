#ifndef _TAGDETECTOR_H_
#define _TAGDETECTOR_H_

#include "Geometry.h"
#include "TagFamily.h"

class TagDetector {
public:

  typedef at::uint uint;

  const TagFamily& tagFamily;

  at::real sigma;
  at::real segSigma;
  bool     segDecimate;
  at::real minMag;
  at::real maxEdgeCost;
  at::real thetaThresh;
  at::real magThresh;
  at::real minimumLineLength;
  at::real minimumSegmentSize;
  at::real minimumTagSize;
  at::real maxQuadAspectRatio;

  bool        debug;
  std::string debugWindowName;

  enum { WEIGHT_SCALE = 100 };//10000;

  TagDetector(const TagFamily& f);

  static at::real arctan2(at::real y, at::real x);

  int edgeCost(at::real theta0, at::real mag0, 
	       at::real theta1, at::real mag1) const;

  void process(const cv::Mat& image,
               const at::Point& opticalCenter,
               TagDetectionArray& detections) const;

  void search(const at::Point& opticalCenter,
              QuadArray& quads, 
              Segment* path[5],
              Segment* parent, 
              int depth) const;
  
  static void initTimers();
  static void reportTimers();

};



#endif
