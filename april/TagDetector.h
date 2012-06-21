#ifndef _TAGDETECTOR_H_
#define _TAGDETECTOR_H_

#include "Geometry.h"
#include "TagFamily.h"

class TagDetectorParams {
 public:
  static const at::real kDefaultSigma = 0;
  static const at::real kDefaultSegSigma = 0.8;
  static const bool     kDefaultSegDecimate = false;
  static const at::real kDefaultMinMag = 0.004;
  static const at::real kDefaultMaxEdgeCost = 30*M_PI/180;
  static const at::real kDefaultThetaThresh = 100;
  static const at::real kDefaultMagThresh = 1200;
  static const at::real kDefaultMinimumLineLength = 4;
  static const at::real kDefaultMinimumSegmentSize = 4;
  static const at::real kDefaultMinimumTagSize = 6;
  static const at::real kDefaultMaxQuadAspectRatio = 32;
  static const bool     kDefaultRefineCorners = false;
  static const int      kDefaultCornerBlockSize = 3;
  static const int      kDefaultCornerSearchRadius = 3;

  TagDetectorParams() :
      sigma(kDefaultSigma),
      segSigma(kDefaultSegSigma),
      segDecimate(kDefaultSegDecimate),
      minMag(kDefaultMinMag),
      maxEdgeCost(kDefaultMaxEdgeCost),
      thetaThresh(kDefaultThetaThresh),
      magThresh(kDefaultMagThresh),
      minimumLineLength(kDefaultMinimumLineLength),
      minimumSegmentSize(kDefaultMinimumSegmentSize),
      minimumTagSize(kDefaultMinimumTagSize),
      maxQuadAspectRatio(kDefaultMaxQuadAspectRatio),
      refineCorners(kDefaultRefineCorners),
      cornerBlockSize(kDefaultCornerBlockSize),
      cornerSearchRadius(kDefaultCornerSearchRadius) {
  }

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
  bool     refineCorners;
  int      cornerBlockSize;
  int      cornerSearchRadius;
};


class TagDetector {
public:

  typedef at::uint uint;
  enum { WEIGHT_SCALE = 100 };//10000;

  static const bool kDefaultDebug = false;
  static const bool kDefaultDebugNumberFiles = false;
  static const char* kDefaultDebugWindowName;

  explicit TagDetector(const TagFamily& f,
                       const TagDetectorParams& parameters =
                       TagDetectorParams());

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

  const TagFamily& tagFamily;
  TagDetectorParams params;

  bool debug;
  bool debugNumberFiles;
  std::string debugWindowName; // if this is empty, will instead emit files
};



#endif
