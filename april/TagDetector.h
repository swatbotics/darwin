#ifndef _TAGDETECTOR_H_
#define _TAGDETECTOR_H_

#include "Geometry.h"
#include "TagFamily.h"
#include "Refine.h"

class TagDetectorParams {
public:

  TagDetectorParams();

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
  at::real adaptiveThresholdValue;
  int      adaptiveThresholdRadius;
  bool     refineQuads;
  bool     refineBad;
  bool     newQuadAlgorithm;
  bool     hasNewQuadAlgorithm;
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

private:

  struct Images {

    cv::Mat orig;
    cv::Mat_<cv::Vec3b> origRGB;
    cv::Mat origBW;
    cv::Mat_<unsigned char> origBW8;

    at::Mat fimOrig; 
    at::Mat fim;
    
    at::Point opticalCenter;

  };
  
  static void sampleGradient(const Images& images, 
                             int x, int y,
                             at::real& gx, at::real& gy);


  TPointArray tpoints;

  void makeImages(const cv::Mat& image, 
                  const at::Point& opticalCenter,
                  Images& images) const;
  
  void getQuads_AT(const Images& images,
                   QuadArray& quads) const;

  void getQuads_MZ(const Images& images,
                   QuadArray& quads) const;

  void refineQuadL(const Images& images, Quad& quad) const;

  void refineQuadTT(const Images& images, Quad& quad) const;

  void refineQuadLSQ(const Images& images, Quad& quad) const;

  void refineQuads(const Images& images,
                   QuadArray& quads) const;
  
  void debugShowQuads(const Images& images,
                      const QuadArray& quads,
                      int step, const std::string& label) const;

  void decode(const Images& images,
              const QuadArray& quads,
              TagDetectionArray& detections) const;

  bool decodeQuad(const Images& images,
                  const Quad& quad, 
                  size_t quadIndex,
                  TagDetectionArray& detections) const;


};



#endif
