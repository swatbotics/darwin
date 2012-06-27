#include "TagDetector.h"
#include "UnionFindSimple.h"
#include <map>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include "MathUtil.h"
#include "Geometry.h"
#include "GrayModel.h"

#include <sys/time.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define _USE_FAST_MATH_

typedef std::vector<at::real> RealArray;
typedef std::vector<size_t> SizeArray;
typedef std::vector<uint64_t> Uint64Array;

enum TimerIndex {
  total_time = 0,
  step1_time,
  step2_time,
  step3_time,
  step3a_time,
  step3b_time,
  step3c_time,
  step4_time,
  step5_time,
  step6_time,
  step7_time,
  step7b_time,
  step8_time,
  step9_time,
  cleanup_time,
  num_timers
};

static double start_times[num_timers];
static double run_times[num_timers];
static const char* descriptions[num_timers];
static int num_iterations = -1;
static int num_detections;

#define START_PROFILE(which, what) \
  start_times[which] = getTimeAsDouble(); descriptions[which] = what;
#define END_PROFILE(which) \
  run_times[which] += (getTimeAsDouble() - start_times[which]); \
  num_iterations += (which == total_time) ? 1 : 0;
#define REPORT_PROFILE(which) \
  printf("% 6.1fms - %s - %s\n", (run_times[which] / num_iterations * 1000), \
         #which, (descriptions[which] ? descriptions[which] : ""));

static double getTimeAsDouble() {
  struct timeval tp;
  gettimeofday(&tp, 0);
  return double(tp.tv_sec) + double(tp.tv_usec) * 1e-6;
}

void TagDetector::initTimers() {
  num_iterations = 0;
  num_detections = 0;
  memset(run_times, 0, sizeof(run_times));
  memset(descriptions, 0, sizeof(descriptions));
}

void TagDetector::reportTimers() {
  std::cout << "report averaged over " << num_iterations << " frames with " << num_detections << " detections (" << (double(num_detections)/num_iterations) << " per frame)\n\n";
  REPORT_PROFILE(total_time);
  std::cout << "\n";
  REPORT_PROFILE(step1_time);
  REPORT_PROFILE(step2_time);
  REPORT_PROFILE(step3_time);
  REPORT_PROFILE(step3a_time);
  REPORT_PROFILE(step3b_time);
  REPORT_PROFILE(step3c_time);
  REPORT_PROFILE(step4_time);
  REPORT_PROFILE(step5_time);
  REPORT_PROFILE(step6_time);
  REPORT_PROFILE(step7_time);
  REPORT_PROFILE(step7b_time);
  REPORT_PROFILE(step8_time);
  REPORT_PROFILE(step9_time);
  REPORT_PROFILE(cleanup_time);
}


//////////////////////////////////////////////////////////////////////

typedef std::vector< cv::Scalar > ScalarVec;

static const ScalarVec& getCColors() {
  static ScalarVec ccolors;
  if (ccolors.empty()) {
    ccolors.push_back(CV_RGB(  0, 255, 255));
    ccolors.push_back(CV_RGB(255,   0,   0));
    ccolors.push_back(CV_RGB(  0, 191, 255));
    ccolors.push_back(CV_RGB(255,  63,   0));
    ccolors.push_back(CV_RGB(  0, 127, 255));
    ccolors.push_back(CV_RGB(255, 127,   0));
    ccolors.push_back(CV_RGB(  0,  63, 255));
    ccolors.push_back(CV_RGB(255, 191,   0));
    ccolors.push_back(CV_RGB(  0,   0, 255));
    ccolors.push_back(CV_RGB(255, 255,   0));
    ccolors.push_back(CV_RGB( 63,   0, 255));
    ccolors.push_back(CV_RGB(191, 255,   0));
    ccolors.push_back(CV_RGB(127,   0, 255));
    ccolors.push_back(CV_RGB(127, 255,   0));
    ccolors.push_back(CV_RGB(191,   0, 255));
    ccolors.push_back(CV_RGB( 63, 255,   0));
    ccolors.push_back(CV_RGB(255,   0, 255));
    ccolors.push_back(CV_RGB(  0, 255,   0));
    ccolors.push_back(CV_RGB(255,   0, 191));
    ccolors.push_back(CV_RGB(  0, 255,  63));
    ccolors.push_back(CV_RGB(255,   0, 127));
    ccolors.push_back(CV_RGB(  0, 255, 127));
    ccolors.push_back(CV_RGB(255,   0,  63));
    ccolors.push_back(CV_RGB(  0, 255, 191));
  }
  return ccolors;
}

//////////////////////////////////////////////////////////////////////

static void countingSortLongArray(Uint64Array& v,
				  size_t vlength, 
				  int maxv, uint64_t mask) {

  if (maxv < 0) {
    for (size_t i = 0; i < vlength; i++)
      maxv = std::max(maxv, (int) (v[i]&mask));
  }

  // For weight 'w', counts[w] will give the output position for
  // the next sample with weight w.  To build this, we begin by
  // counting how many samples there are with each weight. (Note
  // that the initial position for weight w is only affected by
  // the number of weights less than w, hence the +1 in
  // counts[w+1].
  //  int counts[] = new int[maxv+2];
  SizeArray counts(maxv+2, 0);
  
  for (size_t i = 0; i < vlength; i++) {
    int w = (int) (v[i]&mask);
    counts[w+1]++;
  }
  
  // accumulate.
  for (size_t i = 1; i < counts.size(); i++) {
    counts[i] += counts[i-1];
  }
  
  //long newv[] = new long[vlength];
  Uint64Array newv(vlength);

  for (size_t i = 0; i < vlength; i++) {
    int w = (int) (v[i]&mask);
    newv[counts[w]] = v[i];
    counts[w]++;
  }
  
  /*       // test (debugging code)
           for (int i = 0; i+1 < newv.length; i++) {
           int w0 = (int) (newv[i]&mask);
           int w1 = (int) (newv[i+1]&mask);
           assert(w0 <= w1);
           }
  */
  
  newv.swap(v);

}

//////////////////////////////////////////////////////////////////////

bool detectionsOverlapTooMuch(const TagDetection& a, const TagDetection& b)
{
  // Compute a sort of "radius" of the two targets. We'll do
  // this by computing the average length of the edges of the
  // quads (in pixels).
  at::real radius = 0.0625*(pdist(a.p[0], a.p[1]) +
                          pdist(a.p[1], a.p[2]) +
                          pdist(a.p[2], a.p[3]) +
                          pdist(a.p[3], a.p[0]) +
                          pdist(b.p[0], b.p[1]) +
                          pdist(b.p[1], b.p[2]) +
                          pdist(b.p[2], b.p[3]) +
                          pdist(b.p[3], b.p[0]));
  
  // distance (in pixels) between two tag centers.
  at::real d = pdist(a.cxy, b.cxy);
  
  // reject pairs where the distance between centroids is
  // smaller than the "radius" of one of the tags.
  return (d < radius);

}



//////////////////////////////////////////////////////////////////////

const char* TagDetector::kDefaultDebugWindowName = "";

TagDetector::TagDetector(const TagFamily& f,
                         const TagDetectorParams& parameters) :
    tagFamily(f),
    params(parameters),
    debug(kDefaultDebug),
    debugNumberFiles(kDefaultDebugNumberFiles),
    debugWindowName(kDefaultDebugWindowName) {
}

//////////////////////////////////////////////////////////////////////
// lousy approximation of arctan function, but good enough for our
// purposes (about 4 degrees)

at::real TagDetector::arctan2(at::real y, at::real x) {

  at::real coeff_1 = at::real(M_PI/4);
  at::real coeff_2 = 3*coeff_1;
  at::real abs_y = fabs(y)+MathUtil::epsilon;      // kludge to prevent 0/0 condition
  
  at::real angle;
  
  if (x >= 0) {
    at::real r = (x - abs_y) / (x + abs_y);
    angle = coeff_1 - coeff_1 * r;
  } else {
    at::real r = (x + abs_y) / (abs_y - x);
      angle = coeff_2 - coeff_1 * r;
  }
  
  if (y < 0) {
    return -angle;     // negate if in quad III or IV
  } else {
    return angle;
  }
  
}

int TagDetector::edgeCost(at::real theta0, at::real mag0, 
			  at::real theta1, at::real mag1) const {

  if (mag0 < params.minMag || mag1 < params.minMag) {
    return -1;
  }

  at::real thetaErr = fabs(MathUtil::mod2pi(theta1 - theta0));
  if (thetaErr > params.maxEdgeCost) {
    return -1;
  }

  at::real normErr = thetaErr / params.maxEdgeCost;

  assert( int(normErr*WEIGHT_SCALE) >= 0 );
    
  return (int) (normErr * WEIGHT_SCALE);

}

cv::Mat gaussianBlur(const cv::Mat& input, at::real sigma) {

  cv::Mat output;
  cv::GaussianBlur(input, output, cv::Size(0,0), sigma);
  return output;
  
}

void emitDebugImage(const std::string& windowName,
                    int step, int substep, bool number,
                    const std::string& label,
                    const cv::Mat& img,
                    ScaleType type) {

  static int num = -1;

  if (windowName.empty()) {

    char buf[1024];
    if (number) {
      if (step == 0 && substep == 0) { ++num; }
      snprintf(buf, 1024, "debug_%04d_%d_%d.png", num, step, substep);
    } else {
      snprintf(buf, 1024, "debug_%d_%d.png", step, substep);
    }

    cv::Mat tmp = rescaleImage(img, type);
    labelImage(tmp, label);

    cv::imwrite(buf, tmp);

  } else {

    labelAndWaitForKey(windowName, label, img, type);

  }

}

void TagDetector::process(const cv::Mat& orig,
                          const at::Point& opticalCenter,
                          TagDetectionArray& detections) const {

  if (num_iterations < 0) { initTimers(); }

  START_PROFILE(total_time, "overall time");

  if (debug) { 
    emitDebugImage(debugWindowName, 
                   0, 0, debugNumberFiles,
                   "Orig", 
                   orig, ScaleNone); 
  }

  // This is a very long function, but it can't really be
  // factored any more simply: it's just a long sequence of
  // sequential operations.

  ///////////////////////////////////////////////////////////
  // Step one. Preprocess image (convert to float (grayscale)
  // and low pass if necessary.)

  START_PROFILE(step1_time, "preprocess image and lopass");

  cv::Mat origbw;

  if (orig.channels() == 1) {
    origbw = orig;
  } else {
    cv::cvtColor(orig, origbw, cv::COLOR_RGB2GRAY);
  }

  at::Mat fimOrig;
  if (orig.depth() != at::IMAGE_TYPE) {
    origbw.convertTo(fimOrig, at::IMAGE_TYPE, 1.0/255);
  } else {
    fimOrig = origbw;
  }

  at::Mat fim;

  if (params.sigma > 0) {
    cv::GaussianBlur(fimOrig, fim, cv::Size(0,0), params.sigma);
    if (debug) { 
      emitDebugImage(debugWindowName, 
                     1, 0, debugNumberFiles,
                     "Blur", 
                     fim, ScaleNone); 
    }
  } else {
    fim = fimOrig;
  }


  cv::Mat rgbOrig;

  if (debug) {
    cv::Mat rgb;
    if (orig.channels() == 3) {
      rgb = orig;
    } else {
      cv::cvtColor(orig, rgb, cv::COLOR_GRAY2RGB);
    }
    if (rgb.depth() != CV_8U) {
      at::real scl = 1;
      if (rgb.depth() == CV_32F || rgb.depth() == CV_64F) {
        scl = 255;
      }
      rgb.convertTo(rgbOrig, scl);
    } else {
        rgbOrig = rgb;
    }
  }

  END_PROFILE(step1_time);

  ///////////////////////////////////////////////////////////
  // Step two. For each pixel, compute the local gradient. We
  // store the direction and magnitude.

  START_PROFILE(step2_time, "compute gradient direction and magnitude");

  // This step is quite sensitive to noise, since a few bad
  // theta estimates will break up segments, causing us to miss
  // quads. It is helpful to do a Gaussian low-pass on this step
  // even if we don't want it for decoding.

  at::Mat fimseg;

  if (params.segSigma > 0) {
    if (params.segSigma == params.sigma) {
      fimseg = fim;
    } else {
      cv::GaussianBlur(fimOrig, fimseg, cv::Size(0,0), params.segSigma);
    }
    if (debug) { 
      emitDebugImage(debugWindowName, 
                     2, 0, debugNumberFiles,
                     "Seg. Blur", 
                     fimseg, ScaleNone); 
    }
  } else {
    fimseg = fimOrig;
  }

  if (params.segDecimate) {
    at::Mat small(fimseg.rows/2, fimseg.cols/2);
    for (int y=0; y<small.rows; ++y) {
      for (int x=0; x<small.cols; ++x) {
        small(y,x) = 0.25 * ( fimseg(2*y+0, 2*x+0) +
                              fimseg(2*y+0, 2*x+1) +
                              fimseg(2*y+1, 2*x+0) + 
                              fimseg(2*y+1, 2*x+1) );
      }
    }
    fimseg = small;
  }

  at::Mat fimTheta( fimseg.size() );
  at::Mat fimMag( fimseg.size() );

  for (int x=0; x<fimseg.cols; ++x) {
    fimTheta(0, x) = fimTheta(fimseg.rows-1, x) =
      fimMag(0, x) = fimMag(fimseg.rows-1, x) = 0;
  }

  for (int y=0; y<fimseg.rows; ++y) {
    fimTheta(y, 0) = fimTheta(y, fimseg.cols-1) =
      fimMag(y, 0) = fimMag(y, fimseg.cols-1) = 0;
  }

  for (int y=1; y+1<fimseg.rows; ++y) {
    for (int x=1; x+1<fimseg.cols; ++x) {

      at::real Ix = fimseg(y, x+1) - fimseg(y, x-1);
      at::real Iy = fimseg(y+1, x) - fimseg(y-1, x);
        
      at::real mag = Ix*Ix + Iy*Iy;

      fimMag(y, x) = mag;

#ifdef _USE_FAST_MATH_
      at::real theta = arctan2(Iy, Ix);
#else
      at::real theta = atan2(Iy, Ix);
#endif
      
      fimTheta(y, x) = theta;

    }
  }

  END_PROFILE(step2_time);

  if (debug) {

    at::real mmin =  AT_REAL_MAX;
    at::real mmax = -AT_REAL_MAX;
    for (int y=1; y+1<fimseg.rows; ++y) {
      for (int x=1; x+1<fimseg.cols; ++x) {
        at::real m = fimMag(y,x);
        mmin = std::min(mmin, m);
        mmax = std::max(mmax, m);
      }
    }

    size_t nbins = 10;
    std::vector<int> hist(nbins, 0);
    for (int y=1; y+1<fimseg.rows; ++y) {
      for (int x=1; x+1<fimseg.cols; ++x) {
        at::real m = fimMag(y,x);
        at::real f = (m-mmin)/(mmax-mmin);
        int bin = int(f*(nbins-1) + 0.5);
        ++hist[bin];
      }
    }

    std::cout << "max mag = " << mmax << ", min mag = " << mmin << "\n";

    for (size_t i=0; i<nbins; ++i) {
      at::real rmin = mmin + i*(mmax-mmin)/nbins;
      at::real rmax = mmin + (i+1)*(mmax-mmin)/nbins;
      std::cout << rmin << "-" << rmax << ": " << hist[i] << "\n";
    }

    std::cout << "\n";
    
    emitDebugImage(debugWindowName,
                   2, 1, debugNumberFiles,
                   "Theta", 
                   fimTheta, ScaleMinMax);
    
    emitDebugImage(debugWindowName, 
                   2, 2, debugNumberFiles,
                   "Magnitude", 
                   fimMag, ScaleMinMax);

  }


  ///////////////////////////////////////////////////////////
  // Step three. Segment the edges, grouping pixels with similar
  // thetas together. This is a greedy algorithm: we start with
  // the most similar pixels.  We use 4-connectivity.

  START_PROFILE(step3_time, "segment");

  UnionFindSimple uf(fimseg.cols*fimseg.rows);

  if (true) {
    
    START_PROFILE(step3a_time, "build edges array");

    int width = fimseg.cols;
    int height = fimseg.rows;
    
    Uint64Array edges(width*height*4);

    int nedges = 0;

    // for efficiency, each edge is encoded as a single
    // long. The constants below are used to pack/unpack the
    // long.
    const uint64_t IDA_SHIFT = 40, IDB_SHIFT = 16, 
      INDEX_MASK = (1<<24) - 1, WEIGHT_MASK=(1<<16)-1;
    
    // bounds on the thetas assigned to this group. Note that
    // because theta is periodic, these are defined such that the
    // average value is contained *within* the interval.
    RealArray tmin(width*height);
    RealArray tmax(width*height);
    RealArray mmin(width*height);
    RealArray mmax(width*height);
    
    for (int y = 2; y+2 < fimseg.rows; y++) {
      for (int x = 2; x+2 < fimseg.cols; x++) {

        at::real mag0 = fimMag(y,x);

        if (mag0 < params.minMag) {
          continue;
        }

        mmax[y*width+x] = mag0;
        mmin[y*width+x] = mag0;

        at::real theta0 = fimTheta(y,x);
        tmin[y*width+x] = theta0;
        tmax[y*width+x] = theta0;

        int edgeCost;

        // RIGHT
        edgeCost = this->edgeCost(theta0, mag0, 
                                  fimTheta(y, x+1), 
                                  fimMag(y,x+1));

        if (edgeCost >= 0) {
          edges[nedges++] = 
            (uint64_t(y*width+x)<<IDA_SHIFT) + 
	    (uint64_t(y*width+x+1)<<IDB_SHIFT) + edgeCost;
        }

        // DOWN
        edgeCost = this->edgeCost(theta0, mag0, 
                                  fimTheta(y+1,x), 
                                  fimMag(y+1,x));

        if (edgeCost >= 0) {
          edges[nedges++] = 
            (uint64_t(y*width+x)<<IDA_SHIFT) + 
            (uint64_t((y+1)*width+x)<<IDB_SHIFT) + edgeCost;
        }

        // DOWN & RIGHT
        edgeCost = this->edgeCost(theta0, mag0, 
                                  fimTheta(y+1,x+1), 
                                  fimMag(y+1,x+1));

        if (edgeCost >= 0) {
          edges[nedges++] = 
            (uint64_t(y*width+x)<<IDA_SHIFT) + 
            (uint64_t((y+1)*width+x+1)<<IDB_SHIFT) + edgeCost;
        }



        // DOWN & LEFT
        edgeCost = (x == 0) ? -1 : this->edgeCost(theta0, mag0, 
                                                  fimTheta(y+1,x-1), 
                                                  fimMag(y+1,x-1));

        if (edgeCost >= 0) {
          edges[nedges++] = 
            (uint64_t(y*width+x)<<IDA_SHIFT) + 
            (uint64_t((y+1)*width+x-1)<<IDB_SHIFT) + edgeCost;
        }

        // XXX Would 8 connectivity help for rotated tags?
        // (Probably not much, so long as input filtering
        // hasn't been disabled.)
      }
    }
    
    END_PROFILE(step3a_time);

    START_PROFILE(step3b_time, "sort edges array");

    //sort those edges by weight (lowest weight first).
    countingSortLongArray(edges, nedges, -1, WEIGHT_MASK);

    END_PROFILE(step3b_time);

    START_PROFILE(step3c_time, "merge edges");

    // process edges in order of increasing weight, merging
    // clusters if we can do so without exceeding the
    // params.thetaThresh.
    for (int i = 0; i < nedges; i++) {

      int ida = (int) ((edges[i]>>IDA_SHIFT)&INDEX_MASK);
      int idb = (int) ((edges[i]>>IDB_SHIFT)&INDEX_MASK);

      ida = uf.getRepresentative(ida);
      idb = uf.getRepresentative(idb);

      if (ida == idb) {
        continue;
      }

      int sza = uf.getSetSize(ida);
      int szb = uf.getSetSize(idb);

      at::real tmina = tmin[ida], tmaxa = tmax[ida];
      at::real tminb = tmin[idb], tmaxb = tmax[idb];

      at::real costa = (tmaxa-tmina);
      at::real costb = (tmaxb-tminb);

      // bshift will be a multiple of 2pi that aligns the spans
      // of b with a so that we can properly take the union of
      // them.
      at::real bshift = MathUtil::mod2pi((tmina+tmaxa)/2, 
                                       (tminb+tmaxb)/2) - (tminb+tmaxb)/2;

      at::real tminab = std::min(tmina, tminb + bshift);
      at::real tmaxab = std::max(tmaxa, tmaxb + bshift);

      if (tmaxab - tminab > 2*M_PI) {
        // corner case that's probably not useful to handle correctly. oh well.
        tmaxab = tminab + 2*M_PI;
      }

      at::real mmaxab = std::max(mmax[ida], mmax[idb]);
      at::real mminab = std::min(mmin[ida], mmin[idb]);

      // merge these two clusters?
      at::real costab = (tmaxab - tminab);
      if (costab <= (std::min(costa, costb) + params.thetaThresh/(sza+szb)) &&
          (mmaxab-mminab) <= (std::min(mmax[ida]-mmin[ida],
                                       mmax[idb]-mmin[idb]) +
                              params.magThresh/(sza+szb))) {

        int idab = uf.connectNodes(ida, idb);

        tmin[idab] = tminab;
        tmax[idab] = tmaxab;

        mmin[idab] = mminab;
        mmax[idab] = mmaxab;

      }
    }

    END_PROFILE(step3c_time);

  }

  END_PROFILE(step3_time);

  ///////////////////////////////////////////////////////////
  // Step four. Loop over the pixels again, collecting
  // statistics for each cluster. We will soon fit lines to
  // these points.

  START_PROFILE(step4_time, "build clusters");
  
  ClusterLookup clusters;

  for (int y = 0; y+1 < fimseg.rows; y++) {
    for (int x = 0; x+1 < fimseg.cols; x++) {

      if (uf.getSetSize(y*fimseg.cols+x) < params.minimumSegmentSize) {
        continue;
      }
      
      int rep = (int) uf.getRepresentative(y*fimseg.cols + x);
      
      clusters[rep].push_back(XYW(x,y,fimMag(y,x)));

    }
  }

  if (debug) {

    size_t cidx = 0;
    cv::Mat_<cv::Vec3b> m = cv::Mat_<cv::Vec3b>::zeros(fimseg.size());

    const ScalarVec& ccolors = getCColors();

    for (ClusterLookup::const_iterator i=clusters.begin(); 
         i!=clusters.end(); ++i) {

      const XYWArray& xyw = i->second;

      const cv::Scalar& c = ccolors[cidx % ccolors.size()];

      for (size_t j=0; j<xyw.size(); ++j) {
        const XYW& pi = xyw[j];
        const float fmax = 0.5;
        float f = std::min(pi.w, fmax) / fmax;
        assert( pi.x >= 0 && pi.x < m.cols );
        assert( pi.y >= 0 && pi.y < m.rows );
        m(pi.y, pi.x) = cv::Vec3b(c[0]*f, c[1]*f, c[2]*f);
      }

      ++cidx;

    }

    emitDebugImage(debugWindowName, 
                   4, 0, debugNumberFiles,
                   "Clusters", 
                   m, ScaleNone);

  }

  END_PROFILE(step4_time);

  ///////////////////////////////////////////////////////////
  // Step five. Loop over the clusters, fitting lines (which we
  // call Segments).

  START_PROFILE(step5_time, "fit lines to clusters");

  SegmentArray segments;

  for (ClusterLookup::const_iterator i = clusters.begin();
       i != clusters.end(); ++i) {

    //GLineSegment2D gseg = GLineSegment2D.lsqFitXYW(points);
    GLineSegment2D gseg = lsqFitXYW(i->second);

    // filter short lines
    at::real length = gseg.length();

    if (length < params.minimumLineLength) {
      continue;
    }
    
    Segment* seg = new Segment();
    at::real dy = gseg.p2.y - gseg.p1.y;
    at::real dx = gseg.p2.x - gseg.p1.x;

#ifdef _USE_FAST_MATH_
    seg->theta = MathUtil::atan2(dy, dx);
#else
    seg->theta = atan2(dy, dx);
#endif

    seg->length = length;

    // We add an extra semantic to segments: the vector
    // p1->p2 will have dark on the left, white on the right.
    // To do this, we'll look at every gradient and each one
    // will vote for which way they think the gradient should
    // go. (This is way more retentive than necessary: we
    // could probably sample just one point!)
    at::real flip = 0, noflip = 0;

    for (size_t j=0; j<i->second.size(); ++j) {

      const at::Point& cs = i->second[j].point();

      at::real theta = fimTheta(cs.y, cs.x);
      at::real mag = fimMag(cs.y, cs.x);

      // err *should* be +Math.PI/2 for the correct winding,
      // but if we've got the wrong winding, it'll be around
      // -Math.PI/2.
      at::real err = MathUtil::mod2pi(theta - seg->theta);

      if (err < 0) {
        noflip += mag;
      } else {
        flip += mag;
      }

    }

    if (flip > noflip) {
      seg->theta += M_PI;
    }

    at::real dot = dx*cos(seg->theta) + dy*sin(seg->theta);

    if (dot > 0) {
      seg->x0 = gseg.p2.x; seg->y0 = gseg.p2.y;
      seg->x1 = gseg.p1.x; seg->y1 = gseg.p1.y;
    } else {
      seg->x0 = gseg.p1.x; seg->y0 = gseg.p1.y;
      seg->x1 = gseg.p2.x; seg->y1 = gseg.p2.y;
    }

    if (params.segDecimate) {
      seg->x0 = 2*seg->x0 + .5;
      seg->y0 = 2*seg->y0 + .5;
      seg->x1 = 2*seg->x1 + .5;
      seg->y1 = 2*seg->y1 + .5;
      seg->length *= 2;
    }

    segments.push_back(seg);

  }

  END_PROFILE(step5_time);

  if (debug) {
    cv::Mat rgbu = rgbOrig / 2 + 127;
    const ScalarVec& ccolors = getCColors();
    for (size_t i=0; i<segments.size(); ++i) {
      const Segment* seg = segments[i];
      const cv::Scalar& color = ccolors[ i % ccolors.size() ];
      cv::line( rgbu, 
                cv::Point(seg->x0, seg->y0),
                cv::Point(seg->x1, seg->y1),
                color, 1, CV_AA );
    }
    emitDebugImage(debugWindowName, 
                   5, 0, debugNumberFiles,
                   "Segmented", 
                   rgbu, ScaleNone);
  }

  int width = fim.cols, height = fim.rows;
  
  ////////////////////////////////////////////////////////////////
  // Step six. For each segment, find segments that begin where
  // this segment ends. (We will chain segments together
  // next...) The gridder accelerates the search by building
  // (essentially) a 2D hash table.

  START_PROFILE(step6_time, "find children");

  Gridder gridder(0, 0, width, height, 10);
  for (size_t i=0; i<segments.size(); ++i) {
    Segment* seg = segments[i];
    gridder.add(seg->x0, seg->y0, seg);
  }

  SegmentArray findResults;

  for (size_t i=0; i<segments.size(); ++i) {

    Segment* parent = segments[i];

    gridder.find(parent->x1, parent->y1, 0.5*parent->length, findResults);

    for (size_t j=0; j<findResults.size(); ++j) {
      
      Segment* child = findResults[j];

      if (MathUtil::mod2pi(child->theta - parent->theta) > 0) {
        continue;
      }

      at::Point p;
      if (!intersect(parent, child, p)) {
        continue;
      }

      at::real parentDist = pdist(p, parent->x1, parent->y1);
      at::real childDist = pdist(p, child->x0, child->y0);
      
      if (std::max(parentDist, childDist) > parent->length) {
        continue;
      }

      // everything's okay, this child is a reasonable successor.
      parent->children.push_back(child);


    }
  }

  END_PROFILE(step6_time);

  ////////////////////////////////////////////////////////////////
  // Step seven. Search all connected segments to see if any
  // form a loop of length 4. Add those to the quads list.

  START_PROFILE(step7_time, "build quads");
  
  QuadArray quads;

  Segment* path[5];

  for (size_t i=0; i<segments.size(); ++i) {
    Segment* seg = segments[i];
    path[0] = seg;
    search(opticalCenter, quads, path, seg, 0);
  }

  if (debug) {
    std::cout << "got " << quads.size() << " quads\n";
    cv::Mat rgbu = rgbOrig / 2 + 127;
    const ScalarVec& ccolors = getCColors();
    for (size_t i=0; i<quads.size(); ++i) {
      const Quad& q = *(quads[i]);
      cv::Point2i p[4];
      std::cout << "quad " << i << ":\n";
      for (int j=0; j<4; ++j) { 
        p[j] = q.p[j]; 
        std::cout << "  " << p[j].x << ", " << p[j].y << "\n";
      }
      const cv::Scalar& color = ccolors[ i % ccolors.size() ];
      const cv::Point2i* pp = p;
      int n = 4;
      //cv::fillPoly( rgbu, &pp, &n, 1, color, CV_AA );
      cv::polylines(rgbu, &pp, &n, 1, true, color, 1, CV_AA);
    }
    emitDebugImage(debugWindowName, 
                   7, 0, debugNumberFiles,
                   "Quads", 
                   rgbu, ScaleNone);
  }

  END_PROFILE(step7_time);

  ////////////////////////////////////////////////////////////////
  // Step seven - extra. Try to use corner detection to improve
  // corner points for quads.

  START_PROFILE(step7b_time, "fix quad corners");

  if (params.refineCorners) {
    if (debug) {
      std::cout << "\n\nFIXING CORNERS\n\n\n";
    }
    cv::Mat& fimcorner = origbw;
    const int region_radius = (params.cornerSearchRadius +
                               (params.cornerBlockSize + 1) / 2);
    const int region_width = 2 * region_radius + 1;
    const cv::Point2i region_center(region_radius, region_radius);
    const cv::Size region_size(region_width, region_width);
    const cv::Rect region_template(-region_center, region_size);
    const cv::Rect image_rect(cv::Point2i(0, 0), fimcorner.size());
    cv::Mat whole_masks = cv::Mat::zeros(fimcorner.size(), CV_8UC1);
    cv::Mat mask = cv::Mat::zeros(region_size, CV_8UC1);
    cv::circle(mask, region_center, params.cornerSearchRadius, 1.0, -1);

    QuadArray new_quads;
    for (size_t i = 0; i < quads.size(); ++i) {
      Quad& q = *(quads[i]);
      at::Point p[4];
      for (int j = 0; j < 4; ++j) {
        cv::Point2i curr_corner = p[j] = q.p[j];
        if (debug) {
          std::cout << "Processing corner: " << curr_corner << "\n";
        }
        cv::Rect region_rect = (region_template + curr_corner) & image_rect;
        if (region_rect.size().area() == 0) continue;

        cv::Mat image_region = fimcorner(region_rect);
        cv::Rect mask_rect = region_rect - curr_corner + region_center;
        cv::Mat clipped_mask = mask(mask_rect);
        cv::Mat corner_response;
        if (debug) {
          std::cout << "Region: " << image_region.size().width << "x"
                    << image_region.size().height << " "
                    << "Mask: " << clipped_mask.size().width << "x"
                    << clipped_mask.size().height << " "
                    << "Template: " << region_template.size().width << "x"
                    << region_template.size().height << "\n";
        }
        assert(image_region.size() == clipped_mask.size());
        cv::cornerMinEigenVal(image_region, corner_response,
                              params.cornerBlockSize);
        cv::Point2i best_corner;
        cv::minMaxLoc(corner_response, NULL, NULL, NULL, &best_corner,
                      clipped_mask);
        if (params.refineCornersSubPix) {
          std::vector<cv::Point2f> corners(1, best_corner);
          const cv::Size search_size(params.cornerSearchRadius,
                                     params.cornerSearchRadius);
          // Undocumented restriction of cornerSubPix... grr.
          cv::Size min_size = search_size * 2 + cv::Size(5, 5);
          if (image_region.size().width >= min_size.width &&
              image_region.size().height >= min_size.height) {
            const cv::Size kNoZeroZone(-1, -1);
            cv::TermCriteria crit(cv::TermCriteria::MAX_ITER |
                                  cv::TermCriteria::EPS, 10, 0.1);
            cv::cornerSubPix(image_region, corners, search_size, kNoZeroZone,
                             crit);
            best_corner = corners.front();
          }
        }
        p[j] = best_corner - region_center + curr_corner;
        whole_masks(region_rect) = cv::max(whole_masks(region_rect),
                                           clipped_mask);
      }
      new_quads.push_back(new Quad(p, q.opticalCenter, q.observedPerimeter));
    }

    if (debug) {
      std::cout << "\nFIXED CORNER QUADS\n";
      cv::Mat rgbu = rgbOrig / 2 + 127;
      const ScalarVec& ccolors = getCColors();
      for (size_t i=0; i<new_quads.size(); ++i) {
        const Quad& q = *(new_quads[i]);
        cv::Point2i p[4];
        std::cout << "quad " << i << ":\n";
        for (int j=0; j<4; ++j) {
          p[j] = q.p[j];
          std::cout << "  " << p[j].x << ", " << p[j].y << "\n";
        }
        const cv::Scalar& color = ccolors[ i % ccolors.size() ];
        const cv::Point2i* pp = p;
        int n = 4;
        //cv::fillPoly( rgbu, &pp, &n, 1, color, CV_AA );
        cv::polylines(rgbu, &pp, &n, 1, true, color, 1, CV_AA);
      }
      std::string extra = "";
      if (params.refineCornersSubPix) extra += "-SubPix";
      emitDebugImage(debugWindowName,
                     7, 1, debugNumberFiles,
                     std::string("Fix-Corners") + extra,
                     rgbu, ScaleNone);

      const double kBackgroundFade = 1.0 / 8;
      cv::Mat rgb_background = rgbOrig * kBackgroundFade;
      cv::Mat rgb_remainder = rgbOrig * (1 - kBackgroundFade);
      cv::Mat mask_display;
      rgb_remainder.copyTo(mask_display, whole_masks);
      mask_display += rgb_background;
      emitDebugImage(debugWindowName, 7, 2, debugNumberFiles,
                     "Corners Masks", mask_display, ScaleNone);
      double max;
      cv::Mat whole_response;
      cv::cornerMinEigenVal(fimcorner, whole_response, params.cornerBlockSize);
      cv::minMaxLoc(whole_response, NULL, &max);
      whole_response *= 255 / max;
      emitDebugImage(debugWindowName, 7, 3, debugNumberFiles,
                     "Corners MinEigenVal", whole_response, ScaleNone);
      const int kSobelApertureValue = 3;
      const double kHarrisFreeParameter = 0.04;
      cv::cornerHarris(fimcorner, whole_response, params.cornerBlockSize,
                       kSobelApertureValue, kHarrisFreeParameter);
      cv::minMaxLoc(whole_response, NULL, &max);
      whole_response *= 255 / max;
      emitDebugImage(debugWindowName, 7, 4, debugNumberFiles,
                     "Corners Harris", whole_response, ScaleNone);
    }

    // Add the newly "fixed" quads as alternate candidates to the
    // original candidate quads (over-detection is unlikely).
    quads.insert(quads.end(), new_quads.begin(), new_quads.end());
  }


  END_PROFILE(step7b_time);

  ////////////////////////////////////////////////////////////////
  // Step eight. Decode the quads. For each quad, we first
  // estimate a threshold color to decided between 0 and
  // 1. Then, we read off the bits and see if they make sense.

  START_PROFILE(step8_time, "decode quads");

  detections.clear();
  
  for (size_t i=0; i<quads.size(); ++i) {

    const Quad& quad = *(quads[i]);

    GrayModel blackModel, whiteModel;

    // sample points around the black and white border in
    // order to calibrate our gray threshold. This code is
    // simpler if we loop over the whole rectangle and discard
    // the points we don't want.
    int dd = 2*tagFamily.blackBorder + tagFamily.d;
    
    for (int iy = -1; iy <= dd; iy++) {
      for (int ix = -1; ix <= dd; ix++) {

        at::real y = (iy + .5) / dd;
        at::real x = (ix + .5) / dd;
        
        at::Point pxy = quad.interpolate01(x, y);
        int irx = (int) (pxy.x+.5);
        int iry = (int) (pxy.y+.5);
        
        if (irx < 0 || irx >= width || iry < 0 || iry >= height) {
          continue;
        }
        
        at::real v = fim(iry, irx);
        
        if ((iy == -1 || iy == dd) || (ix == -1 || ix == dd)) {
          // part of the outer white border.
          whiteModel.addObservation(x, y, v);
        } else if ((iy == 0 || iy == (dd-1)) || (ix == 0 || ix == (dd-1))) {
          // part of the outer black border.
          blackModel.addObservation(x, y, v);
        }

      }
    }

    if (debug) {
      
      GrayModel* models[2];
      models[0] = &whiteModel;
      models[1] = &blackModel;

      const char* names[2] = { "white", "black" };

      for (int m=0; m<2; ++m) {
	GrayModel* model = models[m];
	model->compute();
	at::Mat Amat(4, 4, &(model->A[0][0]));
	at::Mat bvec(4, 1, model->b);
	at::Mat xvec(4, 1, model->X);
	std::cout << "for quad " << i << ", model " << names[m] << ":\n";
	std::cout << "  A =\n" << Amat << "\n";
	std::cout << "  b = " << bvec << "\n";
	std::cout << "  x = " << xvec << "\n";
      }



    }

    bool bad = false;
    TagFamily::code_t tagCode = 0;

    if (debug) { std::cout << "\n"; }

    // Try reading off the bits.
    // XXX: todo: multiple samples within each cell and vote?
    for (uint iy = tagFamily.d-1; iy < tagFamily.d; iy--) {

      if (debug) { std::cout << "  "; }

      for (uint ix = 0; ix < tagFamily.d; ix++) {

        at::real y = (tagFamily.blackBorder + iy + .5) / dd;
        at::real x = (tagFamily.blackBorder + ix + .5) / dd;

        at::Point pxy = quad.interpolate01(x, y);
        int irx = (int) (pxy.x+.5);
        int iry = (int) (pxy.y+.5);

        if (irx < 0 || irx >= width || iry < 0 || iry >= height) {
	  if (debug) { std::cout << "quad " << i << " was bad!\n"; }
          bad = true;
          continue;
        }

        at::real threshold = (blackModel.interpolate(x, y) + 
                            whiteModel.interpolate(x,y))*.5;

        at::real v = fim(iry, irx);

        tagCode = tagCode << TagFamily::code_t(1);

        if (v > threshold) {
          tagCode |= TagFamily::code_t(1);
        }

	if (debug) {
	  std::cout << ((v > threshold) ? "##" : "  ");
	}

      }

      if (debug) { std::cout << "\n"; }

    }

    if (debug) { std::cout << "\n"; }

    if (!bad) {

      if (debug) {
	std::cout << "for quad " << i << " got tagCode " << tagCode << "\n";
      }

      TagDetection d;
      tagFamily.decode(d, tagCode);

      // rotate points in detection according to decoded
      // orientation. Thus the order of the points in the
      // detection object can be used to determine the
      // orientation of the target.

      for (int i = 0; i < 4; i++) {
        d.p[(4+i-d.rotation)%4] = quad.p[i];
      }

      // compute the homography (and rotate it appropriately)
      d.homography = quad.H;
      d.hxy = quad.opticalCenter;

      if (true) {
        at::real c = cos(d.rotation*M_PI/2.0);
        at::real s = sin(d.rotation*M_PI/2.0);
        at::real R[9] = { 
          c, -s, 0, 
          s,  c, 0, 
          0,  0, 1 
        };
        at::Mat Rmat(3, 3, R);
        d.homography = d.homography * Rmat;
      }

      if (d.good) {
        d.cxy = quad.interpolate01(.5, .5);
        d.observedPerimeter = quad.observedPerimeter;
        detections.push_back(d);
      }

    }
    
  }

  END_PROFILE(step8_time);

  ////////////////////////////////////////////////////////////////
  // Step nine. Some quads may be detected more than once, due
  // to partial occlusion and our aggressive attempts to recover
  // from broken lines. When two quads (with the same id)
  // overlap, we will keep the one with the lowest error, and if
  // the error is the same, the one with the greatest observed
  // perimeter.

  START_PROFILE(step9_time, "refine detections");

  TagDetectionArray goodDetections;

  // NOTE: allow multiple (non-overlapping) detections of the same target.
  for (size_t i=0; i<detections.size(); ++i) {

    const TagDetection& d = detections[i];

    bool newFeature = true;

    for (size_t odidx = 0; odidx < goodDetections.size(); odidx++) {

      const TagDetection& od = goodDetections[odidx];

      if (d.id != od.id || !detectionsOverlapTooMuch(d, od)) {
        continue;
      }

      // there's a conflict. we must pick one to keep.
      newFeature = false;

      // this detection is worse than the previous one... just don't use it.
      if (d.hammingDistance > od.hammingDistance) {
        continue;
      }

      // otherwise, keep the new one if it either has
      // *lower* error, or has greater perimeter
      if (d.hammingDistance < od.hammingDistance || 
          d.observedPerimeter > od.observedPerimeter) {
        goodDetections[odidx] = d;
      }

    }

    if (newFeature) {
      goodDetections.push_back(d);
      ++num_detections;
    }

  }

  goodDetections.swap(detections);

  END_PROFILE(step9_time);

  START_PROFILE(cleanup_time, "cleanup allocations");
  
  while (!segments.empty()) {
    delete segments.back();
    segments.pop_back();
  }

  while (!quads.empty()) {
    delete quads.back();
    quads.pop_back();
  }

  END_PROFILE(cleanup_time);

  END_PROFILE(total_time);

}

void TagDetector::search(const at::Point& opticalCenter,
                         QuadArray& quads, 
                         Segment* path[5],
                         Segment* parent, 
                         int depth) const {

  if (depth == 4) {

    if (path[4] != path[0]) {
      return;
    }

    at::Point p[4];
    at::real observedPerimeter = 0;

    for (int i=0; i<4; ++i) {
      at::Point pinter;
      if (!intersect(path[i], path[i+1], pinter)) {
        return;
      }
      p[i] = pinter;
      observedPerimeter += path[i]->length;
    }

    // eliminate quads that don't form a simply connected
    // loop (i.e., those that form an hour glass, or wind
    // the wrong way.)

#ifdef _USE_FAST_MATH_
    at::real t0 = MathUtil::atan2(p[1].y - p[0].y, p[1].x - p[0].x);
    at::real t1 = MathUtil::atan2(p[2].y - p[1].y, p[2].x - p[1].x);
    at::real t2 = MathUtil::atan2(p[3].y - p[2].y, p[3].x - p[2].x);
    at::real t3 = MathUtil::atan2(p[0].y - p[3].y, p[0].x - p[3].x);
#else
    at::real t0 = atan2(p[1].y - p[0].y, p[1].x - p[0].x);
    at::real t1 = atan2(p[2].y - p[1].y, p[2].x - p[1].x);
    at::real t2 = atan2(p[3].y - p[2].y, p[3].x - p[2].x);
    at::real t3 = atan2(p[0].y - p[3].y, p[0].x - p[3].x);
#endif
      
    at::real ttheta = ( MathUtil::mod2pi(t1-t0) + MathUtil::mod2pi(t2-t1) +
                      MathUtil::mod2pi(t3-t2) + MathUtil::mod2pi(t0-t3) );
        
    // the magic value is -2*PI. It should be exact,
    // but we allow for (lots of) numeric imprecision.
    if (ttheta < -7 || ttheta > -5) {
      return;
    }

    at::real d0 = pdist(p[0], p[1]);
    at::real d1 = pdist(p[1], p[2]);
    at::real d2 = pdist(p[2], p[3]);
    at::real d3 = pdist(p[3], p[0]);
    at::real d4 = pdist(p[0], p[2]);
    at::real d5 = pdist(p[1], p[3]);
      
    // check sizes
    if (d0 < params.minimumTagSize || d1 < params.minimumTagSize ||
        d2 < params.minimumTagSize || d3 < params.minimumTagSize ||
        d4 < params.minimumTagSize || d5 < params.minimumTagSize) {
      return;
    }
    
    // check aspect ratio
    at::real dmax = std::max(std::max(d0, d1), std::max(d2, d3));
    at::real dmin = std::min(std::min(d0, d1), std::min(d2, d3));
    
    if (dmax > dmin * params.maxQuadAspectRatio) {
      return;
    }
    
    Quad* q = new Quad(p, opticalCenter, observedPerimeter);
    quads.push_back(q);
     
  } else {   

    // Not terminal depth. Recurse on any children that obey the correct handedness.

    for (size_t i=0; i<parent->children.size(); ++i) {

      Segment* child = parent->children[i];
      // (handedness was checked when we created the children)
      
      // we could rediscover each quad 4 times (starting from
      // each corner). If we had an arbitrary ordering over
      // points, we can eliminate the redundant detections by
      // requiring that the first corner have the lowest
      // value. We're arbitrarily going to use theta...
      if (child->theta > path[0]->theta) {
        continue;
      }
      
      path[depth+1] = child;
      search(opticalCenter, quads, path, child, depth + 1);

    }

  }


}


/*
void TagDetector::test() {

  LongArray l;
  long maxv = 0;
  long mask = (1<<16)-1;

  for (int i=0; i<100; ++i) {
    l.push_back(rand() % 12345);
    maxv = std::max(maxv, l.back());
  }

  std::cout << "before: [ ";
  for (int i=0; i<100; ++i) {
    std::cout << l[i] << " ";
  }
  std::cout << "]\n";

  countingSortLongArray(l, l.size(), maxv, mask);

  size_t prev = 0;
  std::cout << "after: [ ";
  for (int i=0; i<100; ++i) {
    if (l[i] < prev) {
      std::cerr << "*** CRAP ***";
    }
    prev = l[i];
    std::cout << l[i] << " ";
  }
  std::cout << "]\n";
  



}
*/
