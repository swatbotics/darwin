#include <cstdio>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <iomanip>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "TagDetector.h"
#include "DebugImage.h"
#include "CameraUtil.h"

#include <set>
#include <map>

#define DEFAULT_TAG_FAMILY "Tag36h11"
const double DEFAULT_FOCAL_LENGTH = 525;
const double DEFAULT_TAG_SIZE = 0.15;

typedef struct BundlerOptions {
  BundlerOptions() :
      params(),
      focal_length(DEFAULT_FOCAL_LENGTH),
      tag_size(DEFAULT_TAG_SIZE),
      family_str(DEFAULT_TAG_FAMILY),
      error_fraction(1){
  }
  TagDetectorParams params;
  double focal_length;
  double tag_size;
  std::string family_str;
  double error_fraction;
} BundlerOptions;

void print_usage(const char* tool_name, FILE* output=stderr) {

  TagDetectorParams p;

  fprintf(output, "\
Usage: %s [OPTIONS] IMAGE1 [IMAGE2 ...]\n\
Run a tool to test tag detection. Options:\n\
 -h              Show this help message.\n\
 -D              Use decimation for segmentation stage.\n\
 -S SIGMA        Set the original image sigma value (default %.2f).\n\
 -s SEGSIGMA     Set the segmentation sigma value (default %.2f).\n\
 -a THETATHRESH  Set the theta threshold for clustering (default %.1f).\n\
 -m MAGTHRESH    Set the magnitude threshold for clustering (default %.1f).\n\
 -V VALUE        Set adaptive threshold value for new quad algo (default %f).\n\
 -N RADIUS       Set adaptive threshold radius for new quad algo (default %d).\n\
 -b              Refine bad quads using template tracker.\n\
 -r              Refine all quads using template tracker.\n\
 -n              Use the new quad detection algorithm.\n\
 -F FPX          Set camera focal length to given quantity in pixels (default %g).\n\
 -s SIZE         Set tag size (including black border) to length in m (default %g).\n\
 -f FAMILY       Look for the given tag family (default \"%s\")\n\
 -e FRACTION     Set error detection fraction (default 1)\n",
          tool_name,
          p.sigma,
          p.segSigma,
          p.thetaThresh,
          p.magThresh,
          p.adaptiveThresholdValue,
          p.adaptiveThresholdRadius,
          DEFAULT_FOCAL_LENGTH,
          DEFAULT_TAG_SIZE,
          DEFAULT_TAG_FAMILY);

  fprintf(output, "Known tag families:");
  TagFamily::StringArray known = TagFamily::families();
  for (size_t i = 0; i < known.size(); ++i) {
    fprintf(output, " %s", known[i].c_str());
  }
  fprintf(output, "\n");
}

BundlerOptions parse_options(int argc, char** argv) {
  BundlerOptions opts;
  const char* options_str = "hdtRvxoDS:s:a:m:V:N:brnf:e:";
  int c;
  while ((c = getopt(argc, argv, options_str)) != -1) {
    switch (c) {
      // Reminder: add new options to 'options_str' above and print_usage()!
      case 'h': print_usage(argv[0], stdout); exit(0); break;
      case 'D': opts.params.segDecimate = true; break;
      case 'S': opts.params.sigma = atof(optarg); break;
      case 's': opts.params.segSigma = atof(optarg); break;
      case 'a': opts.params.thetaThresh = atof(optarg); break;
      case 'm': opts.params.magThresh = atof(optarg); break;
      case 'V': opts.params.adaptiveThresholdValue = atof(optarg); break;
      case 'N': opts.params.adaptiveThresholdRadius = atoi(optarg); break;
      case 'b': opts.params.refineBad = true; break;
      case 'r': opts.params.refineQuads = true; break;
      case 'n': opts.params.newQuadAlgorithm = true; break;
      case 'f': opts.family_str = optarg; break;
      case 'e': opts.error_fraction = atof(optarg); break;
      default:
        fprintf(stderr, "\n");
        print_usage(argv[0], stderr);
        exit(1);
    }
  }
  opts.params.adaptiveThresholdRadius += (opts.params.adaptiveThresholdRadius+1) % 2;
  return opts;
}

//////////////////////////////////////////////////////////////////////

at::Mat vec(double x, double y, double z) {
  return (at::Mat(3,1) << x,y,z);
}




//////////////////////////////////////////////////////////////////////

class TagPoseInfo {
public:

  size_t id;

  size_t xyz_index[3];
  double xyz_value[3];

  size_t rot_index[3];
  double rot_value[3];


};

typedef std::vector<TagPoseInfo> TagPoseInfoArray;

//////////////////////////////////////////////////////////////////////

class Transform {
public:

  at::Mat rvec;
  at::Mat tvec;

  mutable at::Mat R;
  mutable at::Mat J;

public:

  Transform(double px, double py, double pz,
            double tx, double ty, double tz,
            double ux, double uy, double uz) {

    at::Mat p = vec(px,py,pz);
    at::Mat t = vec(tx,ty,tz);
    at::Mat u = vec(ux,uy,uz);

    lookAt(p,t,u);

  }

  Transform(const at::Mat& p, 
            const at::Mat& t, 
            const at::Mat& u) {

    lookAt(p,t,u);
    
  }

  void lookAt(const at::Mat& p,
              const at::Mat& t,
              const at::Mat& u) {
    
    at::Mat z = t-p;
    at::Mat x = z.cross(u);
    at::Mat y = z.cross(x);

    R = at::Mat(3,3);
    R.col(0) = x * (1.0/cv::norm(x));
    R.col(1) = y * (1.0/cv::norm(y));
    R.col(2) = z * (1.0/cv::norm(z));

    R = R.t();

    cv::Rodrigues(R, rvec);

    tvec = -R * p;

  }

  Transform(): rvec(at::Mat::zeros(3,1)),
               tvec(at::Mat::zeros(3,1)) {}

  Transform(const Transform& t): 
    rvec(t.rvec.clone()), 
    tvec(t.tvec.clone()) {}

  Transform(const at::Mat& r, 
            const at::Mat& t):
    rvec(r.clone()), tvec(t.clone()) {}
  
  void transform(const at::Mat& pvec, 
                 at::Mat& result,
                 at::Mat* pJrt=0) const {

    if (R.rows != 3 || R.cols != 3) { R = at::Mat(3,3); }

    at::Mat t = tvec.rows > tvec.cols ? tvec : tvec.t();
    at::Mat p = pvec.rows > pvec.cols ? pvec : pvec.t();
    
    if (pJrt) {

      at::Mat& Jrt = *pJrt;

      cv::Rodrigues(rvec, R, J);

      if (Jrt.rows != 3 || Jrt.cols != 6) { Jrt = at::Mat(3,6); }

      at::Mat Jr = Jrt.colRange(0,3);

      Jrt.colRange(3,6) = at::Mat::eye(3,3);

      for (int i=0; i<3; ++i) {
        Jr.row(i) = (J.colRange(3*i, 3*i+3) * p).t();
      }

    } else {
      cv::Rodrigues(rvec, R);
    }

    result = R*p + t;
    
  }


  Transform inverse() const {

    Transform ti;

    if (R.rows != 3 || R.cols != 3) { R = at::Mat::zeros(3,3); }

    cv::Rodrigues(rvec, R);

    at::Mat t = tvec.rows > tvec.cols ? tvec : tvec.t();

    ti.tvec = -R.t() * t;
    ti.rvec = -rvec;

    return ti;
    
  }

};

Transform operator*(const Transform& t2, const Transform& t1) {

  Transform t3;

  cv::composeRT(t1.rvec, t1.tvec,
                t2.rvec, t2.tvec,
                t3.rvec, t3.tvec);

  return t3;

}

at::Mat operator*(const Transform& t, const at::Mat& p) {
  at::Mat rval;
  t.transform(p, rval);
  return rval;
}

std::ostream& operator<<(std::ostream& ostr, const Transform& t) {
  return ostr << "rvec=" << t.rvec << ", tvec=" << t.tvec;
}

typedef std::vector<Transform> TransformArray;

//////////////////////////////////////////////////////////////////////

at::Point project(const Transform& imgPose,
                  const Transform& tagPose,
                  const at::Point& tagPoint,
                  const double focalLength,
                  at::Mat* dq_dimgrt=0,
                  at::Mat* dq_dtagrt=0) {

  cv::Vec<double, 3> tagPoint3(tagPoint.x, tagPoint.y, 0);
  cv::Mat po(tagPoint3);
  at::Mat pw, pc;

  at::Mat dpc_dtagrt, dpc_dimgrt, dpw_dtagrt;

  bool derivs = ( dq_dtagrt && dq_dimgrt);

  if (derivs) {

    tagPose.transform(po, pw, &dpw_dtagrt);
    imgPose.transform(pw, pc, &dpc_dimgrt);

    dpc_dtagrt = imgPose.R * dpw_dtagrt;

  } else {

    pc = imgPose * (tagPose * po);
      
  }


  // do perspective division
  at::Point q;
  q.x = focalLength*pc(0)/pc(2);
  q.y = focalLength*pc(1)/pc(2);

  at::Mat dq_dpc;


  if (derivs) {

    dq_dpc = at::Mat(2, 3);

    dq_dpc(0,0) = focalLength/pc(2);
    dq_dpc(0,1) = 0;
    dq_dpc(0,2) = -focalLength*pc(0)/(pc(2)*pc(2));

    dq_dpc(1,0) =  0;
    dq_dpc(1,1) =  focalLength/pc(2);
    dq_dpc(1,2) = -focalLength*pc(1)/(pc(2)*pc(2));

    *dq_dtagrt = dq_dpc * dpc_dtagrt;
    *dq_dimgrt = dq_dpc * dpc_dimgrt;

  }

  return q;

}

//////////////////////////////////////////////////////////////////////

cv::Mat detectionImage(const TagFamily& family,
                       size_t id,
                       const Transform& imgPose,
                       const Transform& tagPose,
                       double tagSize,
                       const cv::Size& imageSize,
                       const at::Point& opticalCenter,
                       double focalLength,
                       int scale,
                       int type) {

  Quad q;
  q.opticalCenter = opticalCenter;
      
  const at::Point points[4] = {
    at::Point(-1, -1),
    at::Point( 1, -1),
    at::Point( 1,  1),
    at::Point(-1,  1)
  };
        
  for (int k=0; k<4; ++k) {

    at::Point objPoint = 0.5*tagSize * points[k];
    q.p[k] = project(imgPose, tagPose, objPoint, focalLength) + q.opticalCenter;

  }

  q.recomputeHomography();

  TagDetection d;
  d.id = id;
  d.homography = q.H;
  d.hxy = q.opticalCenter;

  return family.detectionImage( d, imageSize, type, CV_RGB(0,0,0) );

}

//////////////////////////////////////////////////////////////////////

struct TagObs {
  size_t    id; // tag id 
  size_t    tagIndex; // tag index in this data set
  at::Point points[4];  // points in image
  Transform relXform; // relative transformation
};

typedef std::vector<TagObs> TagObsArray;

//////////////////////////////////////////////////////////////////////

class ImageObs {
public:

  std::string imagefile; // filename of image
  cv::Mat image;
  at::Point opticalCenter; // optical center
  size_t biggestTag; // index of largest tag
  TagObsArray obs; // observations

  ImageObs(): biggestTag(0) {}

  void process(const std::string& filename,
               const TagDetector& detector, 
               TagDetectionArray& detections,
               double focalLength,
               double tagSize) {

    image = cv::imread(filename);
    if (image.empty()) { throw std::runtime_error("error reading " + filename); }

    imagefile = filename;
    size_t pos = imagefile.rfind('/');
    if (pos != std::string::npos) {
      imagefile = imagefile.substr(pos+1, imagefile.length()-pos-1);
    }
    
    opticalCenter = at::Point(0.5*image.cols, 0.5*image.rows);
    detector.process(image, opticalCenter, detections);

    std::cout << imagefile << ":\n";

    obs.resize(detections.size());

    double bestPerim = 0;

    for (size_t j=0; j<detections.size(); ++j) {

      if (detections[j].observedPerimeter > bestPerim) {
        bestPerim = detections[j].observedPerimeter;
        biggestTag = j;
      }

      size_t id = detections[j].id;

      obs[j].id = id;
      for (size_t k=0; k<4; ++k) { 
        obs[j].points[k] = detections[j].p[k];
      }

      CameraUtil::homographyToPoseCV(focalLength, focalLength, 
                                     tagSize,
                                     detections[j].homography,
                                     obs[j].relXform.rvec,
                                     obs[j].relXform.tvec);



      std::cout << "  tag " << id << " at " << obs[j].relXform << "\n";


      if (0) {

        cv::Mat img = detectionImage(detector.tagFamily, id, 
                                     obs[j].relXform, Transform(), 
                                     tagSize,
                                     image.size(), 
                                     opticalCenter,
                                     focalLength, 
                                     1,
                                     CV_8UC3);

        cv::Mat super = 0.5*img + 0.5*image;
        cv::imshow("win", super);
        cv::waitKey();

      }



    }

    std::cout << "\n";

  }

  size_t findObs(size_t id) const {
    for (size_t i=0; i<obs.size(); ++i) {
      if (obs[i].id == id) { return i; }
    }
    return size_t(-1);
  }

};

//////////////////////////////////////////////////////////////////////

typedef std::vector<ImageObs> ImageObsArray;
typedef std::vector<size_t> IndexArray;

class DataSet {
public:

  const TagDetector& detector;
  const TagFamily& family;
  
  DataSet(int argc, char** argv, 
          const TagDetector& d,
          double l,
          double s):
    detector(d), family(d.tagFamily), fit_focal_length(false), fit_optical_center(false) {

    focalLength = l;
    tagSize = s;
    num_point_obs = 0;

    TagDetectionArray detections;

    for (int i=optind; i<argc; ++i) {
    
      ImageObs iobs;
      iobs.process(argv[i], detector, detections, focalLength, tagSize);

      if (iobs.obs.size() <= 1) {
        continue;
      }

      for (size_t j=0; j<iobs.obs.size(); ++j) {

        size_t id = iobs.obs[j].id;

        size_t tidx = lookupTagID(id);

        iobs.obs[j].tagIndex = tidx;

        if (tidx == tags.size()) {
          TagPoseInfo tinfo;
          tinfo.id = id;
          tags.push_back(tinfo);
        }

      }

      images.push_back(iobs);

      num_point_obs += 4*iobs.obs.size();

    }


  }

  static std::string trim(const std::string& str) {
    size_t start = 0;
    while (start < str.length() && isspace(str[start])) { ++start; }
    size_t end = str.length();
    while (start < end && isspace(str[end-1])) { --end; }
    return str.substr(start, end-start);
  }

  static void split(const std::string& str,
                    char delim,
                    std::vector<std::string>& tokens) {

    tokens.clear();

    size_t start = 0;

    while (1) { 
      char c = tokens.empty() ? delim : ' ';
      size_t pos = str.find(c, start);
      if (pos == std::string::npos) {
        break;
      }
      std::string tok = trim(str.substr(start, pos-start));
      if (!tok.empty()) { 
        tokens.push_back(tok);
      }
      while (pos < str.length() && str[pos+1] == c) { ++pos; }
      start = pos + 1;
    }

    std::string tok = trim(str.substr(start, str.length()-start));
    if (!tok.empty()) { tokens.push_back(tok); }

  }

  static size_t str2idx(const std::string& str) {
    char* endptr = 0;
    size_t rval = strtol(str.c_str(), &endptr, 10);
    assert(endptr && !*endptr);
    return rval;
  }

  void arrange() {

    //cv::theRNG() = cv::RNG(time(NULL));

    num_shared_params = 0;

    if (fit_focal_length) { num_shared_params += 1; }
    if (fit_optical_center) { num_shared_params += 2; }

    num_img_params = 6 * images.size();
    num_tag_params = 6 * tags.size();

    img_param_offs = num_shared_params;
    tag_param_offs = img_param_offs + num_img_params;
    num_params = tag_param_offs + num_tag_params;

    params = at::Mat::zeros(num_params, 1);
    
    if (fit_focal_length) { params(0) = 1; }

    for (size_t t=0; t<tags.size(); ++t) {
      for (int j=0; j<3; ++j) {
        tags[t].rot_index[j] = 6*t + j + 0;
        tags[t].xyz_index[j] = 6*t + j + 3;
      }
    }

    size_t imagesRemaining = images.size();
    std::vector<int> imageStatus(images.size(), 0);

    std::vector<int> tagSeen(tags.size(), 0);

    while (imagesRemaining) {

      IndexArray unusedImages;
      for (size_t i=0; i<imageStatus.size(); ++i) {
        if (imageStatus[i] == 0) { unusedImages.push_back(i); }
      }

      //cv::randShuffle(unusedImages);

      size_t iidx = -1;

      if (imagesRemaining == images.size()) {
        
        // first image
        iidx = unusedImages[0];

        std::cout << "picked " << images[iidx].imagefile << " first.\n";

      } else {

        for (size_t i=0; i<unusedImages.size(); ++i) {

          // see if this image has any previously seen tags
          size_t ii = unusedImages[i];
          const ImageObs& iobs = images[ii];

          for (size_t j=0; j<iobs.obs.size(); ++j) {

            size_t id = iobs.obs[j].id;
            size_t tidx = iobs.obs[j].tagIndex;

            if (tidx < tags.size() && tagSeen[tidx]) {

              std::cout << "picked " << iobs.imagefile << " because tag " << id << " was seen.\n";
              iidx = ii;

              Transform tagPose;
              
              getTagTransform(tidx, tagPose);
              
              Transform imgPose = iobs.obs[j].relXform * tagPose.inverse();
              
              setImgTransform(iidx, imgPose);

              break;

            }
          }

          if (iidx != -1) { break; }

        }

      }

      if (iidx == -1) {
        std::cout << "quitting because remaining images are separate.\n";
        exit(1);
      }

      const ImageObs& iobs = images[iidx];
      imageStatus[iidx] = 1;
      --imagesRemaining;

      for (size_t j=0; j<iobs.obs.size(); ++j) {

          size_t id = iobs.obs[j].id;
        size_t tidx = iobs.obs[j].tagIndex;

        if (tidx < tags.size() && !tagSeen[tidx]) { 
          tagSeen[tidx] = true;
          std::cout << "  marking tag " << id << " as seen\n";
          
          // compute object world pose from 
          //     camera pose and 
          //     relative pose

          Transform imgPose;
          getImgTransform(iidx, imgPose);
          
          Transform tagPose = imgPose.inverse() * iobs.obs[j].relXform;

          setTagTransform(tidx, tagPose);

          //tagPoses[tidx] = imagePoses[iidx].inverse() * iobs.obs[j].relXform;
          std::cout << "  setting tag " << id << " to pose " << tagPose << "\n";

        }

      }

    }

    std::cout << "\n";

  }


  void randomizeParams() {

    for (int i=0; i<images.size(); ++i) {
      for (int j=0; j<3; ++j) {
        params(img_param_offs + j + 0) += cv::theRNG().uniform(-0.1, 0.1);
        params(img_param_offs + j + 3) += cv::theRNG().uniform(-0.5, 0.5);
      }
    }
  
    for (int p=0; p<num_tag_params; ++p) {
      params(tag_param_offs + p) += cv::theRNG().uniform(-0.1, 0.1);
    }

  }

  void setConstrained() {

    size_t num_newparams = num_shared_params + num_img_params;

    at::Mat newparams(num_newparams, 1);

    for (int i=0; i<num_newparams; ++i) {
      newparams(i) = params(i);
    }

    Transform x;
    for (size_t t=0; t<tags.size(); ++t) {
      getTagTransform(t, x);
      for (int j=0; j<3; ++j) {
        tags[t].xyz_value[j] = x.tvec(j);
        tags[t].rot_value[j] = x.rvec(j);
        tags[t].xyz_index[j] = -1;
        tags[t].rot_index[j] = -1;
      }
    }

    num_tag_params = 0;
    num_params = num_newparams;
    params = newparams;

  }

    

  void setUnconstrained() {

    size_t num_newparams = num_shared_params + num_img_params + 6*tags.size();

    at::Mat newparams(num_newparams, 1);

    for (int i=0; i<tag_param_offs; ++i) {
      newparams(i) = params(i);
    }

    Transform x;

    for (size_t t=0; t<tags.size(); ++t) {
      getTagTransform(t, x);
      for (int j=0; j<3; ++j) {
        newparams(tag_param_offs+6*t+j+0) = x.rvec(j);
        newparams(tag_param_offs+6*t+j+3) = x.tvec(j);
        tags[t].xyz_index[j] = 6*t+j+3;
        tags[t].rot_index[j] = 6*t+j+0;
      }
    }

    num_tag_params = 6*tags.size();
    params = newparams;
    num_params = num_newparams;

  }

  void setRotationUnconstrained() {

    int nrp = 0;
    
    std::vector<double> rp;

    for (int t=0; t<tags.size(); ++t) {
      for (int j=0; j<3; ++j) {
        if (tags[t].rot_index[j] == -1) {
          rp.push_back(tags[t].rot_value[j]);
          tags[t].rot_index[j] = num_tag_params + nrp;
          ++nrp;
        }
      }
    }

    at::Mat newparams(num_params+nrp, 1);

    for (int i=0; i<num_params; ++i) {
      newparams(i) = params(i);
    }
    
    for (int i=0; i<nrp; ++i) {
      newparams(tag_param_offs+num_tag_params+i) = rp[i];
    }

    num_tag_params += nrp;
    params = newparams;
    num_params += nrp;

  }

  void initFromConstraints(const std::string& cfilename) {

    std::ifstream istr(cfilename.c_str());
    if (!istr.is_open()) { 
      std::cout << "error reading " << cfilename << "\n";
      exit(1);
    }

    std::string line;
    std::vector<std::string> tokens;

    std::map<std::string, size_t> cindices;
    std::map<std::string, double> cvalues;

    std::set<size_t> seen_tags;

    while (std::getline(istr, line)) {

      if (line.empty() || line[0] == '#') { 

        continue; 

      } else if (line.find(':') != std::string::npos) {

        split(line, ':', tokens);
        size_t id = str2idx(tokens[0]);
        size_t t = lookupTagID(id);

        if (t >= tags.size()) { 
          std::cerr << "warning: ignoring definition for tag " << id << "\n";
          continue;
        }
        
        TagPoseInfo& tinfo = tags[t];
        assert(tinfo.id == id);

        assert(tokens.size() == 5);


        for (int i=1; i<=3; ++i) {

          const std::string& ti = tokens[i];
          char* endptr = 0;
          double dval = strtod(ti.c_str(), &endptr);

          if (endptr && !*endptr) {  

            // it's a double
            tinfo.xyz_value[i-1] = dval;
            tinfo.xyz_index[i-1] = -1;

          } else {

            // search for the name
            std::map<std::string, size_t>::const_iterator it = 
              cindices.find(ti);

            if (it == cindices.end()) {
              cindices.insert(std::make_pair(ti,  cindices.size()));
              it = cindices.find(ti);
            }
            
            tinfo.xyz_index[i-1] = it->second;

          }
        }

        const std::string& rstr = tokens[4];
        assert(rstr.length() == 2);

        int amap[6][4] =  {

          { 'X', 1, 0, 0 },
          { 'Y', 0, 1, 0 },
          { 'Z', 0, 0, 1 },

          { 'x', -1, 0, 0 },
          { 'y', 0, -1, 0 },
          { 'z', 0, 0, -1 },
          
        };

        at::Mat R = at::Mat::zeros(3,3);

        for (int i=0; i<2; ++i) {
          char c = rstr[i];
          bool found = false;
          for (int k=0; k<6; ++k) {
            if (amap[k][0] == c) {
              found = true;
              for (int j=0; j<3; ++j) {
                R(j, 2*i) = amap[k][j+1];
              }
            }
          }
          if (!found) {
            std::cerr << "bad rstr " << rstr << "\n";
            exit(1);
          }
        }

        R.col(1) = 1*R.col(2).cross(R.col(0));
        
        at::Mat rvec;

        cv::Rodrigues(R, rvec);

        for (int j=0; j<3; ++j) {
          tinfo.rot_index[j] = -1;
          tinfo.rot_value[j] = rvec(j);
        }

        seen_tags.insert(id);

      } else if (line.find('=') != std::string::npos) {

        split(line, '=', tokens);
        assert(tokens.size() == 2);
        
        char* endptr = 0;
        double dval = strtod(tokens[1].c_str(), &endptr);
        assert(endptr && !*endptr);

        cvalues[tokens[0]] = dval;
        
      }
      
    }

    std::cout << "cindices.size() = " << cindices.size() << "\n";

    num_shared_params = 0;

    if (fit_focal_length) { num_shared_params += 1; }
    if (fit_optical_center) { num_shared_params += 2; }

    num_img_params = 6 * images.size();
    num_tag_params = cindices.size();

    img_param_offs = num_shared_params;
    tag_param_offs = img_param_offs + num_img_params;
    num_params = tag_param_offs + num_tag_params;

    params = at::Mat::zeros(num_params, 1);

    if (fit_focal_length) { 
      params(0) = 1;
    }

    pnames.resize(cindices.size());

    // make sure we have a value for each named thing
    for (std::map<std::string, size_t>::const_iterator i=cindices.begin();
         i!=cindices.end(); ++i) {

      std::map<std::string, double>::const_iterator j = cvalues.find(i->first);

      assert(j != cvalues.end());

      params(tag_param_offs + i->second) = j->second;
      pnames[i->second] = i->first;

    }

    std::cout << "all matches up!\n";
    std::cout << "have " << num_tag_params << " tag params\n";
    std::cout << "Jacobian will be " << 2*num_point_obs << " x " << num_params << "\n";

    updateBounds();

    Transform tagPose;

    for (size_t i=0; i<images.size(); ++i) {
      
      const ImageObs& iobs = images[i];

      size_t j = iobs.biggestTag;

      const TagObs& oj = iobs.obs[j];

      // rel = img * tag
      // img = rel * inv(tag)
        
      size_t t = oj.tagIndex;

      assert(t < tags.size());
      assert(tags[t].id == oj.id);

      getTagTransform(t, tagPose);
        
      // rel = img * tag
      // rel * inv(tag) = img * tag * inv(tag)
      // rel = img * inv(tag)
      Transform imgPose = oj.relXform * tagPose.inverse();

      std::cout << "setting pose for " << iobs.imagefile << " to " << imgPose << "\n";
      setImgTransform(i, imgPose);

      if (0) { 

        cv::Mat imgbw = simulateViewFrom(imgPose, cv::Size(640,480), 1);
        cv::Mat sim;
        cv::cvtColor(imgbw, sim, CV_GRAY2RGB);

        cv::Mat img = 0.5*iobs.image + 0.5*sim;

        std::ostringstream ostr;
        ostr << iobs.imagefile << ", tag " << oj.id;

        cv::putText(img, ostr.str(), cv::Point(4, 20),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5, CV_RGB(255,255,255), 1, CV_AA);
        
        cv::imshow("win", img);
        cv::waitKey();

      }


    }

  } 

  cv::Mat simulateViewFrom(const Transform& imgPose,
                           const cv::Size& size,
                           int scale) {


    Transform tagPose;

    
    int x0 = floor(tmin[0]);
    int y0 = floor(tmin[1]);

    int x1 = ceil(tmax[0]);
    int y1 = ceil(tmax[1]);


    at::Point opticalCenter(size.width*0.5, size.height*0.5);

    opticalCenter += getOpticalCenterCorrection();

    cv::Mat_<unsigned char> fakeImage = 
      cv::Mat_<unsigned char>::zeros(size.height, size.width);

    for (int x=x0; x<=x1; ++x) {
      for (int y=y0; y<=y1; ++y) {
        at::Mat p = vec(x,y,0);
        p = imgPose*p;
        if (p(2) > 0) {
          at::Point pp(p(0)/p(2), p(1)/p(2));
          pp = scale*focalLength*pp + opticalCenter;
          cv::circle(fakeImage, pp, 2*scale, CV_RGB(127,127,127), -1, CV_AA);
        }
      }
    }

    for (size_t t=0; t<tags.size(); ++t) {

      getTagTransform(t, tagPose);

      Transform relPose = imgPose * tagPose;

      if (relPose.tvec(2) >= 0) {

        cv::Mat img = detectionImage(family, tags[t].id, 
                                     relPose, Transform(),
                                     tagSize, 
                                     size, 
                                     opticalCenter, 
                                     focalLength * getFocalLengthCorrection(),
                                     1, CV_8UC1);

        fakeImage = cv::max(fakeImage, img);

      }

    }

    return fakeImage;

  }

  void keyboardNavigate(const cv::Size& size,
                        int scale) {

    Transform tagPose;

    int x0 = floor(tmin[0]);
    int y0 = floor(tmin[1]);

    int x1 = ceil(tmax[0]);
    int y1 = ceil(tmax[1]);

    at::Mat pos = vec(0.5*(x0+x1), 0.5*(y0+y1), 0.5);
    at::Mat tgt = vec(x1+1, 0.5*(y0+y1), 0.5);

    at::Mat up = vec(0, 0, 1);

    Transform imgPose( pos, tgt, up );

    while (1) {

      cv::Mat img = simulateViewFrom(imgPose, size, scale);

      cv::imshow("win", img);
      int k = cv::waitKey();
      
      std::cout << "imgPose.rvec = " << imgPose.rvec << "\n";
      std::cout << "imgPose.R =\n" << imgPose.R << "\n";

      if (k == 'w') {
        imgPose = Transform(vec(0,0,0), vec(0,0,-0.1)) * imgPose;
      } else if (k == 'q') {
        imgPose = Transform(vec(0,0.05,0), vec(0,0,0)) * imgPose;
      } else if (k == 's') {
        imgPose = Transform(vec(0,0,0), vec(0,0,0.1)) * imgPose;
      } else if (k == 'a') {
        imgPose = Transform(vec(0,0,0), vec(0.1,0,0)) * imgPose;
      } else if (k == 'd') {
        imgPose = Transform(vec(0,0,0), vec(-0.1,0,0)) * imgPose;
      } else if (k == 'e') {
        imgPose = Transform(vec(0,-0.05,0), vec(0,0,0)) * imgPose;
      } else if (k == 27) {
        break;
      }

    }

  }

  void emitGraph() const {

    std::ofstream ostr("graph.dot");
    ostr << "digraph G {\n"
         << "  nodesep=0.05;\n"
         << "  subgraph images {\n"
         << "    edge [style=invisible,dir=none];\n"
         << "    node [shape=rectangle];\n"
         << "    ";

    for (size_t i=0; i<images.size(); ++i) {
      if (i) {
        ostr << " -> ";
      }
      ostr << '"' << images[i].imagefile << '"';

    }
    ostr << "\n  }\n"
         << "  subgraph tags {\n"
         << "    edge [style=invisible,dir=none];\n"
         << "    ";

    for (size_t t=0; t<tags.size(); ++t) {
      if (t) {
        ostr << " -> ";
      }
      ostr << '"' << tags[t].id << '"';

    }
    ostr << "\n  }\n\n";
    ostr << "  edge [constraint=false,dir=none];\n";
    ostr << "  splines=false;\n";

    for (size_t i=0; i<images.size(); ++i) {
      for (size_t j=0; j<images[i].obs.size(); ++j) {
        ostr << "  \"" << images[i].imagefile << "\":e -> \"" << images[i].obs[j].id << "\":w\n";
      }
    }

    ostr << "}\n";

    ostr.close();

    system("dot -Tpdf graph.dot > graph.pdf");

  }

  double getFocalLengthCorrection() const {
    return fit_focal_length ? params(0) : 1;
  }

  at::Point getOpticalCenterCorrection() const {
    if (fit_optical_center) {
      int i = fit_focal_length ? 1 : 0;
      return at::Point(params(i), params(i+1));
    } else {
      return at::Point(0,0);
    }
  }

  double computeReprojectionError(at::Mat* errvec, at::Mat* J) const {

    size_t obsidx = 0;
    double total = 0;

    int m = 2*num_point_obs;
    int n = num_params;

    assert(params.rows == num_shared_params + num_img_params + num_tag_params);

    Transform imgPose, tagPose;

    if ( errvec && (errvec->rows != m || errvec->cols != 1 ) ) {
      *errvec = at::Mat(2*num_point_obs, 1);
    }

    if (J && (J->rows != m || J->cols != n) ) {
      *J = at::Mat(m, n);
    }
    if (J) { *J = 0; }

    at::Mat dq_dtagrt, dq_dimgrt, dq_dparams, dtagrt_dparams;

    for (size_t i=0; i<images.size(); ++i) {

      const ImageObs& iobs = images[i];
      getImgTransform(i, imgPose);

      for (size_t j=0; j<iobs.obs.size(); ++j) {

        const TagObs& oj = iobs.obs[j];

        size_t t = oj.tagIndex;
        getTagTransform(t, tagPose);

        if (J) { getTagJacobian(t, dtagrt_dparams); }

        const at::Point points[4] = {
          at::Point(-1, -1),
          at::Point( 1, -1),
          at::Point( 1,  1),
          at::Point(-1,  1)
        };

        for (int k=0; k<4; ++k) {

          at::Point objPoint = 0.5*tagSize * points[k];
          
          at::Point q = project(imgPose,
                                tagPose, 
                                objPoint,
                                focalLength,
                                J ? &dq_dimgrt : 0,
                                J ? &dq_dtagrt : 0);

          if (fit_focal_length) { 
            double c = getFocalLengthCorrection();
            if (J) { 
              (*J)(obsidx+0, 0) = -q.x;
              (*J)(obsidx+1, 0) = -q.y;
              dq_dimgrt *= c;
              dq_dtagrt *= c;
            }
            q *= c;
          }

          q += iobs.opticalCenter;

          if (fit_optical_center) { 
            q += getOpticalCenterCorrection();
            int i = fit_focal_length ? 1 : 0;
            if (J) { 
              (*J)(obsidx+0, i+0) = -1;
              (*J)(obsidx+1, i+1) = -1;
            }
          }
          
          at::Point y = oj.points[k]; 

          at::Point err = y - q;
          
          if (J) { 

            int jcol_img = img_param_offs + 6*i;
            int jcol_params = tag_param_offs;

            at::Mat dq_dparams = dq_dtagrt * dtagrt_dparams;

            assert(obsidx < J->rows);
            assert(jcol_img + 6 <= J->cols);
            assert(jcol_params + num_tag_params <= J->cols);

            at::Mat Ji(*J, cv::Rect(jcol_img, obsidx, 6, 2));
            at::Mat Jp(*J, cv::Rect(jcol_params, obsidx, num_tag_params, 2));

            dq_dimgrt = -dq_dimgrt;
            dq_dparams = -dq_dparams;

            dq_dimgrt.copyTo(Ji);
            dq_dparams.copyTo(Jp);

          }

          if (errvec) {
            (*errvec)(obsidx+0) = err.x;
            (*errvec)(obsidx+1) = err.y;
          }

          obsidx += 2;

          total += 0.5*err.dot(err);

        }

      }

    }

    return total;

  }

  void getImgTransform(size_t i, Transform& x) const {
    int j = img_param_offs + 6*i;
    params.rowRange(j, j+3).copyTo(x.rvec);
    params.rowRange(j+3, j+6).copyTo(x.tvec);
  }

  void getTagTransform(size_t t, Transform& x) const {
    
    const TagPoseInfo& tinfo = tags[t];

    for (int j=0; j<3; ++j) {

      if (tinfo.xyz_index[j] == -1) {
        x.tvec(j) = tinfo.xyz_value[j];
      } else {
        x.tvec(j) = params(tag_param_offs + tinfo.xyz_index[j]);
      }

      if (tinfo.rot_index[j] == -1) {
        x.rvec(j) = tinfo.rot_value[j];
      } else {
        x.rvec(j) = params(tag_param_offs + tinfo.rot_index[j]);
      }

    }

  }

  void setTagTransform(size_t t, const Transform& x) {
    
    TagPoseInfo& tinfo = tags[t];

    for (int j=0; j<3; ++j) {

      if (tinfo.xyz_index[j] == -1) {
        tinfo.xyz_value[j] = x.tvec(j);
      } else {
        params(tag_param_offs + tinfo.xyz_index[j]) = x.tvec(j);
      }

      if (tinfo.rot_index[j] == -1) {
        tinfo.rot_value[j] = x.rvec(j);
      } else {
        params(tag_param_offs + tinfo.rot_index[j]) = x.rvec(j);
      }

    }

  }

  void getTagJacobian(size_t t, at::Mat& dtagrt_dparams) const {

    assert(params.rows == num_shared_params + num_img_params + num_tag_params);

    dtagrt_dparams = at::Mat::zeros(6, num_tag_params);

    for (int j=0; j<3; ++j) {

      size_t ri = tags[t].rot_index[j];
      if (ri != -1) {
        dtagrt_dparams[j+0][ri] = 1;
      }

      size_t ti = tags[t].xyz_index[j];
      if (ti != -1) {
        dtagrt_dparams[j+3][ti] = 1;
      }

    }

  }

  void setImgTransform(size_t i, const Transform& x) {
    int j = img_param_offs + 6*i;
    x.rvec.copyTo(params.rowRange(j, j+3));
    x.tvec.copyTo(params.rowRange(j+3, j+6));
  }


  size_t lookupTagID(size_t id) {
    for (size_t i=0; i<tags.size(); ++i) {
      if (tags[i].id == id) { return i; }
    }
    return tags.size();
  }

  void lmMinimize(int maxiter, int maxnd) {

    at::Mat errvec, J;

    double lambda = 1e5;

    double lastErr = DBL_MAX;
    at::Mat lastParams = params.clone();

    int nd = 0;

    for (int iter=0; iter<maxiter; ++iter) {

      double err = computeReprojectionError(&errvec, &J);

      std::cout << "iter " << iter << "\n";
      std::cout << "sse = " << err << "\n";
      std::cout << "lambda = " << lambda << "\n";
      std::cout << "nd = " << nd << "\n";

      // we want err < lastErr
      // which means we want reldecrease < -1e-7
      double reldecrease = (lastErr-err)/err;

      if (reldecrease > 1e-5) {
        nd = 0;
      } else {
        ++nd;
        if (nd > maxnd) {
          break;
        }
      }

      if (err > lastErr) {
        lastParams.copyTo(params);
        lambda *= 10;
        continue;
      }


      at::Mat A = J.t() * J;
      for (size_t i=0; i<A.rows; ++i) { 
        A(i,i) *= (1 + lambda);
      }
    
      at::Mat b = J.t() * errvec;
    
      at::Mat delta;

      bool ok = cv::solve(A, b, delta, cv::DECOMP_CHOLESKY);
      std::cout << "ok = " << ok << "\n";

      std::cout << "grad mag = " << cv::norm(b) << "\n";

      lambda *= 0.5;
      lastErr = err;
      params.copyTo(lastParams);
      params -= delta;

    }

    updateBounds();

  }

  void updateBounds() {

    Transform tagPose;

    for (size_t t=0; t<tags.size(); ++t) {
      getTagTransform(t, tagPose);
      for (int j=0; j<3; ++j) {
        double tj = tagPose.tvec(j);
        if (t == 0 || tj < tmin[j]) { tmin[j] = tj; }
        if (t == 0 || tj > tmax[j]) { tmax[j] = tj; }
      }
    }

  }

  void tour() {

    Transform imgPose;

    for (size_t i=0; i<images.size(); ++i) {
      
      getImgTransform(i, imgPose);

      cv::Mat imgbw = simulateViewFrom(imgPose, cv::Size(640,480), 1);
      cv::Mat sim;
      cv::cvtColor(imgbw, sim, CV_GRAY2RGB);

      cv::Mat img = 0.5*images[i].image + 0.5*sim;
      
      cv::imshow("win", img);
      cv::waitKey();

    }


  }


  void testJacobian() {


    at::Mat errvec, J, e1, e0, Jn;

    computeReprojectionError(&errvec, &J);

    Jn = at::Mat::zeros(J.rows, J.cols);
  

    for (int j=0; j<params.rows; ++j) {

      double pj = params(j);

      double h = j < 6*images.size() ? 1e-4 : 0.1;

      params(j) = pj + h;
      computeReprojectionError(&e1, 0);

      params(j) = pj - h;
      computeReprojectionError(&e0, 0);

      params(j) = pj;

      for (int i=0; i<e1.rows; ++i) {
        Jn(i,j) = (e1(i) - e0(i)) / (2*h);
      }

    }


    // print out row by row
    for (int i=0; i<J.rows; ++i) {
      std::cerr << " J.row(" << i << ") = " << J.row(i) << "\n";
      std::cerr << "Jn.row(" << i << ") = " << Jn.row(i) << "\n\n";
    }


  }

  ImageObsArray images;

  double focalLength;
  double tagSize;

  //TransformArray imagePoses;
  //TransformArray tagPoses;

  TagPoseInfoArray tags;

  std::vector<std::string> pnames;

  double tmin[3];
  double tmax[3];

  at::Mat params;

  bool fit_focal_length;
  bool fit_optical_center;

  size_t num_point_obs;
  size_t num_shared_params; 
  size_t num_img_params;
  size_t num_tag_params;

  size_t img_param_offs;
  size_t tag_param_offs;
  size_t num_params;

};


void testTransforms() {

  cv::Vec<double, 3> a(.1, .2, .3);
  cv::Vec<double, 3> b(0.3, 0.1, -.2);

  cv::Mat amat(a);
  cv::Mat bmat(b);

  Transform t1(amat, bmat);
  Transform t2(bmat, amat);

  std::cout << "t1 = " << t1 << "\n";
  std::cout << "t2 = " << t2 << "\n";

  Transform t3 = t2 * t1;

  std::cout << "t3 = " << t3 << "\n";

  cv::Vec<double, 3> p(10,20,30);
  cv::Mat pmat(p);

  std::cout << "t1 * p = " << t1*pmat << "\n";
  std::cout << "t2*t1 * p = " << t2*(t1*pmat) << "\n";
  std::cout << "t3 * p = " << t3*pmat << "\n";

  std::cout << "t2.inverse() * t3 * p = " << t2.inverse()*(t3*pmat) << "\n";

  std::cout << "t1 * t1.inverse() = " << t1*t1.inverse() << "\n";
  std::cout << "t1.inverse() * t1 = " << t1.inverse()*t1 << "\n";


}

//////////////////////////////////////////////////////////////////////

void testDeriv() {

  //cv::Vec<double,3> rv(.1, .2, .3);
  cv::Vec<double,3> rv(M_PI/4, 0, 0);
  cv::Vec<double,3> pv(4, 0, 0);

  cv::Mat rm(rv);
  cv::Mat pm(pv);

  at::Mat r = rm;
  at::Mat p = pm;

  at::Mat R(3,3);
  at::Mat J;

  cv::Rodrigues(r, R, J);

  std::cout << "r = " << r << "\n";
  std::cout << "R =\n" << R << "\n";

  at::Mat dRpdr_f = at::Mat::zeros(3,3);

  dRpdr_f.row(0) = (J.colRange(0, 3) * p).t();
  dRpdr_f.row(1) = (J.colRange(3, 6) * p).t();
  dRpdr_f.row(2) = (J.colRange(6, 9) * p).t();

  std::cout << "formula:\n" << dRpdr_f << "\n";

  at::Mat Rp = R * p;

  at::Mat dRpdr(3,3);
  double h = 1e-4;
  
  for (int i=0; i<3; ++i) {

    double old = r(i);

    r(i) = old + h;
    cv::Rodrigues(r, R);
    at::Mat rp1 = R*p;

    r(i) = old - h;
    cv::Rodrigues(r, R);
    at::Mat rp0 = R*p;

    for (int j=0; j<3; ++j) {
      dRpdr(j, i) = (rp1(j) - rp0(j)) / (2*h);
    };

    r(i) = old;

  }
  
  std::cout << "numerical:\n" << dRpdr << "\n";

  at::Mat px = at::Mat::zeros(3,3);
  px(0,1) = -( px(1,0) = Rp(2) );
  px(2,0) = -( px(0,2) = Rp(1) );
  px(1,2) = -( px(2,1) = Rp(0) );

  std::cout << "px =\n" << px << "\n";
  std::cout << R * px << "\n";
  std::cout << R.t() * px << "\n";
  std::cout << R * px * R.t() << "\n";
  std::cout << R.t() * px * R << "\n";

  

  

}



void testDeriv2() {

  cv::Vec<double, 3> av(.1, .2, .3);
  cv::Vec<double, 3> bv(0.3, 0.1, -.2);
  cv::Vec<double, 3> cv(4, 5, 6);

  cv::Mat am(av);
  cv::Mat bm(bv);
  cv::Mat cm(cv);

  Transform t(am, bm);

  at::Mat Jr, result;
  at::Mat Jn(3,3);

  t.transform(cm, result, &Jr);

  double h = 1e-4;

  for (int j=0; j<3; ++j) {
    
    double old = t.rvec(j);

    t.rvec(j) = old + h;
    at::Mat r1 = t * cm;

    t.rvec(j) = old - h;
    at::Mat r0 = t * cm;

    t.rvec(j) = old;

    Jn.col(j) = (r1-r0) * (0.5/h);

  }

  std::cout << "Jr:\n" << Jr << "\n";
  std::cout << "Jn:\n" << Jn << "\n";
  

}

void composeP(const at::Mat& t2t1, // 12x1
              const at::Mat& p, // 3x1
              at::Mat& q, // 3x1
              at::Mat* pJ = 0) { // 3x12

  at::Mat params = t2t1.rows > t2t1.cols ? t2t1 : t2t1.t();
  at::Mat pt = p.rows > p.cols ? p : p.t();
  
  Transform t2(params.rowRange(0, 3), params.rowRange(3, 6));
  Transform t1(params.rowRange(6, 9), params.rowRange(9, 12));

  if (!pJ) { 

    q = t2 * (t1 * pt);

  } else {

    at::Mat& J = *pJ;
    if (J.rows != 3 || J.cols != 12) { J = at::Mat(3,12); }

    at::Mat Jrt2 = J.colRange(0, 6);

    at::Mat t1p, Jrt1;

    t1.transform(p, t1p, &Jrt1);
    t2.transform(t1p, q, &Jrt2);
    
    J.colRange(6, 12) = t2.R * Jrt1;


  }
               
}

void testDeriv3() {

  at::Mat t21(12,1), p(3,1);
  at::Mat q, J, Jn(3, 12);

  cv::theRNG().fill(t21, cv::RNG::UNIFORM, -1, 1);
  cv::theRNG().fill(p, cv::RNG::UNIFORM, -1, 1);

  composeP(t21, p, q, &J);

  std::cout << "t21 = " << t21 << "\n";
  std::cout << "p = " << p << "\n";
  std::cout << "q = " << q << "\n";
  std::cout << "J =\n" << J << "\n";

  double h = 1e-3;

  for (int j=0; j<12; ++j) {

    double old = t21(j);
    at::Mat q1, q0;

    t21(j) = old + h;
    composeP(t21, p, q1);

    t21(j) = old - h;
    composeP(t21, p, q0);

    t21(j) = old;

    Jn.col(j) = (q1-q0) * (0.5/h);

  }

  std::cout << "Jn =\n" << Jn << "\n";
    

};


void testReproj() {

  cv::Vec<double, 3> av(.1, .2, .3);
  cv::Vec<double, 3> bv(-.3, -.1, -.2);


  cv::Mat am(av);
  cv::Mat bm(bv);

  Transform tagPose(am, bm);
  Transform imgPose(bm, am);

  at::Point tagPoint(0.15, 0.15);
  at::real focalLength = 525;

  at::Mat Jtag(2,6), Jimg(2,6);

  at::Mat Jnt(2,6), Jni(2,6);

  at::Point q = project(imgPose, tagPose, 
                        tagPoint, focalLength,
                        &Jimg, &Jtag);


  std::cout << "q = " << q << "\n";

  for (int which = 0; which < 4; ++which) {

    at::Mat& m = (which == 0) ? tagPose.rvec :
      (which == 1) ? tagPose.tvec : 
      (which == 2) ? imgPose.rvec :
      imgPose.tvec;

    at::Mat& Jn = (which < 2) ? Jnt : Jni;

    double h = 1e-4;

    for (int j=0; j<3; ++j) {

      double old = m(j);
      m(j) = old + h;
      
      at::Point q1 = project(imgPose, tagPose, tagPoint, focalLength);

      m(j) = old - h;

      at::Point q0 = project(imgPose, tagPose, tagPoint, focalLength);

      m(j) = old;

      int jj = j + 3*(which % 2);

      Jn(0,jj) = (q1.x-q0.x)/(2*h);
      Jn(1,jj) = (q1.y-q0.y)/(2*h);

    }

  }

  std::cout << "Jtag = \n" << Jtag << "\n";
  std::cout << "Jnt = \n" << Jnt << "\n\n";

  std::cout << "Jimg = \n" << Jimg << "\n";
  std::cout << "Jni =\n" << Jni << "\n";

}



//////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {

  //testTransforms();
  //testDeriv();
  //testDeriv2();
  //testDeriv3();
  //testReproj();
  //return 0;

  //////////////////////////////////////////////////////////////////////

  const std::string win = "Tag test";

  BundlerOptions opts = parse_options(argc, argv);


  std::cout << "foo\n";
  std::cout << "opts.family_str = " << opts.family_str << "\n";

  TagFamily family(opts.family_str);

  std::cerr << "foo\n";

  if (opts.error_fraction >= 0 && opts.error_fraction <= 1) {
    family.setErrorRecoveryFraction(opts.error_fraction);
  }
  std::cerr << "set error_fraction\n";

  TagDetector detector(family, opts.params);

  std::cerr << "made detector!\n";

  DataSet dataset(argc, argv, detector, opts.focal_length, opts.tag_size);

  //dataset.fit_optical_center = true;
  //dataset.fit_focal_length = true;

  dataset.emitGraph();

  dataset.initFromConstraints("newconstraints.txt");

  //dataset.arrange();
  //dataset.setConstrained();

  dataset.lmMinimize(1000, 10);

  if (dataset.num_params <= 30) {
    dataset.testJacobian();
  }
  
  
  //dataset.setRotationUnconstrained();
  //dataset.setUnconstrained();
  dataset.randomizeParams();
  dataset.lmMinimize(1000, 10);

  if (dataset.num_tag_params == dataset.pnames.size()) {
    for (size_t p=0; p<dataset.num_tag_params; ++p) {
      std::cout << dataset.pnames[p] << " = " << dataset.params(dataset.tag_param_offs+p) << "\n";
    }
  }

  if (dataset.fit_focal_length) {
    std::cout << "focal length: " << dataset.focalLength * dataset.getFocalLengthCorrection() << "\n";
  }

  if (dataset.fit_optical_center) {
    std::cout << "optical center: " << dataset.getOpticalCenterCorrection() << "\n";
  }

  dataset.tour();
  dataset.keyboardNavigate(cv::Size(640, 480), 1);


}
