#include "TagFamily.h"
#include <iostream>
#include <fstream>
#include <math.h>
#include <assert.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

TagFamily::TagFamily() {
}

TagFamily::TagFamily(const std::string& name) {
  init(name);
}

TagFamily::TagFamily(int bits, 
                     int minimumHammingDistance, 
                     size_t count, 
                     const code_t* data) {

  init(bits, minimumHammingDistance, count, data);

}
  
void TagFamily::init(int b,
                     int mhd,
                     size_t count,
                     const code_t* data) {

  whiteBorder = 1;
  blackBorder = 1;

  bits = b;

  d = (int) sqrt(bits);
  assert(d*d == bits);

  minimumHammingDistance = std::max(mhd, 1);
  errorRecoveryBits = std::min(minimumHammingDistance-1, uint(1));
  codes.assign(data, data+count);
  
}

void TagFamily::setErrorRecoveryBits(int b) {
  errorRecoveryBits = b;
}

void TagFamily::setErrorRecoveryFraction(at::real v) {
  errorRecoveryBits = (int) (((int) (minimumHammingDistance-1)/2)*v);
}

TagFamily::code_t TagFamily::rotate90(TagFamily::code_t w, uint d) {

  code_t wr = 0;

  for (uint r = d-1; r < d; r--) {
    for (uint c = 0; c < d; c++) {
      code_t b = r + d*c;

      wr = wr << code_t(1);

      if ((w & (code_t(1) << b))!=0)
        wr |= code_t(1);

    }
  }

  return wr;

}

/** Compute the hamming distance between two code_ts. **/
at::uint TagFamily::hammingDistance(TagFamily::code_t a, TagFamily::code_t b) {
  return popCount(a^b);
}

/** How many bits are set in the code_t? **/
at::uint TagFamily::popCountReal(TagFamily::code_t w) {
  uint cnt = 0;
  while (w != 0) {
    w &= (w-code_t(1));
    cnt++;
  }
  return cnt;
}

const TagFamily::ByteArray& TagFamily::getPopCountTable() {

  static ByteArray tbl;

  if (tbl.empty()) {
    tbl.resize(1<<popCountTableShift);
    for (size_t i = 0; i < tbl.size(); i++) {
      uint pcr = popCountReal(i);
      assert(pcr < uint(256));
      tbl[i] = pcr;
    }
  }

  return tbl;

}

at::uint TagFamily::popCount(TagFamily::code_t w) {

  uint count = 0;
  
  const ByteArray& tbl = getPopCountTable();
  
  while (w != 0) {
    count += tbl[ w & (tbl.size() - 1) ];
    w >>= popCountTableShift;
  }
  
  return count;

}

/** Given an observed tag with code 'rcode', try to recover the
 * id. The corresponding fields of TagDetection will be filled
 * in. **/
void TagFamily::decode(TagDetection& det, TagFamily::code_t rcode) const {

  size_t bestid = -1;
  uint besthamming = uint(-1);
  int  bestrotation = 0;
  code_t bestcode = 0;

  code_t rcodes[4];

  rcodes[0] = rcode;
  rcodes[1] = rotate90(rcodes[0], d);
  rcodes[2] = rotate90(rcodes[1], d);
  rcodes[3] = rotate90(rcodes[2], d);

  for (size_t id = 0; id < codes.size(); id++) {

    for (int rot = 0; rot < 4; rot++) {
      uint thishamming = hammingDistance(rcodes[rot], codes[id]);
      if (thishamming < besthamming) {
        besthamming = thishamming;
        bestrotation = rot;
        bestid = id;
        bestcode = codes[id];
      }
    }
  }

  det.id = bestid;
  det.hammingDistance = besthamming;
  det.rotation = bestrotation;
  det.good = (det.hammingDistance <= errorRecoveryBits);
  det.obsCode = rcode;
  det.code = bestcode;

}


/** Return the dimension of the tag including borders when we render it.**/
at::uint TagFamily::getTagRenderDimension() const {
  return whiteBorder*2 + blackBorder*2 + d;
}


void TagFamily::printHammingDistances() const {
  
  //int hammings[] = new int[d*d+1];
  std::vector<uint> hammings(d*d+1, 0);
  
  for (size_t i = 0; i < codes.size(); i++) {
    code_t r0 = codes[i];
    code_t r1 = rotate90(r0, d);
    code_t r2 = rotate90(r1, d);
    code_t r3 = rotate90(r2, d);
    
    for (size_t j = i+1; j < codes.size(); j++) {
      
      uint d = std::min(std::min(hammingDistance(r0, codes[j]),
				 hammingDistance(r1, codes[j])),
			std::min(hammingDistance(r2, codes[j]),
				 hammingDistance(r3, codes[j])));
      
      assert(d < hammings.size());
      hammings[d]++;
    }
  }
  
  for (size_t i = 0; i < hammings.size(); i++) {
    std::cout << i << "    " << hammings[i] << "\n";
  }
  
}

cv::Mat_<TagFamily::byte> TagFamily::makeImage(size_t id) const {

  code_t v = codes[id];
  
  uint width = getTagRenderDimension();
  uint height = getTagRenderDimension();
  
  cv::Mat_<byte> rval(height, width);
  const byte white = 255;
  const byte black = 0;
  
  // Draw the borders.  It's easier to do this by iterating over
  // the whole tag than just drawing the borders.
  for (uint y = 0; y < width; y++) {
    for (uint x = 0; x < height; x++) {
      if (y < whiteBorder || y+whiteBorder >= height ||
          x < whiteBorder || x+whiteBorder >= width)
        rval(y,x) = white;
      else
        rval(y,x) = black;
    }
  }

  int bb = whiteBorder + blackBorder;
  
  // Now, draw the payload.
  for (uint y = 0; y < d; y++) {
    for (uint x = 0; x < d; x++) {
      if ((v&(code_t(1)<<code_t(bits-1)))!=0)
        rval(y+bb,x+bb) = white;
      else
        rval(y+bb,x+bb) = black;
      
      v = v<<code_t(1);
    }
  }
  
  return rval;

}


void TagFamily::writeAllImages(const std::string& dirpath) const {

  std::string ddir = dirpath;
  if (ddir.length() && ddir[ddir.length()-1] != '/') {
    ddir += "/";
  }

  for (size_t i = 0; i < codes.size(); i++) {
    cv::Mat im = makeImage(i);
    char buf[1024];
    snprintf(buf, 1024, "%stag%02d_%02d_%05u.png",
             ddir.c_str(),
             bits,
             minimumHammingDistance,
             uint(i));
    cv::imwrite( buf, im );
  }
}

void TagFamily::writeAllImagesSVG(const std::string& dirpath) const {

  std::string ddir = dirpath;
  if (ddir.length() && ddir[ddir.length()-1] != '/') {
    ddir += "/";
  }

  for (size_t i = 0; i < codes.size(); i++) {
    char buf[1024];
    snprintf(buf, 1024, "%stag%02d_%02d_%05u.svg",
             ddir.c_str(),
             bits,
             minimumHammingDistance,
             uint(i));
    writeImageSVG(buf, i);
  }
}

void TagFamily::writeImageSVG(const std::string& filename, size_t id) const {
  
  std::ofstream ostr(filename.c_str());
  if (!ostr.is_open()) { return; }

  int w = 612;
  int h = 792;
  at::real draw_size = std::min(w,h)-72;
  at::real img_size = d + 2*blackBorder + 2*whiteBorder;

  at::real scl = draw_size / img_size;
  at::real sw = 1;

  at::real bs = scl*(d + 2*blackBorder);
  at::real bx = 0.5*w - 0.5*bs;
  at::real by = 0.5*h - 0.5*bs;
  at::real x0 = 0.5*w - 0.5*d*scl;
  at::real y0 = 0.5*h - 0.5*d*scl;

  ostr << "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n"
       << "<!DOCTYPE svg PUBLIC \"-//W3C//DTD SVG 1.1//EN\" \"http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd\">\n"
       << "<svg version=\"1.1\" id=\"Layer_1\" xmlns=\"http://www.w3.org/2000/svg\" xmlns:xlink=\"http://www.w3.org/1999/xlink\"\n"
       << "     width=\""<<w<<"px\" height=\""<<h<<"px\">\n"
       << "<rect x=\""<<bx<<"\" y=\""<<by<<"\" fill=\"#000000\" stroke=\"none\" width=\""<<bs<<"\" height=\""<<bs<<"\"/>\n";

  code_t v = codes[id];
  
  for (uint y=0; y<d; ++y) {
    for (uint x=0; x<d; ++x) {
      if ((v&(code_t(1)<<code_t(bits-1)))!=0) {
        ostr << "<rect x=\""<<(x0+scl*x)<<"\" y=\""<<(y0+scl*y)<<"\" fill=\"#ffffff\" stroke=\"#ffffff\" stroke-width=\""<<sw<<"\" width=\""<<scl<<"\" height=\""<<scl<<"\"/>\n";
        
      }
      v = v<<code_t(1);
    }
  }

  ostr << "</svg>";

}


void TagFamily::writeAllImagesPostScript(const std::string& filepath) const {

  int sz = d + 2*whiteBorder + 2*blackBorder;
  int b = whiteBorder;
  int m = whiteBorder + blackBorder;

  //BufferedWriter outs = new BufferedWriter(new FileWriter(filepath));
  std::ofstream ostr(filepath.c_str());
  if (!ostr.is_open()) { return; }

  ostr << "/pagewidth 8.5 72 mul def\n"
       << "/pageheight 11 72 mul def\n"
       << "/b {\n"
       << "newpath moveto\n"
       << "1 0 rlineto\n"
       << "0 1 rlineto\n"
       << "-1 0 rlineto\n"
       << "closepath\n"
       << "gsave\n"
       << "fill\n"
       << "grestore\n"
       << "stroke\n"
       << "} def\n"
       << "/sm {\n"
       << "gsave\n"
       << "pagewidth 2 div pageheight 2 div translate\n"
       << "0 0 moveto\n"
       << "1.0 pagewidth mul dup scale\n"
       << "1 -1 scale\n"
       << "-.5 -.5 translate\n"
       << "1 " << sz << " div 1 " << sz << " div scale\n"
       << b << " " << b << " moveto\n"
       << sz-b << " " << b << " lineto\n"
       << sz-b << " " << sz-b << " lineto\n"
       << b << " " << sz-b << " lineto\n"
       << "closepath\n"
       << "0 setgray fill 1 setgray 0.01 setlinewidth \n"
       << "} def\n"
       << "/em { grestore gsave pagewidth 2 div 72 translate /Helvetica-Bold findfont 20 scalefont setfont dup stringwidth pop -.5 mul 0 moveto show grestore showpage } def\n";
  
  for (size_t i=0; i<codes.size(); ++i) {
    ostr << "sm ";
    code_t v = codes[i];
    for (uint y=0; y<d; ++y) {
      for (uint x=0; x<d; ++x) {
        if ((v&(code_t(1)<<code_t(bits-1)))!=0) {
          ostr << (x + m) << " " << (y + m) << " b ";
        }
        v = v<<code_t(1);
      }
    }
    ostr << "(Tag family " << bits << "h" << minimumHammingDistance << ", tag #" << i << ") em\n";
  }

  /*
  ostr << "/pagewidth 8.5 72 mul def                      \n"
       << "/pageheight 11 72 mul def                      \n"
       << "/maketag                                       \n"
       << "{                                              \n"
       << "  /img exch def                                \n"
       << "  /name exch def                               \n"
       << "  gsave                                        \n"
       << "  pagewidth 2 div pageheight 2 div translate   \n"
       << "  0 0 moveto                                   \n"
       << "  1.0 pagewidth mul dup scale                  \n"
       << "  1 -1 scale                                   \n"
       << "  -.5 -.5 translate                            \n"
       << "  " << sz << " " << sz << " 1 [ " << sz << " 0 0 " << sz << " 0 0 ] { img } image \n"
       << "  0 setlinewidth .5 setgray [0.002 0.01] 0 setdash \n"
       << "  0 0 moveto 1 0 lineto 1 1 lineto 0 1 lineto  \n"
       << "  closepath stroke                             \n"
       << "  grestore                                     \n"
       << "  gsave                                        \n"
       << "  pagewidth 2 div 72 translate                 \n"
       << "  /Helvetica-Bold findfont 20 scalefont setfont \n"
       << "  name                                         \n"
       << "  dup stringwidth pop -.5 mul 0 moveto         \n"
       << "  show                                         \n"
       << "  grestore                                     \n"
       << "  showpage                                     \n"
       << "} def                                          \n";

  for (size_t id = 0; id < codes.size(); id++) {

    cv::Mat_<cv::Vec3b> im = makeImage(id);

    // convert image into a postscript string
    int width = im.cols, height = im.rows;

    std::ostringstream sstr;

    for (int y = 0; y < height; y++) {
      code_t v = 0;
      int vlen = 0;

      for (int x = 0; x < width; x++) {
        int b = im(y,x)[0] > 0 ? 1 : 0;
        v = (v<<1) | b; vlen++;
      }

      // pad to a byte boundary.
      while ((vlen%8) != 0) {
        v = (v<<1) | 0; vlen++;
      }
      char fmt[1024];
      char buf[1024];
      snprintf(fmt, 1024, "%%0%dx", vlen/4);
      snprintf(buf, 1024, fmt, v);
      sstr << buf;

      //imgdata += String.format("%0"+(vlen/4)+"x", v);
    }

    ostr << "(id = " << id << ") <" << sstr.str() << "> maketag\n";


  }
  
  //outs.close();
  */

  

}

cv::Mat TagFamily::getWarp(const TagDetection& det) const {
  
  at::real sz = d + 2*blackBorder;
  at::real bb = 2*whiteBorder - blackBorder;

  at::Mat T = at::Mat::eye(3,3);
  T(0,2) = det.hxy.x;
  T(1,2) = det.hxy.y;
  
  at::Mat H = det.homography;
  
  at::Mat C = at::Mat::eye(3,3);
  
  C(0,0) =  2.0/sz;
  C(0,2) = -1 - bb/sz;
  C(1,1) = -2.0/sz;
  C(1,2) = 1 + bb/sz;
  
  return T * H * C;
  
}

cv::Mat merge(const cv::Mat& overlay, const cv::Mat& image) {
  cv::Mat result;
  result = 0.5*overlay + 0.5*image;
  return result;
}

cv::Mat TagFamily::superimposeDetection(const cv::Mat& image,
                                        const TagDetection& det) const {

  cv::Size dsize(image.cols, image.rows);
  cv::Mat dst = detectionImage(det, dsize, image.type());
  
  //return 0.5*dst + 0.5*image;
  return merge(dst, image);

}

cv::Mat TagFamily::superimposeDetections(const cv::Mat& image,
                                         const TagDetectionArray& detections) const {

  cv::Mat overlay;

  for (size_t i=0; i<detections.size(); ++i) {
    cv::Mat img = detectionImage(detections[i], image.size(), image.type());
    if (overlay.empty()) {
      overlay = img;
    } else {
      overlay = cv::max(overlay, img);
    }
  }

  if (overlay.empty()) {
    overlay = cv::Mat::zeros(image.size(), image.type());
  }

  //return 0.5*overlay + 0.5*image;
  return merge(overlay, image);

}

cv::Mat TagFamily::detectionImage(const TagDetection& det,
                                  const cv::Size& size, 
                                  int type) const {


  cv::Mat dst(size, type);

  cv::Mat im = makeImage(det.id);

  if (im.depth() != dst.depth()) {
    cv::Mat i2;
    at::real scl = 1.0;
    if (dst.depth() == CV_32F || dst.depth() == CV_64F) {
      scl = 1.0/255;
    }
    im.convertTo(i2, scl);
    im = i2;
  }

  if (im.channels() < dst.channels()) {
    cv::Mat i2;
    cv::cvtColor(im, i2, cv::COLOR_GRAY2RGB);
    im = i2;
  }

  cv::Mat W = getWarp(det);
  cv::warpPerspective(im, dst, W, size, CV_INTER_NN);

  return dst;

}


  /*
  static void main(int argc, char** argv) {

    if (argc != 3) {
      System.out.printf("Usage: <tagclass> <outputdir>\n");
      System.out.printf("Example: art.tag.Tag25h11 /tmp/tag25h11\n");
      return;
    }

    String cls = args[1];
    String dirpath = args[2] + "/";

    TagFamily tagFamily = (TagFamily) april.util.ReflectUtil.createObject(cls);
    if (tagFamily == null)
      return;

    try {
      File f = new File(dirpath);
      if (!f.exists())
        f.mkdirs();

      tagFamily.writeAllImagesMosaic(dirpath+"mosaic.png");
      tagFamily.writeAllImages(dirpath);
      tagFamily.writeAllImagesPostScript(dirpath+"alltags.ps");
    } catch (IOException ex) {
      System.out.println("ex: "+ex);
    }
  }
  */

