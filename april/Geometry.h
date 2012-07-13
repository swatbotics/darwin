#ifndef _GEOMETRY_H_
#define _GEOMETRY_H_

#include "AprilTypes.h"

struct Quad;
typedef std::vector<Quad*> QuadArray;

struct Segment;
typedef std::vector<Segment*> SegmentArray;

struct XYW {
  at::real x, y, w;
  XYW();
  XYW(at::real x, at::real y, at::real w);
  at::Point point() const;
};

typedef std::vector<XYW> XYWArray;
typedef std::map< int, XYWArray > ClusterLookup;

struct Segment {
  at::real x0, y0, x1, y1, length, theta;
  SegmentArray children;
  Segment* nextGrid;
};

class Gridder {
public:
  
  SegmentArray cells;
  at::real x0, y0, x1, y1;
  int width, height;
  at::real metersPerCell;

  Gridder(at::real x0, at::real y0, at::real x1, at::real y1, at::real metersPerCell);

  int sub2ind(int x, int y) const;

  void add(at::real x, at::real y, Segment* s);

  void find(at::real x, at::real y, at::real range, SegmentArray& results) const;

};

struct Quad {

  at::Point p[4];

  at::Point opticalCenter;
  at::Mat H;

  at::real observedPerimeter;

  Quad();

  Quad(const at::Point p[4],
       const at::Point& opticalCenter,
       at::real observedPerimeter);

  void recomputeHomography();

  at::Point interpolate(const at::Point& p) const;
  at::Point interpolate(at::real x, at::real y) const;


  at::Point interpolate01(const at::Point& p) const;
  at::Point interpolate01(at::real x, at::real y) const;

                          

};

bool intersect(const Segment* s1, const Segment* s2, at::Point& pinter);
at::real pdist(const at::Point& p1, const at::Point& p2);
at::real pdist(const at::Point& p, int x, int y);

struct GLineSegment2D {

  at::Point p1, p2;
  
  GLineSegment2D();

  GLineSegment2D(const at::Point& p1, const at::Point& p2);

  at::real length() const;

};

bool intersect(const GLineSegment2D& s1, const GLineSegment2D& s2, at::Point& pinter);

GLineSegment2D lsqFitXYW(const XYWArray& points);




#endif
