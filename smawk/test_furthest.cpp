
#include "Smawk.h"
#include "SmawkDebug.h"

#include <ctime>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/extremal_polygon_2.h>
#include <CGAL/all_furthest_neighbors_2.h>

typedef cv::Mat_<cv::Vec3b> Mat3b;
typedef std::vector<cv::Point> PointArray;

typedef CGAL::Simple_cartesian<int>   CGK;
typedef CGK::Point_2                  CGPoint;
typedef CGAL::Polygon_2<CGK>          CGPolygon_2;

CGPoint cv2cg(const cv::Point& p) {
  return CGPoint(p.x, p.y);
}

int distsq(const cv::Point& p0, const cv::Point& p1) {
  cv::Point diff = p1-p0;
  return diff.dot(diff);
}

static int area2x(const cv::Point& a, 
                  const cv::Point& b,
                  const cv::Point& c) {

  cv::Point u = b-a;
  cv::Point v = c-a;
  return u.x*v.y - u.y*v.x;
    
}

class PointDistMat {
public:

  typedef int value_type;
  
  const PointArray& points;

  PointDistMat(const PointArray& p): points(p) {}

  size_t rows() const { return points.size(); }
  size_t cols() const { return points.size() * 2 - 1; }

  int indexOf(size_t r, size_t c) const {

    size_t n = points.size();

    if (c < r) {
      return c-r;
    } else if (c > r + n - 2) {
      return r+n-2 - c;
    } else {
      assert(c >= r);
      return (c + 1) % n;
    }

  }

  int operator()(size_t r, size_t c) const {

    int i = indexOf(r, c);

    if (i < 0) { 
      return i;
    } else {
      return distsq(points[r], points[i]);
    }

  }
  

};

template <class Tmat>
class CGAL_Matrix_Wrapper {
public:

  const Tmat& m;
  typedef typename Tmat::value_type Value;

  CGAL_Matrix_Wrapper(const Tmat& mat): m(mat) {}

  int number_of_rows() const { return m.rows(); }
  int number_of_columns() const { return m.cols(); }

  Value operator()(int i, int j) const {
    return m(i,j);
  }
  

};

namespace CGAL {

  template <class Tmat>
  class DMat {
  private:

    const Tmat& mat;
    IndexArray colidx;
    int lod;
    size_t roffs;

  public:

    typedef typename Tmat::value_type Value;
    typedef DMat<Tmat> ThisType;

    static size_t getOffset(size_t l) { return (1 << l) - 1; }

    DMat(const Tmat& m, int l=0): mat(m), lod(l), roffs(getOffset(l)) {

      colidx.resize(mat.cols());
      for (size_t j=0; j<mat.cols(); ++j) { colidx[j] = j; }
      
    }

    DMat(const Tmat& m, 
         const IndexArray& c,
         int l=0): mat(m), colidx(c), lod(l), roffs(getOffset(l)) {}

    int number_of_rows() const { return (mat.rows()+roffs) >> lod; }
    int number_of_columns() const { return colidx.size(); }

    void shrink_to_quadratic_size() {
      colidx.resize(number_of_rows());
    }

    void replace_column(int o, int n) {
      colidx[o] = colidx[n];
    }

    ThisType* extract_all_even_rows() const {
      return new ThisType( mat, colidx, lod + 1 );
    }

    Value operator()(int i, int j) const {
      return mat( i << lod, colidx[j] );
    }

    Value operator()( std::pair<int, int> p ) const {
      return this->operator()(p.first, p.second);
    }
  

  };

}

class PointTriMat {
public:

  typedef int value_type;
  const PointArray& points;

  PointTriMat(const PointArray& p): points(p) {}

  size_t rows() const { return points.size()-2; }
  size_t cols() const { return points.size()-2; }

  void indexOf(size_t r, size_t c, size_t& i1, size_t& i2) {
    i1 = r+1;
    i2 = c+2;
  }
  
  int operator()(size_t r, size_t c) const {
    if (r > c) {
      return c-r;
    } else {
      return area2x(points[0], points[r+1], points[c+2]);
    }
  }

};

size_t naiveAllFurthest(const PointArray& cpoints, 
                        IndexArray& fidx) {

  size_t count = 0;
  fidx.resize(cpoints.size());

  for (size_t i=0; i<cpoints.size(); ++i) {

    const cv::Point& pi = cpoints[i]; 

    int maxdist = 0;
    fidx[i] = i;

    for (size_t j=0; j<cpoints.size(); ++j) {
      if (j != i) {
        const cv::Point& pj = cpoints[j]; 
        int dist = distsq(pi, pj); ++count;
        if (dist > maxdist) { 
          fidx[i] = j;
          maxdist = dist;
        }
      }
    }

  }

  return count;

}

size_t cgalAllFurthest(const PointArray& cpoints,
                       IndexArray& fidx) {


  PointDistMat morig(cpoints);
  DebugMatrix_t<PointDistMat> dm(morig, 0);
  std::less<int> ccmp;

  //CGAL::Dynamic_matrix< CGAL_Matrix_Wrapper< DebugMatrix_t<PointDistMat> > > m(dm);

  CGAL::DMat< DebugMatrix_t< PointDistMat > > m(dm);
  //CGAL::DMat< PointDistMat > m(morig);

  fidx.resize(cpoints.size());
  CGAL::monotone_matrix_search(m, fidx.begin(), ccmp);

  return dm.count;

}


enum {
  radius = 320,
  width = 750,
  height = 750,
  scl = 32,
};


int main(int argc, char** argv) {

  //cv::RNG rng(time(NULL));
  cv::RNG rng(12345);

  std::greater<int> cmp;

  int npoints = 1000;
  double stddev = 0.01;

  if (argc > 1) {
    npoints = atoi(argv[1]);
  }
  if (argc > 2) {
    stddev = atof(argv[2]);
  }

  while (1) {

    PointArray points;
    PointArray cpoints;

    for (int i=0; i<npoints; ++i) {

      double angle = double(i)*2*M_PI/npoints;
      int x = scl * (width/2 + radius * cos(-angle) + rng.gaussian(stddev));
      int y = scl * (height/2 + radius * sin(-angle) + rng.gaussian(stddev));
      points.push_back( cv::Point(x,y) );

    }

    cv::convexHull( points, cpoints );

    PointDistMat morig(cpoints);

    PointTriMat torig(cpoints);
    IndexedMatrix_t<PointTriMat> t(torig);
    
    //checkMonotone(t, cmp);

    DebugMatrix_t<PointDistMat> dm(morig, 0);
    IndexedMatrix_t< DebugMatrix_t<PointDistMat> > m(dm);


    IndexArray sopt_furthest, bopt_furthest, nopt_furthest, 
      copt_furthest, dopt_furthest;

    Timer stime;
    dm.count = 0;
    smawk(dm, cmp, sopt_furthest);
    stime.stop();
    std::cout << "smawk finished in " << stime.elapsed() << " with " << dm.count << " evaluations.\n";

    Timer btime;
    dm.count = 0;
    ascendingSearch(dm, cmp, bopt_furthest);
    btime.stop();
    std::cout << "brute finished in " << btime.elapsed() << " with " << dm.count << " evaluations (" << 100*btime.elapsed() / stime.elapsed() << "%)\n";

    Timer dtime;
    dm.count = 0;
    divideAndConquer(dm, cmp, dopt_furthest);
    dtime.stop();
    std::cout << "dandc finished in " << dtime.elapsed() << " with " << dm.count << " evaluations (" << 100*dtime.elapsed() / stime.elapsed() << "%)\n";

    Timer ntime;
    size_t ncount = naiveAllFurthest(cpoints, nopt_furthest);
    ntime.stop();
    std::cout << "naive finished in " << ntime.elapsed() << " with " << ncount << " evaluations (" << 100*ntime.elapsed() / stime.elapsed() << "%)\n";

    Timer ctime;
    size_t ccount = cgalAllFurthest(cpoints, copt_furthest);
    ctime.stop();
    std::cout << "cgal  finished in " << ctime.elapsed() << " with " << ccount << " evaluations (" << 100*ctime.elapsed() / stime.elapsed() << "%)\n";


    std::cout << "\n";

    assert(bopt_furthest.size() == sopt_furthest.size());
    assert(nopt_furthest.size() == sopt_furthest.size());
    assert(copt_furthest.size() == sopt_furthest.size());
    assert(dopt_furthest.size() == sopt_furthest.size());

    for (size_t i=0; i<bopt_furthest.size(); ++i) {

      assert( morig(i, bopt_furthest[i]) == morig(i, sopt_furthest[i]) );

      assert( morig(i, bopt_furthest[i]) == morig(i, dopt_furthest[i]) );


      assert( morig(i, sopt_furthest[i]) == 
              distsq( cpoints[i],
                     cpoints[morig.indexOf(i, sopt_furthest[i])] ) );
      
      assert( morig(i, sopt_furthest[i]) ==
              distsq(cpoints[i], cpoints[nopt_furthest[i]]) );

      assert( morig(i, bopt_furthest[i]) == morig(i, copt_furthest[i]) );

    }



    //std::cout << m << "\n";
    //debugPrintMatrix(m, cmp);

    Mat3b image(width, height);
    image = cv::Vec3b(255,255,255);
    double iscl = 1.0/scl;
    
    for (size_t i=0; i<cpoints.size(); ++i) {
      size_t ii = (i+1)%cpoints.size();
      cv::line(image, cpoints[i]*iscl, cpoints[ii]*iscl, CV_RGB(0,0,255), 1, CV_AA);
    }

    for (size_t i=0; i<cpoints.size(); ++i) {
      size_t ii = morig.indexOf(i,sopt_furthest[i]);
      cv::line(image, cpoints[i]*iscl, cpoints[ii]*iscl, CV_RGB(255,0,255), 1, CV_AA);
    }

    for (size_t i=0; i<cpoints.size(); ++i) {
      cv::circle(image, cpoints[i]*iscl, 3, CV_RGB(255,0,0), CV_FILLED, CV_AA);
      std::ostringstream ostr;
      ostr << i;
      cv::putText(image, ostr.str(), 
                  cpoints[i]*iscl + cv::Point(5, 5), 
                  cv::FONT_HERSHEY_SIMPLEX,
                  0.4, CV_RGB(64,0,0), 1, CV_AA);
    }

    cv::namedWindow("furthest");
    cv::imshow("furthest", image);
    int k = cv::waitKey();
    if (k == 27) { break; }

  }

  return 0;

}
