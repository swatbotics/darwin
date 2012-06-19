#include "TagDetector.h"
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "CameraUtil.h"
#include <map>


#ifdef __APPLE__
#include <Glut/Glut.h>
#else
#include <GL/glut.h>
#endif


void check_opengl_errors(const char* context) {
  GLenum error = glGetError();
  if (!context) { context = "error"; }
  if (error) {
    std::cerr << context << ": " << gluErrorString(error) << "\n";
  }
}



cv::VideoCapture* capture = 0;

GLuint camera_texture;
std::map<int, GLuint> tag_textures;

cv::Mat frame;
cv::Mat_<cv::Vec3b> uframe;
std::vector<unsigned char> rgbbuf;

TagFamily family("Tag36h11");
TagDetector detector(family);
TagDetectionArray detections;

bool flip = true;

int width = 640;
int height = 480;

double f = 500;
double s = 0.01;

// wb = 1, bb = 1, offs = 1.5/d
// wb = 2, bb = 1, offs = 3/d

int n = family.getTagRenderDimension();
//double 

void init() {

  family.whiteBorder = 1;

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  
  glLineWidth(2.0);
  glEnable(GL_LINE_SMOOTH);

  glGenTextures(1, &camera_texture);
  glBindTexture(GL_TEXTURE_2D, camera_texture);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  
  check_opengl_errors("init");

}

void drawCube() {


  double ss = 0.5*s;
  double sz = s;

  enum { npoints = 8, nedges = 12 };

  cv::Point3d src[npoints] = {
    cv::Point3d(-ss, -ss, 0),
    cv::Point3d( ss, -ss, 0),
    cv::Point3d( ss,  ss, 0),
    cv::Point3d(-ss,  ss, 0),
    cv::Point3d(-ss, -ss, sz),
    cv::Point3d( ss, -ss, sz),
    cv::Point3d( ss,  ss, sz),
    cv::Point3d(-ss,  ss, sz),
  };

  int edges[nedges][2] = {

    { 0, 1 },
    { 1, 2 },
    { 2, 3 },
    { 3, 0 },

    { 4, 5 },
    { 5, 6 },
    { 6, 7 },
    { 7, 4 },

    { 0, 4 },
    { 1, 5 },
    { 2, 6 },
    { 3, 7 }

  };

  glBegin(GL_LINES);

  for (int i=0; i<nedges; ++i) {
    glVertex3dv(&src[edges[i][0]].x);
    glVertex3dv(&src[edges[i][1]].x);
  }

  glEnd();

}

void setupFrustum(double fx, double fy, double cx, double cy,
                  double width, double height,
                  double n, double f) {
  
  double fwd[4][4];
  
  fwd[0][0] = 2*fx / width;
  fwd[2][0] = -(2*cx - width) / width;
  
  fwd[1][1] = 2*fy / height;
  fwd[2][1] =  (2*cy - height) / height;
  
  fwd[2][2] = -(f+n)/(f-n);
  fwd[3][2] = -(2*f*n)/(f-n);
  
  fwd[2][3] = -1;
  fwd[3][3] = 0;

  glMultMatrixd(&(fwd[0][0]));
  glScaled(flip ? -1 : 1, -1, -1);

}

void bindTexture(int id) {

  std::map<int, GLuint>::const_iterator i = tag_textures.find(id);
  
  if (i != tag_textures.end()) {

    glBindTexture(GL_TEXTURE_2D, i->second);

  } else {

    GLuint tex;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    tag_textures[id] = tex;

    cv::Mat_<unsigned char> img = family.makeImage(id);
    rgbbuf.resize(img.rows*img.cols*4);

    int offs=0;
    for (int y=0; y<img.rows; ++y) {
      for (int x=0; x<img.cols; ++x) {
        rgbbuf[offs++] = img(y,x);
        rgbbuf[offs++] = img(y,x);
        rgbbuf[offs++] = img(y,x);
        rgbbuf[offs++] = 255;
      }
    }

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA,
                 img.cols, img.rows, 0, GL_RGBA,
                 GL_UNSIGNED_BYTE, &(rgbbuf[0]));

    check_opengl_errors("texture");

  }

}

void display() {

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_PROJECTION);
  glPushMatrix();

  if (flip) {
    gluOrtho2D(width, 0, height, 0);
  } else {
    gluOrtho2D(0, width, height, 0);
  }

  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, camera_texture);
  glColor3ub(255,255,255);

  glBegin(GL_QUADS);
  glTexCoord2f(0, 0);  glVertex2f(0, 0);
  glTexCoord2f(1, 0);  glVertex2f(width, 0);
  glTexCoord2f(1, 1);  glVertex2f(width,  height);
  glTexCoord2f(0, 1);  glVertex2f(0,  height);
  glEnd();

  for (size_t i=0; i<detections.size(); ++i) {

    const TagDetection& d = detections[i];

    bindTexture(d.id);

    double k = 1 + (1.5*family.whiteBorder)/family.d;

    cv::Point2d p0 = d.interpolate(-k, -k);
    cv::Point2d p1 = d.interpolate( k, -k);
    cv::Point2d p2 = d.interpolate( k,  k);
    cv::Point2d p3 = d.interpolate(-k,  k);

    glBegin(GL_QUADS);

    glTexCoord2f(0, 1); glVertex2dv(&p0.x);
    glTexCoord2f(1, 1); glVertex2dv(&p1.x);
    glTexCoord2f(1, 0); glVertex2dv(&p2.x);
    glTexCoord2f(0, 0); glVertex2dv(&p3.x);

    glEnd();


  }

  glDisable(GL_TEXTURE_2D);

  glPopMatrix();
  glPushMatrix();

  setupFrustum(500, 500, 
               width*0.5, height*0.5,
               width, height, 0.01, 10);

  glColor3ub(0,255,0);
  glMatrixMode(GL_MODELVIEW);
  for (size_t i=0; i<detections.size(); ++i) {
    
    cv::Mat_<double> r, R, t, M = cv::Mat_<double>::eye(4,4);
    CameraUtil::homographyToPoseCV(f, f, s, 
                                   detections[i].homography,
                                   r, t);
    cv::Rodrigues(r, R);

    for (int i=0; i<3; ++i) {
      for (int j=0; j<3; ++j) {
        M(i,j) = R(j,i);
      }
      M(3,i) = t(i);
    }

    glPushMatrix();
    glMultMatrixd(&(M(0,0)));
    drawCube();
    glPopMatrix();

  }
  
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();

  glutSwapBuffers();

  check_opengl_errors("display");


}

void reshape(int w, int h) {

  width = w;
  height = h;

  glClearColor(1,1,1,1);
  glViewport(0,0,w,h);

  check_opengl_errors("reshape");

}

void keyboard(unsigned char value, int x, int y) {

  switch (value) {
  case 27: // esc
    detector.reportTimers();
    exit(1);
    break;
  case 'f':
  case 'F':
    std::cout << "flip!\n";
    flip = !flip;
    break;
  }


}

void idle() {

  *capture >> frame;

  if (frame.type() != CV_8UC3) {
    std::cerr << "bad frame!\n";
    exit(1);
  }

  cv::Point2d opticalCenter(0.5*frame.cols, 0.5*frame.rows);
  detector.process(frame, opticalCenter, detections);
  //std::cout << "got " << detections.size() << " detections.\n";

  uframe = frame;


  rgbbuf.resize(frame.cols*frame.rows*3);

  int offs = 0;

  for (int y=0; y<frame.rows; ++y) {
    for (int x=0; x<frame.cols; ++x) {
      rgbbuf[offs++] = uframe(y,x)[0];
      rgbbuf[offs++] = uframe(y,x)[1];
      rgbbuf[offs++] = uframe(y,x)[2];
    }
  }

  glBindTexture(GL_TEXTURE_2D, camera_texture);

  glTexImage2D(GL_TEXTURE_2D, 0, 3,
               frame.cols, frame.rows, 0, GL_BGR,
               GL_UNSIGNED_BYTE, &(rgbbuf[0]));

  glutPostRedisplay();

  check_opengl_errors("idle");

}


int main(int argc, char** argv) {

  capture = new cv::VideoCapture(0);
  cv::Mat frame;
  *capture >> frame;

  width = frame.cols;
  height = frame.rows;

  glutInit(&argc, argv);

  glutInitDisplayMode(GLUT_DOUBLE | GLUT_DEPTH | 
                      GLUT_RGB | GLUT_MULTISAMPLE);

  glutInitWindowPosition(100, 100);
  glutInitWindowSize(width, height);

  glutCreateWindow("AprilTags GL Demo");

  glutDisplayFunc(display);
  glutReshapeFunc(reshape);
  glutKeyboardFunc(keyboard);
  glutIdleFunc(idle);

  
  init();
  glutMainLoop();

  return 0;

}
