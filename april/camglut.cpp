#ifdef __APPLE__
#include <Glut/Glut.h>
#else
#include <GL/glut.h>
#endif

#include <cstdio>
#include <stdlib.h>
#include <opencv2/highgui/highgui.hpp>

// display, reshape, keyboard, idle

cv::VideoCapture capture;
cv::Mat_<cv::Vec3b> frame;
std::vector<unsigned char> rgbbuf;

uint32_t w, h;

GLuint camera_texture = 0;
GLuint font_texture = 0;

const bool do_double = true;

clock_t start = 0;
uint32_t frame_count = 0;

bool deBayer = true;


void init() {

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  glGenTextures(1, &camera_texture);
  glBindTexture(GL_TEXTURE_2D, camera_texture);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

  /*
    glGenTextures(1, &font_texture);
    glBindTexture(GL_TEXTURE_2D, font_texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  */

  /*
  int swap_interval = 0;
  CGLContextObj cgl_context = CGLGetCurrentContext();
  CGLSetParameter(cgl_context, kCGLCPSwapInterval, &swap_interval);
  */


}

void display() {


  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, camera_texture);
  glColor3ub(255,255,255);
    
  glBegin(GL_QUADS);
  glTexCoord2f(0, 1);  glVertex2f(-1, -1);
  glTexCoord2f(1, 1);  glVertex2f(1, -1);
  glTexCoord2f(1, 0);  glVertex2f(1,  1);
  glTexCoord2f(0, 0);  glVertex2f(-1,  1);
  glEnd();

  if (do_double) {
    glutSwapBuffers();
  } else {
    glFlush();
  }

}


void reshape(int w, int h) {
  glViewport(0, 0, w, h);
}

void keyboard(unsigned char value, int x, int y) {
  // TODO: deal
  if (value == 27) { // esc
    exit(0);
  }

}


void idle() {

  capture >> frame;
    

  if (!frame.empty()) {

    if (start == 0) {
      start = clock();
    }

    ++frame_count;

    double elapsed = (clock() - start) / double(CLOCKS_PER_SEC);

    if (frame_count && frame_count % 100 == 0) {
      double fps = frame_count / elapsed;
      printf("%d frames in %f sec (%f fps)\n", frame_count, elapsed, fps);
    }

    char buf[1024];
    sprintf(buf, "%0.3f", elapsed);

    cv::putText(frame, buf, cv::Point(10, h-10), 
                CV_FONT_HERSHEY_SIMPLEX,
                5, CV_RGB(255,255,255),
                2, 8);

    rgbbuf.resize(frame.cols*frame.rows*3);
    
    int offs = 0;
    
    for (int y=0; y<frame.rows; ++y) {
      for (int x=0; x<frame.cols; ++x) {
        rgbbuf[offs++] = frame(y,x)[0];
        rgbbuf[offs++] = frame(y,x)[1];
        rgbbuf[offs++] = frame(y,x)[2];
      }
    }
    
    glBindTexture(GL_TEXTURE_2D, camera_texture);
    
    glTexImage2D(GL_TEXTURE_2D, 0, 3,
                 frame.cols, frame.rows, 0, GL_BGR,
                 GL_UNSIGNED_BYTE, &(rgbbuf[0]));
    
    
    fprintf(stderr, ".");

    glutPostRedisplay();

  } else {

    fprintf(stderr, "X");

  }
  
}

int main(int argc, char** argv) {

  int device = 0;

  if (argc > 1) {
    char* endptr = 0;
    device = strtol(argv[1], &endptr, 10);
    if (!endptr || *endptr) {
      fprintf(stderr, "bad device!\n");
      exit(1);
    }
  }


  capture.open(device);
  
  if (!capture.isOpened()) {
    fprintf(stderr, "error initializing camera!\n");
    exit(1);
  }
  
  capture >> frame;

  if (frame.empty()) {
    fprintf(stderr, "error getting frame!\n");
    exit(1);
  }

  w = frame.cols;
  h = frame.rows;

  glutInitDisplayMode(GLUT_RGB | (do_double ? GLUT_DOUBLE : 0));
  glutInitWindowPosition(100, 100);
  glutInitWindowSize(w, h);
  glutInit(&argc, argv);
  glutCreateWindow("CAM Test");
  init();
  glutDisplayFunc(display);
  glutReshapeFunc(reshape);
  glutKeyboardFunc(keyboard);
  glutIdleFunc(idle);

  glutMainLoop();

  return 0;

}
