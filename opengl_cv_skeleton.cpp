// Skeleton Code for CS290I Homework 1
// 2012-17, Jon Ventura and Chris Sweeney
// 2019-20 Tobias Hollerer

// adapt the include statements for your system:


#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>

#ifdef __APPLE__

#ifdef FREEGLUT
#include <GL/freeglut.h>
#else
#include <GLUT/glut.h>
#endif

#else

#include <GL/gl.h>
#ifdef FREEGLUT
#include <GL/freeglut.h>
#else
#include <GL/glut.h>
#endif

#endif



#include <cstdio>

cv::VideoCapture *cap = NULL;
int width = 640;
int height = 480;
cv::Mat image;

// a useful function for displaying your coordinate system
void drawAxes(float length)
{
  glPushAttrib(GL_POLYGON_BIT | GL_ENABLE_BIT | GL_COLOR_BUFFER_BIT) ;

  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE) ;
  glDisable(GL_LIGHTING) ;

  glBegin(GL_LINES) ;
  glColor3f(1,0,0) ;
  glVertex3f(0,0,0) ;
  glVertex3f(length,0,0);

  glColor3f(0,1,0) ;
  glVertex3f(0,0,0) ;
  glVertex3f(0,length,0);

  glColor3f(0,0,1) ;
  glVertex3f(0,0,0) ;
  glVertex3f(0,0,length);
  glEnd() ;


  glPopAttrib() ;
}

void display()
{
  // clear the window
  glClear( GL_COLOR_BUFFER_BIT );

  // show the current camera frame

  //based on the way cv::Mat stores data, you need to flip it before displaying it
  cv::Mat tempimage;
  cv::flip(image, tempimage, 0);
  glDrawPixels( tempimage.size().width, tempimage.size().height, GL_BGR, GL_UNSIGNED_BYTE, tempimage.ptr() );

  //////////////////////////////////////////////////////////////////////////////////
  // Here, set up new parameters to render a scene viewed from the camera.

  //set viewport
  glViewport(0, 0, tempimage.size().width, tempimage.size().height);

  //set projection matrix using intrinsic camera params
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();


  //glFrustum(-principalX / fx, (width - principalX) / fy, (principalY - height) / fy, principalY / fy, 1, 500);  

  //gluPerspective is arbitrarily set, you will have to determine these values based
  //on the intrinsic camera parameters
  gluPerspective(60, tempimage.size().width*1.0/tempimage.size().height, 1, 20); 

  //you will have to set modelview matrix using extrinsic camera params
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  gluLookAt(0, 0, 5, 0, 0, 0, 0, 1, 0);  


  /////////////////////////////////////////////////////////////////////////////////


// Drawing routine

  //now that the camera params have been set, draw your 3D shapes
  //first, save the current matrix
  glPushMatrix();
    //move to the position where you want the 3D object to go
    glTranslatef(0, 0, 0); //this is an arbitrary position for demonstration
    //you will need to adjust your transformations to match the positions where
    //you want to draw your objects(i.e. chessboard center, chessboard corners)
    glutSolidTeapot(0.5);
    //glutSolidSphere(.3, 100, 100);
    drawAxes(1.0);
  glPopMatrix();
  

  // show the rendering on the screen
  glutSwapBuffers();

  // post the next redisplay
  glutPostRedisplay();
}

void reshape( int w, int h )
{
  // set OpenGL viewport (drawable area)
  glViewport( 0, 0, w, h );
}

void mouse( int button, int state, int x, int y )
{
  if ( button == GLUT_LEFT_BUTTON && state == GLUT_UP )
    {

    }
}

void keyboard( unsigned char key, int x, int y )
{
  switch ( key )
    {
    case 27: //escape key to quit~~
      exit(0);
      break;
    default:
      break;
    }
}



void idle()
{
  // grab a frame from the camera
  (*cap) >> image;
}

int main( int argc, char **argv )
{
  int w,h;

  if ( argc == 1 ) {
    // start video capture from camera
    cap = new cv::VideoCapture(0);
  } else if ( argc == 2 ) {
    // start video capture from file
    cap = new cv::VideoCapture(argv[1]);
  } else {
    fprintf( stderr, "usage: %s [<filename>]\n", argv[0] );
    return 1;
  }

  // check that video is opened
  if ( cap == NULL || !cap->isOpened() ) {
    fprintf( stderr, "could not start video capture\n" );
    return 1;
  }

  // get width and height
  w = (int) cap->get( cv::CAP_PROP_FRAME_WIDTH );
  h = (int) cap->get( cv::CAP_PROP_FRAME_HEIGHT );
  // On Linux, there is currently a bug in OpenCV that returns 
  // zero for both width and height here (at least for video from file)
  // hence the following override to global variable defaults: 
  width = w ? w : width;
  height = h ? h : height;

  // initialize GLUT
  glutInit( &argc, argv );
  glutInitDisplayMode( GLUT_RGBA | GLUT_DOUBLE );
  glutInitWindowPosition( 20, 20 );
  glutInitWindowSize( width, height );
  glutCreateWindow( "OpenGL / OpenCV Example" );

  // set up GUI callback functions
  glutDisplayFunc( display );
  glutReshapeFunc( reshape );
  glutMouseFunc( mouse );
  glutKeyboardFunc( keyboard );
  glutIdleFunc( idle );

  // start GUI loop
  glutMainLoop();

  return 0;
}
