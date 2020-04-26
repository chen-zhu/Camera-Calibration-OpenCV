#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/core/persistence.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core.hpp>
#include <fstream>

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
using namespace std;

cv::VideoCapture *cap = NULL;
int width = 1280;
int height = 720;
cv::Mat image;
cv::Mat grey_image;
cv::Mat camera_matrix = cv::Mat::eye(3,3,CV_64F);;
cv::Mat distortion_coefficients = cv::Mat::zeros(4,1,CV_64F);;

//chessboard dimention
const int w_chessboard = 8;
const int h_chessboard = 6;
cv::Size patternsize(8,6);

int obj = 1; //1 -> teapot, 2 -> sphere

// a useful function for displaying your coordinate system
void drawAxes(float length){
  glPushAttrib(GL_POLYGON_BIT | GL_ENABLE_BIT | GL_COLOR_BUFFER_BIT) ;

  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE) ;
  glDisable(GL_LIGHTING) ;

  glBegin(GL_LINES) ;
  glColor3f(1,0,0) ;
  glVertex3f(0,0,0) ;
  glVertex3f(length,0,0);

  glColor3f(0,1,0) ;
  glVertex3f(0,0,0) ;
  glVertex3f(0,-length,0);

  glColor3f(0,0,1) ;
  glVertex3f(0,0,0) ;
  glVertex3f(0,0,-length);
  glEnd();


  glPopAttrib() ;
}

//https://www.learnopencv.com/camera-calibration-using-opencv/

//https://www.opengl.org/resources/libraries/glut/spec3/node89.html
//https://www.codemiles.com/c-opengl-examples/drawing-teapot-using-opengl-t9010.html
void placeTeapot(){
  glPushAttrib(GL_POLYGON_BIT | GL_ENABLE_BIT | GL_COLOR_BUFFER_BIT);
  glPushMatrix();
    glColor3f(0.37,0.85,0.62);
    //make it vertical to the screen
    //glRotatef(90,1,0,0);
    glRotatef(-90,1,0,0);
    glTranslatef(3.5, 0.0, -2.5);
    glutSolidTeapot(2);
  glPopMatrix();
  glPopAttrib();
}

void placeSphere(){
  glPushAttrib(GL_POLYGON_BIT | GL_ENABLE_BIT | GL_COLOR_BUFFER_BIT);
  glPushMatrix();
  
  glTranslatef(-1.0, 0.0, 0.0);
  glColor3f(0.37,0.85,0.62);

  //glutSolidTeapot(2.0);
  for(int i = 0; i < h_chessboard; i++){
    glPushMatrix();
    for (int j = 0; j < w_chessboard; j++){
        glTranslatef(1.0, 0.0, 0.0);
        glutSolidSphere(0.3, 20, 20);
    }
    glPopMatrix();

    glTranslatef(0.0, -1.0, 0.0);
}


  glPopMatrix();
  glPopAttrib();
}



void display(){
  // clear the window
  glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

  //create chessboard absolute corrdinate (000, 010, 020, etc).
  vector<cv::Point3f> chessboard_cord;
  for (int h = 0; h < h_chessboard; h++) {
    for (int w = 0; w < w_chessboard; w++) {
      chessboard_cord.push_back(cv::Point3f(w, h, 0));
    }
  }

  cv::Mat chessboard_cord_MAT = cv::Mat(chessboard_cord);

  cv::Mat calculated_mat = cv::Mat::eye(4, 4, CV_64FC1);

  // show the current camera frame

  //based on the way cv::Mat stores data, you need to flip it before displaying it
  cv::Mat tempimage;
  bool corner_detected = false; //--> used as a flag to represent if the cornner of chessboard is in the frame. 
  (*cap) >> image;
  //flip image here~
  cv::resize(image,tempimage,cv::Size(width,height));
  cv::undistort(tempimage, image, camera_matrix, distortion_coefficients);
  cv::flip(image, tempimage, 0);

  glDisable(GL_DEPTH_TEST);
  glDrawPixels( tempimage.size().width, tempimage.size().height, GL_BGR, GL_UNSIGNED_BYTE, tempimage.ptr() );
  glEnable(GL_DEPTH_TEST);

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
  //https://stackoverflow.com/questions/16571981/gluperspective-parameters-what-do-they-mean
  //float fovy = 2 * atan((height/2) / camera_matrix.at<double>(1,1) ) * 180 / 3.14159;
  //https://photo.stackexchange.com/questions/21536/how-can-i-calculate-vertical-field-of-view-from-horizontal-field-of-view
  //2arctan(d/2f)*180/pi
  double fx = camera_matrix.at<double>(0,0);
  double fy = camera_matrix.at<double>(1,1); 
  double field_of_view_angle = 2.0 * atan(height/(2.0*fy) ) * 180 / 3.1415926;
  //WTH?? --> double totally messed it up. make sure to multiply by 1.0
  double aspect_ratio = (fy/fx) * ((1.0 * width)/(1.0 * height));
  //float aspect_ratio = width * 1.0 / height;

  if (!image.empty()){
    //cv::Mat viewMAT = cv::Mat::zeros(4, 4, CV_64FC1);
    vector<cv::Point2f> found_corners;

    //https://docs.opencv.org/2.4/modules/imgproc/doc/miscellaneous_transformations.html
    //cv::cvtColor(image, grey_image, CV_RGB2GRAY);
    //https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#bool%20findChessboardCorners(InputArray%20image,%20Size%20patternSize,%20OutputArray%20corners,%20int%20flags)
    
    corner_detected = findChessboardCorners(
              tempimage, 
              patternsize, 
              found_corners, 
              cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE //+ cv::CALIB_CB_FAST_CHECK
          );

    //gluPerspective(field_of_view_angle, //field_of_view_angle
    //          aspect_ratio, //aspect
    //          0.1,  //zNear
    //          20   //zFar
    //        );
    //gluPerspective(field_of_view_angle, //field_of_view_angle
    //          aspect_ratio, //aspect
    //          0.1,  //zNear
    //          200   //zFar
    //        );
    //Setting the projection matrix here~
    //glViewport(0,0,width,height);
    //glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(field_of_view_angle, //field_of_view_angle
              aspect_ratio, //aspect
              0.01,  //zNear
              100   //zFar
            );
    //gluPerspective(60, tempimage.size().width*1.0/tempimage.size().height, 1, 20); 
    //convert cord here~
    if(corner_detected){
      
      //TODO: try it out. 
      //cv::Size zeroZone(-1,-1);
      //vector<cv::Point2f> found_corners2;
      //cv::cornerSubPix(grey_image, cv::Mat(found_corners2), patternsize, zeroZone, (cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE));

      //https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#bool%20solvePnP(InputArray%20objectPoints,%20InputArray%20imagePoints,%20InputArray%20cameraMatrix,%20InputArray%20distCoeffs,%20OutputArray%20rvec,%20OutputArray%20tvec,%20bool%20useExtrinsicGuess,%20int%20flags)
      //Finds an object pose from 3D-2D point correspondences
      cv::Mat rvec; //Output vector of rotation vectors 
      cv::Mat rotation; // -> cv::Rodrigues(rvec, rotation);
      cv::Mat tvec; //Output vector of translation vectors estimated for each pattern view
      cv::solvePnP(chessboard_cord_MAT, found_corners, camera_matrix, distortion_coefficients, rvec, tvec, false);//, CV_ITERATIVE);
      //https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#void%20Rodrigues(InputArray%20src,%20OutputArray%20dst,%20OutputArray%20jacobian)
      //https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#void%20Rodrigues(InputArray%20src,%20OutputArray%20dst,%20OutputArray%20jacobian)
      
      //Hummm Somehow we have to invert x-axis rotation --> because teapot shoudl rotate in the different than chessboard
      rvec.at<double>(1,0) = -rvec.at<double>(1,0);
      //rvec.at<double>(2,0) = -rvec.at<double>(2,0);
      cv::Rodrigues(rvec, rotation);

      /*
      vector<cv::Point2f> projected;
      cv::projectPoints(chessboard_cord_MAT, rvec, tvec, camera_matrix, distortion_coefficients, projected);
      for (int i = 0; i < projected.size(); i++) {
        cv::Point2f pt = projected.at(i);
        cv::circle(image, pt, 5, cv::Scalar(0, 0, 255), 5);
      }*/
      
      //concluded that chessboard_cord_MAT is correct~

      /*GLdouble calced_matrix[16] = {
                    -rotation.at<double>(0,0), -rotation.at<double>(0,1), -rotation.at<double>(0,2), 0,
                    rotation.at<double>(1,0), rotation.at<double>(1,1), rotation.at<double>(1,2), 0,
                    rotation.at<double>(2,0), rotation.at<double>(2,1), rotation.at<double>(2,2), 0,
                    tvec.at<double>(0,0),     -tvec.at<double>(1,0),    tvec.at<double>(2,0),     1
                  };
      */ 

      /*
      GLdouble calced_matrix[16] = {
                    rotation.at<double>(0,0), -rotation.at<double>(1,0), rotation.at<double>(2,0),  tvec.at<double>(0,0),
                    rotation.at<double>(0,1), -rotation.at<double>(1,1), -rotation.at<double>(2,1), -tvec.at<double>(1,0),
                    rotation.at<double>(0,2), rotation.at<double>(1,2),  rotation.at<double>(2,0),  0,
                    0,                        0,                         tvec.at<double>(2,2),      1
                  };
      */

      GLdouble calced_matrix[16] = {
                    rotation.at<double>(0,0), rotation.at<double>(0,1), rotation.at<double>(0,2), 0,
                    rotation.at<double>(1,0), rotation.at<double>(1,1), rotation.at<double>(1,2), 0,
                    rotation.at<double>(2,0), rotation.at<double>(2,1), rotation.at<double>(2,2), 0,
                    tvec.at<double>(0,0),     -tvec.at<double>(1,0),    tvec.at<double>(2,0),     1
                  };
      
      glMatrixMode(GL_MODELVIEW);
      glLoadIdentity();
      glScalef(1.0,-1.0,-1.0);
      //glLoadMatrixd(calced_matrix);
      //Applies subsequent matrix operations to the modelview matrix stack
      glMultMatrixd(calced_matrix);
      //This is a state machine!

      //cv::Mat converted = cv::Mat::eye(4,4,CV_64F);
      //converted.at<double>(1,1) = -1.0f;
      //converted.at<double>(2,2) = -1.0f;
      //TODO: redundant here~
      /*for (int row = 0; row < 3; ++row){
        for (int col = 0; col < 3; ++col) {
          viewMAT.at<double>(row, col) = rotation.at<double>(row, col);
        }
        viewMAT.at<double>(row, 3) = tvec.at<double>(row,0);
      }
      viewMAT.at<double>(3,3) = 1.0f;

      //now transfer openCV to openGL cord.
      viewMAT = converted * viewMAT;

      //completely flip~
      transpose(viewMAT, calculated_mat);
      */


    }
  }

  //you will have to set modelview matrix using extrinsic camera params
  //glMatrixMode(GL_MODELVIEW);
  //glLoadIdentity();
  //gluLookAt(0, 0, 5, 0, 0, 0, 0, 1, 0);  


  /////////////////////////////////////////////////////////////////////////////////


  // Drawing routine

  //now that the camera params have been set, draw your 3D shapes
  //first, save the current matrix

  if(corner_detected){
    glPushMatrix();

      //glLoadIdentity();
      //move to the position where you want the 3D object to go
      //glTranslatef(0, 0, 0); //this is an arbitrary position for demonstration
      //you will need to adjust your transformations to match the positions where
      //you want to draw your objects(i.e. chessboard center, chessboard corners)
      //glutSolidTeapot(0.5);
      //glutSolidSphere(.3, 100, 100);
      //drawAxes(1.0);

      //if(corner_detected){
        //glLoadMatrixd(&calculated_mat.at<double>(0,0));
      //}
      
      //Flip it here~ OpenGL and OpenCV uses different cord system~~~! 
      //glScalef(1.0, -1.0, -1.0);

      glPushMatrix();
        //draw axis @000
        drawAxes(100);
      glPopMatrix();

      if(obj == 1){
        placeTeapot();
      } else {
        placeSphere();
      }

    glPopMatrix();
  }

  //glPushMatrix();
    //drawAxes(1.0);
  //glPopMatrix();

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
    case 'q': //escape key to quit~~
      exit(0);
      break;
    case '1': //teapot~~
      obj = 1;
      break;
    case '2': //sphere~~
      obj = 2;
      break;
    case ' ': //OMG seriously?
      if(obj == 1){obj = 2;}
      else {obj = 1;}
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

//reference: 
//https://docs.opencv.org/master/d4/da4/group__core__xml.html
//https://docs.opencv.org/master/de/dd9/classcv_1_1FileNode.html
//openCV demo/cpp/calibration.cpp
void readCameraXMLConfig(string filename){
  cv::FileStorage xmlFile(filename, 0);
  xmlFile["camera_matrix"] >> camera_matrix;
  xmlFile["distortion_coefficients"] >> distortion_coefficients;
}

int main( int argc, char **argv )
{

  //Read in camera info here
  readCameraXMLConfig("camera_calib_result.xml");

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


  cout << "Input Screen dimention: " << width << " x " << height << "\n";

  // initialize GLUT
  glutInit( &argc, argv );
  glutInitDisplayMode( GLUT_RGBA | GLUT_DOUBLE );
  glutInitWindowSize( width, height );
  glutInitWindowPosition( 20, 20 );
  glutCreateWindow( "OpenGL+OpenCV" );

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
