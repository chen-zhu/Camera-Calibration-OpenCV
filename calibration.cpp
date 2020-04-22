#include <cctype>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <iostream>
#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

using namespace std;
using namespace cv;

const int h_dots = 6;
const int w_dots = 8;

bool camera_calibration(){
	return true;
}


int main( int argc, char** argv ) {
	VideoCapture capture;
	vector<string> imageList;
	vector<vector<Point2f> > imagePoints;
	string outputFilename = "calibrate_result";
	int winSize = 11;

	//Open camera here~
	capture.open(cameraId);

	namedWindow( "Calibration Window", 1 );

	int i = 0;

	while(i >=0){
		Mat view, viewGray;

		if( capture.isOpened() ) {
            Mat view0;
            capture >> view0;
            view0.copyTo(view);
        } else if( i < (int)imageList.size() ){
            view = imread(imageList[i], 1);
        }

        if(view.empty()) {
            if( imagePoints.size() > 0 )
                bool ret = camera_calibration();
            break;
        }

        imageSize = view.size();

        if( flipVertical )
            flip( view, view, 0 );

        vector<Point2f> pointbuf;
        cvtColor(view, viewGray, COLOR_BGR2GRAY);

        bool found = findChessboardCorners( view, boardSize, pointbuf,
                    CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);

        //if chess board is found, then get cornner corrdinate!

        if(found){
        	cornerSubPix( viewGray, pointbuf, Size(winSize,winSize), Size(-1,-1), TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 30, 0.0001 ));
        	drawChessboardCorners( view, boardSize, Mat(pointbuf), found );
        }

        imshow("Detection View", view);

        //detect ESC key here
        if((char)waitKey(capture.isOpened() ? 50 : 500) == 27){
        	break;
        }



	}


}












