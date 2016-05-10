// Ver em: http://hxr99.blogspot.com.br/2011/12/opencv-examples-camera-capture.html

// -------------------------------------------------------------------------------
//  OpenCV C/C++ Examples (Camera Capture)
// There are also python implementation for camera capture and showing image.
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;

void takePicture() {
	CvCapture* capture = 0;
    Mat frame, frameCopy, image;
	Mat gray, cropped;
	Mat canny;


        capture = cvCaptureFromCAM( -1 ); //0=default, -1=any camera, 1..99=your camera

	if ( !capture )
	{
		cout << "No camera detected" << endl;
	}

	cvNamedWindow( "result", CV_WINDOW_AUTOSIZE );

	if ( capture )
	{
		cout << "In capture ..." << endl;
		// for (;;)
		// {
		// 	IplImage* iplImg = cvQueryFrame( capture );
		// 	frame = iplImg;

		// 	if ( frame.empty() )
		// 		break;
		// 	if ( iplImg->origin == IPL_ORIGIN_TL )
		// 		frame.copyTo( frameCopy );
		// 	else
		// 		flip( frame, frameCopy, 0 );

		// 	cvShowImage( "result", iplImg );

		// 	if ( waitKey( 10 ) >= 0 )
		// 		break;
		// }
		IplImage* iplImg = cvQueryFrame( capture );
		cvShowImage( "result", iplImg );
		cvSaveImage("img.png",iplImg);
		//waitKey(0);
	}

	
	cvReleaseCapture( &capture );
	cvDestroyWindow( "result" );
}

void findCircles() {
  Mat gray, cropped;
  Mat canny;
  // src = imread( "img.png", 1 );

  // Transform it into the C++ cv::Mat format
  Mat image = imread( "img.png", 1 );
  // namedWindow( "Hough Circle Transform Demo"); imshow( "Hough Circle Transform Demo", image );

  Rect box = Rect(360, 170, 210, 180);
  // Rect box = Rect(360, 170, 200, 170);
  // Rect box = Rect(560, 260, 300, 260);
  
  // Setup a rectangle to define your region of interest
  // Mat src = imread( "img.png", 1 );
  Mat src(image,box);

  cvtColor( src, gray, CV_BGR2GRAY );
  
  // convertTo(OutputArray m, int rtype, double alpha=1, double beta=0 )
  
  gray.convertTo(gray, -1, 1.3, -200);
  namedWindow("gray"); imshow("gray", gray);
  moveWindow("gray", 270, 0);

  medianBlur(gray, gray, 3);
  namedWindow("blur"); imshow("blur", gray);
  moveWindow("blur", 270, 300);
  // moveWindow("gray", 280, 0);
  // Canny(InputArray image, OutputArray edges, double threshold1, double threshold2, int apertureSize=3, bool L2gradient=false )
  // 10 10
  Canny(gray, canny, 10, 140, 3, true);
  // Canny(gray, canny, 10, 150, 3, true);
  // Canny(gray, canny, 10, 150);
  // Canny(gray, canny, 20, 20);
  // Canny(gray, canny, 10, 20);
  namedWindow("canny1"); imshow("canny1", canny>0);
  moveWindow("canny1", 270*2, 0);
  // moveWindow("canny1", 280+220, 0);

  // medianBlur(InputArray src, OutputArray dst, int ksize)
  // medianBlur(canny, canny, 3);
  // namedWindow("canny2"); imshow("canny2", canny>0);
  // moveWindow("canny2", 270*3, 0);
  // // moveWindow("canny2", 500+220, 0);
  
  // Reduce the noise so we avoid false circle detection
/*  GaussianBlur(InputArray src, 
  			   OutputArray dst,
  			   Size ksize,
  			   double sigmaX,
  			   double sigmaY=0,
  			   int borderType=BORDER_DEFAULT )

  	Parameters:	

    src – input image; the image can have any number of channels, which are processed independently, but the depth should be CV_8U, CV_16U, CV_16S, CV_32F or CV_64F.
    dst – output image of the same size and type as src.
    ksize – Gaussian kernel size. ksize.width and ksize.height can differ but they both must be positive and odd. Or, they can be zero’s and then they are computed from sigma* .
    sigmaX – Gaussian kernel standard deviation in X direction.
    sigmaY – Gaussian kernel standard deviation in Y direction; if sigmaY is zero, it is set to be equal to sigmaX, if both sigmas are zeros, they are computed from ksize.width and ksize.height , respectively (see getGaussianKernel() for details); to fully control the result regardless of possible future modifications of all this semantics, it is recommended to specify all of ksize, sigmaX, and sigmaY.
    borderType – pixel extrapolation method (see borderInterpolate() for details).
*/
  GaussianBlur( canny, canny, Size(99, 99), 0.6, 0.6);
  // GaussianBlur( canny, canny, Size(99, 99), 3, 3);
  namedWindow("canny3"); imshow("canny3", canny>0);
  moveWindow("canny3", 270*4, 0);
  medianBlur(canny, canny, 5);
  namedWindow("canny3Blur"); imshow("canny3Blur", canny>0);
  moveWindow("canny3Blur", 270*4, 250);
  // moveWindow("canny3", 720+220, 0);
 
  vector<Vec3f> circles;
 
  // Apply the Hough Transform to find the circles
  HoughCircles( canny, circles, CV_HOUGH_GRADIENT, 0.5, 30, 200, 50, 0, 0 );
 
  // Draw the circles detected
  for( size_t i = 0; i < circles.size(); i++ )
  {
      Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
      int radius = cvRound(circles[i][2]);     
      circle( src, center, 3, Scalar(0,255,0), -1, 8, 0 );// circle center     
      circle( src, center, radius, Scalar(0,0,255), 3, 8, 0 );// circle outline
      // circle( src, center, 3, Scalar(0,255,0), CV_FILLED, 8, 0 );// circle center     
      // circle( src, center, radius, Scalar(0,0,255), 3, 8, 0 );// circle outline
      cout << "center : " << center << "\nradius : " << radius << endl;
   }
 
  // Show your results
  namedWindow( "src"); imshow( "src", src );
  moveWindow("src", 0, 0);

  waitKey(0);
}

int main( int argc, const char** argv )
{

	// takePicture();
	findCircles();

	return 0;
}
