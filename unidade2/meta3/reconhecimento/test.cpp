#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
 
using namespace cv;
using namespace std;
 
int main()
{
  Mat gray, cropped;
  Mat canny;
  // src = imread( "img.png", 1 );

  // Transform it into the C++ cv::Mat format
  Mat image = imread( "img.png", 1 );
  namedWindow( "Hough Circle Transform Demo"); imshow( "Hough Circle Transform Demo", image );

  Rect box = Rect(560, 260, 300, 260);
  // Setup a rectangle to define your region of interest
  Mat src(image,box);
  // Crop the full image to that image contained by the rectangle myROI
  // Note that this doesn't copy the data
  //Mat croppedImage = image(myROI);
  // namedWindow( "Croped"); imshow( "Croped", crop );


  //resize(src,src,Size(663,690));
  cvtColor( src, gray, CV_BGR2GRAY );
  // Canny(InputArray image, OutputArray edges, double threshold1, double threshold2, int apertureSize=3, bool L2gradient=false )
  // 10 10
  Canny(gray, canny, 10,20);
  namedWindow("canny2"); imshow("canny2", canny>0);
  // Reduce the noise so we avoid false circle detection
  GaussianBlur( canny, canny, Size(9, 9), 2, 2 );
 
  vector<Vec3f> circles;
 
  // Apply the Hough Transform to find the circles
  HoughCircles( canny, circles, CV_HOUGH_GRADIENT, 0.5, 30, 200, 50, 0, 0 );
 
  // Draw the circles detected
  for( size_t i = 0; i < circles.size(); i++ )
  {
      Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
      int radius = cvRound(circles[i][2]);     
      // circle( src, center, 3, Scalar(0,255,0), -1, 8, 0 );// circle center     
      // circle( src, center, radius, Scalar(0,0,255), 3, 8, 0 );// circle outline
      circle( src, center, 3, Scalar(0,255,0), CV_FILLED, 8, 0 );// circle center     
      circle( src, center, radius, Scalar(0,0,255), 3, 8, 0 );// circle outline
      cout << "center : " << center << "\nradius : " << radius << endl;
   }
 
  // Show your results
  namedWindow( "Hough Circle Transform Demo"); imshow( "Hough Circle Transform Demo", src );

 
  waitKey(0);
  return 0;
}