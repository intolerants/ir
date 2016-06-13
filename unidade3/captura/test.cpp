#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>
 
using namespace cv;
 
void conv2(Mat src, int kernel_size)
{
    Mat dst,kernel;
    kernel = Mat::ones( kernel_size, kernel_size, CV_32F )/ (float)(kernel_size*kernel_size);
 
    /// Apply filter
    filter2D(src, dst, -1 , kernel, Point( -1, -1 ), 0, BORDER_DEFAULT );
    namedWindow( "filter2D Demo", CV_WINDOW_AUTOSIZE );imshow( "filter2D Demo", dst );
}
 
int main ( int argc, char** argv )
{
    Mat src;
 
    /// Load an image
    src = imread( "1.jpg" );
    if( !src.data )  { return -1; }
 
    conv2(src,3);
 
    waitKey(0);
    return 0;
}